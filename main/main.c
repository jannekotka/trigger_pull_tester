#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"

// TB6600 Stepper Driver pins
#define STEP_PIN GPIO_NUM_25
#define DIR_PIN  GPIO_NUM_26
#define EN_PIN   GPIO_NUM_27

// HX711 Load Cell pins
#define HX711_DT_PIN  GPIO_NUM_16
#define HX711_SCK_PIN GPIO_NUM_17

#define TAG "STEPPER"

// Step pulse timing (microseconds)
#define STEP_PULSE_WIDTH_US 5   // Pulse width (10us for reliability)
#define STEP_DELAY_MIN_US   400 // Slower speed for 1/8 microstepping (500 Hz)
#define ACCEL_STEPS         640   // Number of steps to accelerate/decelerate

// Motor and lead screw configuration
#define STEPS_PER_REV       200   // 1.8° motor = 200 steps/rev
#define MICROSTEPS          32     // Microstepping setting
#define LEAD_SCREW_PITCH_MM 2.0   // 2mm lead screw
#define STEPS_PER_MM        ((STEPS_PER_REV * MICROSTEPS) / LEAD_SCREW_PITCH_MM)  // 800 steps/mm

// Phase accumulator timer configuration
#define TICK_HZ        20000       // fixed timer tick rate (20 kHz)
#define DITHER_HZ      20          // dither frequency
#define LUT_SIZE       256
#define Q              16          // Q16.16 fixed-point

// Motor state for non-blocking movement
static volatile int32_t target_position_steps = 0;
static volatile int32_t current_position_steps = 0;
static volatile int32_t start_position_steps = 0;
static volatile int32_t total_steps = 0;
static volatile bool motor_moving = false;
static volatile bool motor_direction = 0; // 0 = backward, 1 = forward
static volatile bool measuring_active = false; // Only output load cell data when measuring

// Base speed in steps/sec (set from control task)
static volatile uint32_t base_sps = 1000;          // steps per second (max speed)
static volatile uint16_t dither_eps_permille = 100; // 100 = 10% speed dither

// Internal ISR state for phase accumulator
static uint32_t phase_q = 0; // Q16.16 where 1.0 == (1<<Q)
static uint32_t sin_phase_q = 0; // Q16.16 over [0, LUT_SIZE)
static uint32_t sin_inc_q = 0;   // Q16.16 increment per tick

// Sine LUT in Q1.15 (-32768..32767)
static int16_t sin_lut[LUT_SIZE];

static gptimer_handle_t step_timer = NULL;

// Forward declarations
void stepper_init(void);

// Initialize sine LUT (call once at init)
static void init_sin_lut(void)
{
    for (int i = 0; i < LUT_SIZE; i++) {
        float a = (2.0f * 3.14159265f * i) / LUT_SIZE;
        sin_lut[i] = (int16_t)(32767.0f * sinf(a));
    }

    // sin_inc_q = LUT_SIZE * DITHER_HZ / TICK_HZ in Q16.16
    sin_inc_q = (uint32_t)((((uint64_t)LUT_SIZE * DITHER_HZ) << 16) / TICK_HZ);
}

static inline void IRAM_ATTR pulse_step_pin(void)
{
    gpio_set_level(STEP_PIN, 1);
}

// Hardware timer ISR callback for motor stepping with phase accumulator
static bool IRAM_ATTR step_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    if (!motor_moving) return false;
    gpio_set_level(STEP_PIN, 0);
    // Stop condition
    if (current_position_steps == target_position_steps) {
        motor_moving = false;
        gpio_set_level(EN_PIN, 1);  // Disable motor when done
        return false;
    }

    // Calculate current step in move (absolute value)
    int32_t steps_taken;
    if (motor_direction) {
        steps_taken = current_position_steps - start_position_steps;
    } else {
        steps_taken = start_position_steps - current_position_steps;
    }

    // Calculate steps remaining
    int32_t steps_remaining = total_steps - steps_taken;

    // Acceleration profile: ramp up and down over ACCEL_STEPS
    uint32_t accel_steps = (total_steps < ACCEL_STEPS * 2) ? total_steps / 2 : ACCEL_STEPS;
    uint32_t sps;

    if (steps_taken < accel_steps) {
        // Acceleration phase: linear ramp from 20% to 100% of base_sps
        sps = (base_sps * 20) / 100 + ((base_sps * 80 * steps_taken) / (100 * accel_steps));
    } else if (steps_remaining < accel_steps) {
        // Deceleration phase: linear ramp from 100% to 20% of base_sps
        sps = (base_sps * 20) / 100 + ((base_sps * 80 * steps_remaining) / (100 * accel_steps));
    } else {
        // Constant speed phase
        sps = base_sps;
    }

    if (sps < 1) sps = 1;

    // Apply dither on top of acceleration profile

    sin_phase_q += sin_inc_q;
    uint16_t idx = (sin_phase_q >> 16) & (LUT_SIZE - 1);
    int16_t s = sin_lut[idx];  // Q1.15

    // Apply epsilon in permille (0..1000). Convert sin Q1.15 to a signed multiplier.
    // delta_sps = sps * eps * sin
    // scale: eps_permille/1000 * s/32768
    int32_t delta = (int32_t)(( (int64_t)sps * dither_eps_permille * s ) / (1000LL * 32768LL));
    int32_t sps_dithered = (int32_t)sps + delta;
    if (sps_dithered < 1) sps_dithered = 1;

    uint32_t rate_q = (uint32_t)(((uint64_t)sps_dithered << Q) / TICK_HZ);

    // Phase accumulator decides when to step
    phase_q += rate_q;
    if (phase_q >= (1u << Q)) {
        phase_q -= (1u << Q);

        pulse_step_pin();

        if (motor_direction) current_position_steps++;
        else                 current_position_steps--;
    }

    return false;
}

// HX711 Functions
void hx711_init(void)
{
    gpio_reset_pin(HX711_DT_PIN);
    gpio_set_direction(HX711_DT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(HX711_DT_PIN, GPIO_PULLUP_ONLY);  // Add pull-up

    gpio_reset_pin(HX711_SCK_PIN);
    gpio_set_direction(HX711_SCK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(HX711_SCK_PIN, 0);

    ESP_LOGI(TAG, "HX711 initialized on DT=GPIO%d, SCK=GPIO%d", HX711_DT_PIN, HX711_SCK_PIN);

    // Wait a bit for HX711 to stabilize
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Check if DT pin is stuck high or low
    int dt_level = gpio_get_level(HX711_DT_PIN);
    ESP_LOGI(TAG, "DT pin initial level: %d (should toggle)", dt_level);

    // Power-down test: set SCK high for >60us to power down HX711
    ESP_LOGI(TAG, "Testing HX711 power-down...");
    gpio_set_level(HX711_SCK_PIN, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    int dt_during_pd = gpio_get_level(HX711_DT_PIN);
    ESP_LOGI(TAG, "DT during power-down: %d", dt_during_pd);

    // Wake it up
    gpio_set_level(HX711_SCK_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    int dt_after_wakeup = gpio_get_level(HX711_DT_PIN);
    ESP_LOGI(TAG, "DT after wake-up: %d (should go LOW if working)", dt_after_wakeup);
}

bool hx711_is_ready(void)
{
    return gpio_get_level(HX711_DT_PIN) == 0;
}



int32_t hx711_read_raw_no_wait(void)
{
    // Assumes DT is already low (HX711 is ready)
    // Read 24 bits
    uint32_t data = 0;
    for(int i = 0; i < 24; i++) {
        gpio_set_level(HX711_SCK_PIN, 1);
        esp_rom_delay_us(1);
        int bit = gpio_get_level(HX711_DT_PIN);
        data = (data << 1) | bit;
        gpio_set_level(HX711_SCK_PIN, 0);
        esp_rom_delay_us(1);
    }

    // 1 extra clock pulse for Channel A gain 128 (next read)
    gpio_set_level(HX711_SCK_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(HX711_SCK_PIN, 0);
    esp_rom_delay_us(1);

    // Convert to signed 24-bit
    if(data & 0x800000) {
        data |= 0xFF000000;
    }

    return (int32_t)data;
}


void stepper_init(void)
{
    // Configure STEP pin
    gpio_reset_pin(STEP_PIN);
    gpio_set_direction(STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(STEP_PIN, 0);

    // Configure DIR pin
    gpio_reset_pin(DIR_PIN);
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DIR_PIN, 0);

    // Configure ENABLE pin (active low on TB6600)
    gpio_reset_pin(EN_PIN);
    gpio_set_direction(EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(EN_PIN, 0);  // 0 = enabled, 1 = disabled

    ESP_LOGI(TAG, "Stepper driver initialized");

    // Initialize sine LUT for dithering
    init_sin_lut();
    ESP_LOGI(TAG, "Sine LUT initialized (LUT_SIZE=%d, sin_inc_q=0x%08lx)", LUT_SIZE, sin_inc_q);

    // Create hardware timer at fixed 20kHz
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TICK_HZ, // 20 kHz timer
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &step_timer));

    // Set alarm to trigger every tick (50us at 20kHz)
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1, // Trigger every tick
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(step_timer, &alarm_config));

    // Register callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = step_timer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(step_timer, &cbs, NULL));

    // Enable and start timer
    ESP_ERROR_CHECK(gptimer_enable(step_timer));
    ESP_ERROR_CHECK(gptimer_start(step_timer));

    ESP_LOGI(TAG, "Hardware timer started at %d Hz with phase accumulator", TICK_HZ);
}

// Start non-blocking movement with speed control and acceleration
void start_move(int32_t steps, bool direction, float speed_mm_per_sec)
{
    start_position_steps = current_position_steps;
    total_steps = steps;
    target_position_steps = current_position_steps + (direction ? steps : -steps);
    motor_direction = direction;

    // Convert speed from mm/s to steps/sec (this is max speed)
    // steps/sec = mm/s * steps/mm
    base_sps = (uint32_t)(speed_mm_per_sec * STEPS_PER_MM);
    if (base_sps < 1) base_sps = 1;

    // Reset phase accumulator for new move
    phase_q = 0;

    motor_moving = true;
    gpio_set_level(DIR_PIN, direction ? 1 : 0);
    gpio_set_level(EN_PIN, 0);  // Enable motor
}

// Stop movement
void stop_move(void)
{
    motor_moving = false;
    gpio_set_level(EN_PIN, 1);  // Disable motor
}

void process_command(const char* cmd)
{
    if (strncmp(cmd, "MOVE ", 5) == 0) {
        // Parse distance and optional speed from "MOVE 10.5" or "MOVE 10.5 2.5"
        // Skip any leading spaces after MOVE
        const char* params = cmd + 5;
        while (*params == ' ') params++;

        float distance = atof(params);
        bool direction = distance >= 0;
        if (distance < 0) distance = -distance;

        // Check for optional speed parameter (in mm/s)
        float speed_mm_per_sec = 1.25; // Default 1.25 mm/s
        const char* space_pos = strchr(params, ' ');
        if (space_pos != NULL) {
            float speed_float = atof(space_pos + 1);
            if (speed_float > 0.0) {
                speed_mm_per_sec = speed_float;
            }
        }

        int32_t steps = (int32_t)(distance * STEPS_PER_MM);
        start_move(steps, direction, speed_mm_per_sec);
        printf("OK\n");
    }
    else if (strcmp(cmd, "STOP") == 0) {
        stop_move();
        printf("OK\n");
    }
    else if (strcmp(cmd, "ZERO") == 0) {
        current_position_steps = 0;
        printf("OK\n");
    }
    else if (strcmp(cmd, "ENABLE") == 0) {
        gpio_set_level(EN_PIN, 0);  // Enable motor
        printf("OK\n");
    }
    else if (strcmp(cmd, "DISABLE") == 0) {
        stop_move();
        gpio_set_level(EN_PIN, 1);  // Disable motor
        printf("OK\n");
    }
    else if (strcmp(cmd, "MEASURE_START") == 0) {
        measuring_active = true;
        printf("OK\n");
    }
    else if (strcmp(cmd, "MEASURE_STOP") == 0) {
        measuring_active = false;
        printf("OK\n");
    }
    else {
        printf("ERROR: Unknown command\n");
    }
}

void app_main(void)
{
    // Add this task to the watchdog
    esp_task_wdt_add(NULL);

    // Initialize hardware
    hx711_init();
    stepper_init();

    // Install UART driver for command reception (use small buffers)
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Disable stdout buffering for immediate output
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("READY\n");

    // Command buffer for incoming serial data
    char cmd_buffer[64];
    int cmd_idx = 0;

    // Continuous reading loop with command processing
    while(1) {
        // Check for incoming commands (non-blocking)
        uint8_t byte;
        int len = uart_read_bytes(UART_NUM_0, &byte, 1, 0);
        if (len > 0) {
            if (byte == '\n' || byte == '\r') {
                if (cmd_idx > 0) {
                    cmd_buffer[cmd_idx] = '\0';
                    process_command(cmd_buffer);
                    cmd_idx = 0;
                }
            } else if (cmd_idx < sizeof(cmd_buffer) - 1) {
                cmd_buffer[cmd_idx++] = byte;
            }
        }

        // Check if motor just finished moving
        static bool was_moving = false;
        if (was_moving && !motor_moving) {
            float actual_distance_mm = (float)current_position_steps / STEPS_PER_MM;
            printf("MOVE_COMPLETE pos=%.3f mm\n", actual_distance_mm);
            fflush(stdout);
        }
        was_moving = motor_moving;

        // Read and output load cell data with position (only when measuring)
        if (measuring_active) {
            if (hx711_is_ready()) {
                int32_t pos_steps = current_position_steps;
                int32_t value = hx711_read_raw_no_wait();
                float position_mm = (float)pos_steps / STEPS_PER_MM;
                printf("%.3f,%ld\n", position_mm, value);
                fflush(stdout);  // Force immediate transmission
            }
        }

        // Reset watchdog and minimal delay to allow idle task to run
        esp_task_wdt_reset();
        vTaskDelay(1);  // 1 tick (~1ms) - minimal delay for idle task
    }
}