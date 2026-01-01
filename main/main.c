#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

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
#define ACCEL_STEPS         50   // Number of steps to accelerate/decelerate

// Motor and lead screw configuration
#define STEPS_PER_REV       200   // 1.8° motor = 200 steps/rev
#define MICROSTEPS          8     // Microstepping setting
#define LEAD_SCREW_PITCH_MM 2.0   // 2mm lead screw
#define STEPS_PER_MM        ((STEPS_PER_REV * MICROSTEPS) / LEAD_SCREW_PITCH_MM)  // 800 steps/mm

// Motor state for non-blocking movement
static volatile int32_t target_position_steps = 0;
static volatile int32_t current_position_steps = 0;
static volatile bool motor_moving = false;
static volatile bool motor_direction = 0; // 0 = backward, 1 = forward
static volatile bool measuring_active = false; // Only output load cell data when measuring
static volatile uint32_t current_speed_ms = 1; // Current timer period in ms (1ms = 1000 steps/sec)
static gptimer_handle_t step_timer = NULL;

// Forward declarations
void move_distance_mm(float distance_mm, bool direction);
void stepper_init(void);

// Hardware timer ISR callback for motor stepping
static bool IRAM_ATTR step_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    if (!motor_moving) return false;

    if (current_position_steps == target_position_steps) {
        motor_moving = false;
        gpio_set_level(EN_PIN, 1);  // Disable motor when done
        return false;
    }

    // Perform one step - keep it minimal in ISR
    gpio_set_level(STEP_PIN, 1);
    // Minimal delay for pulse width
    for (volatile int i = 0; i < 5; i++);
    gpio_set_level(STEP_PIN, 0);

    // Update position
    if (motor_direction) {
        current_position_steps++;
    } else {
        current_position_steps--;
    }

    return false;  // Don't yield from ISR
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

int32_t hx711_read_raw(void)
{
    // Check initial DT state
    int dt_before = gpio_get_level(HX711_DT_PIN);

    // Wait for HX711 to be ready (DT goes low)
    uint32_t timeout = 1000000;
    while(!hx711_is_ready() && timeout--) {
        esp_rom_delay_us(1);
    }

    if(timeout == 0) {
        ESP_LOGW(TAG, "HX711 timeout - DT stuck at %d", dt_before);
        return -1;
    }

    ESP_LOGI(TAG, "HX711 ready, starting read...");

    // Read 24 bits
    uint32_t data = 0;
    uint8_t bits_high = 0;
    int first_bit = -1;
    int last_bit = -1;
    for(int i = 0; i < 24; i++) {
        gpio_set_level(HX711_SCK_PIN, 1);
        esp_rom_delay_us(10);  // Longer delay for 10Hz mode
        int bit = gpio_get_level(HX711_DT_PIN);
        if(i == 0) first_bit = bit;
        if(i == 23) last_bit = bit;
        if(bit) bits_high++;
        data = (data << 1) | bit;
        gpio_set_level(HX711_SCK_PIN, 0);
        esp_rom_delay_us(10);  // Longer delay for 10Hz mode
    }

    ESP_LOGI(TAG, "Read: %d/24 bits HIGH, first=%d, last=%d, data=0x%06lX",
             bits_high, first_bit, last_bit, data);

    // Try Channel B instead (3 extra clock pulses for gain 32)
    // This is in case the load cell is wired to B instead of A
    for(int i = 0; i < 3; i++) {
        gpio_set_level(HX711_SCK_PIN, 1);
        esp_rom_delay_us(1);
        gpio_set_level(HX711_SCK_PIN, 0);
        esp_rom_delay_us(1);
    }

    // Convert to signed 24-bit
    if(data & 0x800000) {
        data |= 0xFF000000;
    }

    return (int32_t)data;
}

float hx711_read_average(uint8_t samples)
{
    int64_t sum = 0;
    for(uint8_t i = 0; i < samples; i++) {
        sum += hx711_read_raw();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    return (float)sum / samples;
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

    // Create hardware timer (1ms period = 1000 steps/sec max = 1.25 mm/sec)
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &step_timer));

    // Set alarm to trigger every 1ms (1000 us)
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000, // 1ms at 1MHz
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

    ESP_LOGI(TAG, "Hardware timer started (1ms period, 1000 steps/sec)");
}

// Start non-blocking movement with speed control
void start_move(int32_t steps, bool direction, uint32_t speed_us)
{
    target_position_steps = current_position_steps + (direction ? steps : -steps);
    motor_direction = direction;
    current_speed_ms = speed_us;

    // Stop timer to reconfigure
    gptimer_stop(step_timer);

    // Update timer period for new speed (speed_us is already in microseconds)
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = speed_us, // Period in microseconds
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(step_timer, &alarm_config);

    // Restart timer with new speed
    gptimer_start(step_timer);

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
        // Parse distance and optional speed from "MOVE 10.5" or "MOVE 10.5 500"
        float distance = atof(cmd + 5);
        bool direction = distance >= 0;
        if (distance < 0) distance = -distance;

        // Check for optional speed parameter (in microseconds per step)
        uint32_t speed_us = 1000; // Default 1000us (1ms) = 1000 steps/sec
        const char* space_pos = strchr(cmd + 5, ' ');
        if (space_pos != NULL) {
            int speed_int = atoi(space_pos + 1);
            if (speed_int > 0) {
                speed_us = (uint32_t)speed_int;
            }
        }

        int32_t steps = (int32_t)(distance * STEPS_PER_MM);
        ESP_LOGI(TAG, "Command: Move %.2f mm (%ld steps) at %lu us/step",
                 direction ? distance : -distance, steps, speed_us);
        start_move(steps, direction, speed_us);
        printf("OK\n");
    }
    else if (strcmp(cmd, "STOP") == 0) {
        stop_move();
        ESP_LOGI(TAG, "Motor stopped");
        printf("OK\n");
    }
    else if (strcmp(cmd, "ZERO") == 0) {
        current_position_steps = 0;
        ESP_LOGI(TAG, "Position zeroed");
        printf("OK\n");
    }
    else if (strcmp(cmd, "ENABLE") == 0) {
        gpio_set_level(EN_PIN, 0);  // Enable motor
        ESP_LOGI(TAG, "Motor enabled");
        printf("OK\n");
    }
    else if (strcmp(cmd, "DISABLE") == 0) {
        stop_move();
        gpio_set_level(EN_PIN, 1);  // Disable motor
        ESP_LOGI(TAG, "Motor disabled");
        printf("OK\n");
    }
    else if (strcmp(cmd, "MEASURE_START") == 0) {
        measuring_active = true;
        ESP_LOGI(TAG, "Measurement started");
        printf("OK\n");
    }
    else if (strcmp(cmd, "MEASURE_STOP") == 0) {
        measuring_active = false;
        ESP_LOGI(TAG, "Measurement stopped");
        printf("OK\n");
    }
    else {
        printf("ERROR: Unknown command\n");
    }
}

void stepper_step_accel(uint32_t steps, bool direction)
{
    // Set direction
    gpio_set_level(DIR_PIN, direction ? 1 : 0);
    ESP_LOGI(TAG, "Moving %ld steps in %s direction", steps, direction ? "CW" : "CCW");

    // Small delay after changing direction
    vTaskDelay(2 / portTICK_PERIOD_MS);

    uint32_t accel_steps = (steps < ACCEL_STEPS * 2) ? steps / 2 : ACCEL_STEPS;

    // Generate step pulses with acceleration
    for(uint32_t i = 0; i < steps; i++) {
        uint32_t delay_us;

        // Acceleration phase
        if(i < accel_steps) {
            delay_us = STEP_DELAY_MIN_US + ((accel_steps - i) * STEP_DELAY_MIN_US / accel_steps);
        }
        // Deceleration phase
        else if(i >= steps - accel_steps) {
            delay_us = STEP_DELAY_MIN_US + ((i - (steps - accel_steps)) * STEP_DELAY_MIN_US / accel_steps);
        }
        // Constant speed phase
        else {
            delay_us = STEP_DELAY_MIN_US;
        }

        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(STEP_PULSE_WIDTH_US);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(delay_us);
    }

    ESP_LOGI(TAG, "Movement complete");
}

void move_distance_mm(float distance_mm, bool direction)
{
    // Calculate steps needed
    uint32_t steps = (uint32_t)(distance_mm * STEPS_PER_MM);

    ESP_LOGI(TAG, "Moving %.2f mm (%ld steps) in %s direction",
             distance_mm, steps, direction ? "forward" : "backward");

    stepper_step_accel(steps, direction);
}

void app_main(void)
{
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
            printf("MOVE_COMPLETE\n");
            fflush(stdout);
        }
        was_moving = motor_moving;

        // Read and output load cell data with position (only when measuring)
        if (measuring_active) {
            int32_t value = hx711_read_raw();
            float position_mm = (float)current_position_steps / STEPS_PER_MM;
            printf("%.3f,%ld\n", position_mm, value);
            fflush(stdout);  // Force immediate transmission
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms delay = 100Hz sample rate
    }
}