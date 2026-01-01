import serial
import time

print("Testing serial connection to COM3...")

try:
    # Open serial port with same settings as GUI
    # Disable DTR and RTS to prevent ESP32 reset
    ser = serial.Serial(
        port='COM3',
        baudrate=115200,
        timeout=1.0,
        dsrdtr=False,
        rtscts=False
    )

    # Explicitly set DTR and RTS low to prevent reset
    ser.dtr = False
    ser.rts = False

    print(f"Successfully opened {ser.port}")
    print(f"Baudrate: {ser.baudrate}")
    print("Waiting for ESP32 to boot...")
    time.sleep(2)  # Wait for ESP32 to boot if it reset
    ser.reset_input_buffer()  # Clear boot messages
    print("Waiting for data...\n")

    # Read and print lines with timestamps using raw read
    start_time = time.time()
    line_count = 0
    last_time = start_time
    buffer = b''

    while time.time() - start_time < 10:
        # Read all available bytes
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buffer += chunk

            # Process complete lines
            while b'\n' in buffer:
                line_bytes, buffer = buffer.split(b'\n', 1)
                line = line_bytes.decode('utf-8', errors='ignore').strip()

                if line:
                    line_count += 1
                    current_time = time.time()
                    elapsed = current_time - start_time
                    delta = current_time - last_time
                    last_time = current_time
                    print(f"[{line_count}] {elapsed:.3f}s (+{delta*1000:.1f}ms) {line}")
        else:
            time.sleep(0.001)  # 1ms sleep
except Exception as e:
    print(f"Unexpected error: {e}")
