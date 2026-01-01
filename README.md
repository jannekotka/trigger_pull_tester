# Trigger Tester

A measurement system for analyzing pistol trigger pull force over distance using ESP32, load cell, and stepper motor.

## Overview

This system continuously measures trigger pull force while a stepper motor moves the trigger through a known distance, producing detailed force vs. position curves. The PyQt6 GUI provides real-time visualization with calibration support for gram-based measurements.

## Hardware Requirements

example setup

- **ESP32 DevKit** (CP2102 USB-to-serial)
- **NEMA 17 Stepper Motor** (200 steps/rev)
- **TB6600 Stepper Driver** (configured for 1/8 microstepping = 1600 steps/rev)
- **HX711 24-bit ADC** with bar load cell
- **2mm Lead Screw** (800 steps/mm travel)
- **Power Supply**: 20V USB-PD for motor

### Wiring

**TB6600 Stepper Driver:**
- GPIO 25 → STEP
- GPIO 26 → DIR
- GPIO 27 → EN

**HX711 Load Cell:**
- GPIO 16 → DT (Data)
- GPIO 17 → SCK (Clock)

## Software Requirements

### ESP32 Firmware
- ESP-IDF v5.5.2
- Configured for hardware timer-based stepping

### Python GUI
- Python 3.8+
- PyQt6 6.7.1
- pyqtgraph 0.13+
- pyserial 3.5

## Setup Instructions

### 1. ESP32 Firmware

```powershell
# Set up ESP-IDF environment (if not already done)
# Follow ESP-IDF installation guide for your platform

# Build and flash
idf.py build
idf.py -p COM3 flash

# Monitor output (optional)
idf.py -p COM3 monitor
```

### 2. Python Environment

```powershell
# Create virtual environment
python -m venv venv_clean

# Activate virtual environment
.\venv_clean\Scripts\Activate.ps1

# Install dependencies
pip install PyQt6==6.7.1 pyqtgraph pyserial
```

### 3. Run the Application

```powershell
python load_cell_plotter.py
```

## Usage

### Initial Setup

1. **Connect Hardware**: Plug in ESP32 via USB (COM3)
2. **Launch GUI**: Run `load_cell_plotter.py`
3. **Select Port**: Choose your ESP32's COM port from dropdown
4. **Connect**: Click "Connect" button

### Calibration (First Time Setup)

1. Click **"Calibrate"** button
2. Apply **no load** to the load cell:
   - Wait for raw value to stabilize
   - Click "Measure Current Value"
   - Enter `0` in the Grams field
   - Click "Add Calibration Point"
3. Apply **known weight** (e.g., 1000g):
   - Place weight on load cell
   - Wait for value to stabilize
   - Click "Measure Current Value"
   - Enter actual weight in grams
   - Click "Add Calibration Point"
4. Add more points for better accuracy (optional)
5. Close calibration dialog

**Note**: Calibration points are automatically saved to `calibration.json`

### Taking Measurements
1. Ensure that the weapons chamber is empty
2. Fix the gun so that the measuring rod is lightly touching the trigger
3. Enter **distance** in mm (e.g., `-5` for 5mm trigger travel)
4. Click **"Measure"**
   - Motor moves at measurement speed (1000 µs/step)
   - Load cell records force continuously (~12-13 Hz)
   - Plot is updated in real time
   - Motor automatically returns to start position at higher speed
5. Each measurement appears as a new colored line on the graph
6. Take multiple measurements to compare consistency or different triggers

### Manual Motor Control

- **Move**: Enter distance in mm and click "Move" (positive = forward, negative = reverse)
- **Enable/Disable**: Control motor power
- **Clear**: Remove all data from graph and reset position

## Features

- **Non-blocking Motor Control**: Hardware timer (1 MHz) for precise stepping
- **Variable Speed**: Different speeds for measurement (slow, accurate) vs. return (fast)
- **Real-time Plotting**: Force vs. position curves updated at 30 Hz
- **Multi-measurement**: Overlay multiple measurements with unique colors
- **Calibration System**: Linear interpolation/extrapolation for gram conversion
- **Data Persistence**: Calibration saved to JSON automatically
- **Calibration Management**: View and delete individual calibration points

## Serial Communication

- **Baud Rate**: 115200
- **Data Format**: CSV `position_mm,force_value`
- **Commands**:
  - `MOVE [distance] [speed_us]` - Move motor (optional speed in microseconds/step)
  - `MEASURE_START` - Begin outputting data
  - `MEASURE_STOP` - Stop outputting data
  - `ZERO` - Reset position counter
  - `ENABLE` - Enable motor
  - `DISABLE` - Disable motor
  - `STOP` - Emergency stop

## Technical Specifications

- **Resolution**: 800 steps/mm (1600 steps/rev × 1 rev/2mm)
- **Measurement Speed**: 1000 µs/step = 1.25 mm/s
- **Return Speed**: 250 µs/step = 5.0 mm/s
- **ADC Resolution**: 24-bit (HX711)
- **Sampling Rate**: ~12-13 Hz during measurement
- **Maximum Travel**: Limited by mechanical setup

## Troubleshooting

**ESP32 won't connect:**
- Check COM port in Device Manager
- Ensure drivers installed (CP2102)
- Try different USB cable/port

**Motor not moving:**
- Check "Enable Motor" is clicked
- Verify TB6600 power supply (12-20V)
- Check wiring connections

**Erratic load cell readings:**
- Ensure stable 5V power to HX711
- Check load cell wiring (red/black = excitation, white/green = signal)
- Avoid electrical noise sources
- Let sensor stabilize after power-on

**Calibration shows wrong values:**
- Delete corrupted points in calibration dialog
- Recalibrate with known weights
- Ensure at least 2 calibration points for activation

## File Structure

```
triggertester/
├── main/
│   └── main.c              # ESP32 firmware
├── load_cell_plotter.py    # Python GUI application
├── calibration.json        # Calibration data (auto-generated)
├── README.md               # This file
└── .gitignore
```

## License

MIT License - see [LICENSE](LICENSE) file for details
