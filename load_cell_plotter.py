import sys
import os
import json

from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QComboBox, QLineEdit, QDialog, QFormLayout, QListWidget
from PyQt6.QtCore import QTimer, QThread, pyqtSignal
import serial
import serial.tools.list_ports
from collections import deque
import numpy as np

# Import and configure pyqtgraph to use PyQt6
import pyqtgraph as pg
pg.setConfigOptions(useOpenGL=False)  # Disable OpenGL to avoid issues

CALIBRATION_FILE = "calibration.json"

class SerialReaderThread(QThread):
    """Background thread for reading serial data without blocking UI"""
    data_received = pyqtSignal(float, int)  # Signal emits (position_mm, force_value)
    error_occurred = pyqtSignal(str)  # Signal for errors

    def __init__(self, serial_port):
        super().__init__()
        self.serial_port = serial_port
        self.running = True
        self.buffer = b''

    def run(self):
        """Main thread loop - runs in background"""
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                # Read raw bytes (faster than readline())
                chunk = self.serial_port.read(self.serial_port.in_waiting or 1)
                if chunk:
                    self.buffer += chunk

                    # Process complete lines
                    while b'\n' in self.buffer:
                        line_bytes, self.buffer = self.buffer.split(b'\n', 1)
                        line = line_bytes.decode('utf-8', errors='ignore').strip()

                        if line and line != "READY" and not line.startswith("I ("):
                            # Handle MOVE_COMPLETE message
                            if line == 'MOVE_COMPLETE':
                                # Emit special marker for move complete
                                self.data_received.emit(-999.0, 0)
                                continue

                            try:
                                # Parse "position,force" format
                                if ',' in line:
                                    parts = line.split(',')
                                    position = float(parts[0])
                                    force = int(parts[1])
                                    self.data_received.emit(position, force)
                            except (ValueError, IndexError):
                                # Skip invalid lines
                                pass

            except Exception as e:
                self.error_occurred.emit(str(e))
                break

    def stop(self):
        """Stop the thread gracefully"""
        self.running = False


class CalibrationDialog(QDialog):
    """Dialog for calibrating load cell"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.setWindowTitle("Load Cell Calibration")
        self.setMinimumWidth(400)

        layout = QVBoxLayout(self)

        # Current raw value display
        self.current_raw_label = QLabel("Current Raw Value: ---")
        self.current_raw_label.setStyleSheet("font-size: 16pt; font-weight: bold;")
        layout.addWidget(self.current_raw_label)

        # Form for calibration point entry
        form_layout = QFormLayout()

        self.raw_input = QLineEdit()
        self.raw_input.setPlaceholderText("Raw value")
        form_layout.addRow("Raw Value:", self.raw_input)

        self.grams_input = QLineEdit()
        self.grams_input.setPlaceholderText("Weight in grams")
        form_layout.addRow("Grams:", self.grams_input)

        layout.addLayout(form_layout)

        # Measure button to capture current raw value
        self.measure_btn = QPushButton("Measure Current Value")
        self.measure_btn.clicked.connect(self.capture_raw_value)
        layout.addWidget(self.measure_btn)

        # Add calibration point button
        self.add_point_btn = QPushButton("Add Calibration Point")
        self.add_point_btn.clicked.connect(self.add_calibration_point)
        layout.addWidget(self.add_point_btn)

        # Calibration points list
        points_label = QLabel("Current Calibration Points:")
        layout.addWidget(points_label)

        self.points_list = QListWidget()
        self.points_list.setMaximumHeight(150)
        layout.addWidget(self.points_list)

        # Delete selected point button
        self.delete_btn = QPushButton("Delete Selected Point")
        self.delete_btn.clicked.connect(self.delete_selected_point)
        layout.addWidget(self.delete_btn)

        # Status label for calibration points
        self.calibration_status = QLabel("Calibration Points: 0")
        layout.addWidget(self.calibration_status)

        # Status label for feedback
        self.status_label = QLabel("")
        layout.addWidget(self.status_label)

        # Update the list with existing points
        self.update_points_list()

        # Timer to update current raw value display
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # Update every 100ms

        # Start measuring when dialog opens
        if self.parent_window:
            self.parent_window.send_command("MEASURE_START")

    def update_display(self):
        """Update the current raw value display"""
        if self.parent_window and len(self.parent_window.raw_data_buffer) > 0:
            current_value = self.parent_window.raw_data_buffer[-1]
            self.current_raw_label.setText(f"Current Raw Value: {current_value}")

    def capture_raw_value(self):
        """Capture the current raw value into the raw field"""
        if self.parent_window and len(self.parent_window.raw_data_buffer) > 0:
            current_value = self.parent_window.raw_data_buffer[-1]
            self.raw_input.setText(str(current_value))
            self.grams_input.setFocus()  # Move focus to grams field for easy entry

    def add_calibration_point(self):
        """Add a calibration point from the input fields"""
        try:
            raw_value = float(self.raw_input.text())
            grams = float(self.grams_input.text())

            if self.parent_window:
                self.parent_window.add_calibration_point(raw_value, grams)
                self.calibration_status.setText(f"Calibration Points: {len(self.parent_window.calibration_points)}")
                self.raw_input.clear()
                self.grams_input.clear()
                self.status_label.setText(f"Added: {raw_value} -> {grams}g")
                self.update_points_list()
        except ValueError:
            if hasattr(self, 'status_label'):
                self.status_label.setText("Error: Invalid values")

    def update_points_list(self):
        """Update the list widget with current calibration points"""
        self.points_list.clear()
        if self.parent_window:
            for raw, grams in self.parent_window.calibration_points:
                self.points_list.addItem(f"Raw: {raw:.1f} → {grams:.1f}g")

    def delete_selected_point(self):
        """Delete the selected calibration point"""
        current_row = self.points_list.currentRow()
        if current_row >= 0 and self.parent_window:
            if current_row < len(self.parent_window.calibration_points):
                deleted_point = self.parent_window.calibration_points.pop(current_row)

                # Update calibration active status
                if len(self.parent_window.calibration_points) < 2:
                    self.parent_window.calibration_active = False
                    self.parent_window.update_y_axis_label()

                self.parent_window.save_calibration()
                self.update_points_list()
                self.calibration_status.setText(f"Calibration Points: {len(self.parent_window.calibration_points)}")
                self.status_label.setText(f"Deleted: {deleted_point[0]:.1f} → {deleted_point[1]:.1f}g")

    def closeEvent(self, event):
        """Stop measuring when dialog closes"""
        if self.parent_window:
            self.parent_window.send_command("MEASURE_STOP")
        self.update_timer.stop()
        event.accept()


class LoadCellPlotter(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.serial_thread = None
        self.data_buffer = deque(maxlen=1000)  # Store last 1000 force values (converted to grams if calibrated)
        self.raw_data_buffer = deque(maxlen=1000)  # Store raw values before calibration conversion
        self.position_data = deque(maxlen=1000)  # Store position values
        self.sample_count = 0

        # Measurement state
        self.measuring = False
        self.measurement_distance = 0.0
        self.measurement_count = 0
        self.measurement_colors = ['b', 'r', 'g', 'm', 'c', 'y', 'w']
        self.current_color_index = 0

        # Calibration state
        self.calibration_points = []  # List of (raw, grams) tuples
        self.calibration_active = False

        # Load saved calibration
        self.load_calibration()

        self.init_ui()

        # Update y-axis label if calibration is already active
        self.update_y_axis_label()

        # Timer for updating plot display (not for reading serial)
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.refresh_plot)
        self.plot_timer.start(30)  # Refresh display every 30ms for smoother updates

    def init_ui(self):
        import pyqtgraph as pg

        self.setWindowTitle("Load Cell Monitor")
        self.setGeometry(100, 100, 1200, 600)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Control panel - Connection row
        connection_layout = QHBoxLayout()

        # Port selection
        self.port_label = QLabel("Serial Port:")
        connection_layout.addWidget(self.port_label)

        self.port_combo = QComboBox()
        self.refresh_ports()
        connection_layout.addWidget(self.port_combo)

        # Refresh ports button
        self.refresh_btn = QPushButton("Refresh Ports")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        connection_layout.addWidget(self.refresh_btn)

        # Connect/Disconnect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        connection_layout.addWidget(self.connect_btn)

        connection_layout.addStretch()
        layout.addLayout(connection_layout)

        # Status and data row
        status_layout = QHBoxLayout()

        # Status label
        self.status_label = QLabel("Status: Disconnected")
        status_layout.addWidget(self.status_label)

        # Position label
        self.position_label = QLabel("Position: 0.000 mm")
        status_layout.addWidget(self.position_label)

        # Stats labels
        self.min_label = QLabel("Min: ---")
        status_layout.addWidget(self.min_label)

        self.max_label = QLabel("Max: ---")
        status_layout.addWidget(self.max_label)

        self.current_label = QLabel("Current: ---")
        status_layout.addWidget(self.current_label)

        status_layout.addStretch()

        # Calibrate button
        self.calibrate_btn = QPushButton("Calibrate")
        self.calibrate_btn.clicked.connect(self.open_calibration_dialog)
        self.calibrate_btn.setEnabled(False)
        status_layout.addWidget(self.calibrate_btn)

        # Clear button
        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self.clear_data)
        status_layout.addWidget(self.clear_btn)

        layout.addLayout(status_layout)

        # Motor control panel
        motor_layout = QHBoxLayout()

        motor_label = QLabel("Motor Control:")
        motor_layout.addWidget(motor_label)

        self.distance_input = QLineEdit()
        self.distance_input.setPlaceholderText("Distance (mm)")
        self.distance_input.setMaximumWidth(100)
        motor_layout.addWidget(self.distance_input)

        self.move_btn = QPushButton("Move")
        self.move_btn.clicked.connect(self.move_motor)
        self.move_btn.setEnabled(False)
        motor_layout.addWidget(self.move_btn)

        self.enable_btn = QPushButton("Enable Motor")
        self.enable_btn.clicked.connect(lambda: self.send_command("ENABLE"))
        self.enable_btn.setEnabled(False)
        motor_layout.addWidget(self.enable_btn)

        self.disable_btn = QPushButton("Disable Motor")
        self.disable_btn.clicked.connect(lambda: self.send_command("DISABLE"))
        self.disable_btn.setEnabled(False)
        motor_layout.addWidget(self.disable_btn)

        motor_layout.addStretch()
        layout.addLayout(motor_layout)

        # Measurement control panel
        measure_layout = QHBoxLayout()

        measure_label = QLabel("Measurement:")
        measure_layout.addWidget(measure_label)

        self.measure_distance_input = QLineEdit()
        self.measure_distance_input.setPlaceholderText("Distance (mm)")
        self.measure_distance_input.setMaximumWidth(100)
        measure_layout.addWidget(self.measure_distance_input)

        self.measure_btn = QPushButton("Measure")
        self.measure_btn.clicked.connect(self.start_measurement)
        self.measure_btn.setEnabled(False)
        measure_layout.addWidget(self.measure_btn)

        self.stop_measure_btn = QPushButton("Stop Measurement")
        self.stop_measure_btn.clicked.connect(self.stop_measurement)
        self.stop_measure_btn.setEnabled(False)
        measure_layout.addWidget(self.stop_measure_btn)

        measure_layout.addStretch()
        layout.addLayout(measure_layout)

        # Plot widget - now plots force vs position
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        self.plot_widget.setLabel('left', 'Force (Load Cell Value)', color='black')
        self.plot_widget.setLabel('bottom', 'Position (mm)', color='black')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setTitle("Force vs Position", color='black')

        # Add legend at top right corner
        self.legend = self.plot_widget.addLegend(offset=(-10, 10), anchor=(1, 0))

        # Enable downsampling for better performance
        self.plot_widget.setDownsampling(auto=True, mode='peak')
        self.plot_widget.setClipToView(True)

        # Store multiple plot lines for different measurements
        self.plot_lines = []

        # Initial plot line (for non-measurement data, hidden by default)
        pen = pg.mkPen(color=self.measurement_colors[0], width=2)
        self.plot_line = self.plot_widget.plot([], [], pen=pen, name='Current')
        self.plot_lines.append(self.plot_line)

        layout.addWidget(self.plot_widget)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")

    def toggle_connection(self):
        if self.serial_port is None or not self.serial_port.is_open:
            self.connect_serial()
        else:
            self.disconnect_serial()

    def connect_serial(self):
        try:
            port_text = self.port_combo.currentText()
            if not port_text:
                self.status_label.setText("Status: No port selected")
                return

            port_name = port_text.split(" - ")[0]
            # Disable DTR/RTS to prevent ESP32 reset on connection
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=115200,
                timeout=0.1,
                dsrdtr=False,
                rtscts=False
            )

            # Explicitly set DTR and RTS low
            self.serial_port.dtr = False
            self.serial_port.rts = False

            # Wait for "READY" message
            self.status_label.setText("Status: Waiting for ESP32...")
            QApplication.processEvents()

            # Clear any stale data in the buffer
            self.serial_port.reset_input_buffer()
            import time
            time.sleep(0.2)  # Wait for buffer to clear
            self.serial_port.reset_input_buffer()  # Clear again

            # Start background thread for serial reading
            self.serial_thread = SerialReaderThread(self.serial_port)
            self.serial_thread.data_received.connect(self.on_data_received)
            self.serial_thread.error_occurred.connect(self.on_serial_error)
            self.serial_thread.start()

            self.connect_btn.setText("Disconnect")
            self.status_label.setText(f"Status: Connected to {port_name}")

            # Enable motor controls
            self.move_btn.setEnabled(True)
            self.enable_btn.setEnabled(True)
            self.disable_btn.setEnabled(True)
            self.measure_btn.setEnabled(True)
            self.calibrate_btn.setEnabled(True)

        except Exception as e:
            self.status_label.setText(f"Status: Error - {str(e)}")

    def disconnect_serial(self):
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread.wait()  # Wait for thread to finish
            self.serial_thread = None

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_port = None

        self.connect_btn.setText("Connect")
        self.status_label.setText("Status: Disconnected")

        # Disable motor controls
        self.move_btn.setEnabled(False)
        self.enable_btn.setEnabled(False)
        self.disable_btn.setEnabled(False)
        self.measure_btn.setEnabled(False)
        self.stop_measure_btn.setEnabled(False)
        self.calibrate_btn.setEnabled(False)

    def send_command(self, command):
        """Send a command to the ESP32"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{command}\n".encode('utf-8'))
                print(f"Sent command: {command}")
            except Exception as e:
                self.status_label.setText(f"Status: Error - {str(e)}")

    def move_motor(self):
        """Send move command with distance from input"""
        try:
            distance = float(self.distance_input.text())
            self.send_command(f"MOVE {distance}")
        except ValueError:
            self.status_label.setText("Status: Invalid distance value")

    def on_data_received(self, position, force):
        """Slot called when worker thread receives data (runs in main thread)"""
        # Check for MOVE_COMPLETE marker
        if position == -999.0:
            self.on_move_complete()
            return

        # Always store raw value before conversion
        self.raw_data_buffer.append(force)

        # Convert force to grams if calibration is active
        if self.calibration_active:
            force_value = self.raw_to_grams(force)
        else:
            force_value = force

        self.position_data.append(position)
        self.data_buffer.append(force_value)
        self.sample_count += 1

    def on_serial_error(self, error_msg):
        """Slot called when worker thread encounters error"""
        self.status_label.setText(f"Status: Error - {error_msg}")
        self.disconnect_serial()

    def refresh_plot(self):
        """Refresh the plot display (called by timer in main thread)"""
        if len(self.data_buffer) > 0:
            # Update plot - force vs position
            self.plot_line.setData(np.array(self.position_data), np.array(self.data_buffer))

            # Update stats
            current_force = self.data_buffer[-1]
            current_position = self.position_data[-1]
            min_force = min(self.data_buffer)
            max_force = max(self.data_buffer)

            self.position_label.setText(f"Position: {current_position:.3f} mm")
            self.current_label.setText(f"Current: {current_force}")
            self.min_label.setText(f"Min: {min_force}")
            self.max_label.setText(f"Max: {max_force}")

    def clear_data(self):
        """Clear all data buffers and reset display"""
        self.data_buffer.clear()
        self.raw_data_buffer.clear()
        self.position_data.clear()
        self.sample_count = 0

        # Remove all plot lines from plot widget
        for plot_line in self.plot_lines:
            self.plot_widget.removeItem(plot_line)

        # Clear the plot lines list
        self.plot_lines.clear()

        # Clear and recreate legend at top right corner
        self.plot_widget.removeItem(self.legend)
        self.legend = self.plot_widget.addLegend(offset=(-10, 10), anchor=(1, 0))

        # Recreate initial plot line
        pen = pg.mkPen(color=self.measurement_colors[0], width=2)
        self.plot_line = self.plot_widget.plot([], [], pen=pen, name='Current')
        self.plot_lines.append(self.plot_line)

        self.position_label.setText("Position: 0.000 mm")
        self.min_label.setText("Min: ---")
        self.max_label.setText("Max: ---")
        self.current_label.setText("Current: ---")

        # Reset measurement count and color
        self.measurement_count = 0
        self.current_color_index = 0

        # Send ZERO command to reset ESP32 position
        if self.serial_port and self.serial_port.is_open:
            self.send_command("ZERO")

    def start_measurement(self):
        """Start a measurement sequence"""
        print("start_measurement called")  # Debug
        try:
            distance = float(self.measure_distance_input.text())
            print(f"Distance: {distance}")  # Debug
            if distance == 0:
                self.status_label.setText("Status: Distance cannot be zero")
                return

            self.measurement_distance = distance
            self.measuring = True
            self.measurement_count += 1
            print(f"Starting measurement {self.measurement_count}")  # Debug

            # Update UI
            self.measure_btn.setEnabled(False)
            self.stop_measure_btn.setEnabled(True)
            self.move_btn.setEnabled(False)
            self.status_label.setText(f"Status: Measuring ({self.measurement_count})...")

            # Create new plot line with next color
            self.current_color_index = (self.measurement_count - 1) % len(self.measurement_colors)
            pen = pg.mkPen(color=self.measurement_colors[self.current_color_index], width=2)
            new_line = self.plot_widget.plot([], [], pen=pen, name=f"Measurement {self.measurement_count}")
            self.plot_lines.append(new_line)
            self.plot_line = new_line  # Update current plot line

            # Clear current data buffers for new measurement
            self.data_buffer.clear()
            self.position_data.clear()

            # Send commands to ESP32
            print("Sending MEASURE_START")  # Debug
            self.send_command("MEASURE_START")
            import time
            time.sleep(0.1)  # Small delay between commands
            print(f"Sending MOVE {distance} 2000")
            self.send_command(f"MOVE {distance} 2000")

        except ValueError as e:
            print(f"ValueError: {e}")  # Debug
            self.status_label.setText("Status: Invalid distance value")

    def stop_measurement(self):
        """Manually stop measurement"""
        if self.measuring:
            self.send_command("STOP")
            self.send_command("MEASURE_STOP")
            self.measuring = False
            self.measure_btn.setEnabled(True)
            self.stop_measure_btn.setEnabled(False)
            self.move_btn.setEnabled(True)
            self.status_label.setText("Status: Measurement stopped")

    def on_move_complete(self):
        """Called when ESP32 reports movement is complete"""
        if self.measuring:
            # Stop measuring and move back at 2x speed (500us = 0.5ms per step)
            self.send_command("MEASURE_STOP")
            import time
            time.sleep(0.1)
            self.send_command(f"MOVE {-self.measurement_distance} 250")

            # Update UI
            self.measuring = False
            self.measure_btn.setEnabled(True)
            self.stop_measure_btn.setEnabled(False)
            self.move_btn.setEnabled(True)
            self.status_label.setText(f"Status: Measurement {self.measurement_count} complete, returning...")

    def open_calibration_dialog(self):
        """Open the calibration dialog"""
        was_calibrated = self.calibration_active
        dialog = CalibrationDialog(self)
        dialog.exec()
        # Update plot label and clear data if calibration was just activated
        if self.calibration_active:
            self.update_y_axis_label()
            if not was_calibrated:
                # Calibration was just activated, clear old raw data
                self.clear_data()

    def update_y_axis_label(self):
        """Update y-axis label based on calibration status"""
        if self.calibration_active:
            self.plot_widget.setLabel('left', 'Force (grams)', color='black')
        else:
            self.plot_widget.setLabel('left', 'Force (Load Cell Value)', color='black')

    def add_calibration_point(self, raw_value, grams):
        """Add a calibration point and activate calibration if we have at least 2 points"""
        was_active = self.calibration_active
        self.calibration_points.append((raw_value, grams))
        self.calibration_points.sort(key=lambda x: x[0])  # Sort by raw value

        if len(self.calibration_points) >= 2:
            self.calibration_active = True
            print(f"Calibration active with {len(self.calibration_points)} points")
            # Update plot label and clear data immediately when calibration becomes active
            if not was_active:
                self.update_y_axis_label()
                self.clear_data()
        else:
            print(f"Added calibration point: {raw_value} -> {grams}g. Need at least 2 points.")

        # Save calibration after adding point
        self.save_calibration()

    def raw_to_grams(self, raw_value):
        """Convert raw value to grams using linear interpolation"""
        if not self.calibration_active or len(self.calibration_points) < 2:
            return raw_value

        # Linear interpolation between calibration points
        for i in range(len(self.calibration_points) - 1):
            raw1, grams1 = self.calibration_points[i]
            raw2, grams2 = self.calibration_points[i + 1]

            if raw1 <= raw_value <= raw2:
                # Interpolate
                ratio = (raw_value - raw1) / (raw2 - raw1)
                return grams1 + ratio * (grams2 - grams1)

        # Extrapolate if outside range
        if raw_value < self.calibration_points[0][0]:
            raw1, grams1 = self.calibration_points[0]
            raw2, grams2 = self.calibration_points[1]
        else:
            raw1, grams1 = self.calibration_points[-2]
            raw2, grams2 = self.calibration_points[-1]

        slope = (grams2 - grams1) / (raw2 - raw1)
        return grams1 + slope * (raw_value - raw1)

    def save_calibration(self):
        """Save calibration points to file"""
        try:
            data = {
                'calibration_points': self.calibration_points,
                'calibration_active': self.calibration_active
            }
            with open(CALIBRATION_FILE, 'w') as f:
                json.dump(data, f)
            print(f"Calibration saved: {len(self.calibration_points)} points")
        except Exception as e:
            print(f"Error saving calibration: {e}")

    def load_calibration(self):
        """Load calibration points from file"""
        try:
            if os.path.exists(CALIBRATION_FILE):
                with open(CALIBRATION_FILE, 'r') as f:
                    data = json.load(f)
                self.calibration_points = [tuple(point) for point in data.get('calibration_points', [])]
                self.calibration_active = data.get('calibration_active', False)
                print(f"Calibration loaded: {len(self.calibration_points)} points, active={self.calibration_active}")
        except Exception as e:
            print(f"Error loading calibration: {e}")

    def closeEvent(self, event):
        self.save_calibration()
        self.disconnect_serial()
        event.accept()


def main():
    print("Starting application...")
    app = QApplication(sys.argv)
    print("QApplication created")
    window = LoadCellPlotter()
    print("Window created")
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
