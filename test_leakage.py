"""
Pouch Leakage Test Tool

This tool tests soft robot pouches for leakage by:
1. Applying constant pressure via a 'Pump' Arduino.
2. Monitoring sensor readings from a 'Sensor' Arduino in real-time.
3. Detecting pressure decay that indicates leakage.

Usage:
    python leakage_test.py
"""

import logging
import signal
import socket
import struct
import sys
import threading
import time
from typing import List

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class NetworkConfig:
    PC_ADDRESS = "10.211.215.251"
    ARDUINO_PORTS = [10001, 10002, 10003, 10004, 10005, 10006, 10007, 10008]


class LeakageTestConfig:
    TEST_PRESSURE = 10.0  # PSI
    STABILIZATION_TIME = 10.0  # seconds to let pressure stabilize
    TEST_DURATION = 60.0  # seconds to monitor for leakage
    LEAKAGE_THRESHOLD = 0.5  # PSI drop that indicates leakage
    SAMPLE_RATE = 10  # Hz


class ArduinoConnection:
    """Manages connection to a single Arduino"""

    def __init__(self, arduino_id: int):
        self.arduino_id = arduino_id
        self.port = NetworkConfig.ARDUINO_PORTS[arduino_id - 1]
        self.server_socket = None
        self.client_socket = None
        self.client_address = None

    def connect(self) -> bool:
        """Establish connection to Arduino"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((NetworkConfig.PC_ADDRESS, self.port))
            self.server_socket.listen(1)

            logger.info(f"Waiting for Arduino {self.arduino_id} on port {self.port}...")
            self.client_socket, self.client_address = self.server_socket.accept()
            logger.info(
                f"Arduino {self.arduino_id} connected from {self.client_address}"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to connect to Arduino {self.arduino_id}: {e}")
            return False

    def send_pressure_read_sensors(self, pressure: float) -> List[float]:
        """Send pressure command and read sensor values back."""
        try:
            # Send pressure value
            packed_data = struct.pack("f", pressure)
            self.client_socket.send(packed_data)

            # Read sensor data
            received_data = b""
            while len(received_data) < 8:
                chunk = self.client_socket.recv(8 - len(received_data))
                if not chunk:
                    return [0.0] * 4
                received_data += chunk

            # Unpack and convert sensor readings
            received_data_unpacked = struct.unpack(">4h", received_data)
            sensor_values = []

            for raw_value in received_data_unpacked:
                pressure_volt = raw_value * (12.288 / 65536.0)
                pressure_psi = ((30.0 - 0.0) * (pressure_volt - (0.1 * 5.0))) / (
                    0.8 * 5.0
                )
                sensor_values.append(round(pressure_psi, 4))

            return sensor_values

        except Exception as e:
            logger.error(f"Error communicating with Arduino {self.arduino_id}: {e}")
            return [0.0] * 4

    def cleanup(self):
        """Close connections"""
        if self.client_socket:
            try:
                self.client_socket.close()
            except Exception:
                pass
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass


class LeakageTest:
    """Main leakage test controller"""

    def __init__(
        self, pump_arduino_id: int, sensor_arduino_id: int, target_pressure: float
    ):
        self.pump_arduino_id = pump_arduino_id
        self.sensor_arduino_id = sensor_arduino_id
        self.target_pressure = target_pressure
        self.is_separate_devices = pump_arduino_id != sensor_arduino_id

        self.pump_arduino = ArduinoConnection(pump_arduino_id)
        if self.is_separate_devices:
            self.sensor_arduino = ArduinoConnection(sensor_arduino_id)
        else:
            self.sensor_arduino = self.pump_arduino  # Use the same connection

        # Data storage
        self.timestamps = []
        self.sensor_data = [[], [], [], []]
        self.start_time = None
        self.running = False
        self.test_thread = None

        # Test results
        self.stabilization_complete = False
        self.baseline_pressure = None
        self.leakage_detected = False
        self.leakage_sensor = None
        self.leakage_time = None

    def connect(self) -> bool:
        """Connect to the required Arduino(s)"""
        logger.info(f"Connecting to Pump Arduino {self.pump_arduino_id}...")
        if not self.pump_arduino.connect():
            return False

        if self.is_separate_devices:
            logger.info(f"Connecting to Sensor Arduino {self.sensor_arduino_id}...")
            if not self.sensor_arduino.connect():
                self.pump_arduino.cleanup()  # Clean up the first connection
                return False
        return True

    def run_test(self):
        """Main test loop"""
        self.running = True
        self.start_time = time.time()

        logger.info(f"\n{'=' * 60}")
        logger.info(f"LEAKAGE TEST STARTED")
        logger.info(f"Pump Arduino ID: {self.pump_arduino_id}")
        logger.info(f"Sensor Arduino ID: {self.sensor_arduino_id}")
        logger.info(f"Target Pressure: {self.target_pressure} PSI")
        logger.info(f"{'=' * 60}\n")

        logger.info(
            f"Phase 1: Pressurizing and stabilizing ({LeakageTestConfig.STABILIZATION_TIME}s)..."
        )

        while self.running:
            current_time = time.time()
            elapsed = current_time - self.start_time

            # Send pressure command and read sensor data
            if self.is_separate_devices:
                # Send pressure to pump, ignore its sensor readings
                self.pump_arduino.send_pressure_read_sensors(self.target_pressure)
                # Send dummy command (0 pressure) to sensor Arduino to trigger data send
                sensors = self.sensor_arduino.send_pressure_read_sensors(0.0)
            else:
                # Pump and sensor are the same device
                sensors = self.pump_arduino.send_pressure_read_sensors(
                    self.target_pressure
                )

            # Store data
            self.timestamps.append(elapsed)
            for i, value in enumerate(sensors):
                self.sensor_data[i].append(value)

            # Check stabilization phase
            if (
                not self.stabilization_complete
                and elapsed >= LeakageTestConfig.STABILIZATION_TIME
            ):
                self.stabilization_complete = True
                self.baseline_pressure = [
                    sum(s[-10:]) / 10 for s in self.sensor_data if len(s) >= 10
                ]
                logger.info(
                    f"\nPhase 2: Monitoring for leakage ({LeakageTestConfig.TEST_DURATION}s)..."
                )
                logger.info(
                    f"Baseline pressures: {[f'{p:.2f}' for p in self.baseline_pressure]} PSI"
                )

            # Check for leakage (after stabilization)
            if self.stabilization_complete and not self.leakage_detected:
                for i, sensor_readings in enumerate(self.sensor_data):
                    if len(sensor_readings) >= 10:
                        recent_avg = sum(sensor_readings[-10:]) / 10
                        pressure_drop = self.baseline_pressure[i] - recent_avg

                        if pressure_drop > LeakageTestConfig.LEAKAGE_THRESHOLD:
                            self.leakage_detected = True
                            self.leakage_sensor = i + 1
                            self.leakage_time = elapsed
                            logger.warning(
                                f"\n⚠️  LEAKAGE DETECTED on Sensor {i + 1} (Arduino {self.sensor_arduino_id})!"
                            )
                            logger.warning(f"   Time: {elapsed:.1f}s")
                            logger.warning(f"   Pressure drop: {pressure_drop:.2f} PSI")

            # Check if test duration exceeded
            if self.stabilization_complete and elapsed >= (
                LeakageTestConfig.STABILIZATION_TIME + LeakageTestConfig.TEST_DURATION
            ):
                logger.info("\nTest duration completed.")
                break

            time.sleep(1.0 / LeakageTestConfig.SAMPLE_RATE)

        self._print_results()

    def _print_results(self):
        """Print test results summary"""
        logger.info(f"\n{'=' * 60}")
        logger.info(f"LEAKAGE TEST RESULTS")
        logger.info(f"{'=' * 60}")
        logger.info(f"Pump Arduino ID: {self.pump_arduino_id}")
        logger.info(f"Sensor Arduino ID: {self.sensor_arduino_id}")
        logger.info(f"Target Pressure: {self.target_pressure} PSI")
        logger.info(f"Test Duration: {self.timestamps[-1]:.1f}s")

        if self.leakage_detected:
            logger.info(f"\n❌ RESULT: LEAKAGE DETECTED")
            logger.info(
                f"   Sensor: {self.leakage_sensor} on Arduino {self.sensor_arduino_id}"
            )
            logger.info(f"   Detection Time: {self.leakage_time:.1f}s")
        else:
            logger.info(f"\n✓ RESULT: NO LEAKAGE DETECTED")
            logger.info(f"   All sensors maintained pressure within tolerance.")

        logger.info(f"\nFinal Sensor Readings (from Arduino {self.sensor_arduino_id}):")
        for i, readings in enumerate(self.sensor_data):
            if readings:
                final = readings[-1]
                if self.baseline_pressure:
                    drop = self.baseline_pressure[i] - final
                    logger.info(
                        f"   Sensor {i + 1}: {final:.2f} PSI (drop: {drop:.2f} PSI)"
                    )
                else:
                    logger.info(f"   Sensor {i + 1}: {final:.2f} PSI")

        logger.info(f"{'=' * 60}\n")

    def start_test_thread(self):
        """Start test in background thread"""
        self.test_thread = threading.Thread(target=self.run_test, daemon=True)
        self.test_thread.start()

    def stop(self):
        """Stop the test"""
        self.running = False
        if self.test_thread:
            self.test_thread.join(timeout=2.0)

    def cleanup(self):
        """Clean up resources"""
        self.stop()
        # Send zero pressure to pump before disconnecting
        try:
            self.pump_arduino.send_pressure_read_sensors(0.0)
            time.sleep(0.5)
        except Exception:
            pass

        self.pump_arduino.cleanup()
        if self.is_separate_devices:
            self.sensor_arduino.cleanup()


class RealtimePlotter:
    """Real-time plotting of sensor data"""

    def __init__(self, test: LeakageTest):
        self.test = test
        self.fig, self.ax = plt.subplots(figsize=(12, 6))
        self.lines = []
        self.colors = ["blue", "green", "red", "purple"]

        # Setup plot
        for i in range(4):
            (line,) = self.ax.plot(
                [], [], label=f"Sensor {i + 1}", color=self.colors[i], linewidth=2
            )
            self.lines.append(line)

        self.ax.set_xlabel("Time (seconds)", fontsize=12)
        self.ax.set_ylabel("Pressure (PSI)", fontsize=12)
        self.ax.set_title(
            f"Leakage Test - Pump: {test.pump_arduino_id} | Sensor: {test.sensor_arduino_id}",
            fontsize=14,
            fontweight="bold",
        )
        self.ax.legend(loc="upper right")
        self.ax.grid(True, alpha=0.3)

        self.threshold_line = None

    def update(self, frame):
        """Update plot with new data"""
        if not self.test.timestamps:
            return self.lines

        for i, line in enumerate(self.lines):
            line.set_data(self.test.timestamps, self.test.sensor_data[i])

        if self.test.timestamps:
            self.ax.set_xlim(0, max(10, self.test.timestamps[-1] + 2))
            all_values = [v for s in self.test.sensor_data for v in s if v > 0]
            if all_values:
                y_min = max(0, min(all_values) - 2)
                y_max = max(all_values) + 2
                self.ax.set_ylim(y_min, y_max)

        if (
            self.test.stabilization_complete
            and self.threshold_line is None
            and self.test.baseline_pressure
        ):
            min_baseline = min(self.test.baseline_pressure)
            threshold_y = min_baseline - LeakageTestConfig.LEAKAGE_THRESHOLD
            self.threshold_line = self.ax.axhline(
                y=threshold_y,
                color="red",
                linestyle="--",
                linewidth=2,
                label="Leakage Threshold",
                alpha=0.7,
            )
            self.ax.legend(loc="upper right")

        if self.test.leakage_detected and self.test.leakage_time:
            self.ax.axvline(
                x=self.test.leakage_time,
                color="red",
                linestyle=":",
                linewidth=2,
                alpha=0.5,
            )

        return self.lines

    def show(self):
        """Start animation and show plot"""
        ani = FuncAnimation(self.fig, self.update, interval=100, blit=False)
        plt.tight_layout()
        plt.show()


def get_device_id(prompt_message: str) -> int:
    """Get a valid Arduino ID (1-8) from the user."""
    available_ids = list(range(1, 9))
    while True:
        try:
            device_id = int(input(f"{prompt_message} {available_ids}: "))
            if device_id in available_ids:
                return device_id
            else:
                print(f"Invalid ID. Please choose from: {available_ids}")
        except ValueError:
            print("Please enter a valid number.")


def get_test_pressure() -> float:
    """Get test pressure from user"""
    default_pressure = LeakageTestConfig.TEST_PRESSURE
    while True:
        try:
            response = input(
                f"\nEnter test pressure in PSI (default {default_pressure}): "
            ).strip()
            if not response:
                return default_pressure
            pressure = float(response)
            if 0 < pressure <= 30:
                return pressure
            else:
                print("Pressure must be between 0 and 30 PSI.")
        except ValueError:
            print("Please enter a valid number.")


def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print("\n\nTest interrupted by user. Cleaning up...")
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)

    print("\n" + "=" * 60)
    print("POUCH LEAKAGE TEST TOOL")
    print("=" * 60)
    pump_arduino_id = 3
    # pump_arduino_id = get_device_id("\nEnter Pump Arduino ID")
    sensor_arduino_id = 3

    # sensor_arduino_id = get_device_id(
    #     "Enter Sensor Arduino ID (where pressure is measured)"
    # )
    test_pressure = 5
    # test_pressure = get_test_pressure()

    test = LeakageTest(pump_arduino_id, sensor_arduino_id, test_pressure)

    try:
        if not test.connect():
            logger.error("Failed to connect to one or more Arduinos. Exiting.")
            return 1

        test.start_test_thread()
        plotter = RealtimePlotter(test)
        plotter.show()

    except Exception as e:
        logger.error(f"A critical error occurred: {e}")
        return 1
    finally:
        logger.info("Cleaning up connections...")
        test.cleanup()

    return 0


if __name__ == "__main__":
    sys.exit(main())
