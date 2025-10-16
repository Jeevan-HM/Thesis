"""
PC Client for Soft Robot Pressure Control System

This module provides the main pressure control client for interfacing with
Arduino-based pressure sensors and actuators, with motion capture integration.

Note: External dependencies (numpy, zmq, vtech_config, wave_generation) have been
removed to create a dependency-free version.
"""

import datetime
import logging
import os
import socket
import threading
import time
from typing import List, Optional, Tuple

# Import wave generation functions
import wave_functions

# Import ZMQ for mocap communication
try:
    import numpy as np
    import zmq

    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False
    print("Warning: ZMQ and/or numpy not available. Mocap functionality disabled.")

# Setup logging
logger = logging.getLogger(__name__)


# Configuration constants (moved from vtech_config)
class NetworkConfig:
    # PC_ADDRESS = "10.203.49.197"
    PC_ADDRESS = "10.211.215.251"
    ARDUINO_PORTS = [10001, 10002, 10003, 10004, 10005, 10006, 10007, 10008]
    ZMQ_PUB_PORT = "tcp://127.0.0.1:5555"
    ZMQ_SUB_PORT = (
        "tcp://127.0.0.1:3885"  # Port where arm_mocap_pubsub publishes mocap data
    )


class DataConfig:
    MOCAP_DATA_SIZE = 21
    PRESSURE_CHANNELS = 16


class ControlConfig:
    MAX_PRESSURE = 100.0
    MIN_PRESSURE = 0.0
    SAMPLE_RATE = 100


class ThreadConfig:
    MOCAP_THREAD_DELAY = 0.01
    CONTROL_THREAD_DELAY = 0.01


class CommunicationManager:
    """Manages network communications with Arduino devices and mocap system"""

    def __init__(self, arduino_ids: List[int]):
        self.arduino_ids = arduino_ids
        self.server_sockets = []
        self.client_sockets = []
        self.client_addresses = []
        self.zmq_context = None
        self.pub_socket = None
        self.sub_socket = None

    def initialize_arduino_connections(self) -> bool:
        """Initialize TCP connections to Arduino devices"""
        try:
            for arduino_id in self.arduino_ids:
                server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                port = NetworkConfig.ARDUINO_PORTS[arduino_id - 1]
                server_socket.bind((NetworkConfig.PC_ADDRESS, port))
                server_socket.listen(1)
                logger.info(f"Listening for Arduino {arduino_id} on port {port}")
                self.server_sockets.append(server_socket)

                client_socket, client_address = server_socket.accept()
                self.client_sockets.append(client_socket)
                self.client_addresses.append(client_address)
                logger.info(f"Arduino {arduino_id} connected from {client_address}")

            return True
        except Exception as e:
            logger.error(f"Failed to initialize Arduino connections: {e}")
            return False

    def initialize_mocap_connections(self) -> bool:
        """Initialize ZMQ connections for mocap data reception"""
        if not ZMQ_AVAILABLE:
            logger.warning("ZMQ not available - mocap connections disabled")
            return False

        try:
            self.zmq_context = zmq.Context()

            self.pub_socket = self.zmq_context.socket(zmq.PUB)
            self.pub_socket.setsockopt(zmq.CONFLATE, True)
            self.pub_socket.bind(NetworkConfig.ZMQ_PUB_PORT)

            self.sub_socket = self.zmq_context.socket(zmq.SUB)
            self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "", encoding="utf-8")
            self.sub_socket.setsockopt(zmq.CONFLATE, True)
            self.sub_socket.connect(NetworkConfig.ZMQ_SUB_PORT)

            logger.info(
                f"Connected to motion capture system on {NetworkConfig.ZMQ_SUB_PORT}"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to initialize mocap connections: {e}")
            return False

    def cleanup(self):
        """Clean up network resources"""
        for client_socket in self.client_sockets:
            try:
                client_socket.close()
            except Exception:
                pass
        self.client_sockets.clear()

        for server_socket in self.server_sockets:
            try:
                server_socket.close()
            except Exception:
                pass
        self.server_sockets.clear()

        if ZMQ_AVAILABLE:
            try:
                if self.pub_socket:
                    self.pub_socket.close()
                if self.sub_socket:
                    self.sub_socket.close()
                if self.zmq_context:
                    self.zmq_context.term()
            except Exception:
                pass


class MotionCaptureManager:
    """Manages motion capture data reception via ZMQ"""

    def __init__(self, comm_manager=None):
        self.mocap_data = [0.0] * DataConfig.MOCAP_DATA_SIZE
        self.running = False
        self.thread = None
        self.comm_manager = comm_manager
        self._last_mocap_data = None

    def set_comm_manager(self, comm_manager):
        self.comm_manager = comm_manager

    def start_mocap_thread(self):
        if (
            not ZMQ_AVAILABLE
            or not self.comm_manager
            or not self.comm_manager.sub_socket
        ):
            logger.warning(
                "Mocap thread not started - ZMQ not available or not connected"
            )
            return

        self.running = True
        self.thread = threading.Thread(target=self._mocap_loop, daemon=True)
        self.thread.start()
        logger.info("Motion capture data acquisition started")

    def _mocap_loop(self):
        while self.running:
            try:
                if self.comm_manager and self.comm_manager.sub_socket:
                    strMsg = self.comm_manager.sub_socket.recv(zmq.NOBLOCK)
                    if ZMQ_AVAILABLE:
                        floatArray = np.fromstring(
                            strMsg.decode("utf-8"), dtype=float, sep=","
                        )
                        if len(floatArray) >= DataConfig.MOCAP_DATA_SIZE:
                            self.mocap_data = floatArray[
                                : DataConfig.MOCAP_DATA_SIZE
                            ].tolist()
                            self._last_mocap_data = self.mocap_data.copy()
                        logger.debug(f"Received mocap data: {len(floatArray)} values")

            except Exception as e:
                if "zmq.Again" in str(
                    type(e)
                ) or "Resource temporarily unavailable" in str(e):
                    if self._last_mocap_data:
                        self.mocap_data = self._last_mocap_data.copy()
                else:
                    logger.error(f"Error receiving mocap data: {e}")

            time.sleep(ThreadConfig.MOCAP_THREAD_DELAY)

    def stop_mocap_thread(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        logger.info("Motion capture data acquisition stopped")

    def get_mocap_data(self) -> List[float]:
        return self.mocap_data.copy()


class PressureController:
    """Controls pressure actuators with active control loop"""

    def __init__(self):
        self.desired_pressures = [0.0] * DataConfig.PRESSURE_CHANNELS
        self.measured_pressures = [0.0] * DataConfig.PRESSURE_CHANNELS
        self.running = False
        self.control_thread = None
        self.client_ref = None

    def set_client_reference(self, client):
        self.client_ref = client

    def set_pressure(self, channel: int, pressure: float):
        if 0 <= channel < DataConfig.PRESSURE_CHANNELS:
            pressure = max(
                ControlConfig.MIN_PRESSURE, min(ControlConfig.MAX_PRESSURE, pressure)
            )
            self.desired_pressures[channel] = pressure
            logger.debug(f"Set pressure channel {channel} to {pressure}")

    def get_pressures(self) -> Tuple[List[float], List[float]]:
        return self.desired_pressures.copy(), self.measured_pressures.copy()

    def _control_loop(self):
        logger.info("Pressure control thread started")
        while self.running:
            try:
                if self.client_ref and self.client_ref.comm_manager.client_sockets:
                    for i, sock in enumerate(
                        self.client_ref.comm_manager.client_sockets
                    ):
                        if i < len(self.client_ref.NArs):
                            arduino_id = self.client_ref.NArs[i]
                            desired_pressure = (
                                self.desired_pressures[i]
                                if i < len(self.desired_pressures)
                                else 0.0
                            )
                            sensor_data = self.client_ref.ard_socket(
                                desired_pressure, sock
                            )
                            base_idx = i * 4
                            for j, sensor_val in enumerate(sensor_data):
                                if base_idx + j < len(self.measured_pressures):
                                    self.measured_pressures[base_idx + j] = sensor_val
                            logger.debug(
                                f"Arduino {arduino_id}: Sent {desired_pressure} PSI, Got sensors: {sensor_data}"
                            )
                time.sleep(ThreadConfig.CONTROL_THREAD_DELAY)
            except Exception as e:
                logger.error(f"Error in pressure control loop: {e}")
                time.sleep(0.1)
        logger.info("Pressure control thread stopped")

    def start_control_loop(self):
        if not self.running:
            self.running = True
            self.control_thread = threading.Thread(
                target=self._control_loop, daemon=True
            )
            self.control_thread.start()
            logger.info("Pressure control loop started")

    def stop_control_loop(self):
        self.running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        logger.info("Pressure control loop stopped")


class DataLogger:
    """Logs experimental data to files"""

    def __init__(self, base_path: str = "experiments"):
        self.base_path = base_path
        self.log_file = None
        self.logging = False
        self.start_time = None

    def start_logging(
        self, arduino_ids: List[int], experiment_name: Optional[str] = None
    ) -> bool:
        """Start data logging to file with dynamic headers."""
        try:
            if experiment_name is None:
                now = datetime.datetime.now()
                date_folder = now.strftime("%B-%d")
                date_folder_path = os.path.join(self.base_path, date_folder)
                os.makedirs(date_folder_path, exist_ok=True)
                experiment_number = self._get_next_experiment_number(date_folder_path)
                experiment_name = f"Experiment_{experiment_number}"
                log_path = os.path.join(date_folder_path, f"{experiment_name}.csv")
            else:
                os.makedirs(self.base_path, exist_ok=True)
                log_path = os.path.join(self.base_path, f"{experiment_name}.csv")

            full_path = os.path.abspath(log_path)
            self.log_file = open(log_path, "w")

            header_parts = ["time"]
            for arduino_id in arduino_ids:
                header_parts.append(f"pd_{arduino_id}")
            for arduino_id in arduino_ids:
                for sensor_num in range(1, 5):
                    header_parts.append(f"pm_{arduino_id}_{sensor_num}")

            mocap_labels = ["x", "y", "z", "qx", "qy", "qz", "qw"]
            if DataConfig.MOCAP_DATA_SIZE % len(mocap_labels) == 0:
                num_bodies = DataConfig.MOCAP_DATA_SIZE // len(mocap_labels)
                for body_num in range(1, num_bodies + 1):
                    for label in mocap_labels:
                        header_parts.append(f"mocap_{body_num}_{label}")
            else:
                for i in range(DataConfig.MOCAP_DATA_SIZE):
                    header_parts.append(f"mocap_{i + 1}")

            header = ",".join(header_parts)
            self.log_file.write(header + "\n")

            self.start_time = time.time()
            self.logging = True
            logger.info("DATA LOGGING STARTED")
            logger.info(f"CSV file location: {full_path}")
            logger.info(f"Reference start time: {self.start_time:.6f}")
            return True
        except Exception as e:
            logger.error(f"Failed to start logging: {e}")
            return False

    def log_data(self, timestamp: float, pressures: List[float], mocap: List[float]):
        if self.logging and self.log_file and self.start_time is not None:
            try:
                rel_time = timestamp - self.start_time
                line = f"{rel_time:.6f}"
                for p in pressures:
                    line += f",{p:.3f}"
                for m in mocap:
                    line += f",{m:.6f}"
                self.log_file.write(line + "\n")
                self.log_file.flush()
            except Exception as e:
                logger.error(f"Failed to log data: {e}")

    def _get_next_experiment_number(self, folder_path: str) -> int:
        try:
            if not os.path.exists(folder_path):
                return 1
            experiment_files = []
            for filename in os.listdir(folder_path):
                if filename.startswith("Experiment_") and filename.endswith(".csv"):
                    try:
                        number_str = filename[11:-4]
                        number = int(number_str)
                        experiment_files.append(number)
                    except ValueError:
                        continue
            return max(experiment_files) + 1 if experiment_files else 1
        except Exception as e:
            logger.warning(f"Error finding next experiment number: {e}")
            return 1

    def stop_logging(self):
        if self.log_file:
            file_path = self.log_file.name
            self.log_file.close()
            self.log_file = None
            full_path = os.path.abspath(file_path)
            logger.info("DATA LOGGING STOPPED")
            logger.info(f"Final CSV data saved to: {full_path}")
        self.logging = False


class pc_client:
    """Main pressure control client"""

    def __init__(self):
        self.NArs = [4, 3, 7, 8]
        self.pressure_array = [2.0, 10.0, 10.0]
        self.comm_manager = CommunicationManager(self.NArs)
        self.mocap_manager = MotionCaptureManager(self.comm_manager)
        self.pressure_controller = PressureController()
        self.pressure_controller.set_client_reference(self)
        self.data_logger = DataLogger()
        self.running = False
        self.pd_array_1 = [0.0] * len(self.NArs)
        self.pm_array_1 = [[0.0] * 4 for _ in range(len(self.NArs))]
        self.th1_flag = True
        self.th2_flag = True
        self.th3_flag = True
        self.t0_on_trial = None
        self.trailDuriation = 600.0
        self.flag_reset = True
        self.flag_use_mocap = True
        self.positionProfile_flag = 3
        self.logging_thread = None
        self.logging_active = False
        self.pressure_execution_thread = None
        self.pressure_execution_active = False
        self.end_after_wave = True

    def initialize(self) -> bool:
        try:
            logger.info("Initializing pressure control client...")
            if not self.comm_manager.initialize_arduino_connections():
                return False
            if not self.comm_manager.initialize_mocap_connections():
                logger.warning(
                    "Motion capture initialization failed, continuing without mocap data"
                )
            else:
                self.mocap_manager.set_comm_manager(self.comm_manager)
            logger.info("Client initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize client: {e}")
            return False

    def _data_logging_loop(self):
        logger.info("Data logging thread started")
        log_count = 0
        start_time = time.time()
        while self.logging_active and self.running:
            try:
                mocap_data = self.mocap_manager.get_mocap_data()
                pressure_log_data = []

                for i in range(len(self.NArs)):
                    pressure_log_data.append(
                        self.pd_array_1[i] if i < len(self.pd_array_1) else 0.0
                    )
                for i in range(len(self.NArs)):
                    if i < len(self.pm_array_1):
                        for j in range(4):
                            pressure_log_data.append(
                                self.pm_array_1[i][j]
                                if j < len(self.pm_array_1[i])
                                else 0.0
                            )
                    else:
                        pressure_log_data.extend([0.0] * 4)

                current_time = time.time()
                self.data_logger.log_data(current_time, pressure_log_data, mocap_data)
                log_count += 1
                time.sleep(0.01)
            except Exception as e:
                logger.error(f"Error in data logging loop: {e}")
                time.sleep(0.1)

        total_elapsed = time.time() - start_time
        avg_rate = log_count / total_elapsed if total_elapsed > 0 else 0
        logger.info("Data logging thread stopped")
        logger.info(f"   Total samples logged: {log_count}")
        logger.info(f"   Average sample rate: {avg_rate:.1f} Hz")

    def _pressure_execution_loop(self):
        logger.info("Pressure execution thread started")
        logger.info(
            f"Running pressure patterns on Arduinos {self.NArs} with pressures {self.pressure_array}"
        )
        try:
            while self.pressure_execution_active and self.running:
                pressure_descriptions = [
                    f"Arduino {self.NArs[i]} (0→{self.pressure_array[i]}→0 psi)"
                    if i < len(self.pressure_array)
                    else f"Arduino {self.NArs[i]} (0 psi)"
                    for i in range(len(self.NArs))
                ]
                logger.info(f"Sequential pattern: {', '.join(pressure_descriptions)}")

                # wave_functions.pressure_sequential_custom_response(
                #     self,
                #     ramp_up_rate=5.0,
                #     ramp_down_rate=5.0,
                #     hold_time=1.0,
                #     stabilization_time=2.0,
                #     duration=300.0,
                # )
                wave_functions.elliptical_with_one_static(
                    self,
                    # freq_hz=0.1,  # 0.1 Hz => 10 s per circle
                    # center=6.0,  # mean pressure (psi)
                    # amp=3.0,  # sine amplitude (psi); ensure center-amp >= 0
                    duration=120.0,  # run for 2 minutes (or whatever you like)
                )

                if getattr(self, "end_after_wave", False):
                    logger.info("Flag set: Ending experiment after wave cycle.")
                    self.running = False
                    break

                time.sleep(1.0)
        except Exception as e:
            logger.error(f"Error in pressure execution: {e}")
        finally:
            self.running = False
            self.pressure_execution_active = False
            self.logging_active = False
            logger.info("Pressure execution thread completed")

    def start_experiment(self, experiment_name: Optional[str] = None) -> bool:
        try:
            if not self.data_logger.start_logging(self.NArs, experiment_name):
                return False

            self.mocap_manager.start_mocap_thread()
            self.running = True

            self.logging_active = True
            self.logging_thread = threading.Thread(
                target=self._data_logging_loop, daemon=True
            )
            self.logging_thread.start()

            self.pressure_execution_active = True
            self.pressure_execution_thread = threading.Thread(
                target=self._pressure_execution_loop, daemon=True
            )
            self.pressure_execution_thread.start()

            logger.info("Experiment started")
            return True
        except Exception as e:
            logger.error(f"Failed to start experiment: {e}")
            return False

    def stop_experiment(self):
        self.running = False
        self.logging_active = False
        self.pressure_execution_active = False

        self.mocap_manager.stop_mocap_thread()

        if self.logging_thread and self.logging_thread.is_alive():
            self.logging_thread.join(timeout=2.0)
        if self.pressure_execution_thread and self.pressure_execution_thread.is_alive():
            self.pressure_execution_thread.join(timeout=2.0)

        self.data_logger.stop_logging()
        logger.info("Experiment stopped")

    def set_pressure(self, channel: int, pressure: float):
        self.pressure_controller.set_pressure(channel, pressure)

    def get_current_data(self) -> dict:
        desired_p, measured_p = self.pressure_controller.get_pressures()
        mocap_data = self.mocap_manager.get_mocap_data()
        return {
            "timestamp": time.time(),
            "desired_pressures": desired_p,
            "measured_pressures": measured_p,
            "mocap_data": mocap_data,
        }

    def is_experiment_running(self) -> bool:
        return self.running and self.pressure_execution_active

    def cleanup(self):
        self.stop_experiment()
        self.comm_manager.cleanup()
        logger.info("Client cleanup completed")

    def ard_socket(self, pd_value, client_socket, flags=0):
        import struct

        try:
            packed_data = struct.pack("f", pd_value)
            client_socket.send(packed_data)

            received_data = b""
            while len(received_data) < 8:
                chunk = client_socket.recv(8 - len(received_data))
                if not chunk:
                    return [0.0] * 4
                received_data += chunk

            received_data_unpacked = struct.unpack(">4h", received_data)
            sensor_values = list(received_data_unpacked)

            for i in range(len(sensor_values)):
                pressure_volt = sensor_values[i] * (12.288 / 65536.0)
                sensor_values[i] = round(
                    (((30.0 - 0.0) * (pressure_volt - (0.1 * 5.0))) / (0.8 * 5.0)), 4
                )
            return sensor_values
        except Exception as e:
            logger.error(f"Error in Arduino communication: {e}")
            return [0.0] * 4

    @property
    def client_sockets(self):
        return self.comm_manager.client_sockets


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    client = pc_client()
    client.comm_manager.arduino_ids = client.NArs

    try:
        if client.initialize():
            print("Client initialized successfully")
            print(f"Using Arduinos: {client.NArs}")
            print(f"Arduino pressures: {client.pressure_array}")
            print(
                f"Active Arduinos: {client.NArs} with pressures {client.pressure_array} PSI"
            )

            if client.start_experiment():
                print("Experiment started")
                print("Running step response...")
                print(f"Sending STEP SIGNALS to Arduinos {client.NArs} for 10 seconds")
                pressure_details = [
                    f"Arduino {client.NArs[i]}={client.pressure_array[i]} PSI"
                    for i in range(len(client.NArs))
                ]

                print("Running ramp response...")
                print(f"Sending RAMP SIGNALS to Arduinos {client.NArs}")
                wave_functions.pressure_three_phase_sine(
                    client,
                    freq_hz=0.1,
                    center=6.0,
                    amp=3.0,
                    duration=120.0,
                )

                client.stop_experiment()
                print("Experiment completed")
            else:
                print("Failed to start experiment")
        else:
            print("Failed to initialize client")
    finally:
        client.cleanup()
