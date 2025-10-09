"""
PC Client for Soft Robot Pressure Control System

This module provides the main pressure control client for interfacing with
Arduino-based pressure sensors and actuators, with motion capture integration.
"""

import datetime
import logging
import os
import pickle
import re
import socket
import struct
import threading
import time
import zlib
from typing import List, Optional, Tuple

import numpy as np
import zmq

from vtech_config import ControlConfig, DataConfig, NetworkConfig, ThreadConfig
from wave_generation import (
    MultiChannelSignalGenerator,
    SignalParameters,
    SignalType,
    create_ramp_response,
    create_sine_wave,
    create_step_response,
)

# Setup logging
logger = logging.getLogger(__name__)


class CommunicationManager:
    """Manages network communications with Arduino devices and motion capture system"""

    def __init__(self, arduino_ids: List[int]):
        self.arduino_ids = arduino_ids
        self.client_sockets = []
        self.client_addresses = []
        self.zmq_context = None
        self.pub_socket = None
        self.sub_socket = None

    def initialize_arduino_connections(self) -> bool:
        """Initialize TCP connections to Arduino devices"""
        try:
            logger.info(
                f"Initializing connections to Arduino devices: {self.arduino_ids}"
            )

            for arduino_id in self.arduino_ids:
                server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                port = NetworkConfig.ARDUINO_PORTS[arduino_id - 1]
                server_socket.bind((NetworkConfig.PC_ADDRESS, port))
                server_socket.listen(1)

                logger.info(
                    f"Waiting for Arduino {arduino_id} connection on port {port}..."
                )
                client_socket, client_address = server_socket.accept()
                logger.info(f"Connected to Arduino {arduino_id} at {client_address}")

                self.client_sockets.append(client_socket)
                self.client_addresses.append(client_address)

            return True

        except Exception as e:
            logger.error(f"Failed to initialize Arduino connections: {e}")
            return False

    def initialize_zmq_sockets(self, use_mocap: bool = True) -> bool:
        """Initialize ZMQ sockets for data publishing and motion capture subscription"""
        try:
            self.zmq_context = zmq.Context()

            # Publisher socket for data recording
            self.pub_socket = self.zmq_context.socket(zmq.PUB)
            self.pub_socket.setsockopt(zmq.CONFLATE, True)
            self.pub_socket.bind(NetworkConfig.ZMQ_PUB_PORT)

            # Subscriber socket for motion capture data
            self.sub_socket = self.zmq_context.socket(zmq.SUB)
            self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "", encoding="utf-8")
            self.sub_socket.setsockopt(zmq.CONFLATE, True)

            if use_mocap:
                self.sub_socket.connect(NetworkConfig.ZMQ_SUB_PORT)
                logger.info("Connected to motion capture system")

            logger.info("ZMQ sockets initialized successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to initialize ZMQ sockets: {e}")
            return False

    def cleanup(self):
        """Clean up network connections"""
        try:
            for socket in self.client_sockets:
                socket.close()

            if self.pub_socket:
                self.pub_socket.close()
            if self.sub_socket:
                self.sub_socket.close()
            if self.zmq_context:
                self.zmq_context.term()

            logger.info("Network connections cleaned up")

        except Exception as e:
            logger.error(f"Error during cleanup: {e}")


class DataManager:
    """Manages data arrays and recording functionality"""

    def __init__(self, num_arduinos: int):
        self.num_arduinos = num_arduinos
        self.initialize_data_arrays()

    def initialize_data_arrays(self):
        """Initialize data storage arrays"""
        # Motion capture data: base(x y z qw qx qy qz) + top(x1 y1 z1 qw1 qx1 qy1 qz1)
        self.mocap_data = np.zeros(DataConfig.MOCAP_DATA_SIZE)

        # Pressure data arrays
        self.desired_pressure = np.zeros(self.num_arduinos)
        self.measured_pressure = np.zeros(
            (self.num_arduinos, DataConfig.PRESSURE_SENSORS_PER_ARDUINO)
        )

        # Combined recording array
        self.update_combined_record()

    def update_combined_record(self):
        """Update the combined data array for recording"""
        pm_flattened = self.measured_pressure.flatten()
        self.combined_record = np.concatenate(
            (self.desired_pressure, pm_flattened, self.mocap_data), axis=None
        )


class PressureController:
    """docstring for PressureController"""

    def __init__(self, arduino_ids: Optional[List[int]] = None):
        """Initialize the pressure control system

        Args:
            arduino_ids: List of Arduino device IDs to connect to
        """
        # Configuration
        self.arduino_ids = arduino_ids or ControlConfig.DEFAULT_ARDUINO_IDS.copy()
        self.ready = self._check_user_ready()

        if not self.ready:
            logger.warning("User not ready, initialization aborted")
            return

        # Control flags
        self.flag_use_mocap = True
        self.flag_control_mode = ControlConfig.CONTROL_MODE_SMC_ILC
        self.flag_reset = True
        self.trial_start_reset = True

        # Initialize components
        self.comm_manager = CommunicationManager(self.arduino_ids)
        self.data_manager = DataManager(len(self.arduino_ids))

        # Initialize connections
        if not self._initialize_system():
            logger.error("Failed to initialize system")
            return

        # Initialize threading components
        self._initialize_threading()

        # Set default control parameters
        self.positionProfile_flag = ControlConfig.DEFAULT_POSITION_PROFILE
        self.trailDuriation = ControlConfig.DEFAULT_TRIAL_DURATION

        # Initialize wave generation system
        self.wave_generator = MultiChannelSignalGenerator(len(self.arduino_ids))
        self.t0_on_trial = None  # Start time for current trial

        logger.info("Pressure controller initialized successfully")

    def _check_user_ready(self) -> bool:
        """Check if user is ready to start the system"""
        try:
            ready = input("If Ready Enter 'y': ")
            return ready.lower() == "y"
        except Exception as e:
            logger.error(f"Error getting user input: {e}")
            return False

    def _initialize_system(self) -> bool:
        """Initialize the complete system"""
        # Initialize Arduino connections
        if not self.comm_manager.initialize_arduino_connections():
            return False

        # Initialize ZMQ sockets
        if not self.comm_manager.initialize_zmq_sockets(self.flag_use_mocap):
            return False

        logger.info("System initialization complete")
        return True

    def _initialize_threading(self):
        """Initialize threading components"""
        self.th1_flag = True
        self.th2_flag = True
        self.th3_flag = True

        self.run_event = threading.Event()
        self.run_event.set()

        # Create threads
        self.th1 = threading.Thread(name="pressure_generation", target=self.th_pd_gen)
        self.th2 = threading.Thread(name="mocap_data", target=self.th_data_exchange)
        self.th3 = threading.Thread(
            name="data_publishing", target=self.th_data_exchange_high
        )

        logger.info("Threading components initialized")

    @property
    def NArs(self) -> List[int]:
        """Compatibility property for existing code"""
        return self.arduino_ids

    @property
    def client_sockets(self) -> List[socket.socket]:
        """Compatibility property for existing code"""
        return self.comm_manager.client_sockets

    @property
    def socket1(self) -> zmq.Socket:
        """Compatibility property for existing code"""
        return self.comm_manager.pub_socket

    @property
    def socket2(self) -> zmq.Socket:
        """Compatibility property for existing code"""
        return self.comm_manager.sub_socket

    @property
    def pd_array_1(self) -> np.ndarray:
        """Compatibility property for existing code"""
        return self.data_manager.desired_pressure

    @property
    def pm_array_1(self) -> np.ndarray:
        """Compatibility property for existing code"""
        return self.data_manager.measured_pressure

    @property
    def arr_comb_record(self) -> np.ndarray:
        """Compatibility property for existing code"""
        return self.data_manager.combined_record

    @property
    def array3setswithrotation(self) -> np.ndarray:
        """Compatibility property for existing code"""
        return self.data_manager.mocap_data

    @array3setswithrotation.setter
    def array3setswithrotation(self, value: np.ndarray):
        """Setter for mocap data"""
        self.data_manager.mocap_data = value
        self.data_manager.update_combined_record()

    @arr_comb_record.setter
    def arr_comb_record(self, value: np.ndarray):
        """Setter for combined record"""
        self.data_manager.combined_record = value

    def test_triangular_wave(
        self, frequency=0.2, upper_bound=5.0, lower_bound=0.0, duration=30.0
    ):
        """
        Test function to demonstrate the triangular wave functionality

        Args:
            frequency (float): Frequency in Hz (default: 0.2 Hz = 5 second cycle)
            upper_bound (float): Maximum pressure in psi (default: 5.0)
            lower_bound (float): Minimum pressure in psi (default: 0.0)
            duration (float): Test duration in seconds (default: 30.0)
        """
        print("=== TRIANGULAR WAVE TEST ===")
        print(f"Frequency: {frequency} Hz ({1 / frequency:.1f} second cycle)")
        print(f"Pressure range: {lower_bound} to {upper_bound} psi")
        print(f"Duration: {duration} seconds")
        print("=" * 50)

        # Reset trial time
        self.t0_on_trial = time.time()

        # Run the triangular wave
        self.pres_single_triangular_response(
            frequency, upper_bound, lower_bound, duration
        )

        # Reset to zero pressure after test
        print("Resetting all actuators to 0 psi...")
        for i in range(len(self.NArs)):
            self.pd_array_1[i] = 0.0
            self.pm_array_1[i] = self.ard_socket(0.0, self.client_sockets[i])

        print("Triangular wave test completed!")

    def cleanup(self):
        """Clean up resources"""
        if self.comm_manager:
            self.comm_manager.cleanup()
        if self.wave_generator:
            self.wave_generator.stop_all()
        logger.info("Pressure controller cleaned up")

    # Enhanced Wave Generation Methods using the new module
    def setup_step_response(
        self,
        channel: int,
        step_value: float,
        step_time: float = 1.0,
        duration: float = 10.0,
    ):
        """Setup step response for a specific channel"""
        generator = create_step_response(step_value, step_time, duration)
        self.wave_generator.set_channel_generator(channel, generator)
        logger.info(
            f"Setup step response for channel {channel}: value={step_value}, time={step_time}"
        )

    def setup_ramp_response(
        self,
        channel: int,
        start: float,
        end: float,
        up_rate: float,
        down_rate: float,
        hold_time: float = 2.0,
        duration: float = 20.0,
    ):
        """Setup ramp response for a specific channel"""
        generator = create_ramp_response(
            start, end, up_rate, down_rate, hold_time, duration
        )
        self.wave_generator.set_channel_generator(channel, generator)
        logger.info(
            f"Setup ramp response for channel {channel}: {start}→{end}, rates=({up_rate},{down_rate})"
        )

    def setup_sine_wave(
        self,
        channel: int,
        amplitude: float,
        frequency: float,
        offset: float = 0.0,
        phase: float = 0.0,
        duration: float = 10.0,
    ):
        """Setup sine wave for a specific channel"""
        generator = create_sine_wave(amplitude, frequency, offset, phase, duration)
        self.wave_generator.set_channel_generator(channel, generator)
        logger.info(
            f"Setup sine wave for channel {channel}: amp={amplitude}, freq={frequency}Hz"
        )

    def start_wave_generation(self):
        """Start all configured wave generators"""
        self.t0_on_trial = time.time()
        self.wave_generator.start_all()
        logger.info("Started wave generation for all channels")

    def stop_wave_generation(self):
        """Stop all wave generators"""
        self.wave_generator.stop_all()
        logger.info("Stopped wave generation for all channels")

    def get_generated_pressures(self) -> List[float]:
        """Get current pressure values from wave generators"""
        return self.wave_generator.get_all_values()

    def th_pd_gen(self):
        print("th_Pd_G")
        try:
            if self.flag_reset == 1:
                # seg_5 = 0 #r1
                # seg_6 = 0 #sens
                # seg_7 = 0 #r2
                # seg_8 = 0
                # seg_4 = 2

                seg_1 = 5
                # seg_2 = 0
                seg_3 = 0
                seg_4 = 0

                array = [seg_1]

                # array = [seg_7,seg_8]

                self.t0_on_trial = time.time()
                self.pres_single_step_response((array), 10)

                self.flag_reset = 0
            self.t0_on_glob = time.time()
            # print(time.time()-self.t0_on_glob)
            while time.time() - self.t0_on_glob < self.trailDuriation:
                print(
                    f"Main control loop iteration - elapsed: {time.time() - self.t0_on_glob:.2f}s"
                )
                try:
                    print("Receiving mocap data...")
                    # if self.flag_use_mocap == True:
                    self.array3setswithrotation = self.recv_cpp_socket2()
                    print("Mocap data received, starting ramp cycles...")
                    up_rate = 1.0  # psi/s
                    down_rate = 1.0  # psi/s
                    lower_bound = 0.0  # psi
                    upper_bound = 2.0  # psi

                    for i in range(6):
                        print(f"Starting ramp cycle {i + 1}/6")
                        if self.trial_start_reset == 1:
                            self.t0_on_trial = time.time()
                            self.trial_start_reset = 0
                        # self.pres_single_step_response(np.array([0.0]*len(self.NArs)),10)
                        # self.t0_on_trial = time.time()
                        # self.pres_single_step_response(np.array([5.0]*len(self.NArs)),10)
                        # self.pres_single_ramp_response(
                        # up_rate, down_rate, upper_bound, lower_bound
                        # )
                        # print(f"Completed ramp cycle {i + 1}/6")
                        # self.trial_start_reset = 1

                        # Alternative: Use triangular wave instead of ramp
                        # Uncomment the following lines to use triangular wave:
                        frequency = (
                            0.1  # 0.1 Hz = one complete triangle every 10 seconds
                        )
                        self.pres_single_triangular_response(
                            frequency, upper_bound, lower_bound, duration=20.0
                        )

                except KeyboardInterrupt:
                    break
                    print("E-stop")
                    self.th1_flag = False
                    self.th2_flag = False
            if self.flag_reset == 0:
                self.t0_on_trial = time.time()
                self.pres_single_step_response(np.array([0.0] * len(self.NArs)), 10)
                self.flag_reset = 1
            self.th1_flag = False
            self.th2_flag = False
            print("Done")
            exit()
        except KeyboardInterrupt:
            self.th1_flag = False
            self.th2_flag = False
            print("Press Ctrl+C to Stop")

    def th_data_exchange(
        self,
    ):  # thread config of read data from mocap and send packed msg to record file.
        print("th_data_ex")
        # print("Run State: ", self.run_event.is_set())
        # print("th2_flag: ", self.th2_flag)
        # print("Mocap Flag: ", self.flag_use_mocap)
        while self.run_event.is_set() and self.th2_flag:
            try:
                if self.flag_use_mocap:
                    self.array3setswithrotation = (
                        self.recv_cpp_socket2()
                    )  # ADD PUBSUB Pm Pd
                    # print(self.array3setswithrotation)
                # Always send data to socket1 for recording, regardless of reset flag
                self.send_zipped_socket1(self.arr_comb_record)

                # Small delay to prevent overwhelming the mocap data reception
                time.sleep(0.005)  # 5ms delay for ~200Hz data collection rate

            except KeyboardInterrupt:
                print(Exception)
                break
                self.th1_flag = False
                self.th2_flag = False
                exit()

    def experiment_number(self):
        experiment_date = datetime.datetime.today().strftime("%B-%d")
        # Use absolute path to experiments directory in RISE_Lab
        base_dir = os.path.dirname(os.path.abspath(__file__))
        experiment_dir = os.path.join(base_dir, "experiments", experiment_date)

        # Ensure the directory exists
        os.makedirs(experiment_dir, exist_ok=True)

        # Get the next experiment number
        existing_numbers = []
        for f in os.listdir(experiment_dir):
            match = re.search(r"Test_\d+_(\d+)\.csv", f)
            if match:
                existing_numbers.append(int(match.group(1)))

        # Construct and return the dynamic filename
        experiment_number = max(existing_numbers, default=0) + 1
        return os.path.join(
            base_dir,
            "experiments",
            experiment_date,
            f"Test_{self.NArs[0]}_{experiment_number}.csv",
        )

    def th_data_exchange_high(self):
        print("th_data_exHIGH")
        file_name = self.experiment_number()
        print("Logging to: ", file_name)
        start_time = time.time()
        received_count = 0
        self.comm_rate = []

        # Build header
        header = ["time"]
        header += [f"pd_{n}" for n in self.NArs]
        for n in self.NArs:
            header += [f"pm_{n}_{i + 1}" for i in range(4)]
        # Add mocap headers for 3 bodies, each with 7 values
        mocap_labels = ["x", "y", "z", "qx", "qy", "qz", "qw"]
        for body in range(1, 4):
            header += [f"mocap{body}_{label}" for label in mocap_labels]
        header_line = ",".join(header) + "\n"

        with open(file_name, "w+") as data_file:
            data_file.write(header_line)  # Write header
            t0 = time.time()
            while self.run_event.is_set() and self.th3_flag:
                try:
                    # Always use the most current data for recording
                    # Flatten pm_array_1 to match the header structure
                    pm_flattened = self.pm_array_1.flatten()
                    self.arr_comb_record = np.concatenate(
                        (self.pd_array_1, pm_flattened, self.array3setswithrotation),
                        axis=None,
                    )
                    msg = self.arr_comb_record
                    lines = (
                        str(round((time.time() - t0), 6))
                        + ","
                        + np.array2string(msg, separator=",")
                        .replace("[", "")
                        .replace("]", "")
                        .replace("\n", "")
                        + "\n"
                    )
                    data_file.write(lines)
                    data_file.flush()
                    received_count += 1

                    current_time = time.time()
                    if current_time - start_time >= 10:
                        communication_rate = received_count / (
                            current_time - start_time
                        )
                        self.comm_rate.append(communication_rate)
                        start_time = current_time
                        received_count = 0

                    # Always send data to socket1 for other processes
                    self.send_zipped_socket1(self.arr_comb_record)

                    # Small delay to prevent overwhelming the system and ensure proper data collection
                    time.sleep(0.01)  # 10ms delay for ~100Hz data collection rate

                except KeyboardInterrupt:
                    break
                    exit()

    # def pres_single_ramp_response(self, up_rate, down_rate, upper_bound, lower_bound):

    #     # print("ramping")
    #     t = time.time() - self.t0_on_trial  # range from 0
    #     total = (upper_bound - lower_bound) / up_rate + (
    #         upper_bound - lower_bound
    #     ) / down_rate
    #     while self.th1_flag and self.th2_flag and (t <= total):

    #         try:
    #             t = time.time() - self.t0_on_trial  # range from 0
    #             if t <= (upper_bound - lower_bound) / up_rate:
    #                 for i in range(len(self.NArs)):
    #                     self.pd_array_1[i] = lower_bound + up_rate * t
    #                     self.pm_array_1[i] = self.ard_socket(
    #                         self.pd_array_1[i], self.client_sockets[i]
    #                     )
    #                     if i != len(self.NArs) - 1:
    #                         time.sleep(5)

    #             if ((upper_bound - lower_bound) / up_rate < t) and (t <= total):
    #                 for i in range(len(self.NArs)):
    #                     self.pd_array_1[i] = upper_bound - down_rate * (
    #                         t - (upper_bound - lower_bound) / up_rate
    #                     )
    #                     self.pm_array_1[i] = self.ard_socket(
    #                         self.pd_array_1[i], self.client_sockets[i]
    #                     )
    #                     if i != len(self.NArs) - 1:
    #                         time.sleep(5)

    #             # for i in range(len(self.NArs)):
    #             #     # if i == 2 or i == 3 :
    #             #     #     self.pm_array_1[i] = self.ard_socket(3,self.client_sockets[i])
    #             #     # else:
    #             #     #     self.pm_array_1[i] = self.ard_socket(self.pd_array_1[i],self.client_sockets[i])
    #             #     self.pm_array_1[i] = self.ard_socket(
    #             #         self.pd_array_1[i], self.client_sockets[i]
    #             #     )

    #         except KeyboardInterrupt:
    #             break
    #             self.th1_flag = 0
    #             self.th2_flag = 0

    def pres_single_ramp_response(self, up_rate, down_rate, upper_bound, lower_bound):
        """
        Performs a SEQUENTIAL ramp-up and ramp-down of the Arduinos.
        This version includes checks for self.th1_flag to allow for safe thread termination.
        """
        # --- Part 1: Ramp UP, one by one ---
        print("Starting sequential ramp UP...")
        try:
            for i in range(len(self.NArs)):
                # Check for stop signal before starting the next ramp
                if not self.th1_flag:
                    print("Stop signal received, aborting ramp sequence.")
                    return
                if self.NArs[i] == 4:
                    #     self.pm_array_1[j] = self.ard_socket(self.pd_array_1[j], self.client_sockets[j])

                    continue  # This jumps to the next Arduino in the list
                print(f"--> Ramping UP Arduino {self.NArs[i]}...")

                t0_ramp = time.time()
                ramp_duration = (upper_bound - lower_bound) / up_rate

                while True:
                    # Check for stop signal during the ramp
                    if not self.th1_flag:
                        print(" Stop signal received, aborting ramp sequence.")
                        return

                    t_elapsed = time.time() - t0_ramp
                    if t_elapsed > ramp_duration:
                        break

                    self.pd_array_1[i] = lower_bound + up_rate * t_elapsed
                    for j in range(len(self.NArs)):
                        self.pm_array_1[j] = self.ard_socket(
                            self.pd_array_1[j], self.client_sockets[j]
                        )
                    time.sleep(0.02)  # Small delay to prevent flooding sockets

                # After the ramp, lock the current Arduino at the final pressure
                print(f"    Arduino {self.NArs[i]} ramp up complete.")
                self.pd_array_1[i] = upper_bound
                # Send one last update to lock in the state
                for j in range(len(self.NArs)):
                    self.pm_array_1[j] = self.ard_socket(
                        self.pd_array_1[j], self.client_sockets[j]
                    )

            print("\nAll Arduinos are at the upper bound. Pausing before ramp down.\n")
            time.sleep(2.0)  # Optional pause after all are ramped up

            # --- Part 2: Ramp DOWN, one by one ---
            print("Starting sequential ramp DOWN...")
            for i in range(len(self.NArs)):
                # Check for stop signal before starting the next ramp
                if not self.th1_flag:
                    print("Stop signal received, aborting ramp sequence.")
                    return
                if self.NArs[i] == 4:
                    continue  # This jumps to the next Arduino in the list

                print(f"--> Ramping DOWN Arduino {self.NArs[i]}...")

                t0_ramp = time.time()
                ramp_duration = (upper_bound - lower_bound) / down_rate

                while True:
                    # Check for stop signal during the ramp
                    if not self.th1_flag:
                        print("Stop signal received, aborting ramp sequence.")
                        return

                    t_elapsed = time.time() - t0_ramp
                    if t_elapsed > ramp_duration:
                        break

                    self.pd_array_1[i] = upper_bound - down_rate * t_elapsed
                    for j in range(len(self.NArs)):
                        self.pm_array_1[j] = self.ard_socket(
                            self.pd_array_1[j], self.client_sockets[j]
                        )
                    time.sleep(0.02)

                # Lock the Arduino at the lower bound after its ramp is done
                print(f"    Arduino {self.NArs[i]} ramp down complete.")
                self.pd_array_1[i] = lower_bound
                for j in range(len(self.NArs)):
                    self.pm_array_1[j] = self.ard_socket(
                        self.pd_array_1[j], self.client_sockets[j]
                    )
        except KeyboardInterrupt:
            self.th1_flag = 0
            self.th2_flag = 0

    def pres_single_triangular_response(
        self, frequency, upper_bound, lower_bound, duration=None
    ):
        """
        Generates a triangular wave pressure response for all Arduinos simultaneously.

        Args:
            frequency (float): Frequency of the triangular wave in Hz (cycles per second)
                             For example: 0.1 Hz = one complete triangle every 10 seconds
                                         0.5 Hz = one complete triangle every 2 seconds
            upper_bound (float): Maximum pressure value in psi
            lower_bound (float): Minimum pressure value in psi
            duration (float): Duration to run the wave in seconds. If None, runs indefinitely

        The wave goes from lower_bound to upper_bound and back to lower_bound in one cycle.

        Example usage:
            # Slow wave: 0 to 5 psi with 10-second cycles for 60 seconds
            controller.pres_single_triangular_response(0.1, 5.0, 0.0, 60.0)

            # Fast wave: 0 to 3 psi with 2-second cycles for 20 seconds
            controller.pres_single_triangular_response(0.5, 3.0, 0.0, 20.0)
        """
        print(
            f"Starting triangular wave response: {lower_bound} to {upper_bound} psi at {frequency} Hz"
        )

        if self.t0_on_trial is None:
            self.t0_on_trial = time.time()

        try:
            # Calculate period of one complete triangle cycle
            period = 1.0 / frequency  # seconds per cycle
            half_period = period / 2.0  # time for up ramp or down ramp
            amplitude = upper_bound - lower_bound

            t_start = time.time()

            while self.th1_flag and self.th2_flag:
                # Check duration limit if specified
                if duration is not None and (time.time() - t_start) > duration:
                    print("Triangular wave duration completed.")
                    break

                # Calculate current time within the cycle
                t_elapsed = time.time() - t_start
                t_cycle = t_elapsed % period  # time within current cycle (0 to period)

                if t_cycle <= half_period:
                    # First half: ramp up from lower_bound to upper_bound
                    progress = t_cycle / half_period  # 0 to 1
                    current_pressure = lower_bound + (amplitude * progress)
                else:
                    # Second half: ramp down from upper_bound to lower_bound
                    progress = (t_cycle - half_period) / half_period  # 0 to 1
                    current_pressure = upper_bound - (amplitude * progress)

                # Apply the same pressure to all Arduinos
                for i in range(len(self.NArs)):
                    self.pd_array_1[i] = current_pressure

                # Send to all Arduinos and get feedback
                for j in range(len(self.NArs)):
                    self.pm_array_1[j] = self.ard_socket(
                        self.pd_array_1[j], self.client_sockets[j]
                    )

                # Small delay to control update rate
                time.sleep(0.01)  # 100Hz update rate

            print("Triangular wave response completed.")

        except KeyboardInterrupt:
            print("Triangular wave interrupted by user")
            self.th1_flag = False
            self.th2_flag = False

    # def pres_single_ramp_response(self, up_rate, down_rate, upper_bound, lower_bound):
    # # print("ramping")
    # t = time.time() - self.t0_on_trial  # range from 0
    # total = (upper_bound - lower_bound) / up_rate + (
    #     upper_bound - lower_bound
    # ) / down_rate
    # while self.th1_flag and self.th2_flag and (t <= total):

    #     try:
    #         t = time.time() - self.t0_on_trial  # range from 0
    #         if t <= (upper_bound - lower_bound) / up_rate:
    #             for i in range(len(self.NArs)):
    #                 self.pd_array_1[i] = (
    #                     lower_bound + up_rate * t
    #                 )  # comment out the code from this line up until line 250 for sine wave

    #         if ((upper_bound - lower_bound) / up_rate < t) and (t <= total):
    #             for i in range(len(self.NArs)):
    #                 self.pd_array_1[i] = upper_bound - down_rate * (
    #                     t - (upper_bound - lower_bound) / up_rate
    #                 )

    #         for i in range(len(self.NArs)):
    #             # if i == 2 or i == 3 :
    #             #     self.pm_array_1[i] = self.ard_socket(3,self.client_sockets[i])
    #             # else:
    #             #     self.pm_array_1[i] = self.ard_socket(self.pd_array_1[i],self.client_sockets[i])
    #             self.pm_array_1[i] = self.ard_socket(
    #                 self.pd_array_1[i], self.client_sockets[i]
    #             )

    # for i in range(len(self.NArs)):
    #     if i == col:
    #         self.pm_array_1[i] = self.ard_socket(self.pd_array_1[i],self.client_sockets[i])
    #     else:
    #         self.pm_array_1[i] = self.ard_socket(0,self.client_sockets[i])

    # self.pm_array_1 = self.ard_socket(self.pd_array_1[i],self.client_sockets[3])
    # print(self.pm_array_1)

    def pres_single_step_response(self, pd_array, step_time):
        """Legacy step response method - maintained for backward compatibility"""
        if self.t0_on_trial is None:
            logger.warning("Trial start time not set, using current time")
            self.t0_on_trial = time.time()

        t = time.time() - self.t0_on_trial  # range from 0
        while self.th1_flag and self.th2_flag and (t <= step_time):
            try:
                t = time.time() - self.t0_on_trial  # range from 0
                for i in range(len(self.NArs)):
                    self.pd_array_1[i] = pd_array[i]
                    self.pm_array_1[i] = self.ard_socket(
                        self.pd_array_1[i], self.client_sockets[i]
                    )
            except KeyboardInterrupt:
                logger.info("Step response interrupted by user")
                self.th1_flag = False
                self.th2_flag = False
                break

    def pres_wave_controlled_response(self, duration: Optional[float] = None):
        """New method using wave generation module for pressure control"""
        if duration is None:
            duration = self.trailDuriation

        if self.t0_on_trial is None:
            self.t0_on_trial = time.time()

        logger.info(f"Starting wave-controlled pressure response for {duration}s")

        try:
            while self.th1_flag and self.th2_flag:
                t = time.time() - self.t0_on_trial
                if t > duration:
                    break

                # Get pressure values from wave generators
                generated_pressures = self.get_generated_pressures()

                # Apply generated pressures to actuators
                for i in range(len(self.NArs)):
                    if i < len(generated_pressures):
                        self.pd_array_1[i] = generated_pressures[i]
                    else:
                        self.pd_array_1[i] = 0.0

                    # Send to Arduino and get feedback
                    self.pm_array_1[i] = self.ard_socket(
                        self.pd_array_1[i], self.client_sockets[i]
                    )

                time.sleep(0.01)  # Small delay to prevent overwhelming the system

        except KeyboardInterrupt:
            logger.info("Wave-controlled response interrupted by user")
            self.th1_flag = False
            self.th2_flag = False
            self.stop_wave_generation()

    ###############Arduino Sockets###########################

    def ard_socket(self, pd_array, client, flags=0):
        packed_data = struct.pack("f", pd_array)
        client.send(packed_data)
        received_data = b""
        while len(received_data) < 8:
            chunk = client.recv(8)
            if not chunk:
                print("Connection to Arduino closed")
                break
            received_data += chunk

        if len(received_data) < 8:
            print("Data from Arduino is messed")
            return None  # Data is too short, skip processing

        # Unpack the received float
        received_float = struct.unpack(">4h", received_data)
        data = list(received_float)

        for i in range(0, len(data)):
            pressure_volt_1 = received_float[i] * (12.288 / 65536.0)
            data[i] = round(
                (((30.0 - 0.0) * (pressure_volt_1 - (0.1 * 5.0))) / (0.8 * 5.0)), 4
            )

        return data

    def send_zipped_socket1(self, obj, flags=0, protocol=-1):
        """pack and compress an object with pickle and zlib."""
        pobj = pickle.dumps(obj, protocol)
        zobj = zlib.compress(pobj)
        self.socket1.send(zobj, flags=flags)

    def recv_zipped_socket2(self, flags=0):
        """reconstruct a Python object sent with zipped_pickle"""
        zobj = self.socket2.recv(flags)
        pobj = zlib.decompress(zobj)
        return pickle.loads(pobj)

    def recv_cpp_socket2(self):
        """Receive motion capture data from socket"""
        try:
            # Use non-blocking receive with timeout to prevent hanging
            strMsg = self.socket2.recv(zmq.NOBLOCK)
            floatArray = np.fromstring(strMsg.decode("utf-8"), dtype=float, sep=",")
            # Cache the last received data
            self._last_mocap_data = floatArray
            return floatArray
        except zmq.Again:
            # No message available, return previous data or zeros
            logger.debug("No mocap data available, using previous/zero data")
            return getattr(
                self, "_last_mocap_data", np.zeros(DataConfig.MOCAP_DATA_SIZE)
            )
        except Exception as e:
            logger.error(f"Error receiving motion capture data: {e}")
            return getattr(
                self, "_last_mocap_data", np.zeros(DataConfig.MOCAP_DATA_SIZE)
            )


# Compatibility alias for existing code
pc_client = PressureController
