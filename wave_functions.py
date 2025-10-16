"""
Wave Generation Functions for Soft Robot Pressure Control

This module contains all the wave generation and signal processing functions
for pressure control patterns including step, ramp, and triangular responses.
"""

import logging
import math
import time
from typing import List

# Setup logging
logger = logging.getLogger(__name__)


def _calculate_wave_cycle_duration(
    target_pressures,
    ramp_up_rate,
    ramp_down_rate,
    hold_time,
    initial_delay,
    stabilization_time=3.0,
):
    """Calculate estimated duration of one complete wave cycle with robot positioning time"""
    # Get the maximum target pressure to estimate ramp times
    max_target_pressure = max(target_pressures.values()) if target_pressures else 0.0

    if max_target_pressure <= 0:
        return initial_delay + 2.0  # Minimal duration if no pressures

    # Estimate timing components including robot positioning time
    ramp_up_time = max_target_pressure / ramp_up_rate  # Time to ramp up
    ramp_down_time = max_target_pressure / ramp_down_rate  # Time to ramp down

    # Total stabilization time per cycle:
    # - 2 * stabilization_time for individual Arduino positioning (7 and 8)
    # - 2 * stabilization_time for return positioning (7 and 8)
    # - 1 * hold_time for combined system stabilization
    # - 1 * hold_time for inter-cycle pause
    # Note: Arduino 4 setup is done once at start, not per cycle
    total_stabilization = (4 * stabilization_time) + (2 * hold_time)

    # Total cycle: initial delay + ramp times + all stabilization periods + buffer
    total_time = (
        initial_delay
        + (2 * ramp_up_time)
        + (2 * ramp_down_time)
        + total_stabilization
        + 2.0
    )  # 2s buffer

    return total_time


def _send_pressure_commands(client_instance):
    """Helper function to send pressure commands to all Arduinos"""
    for i in range(len(client_instance.NArs)):
        # Send to Arduino and get sensor feedback
        if i < len(client_instance.comm_manager.client_sockets):
            client_instance.pm_array_1[i] = client_instance.ard_socket(
                client_instance.pd_array_1[i],
                client_instance.comm_manager.client_sockets[i],
            )


def create_step_response(
    amplitude: float, duration: float, sample_rate: float = 100
) -> List[float]:
    """Create a step response signal"""
    num_samples = int(duration * sample_rate)
    return [amplitude] * num_samples


def create_sine_wave(
    amplitude: float, frequency: float, duration: float, sample_rate: float = 100
) -> List[float]:
    """Create a sine wave signal"""
    import math

    num_samples = int(duration * sample_rate)
    samples = []
    for i in range(num_samples):
        t = i / sample_rate
        value = amplitude * math.sin(2 * math.pi * frequency * t)
        samples.append(value)
    return samples


def create_ramp_response(
    start_amp: float, end_amp: float, duration: float, sample_rate: float = 100
) -> List[float]:
    """Create a ramp signal"""
    num_samples = int(duration * sample_rate)
    if num_samples <= 1:
        return [start_amp]

    step = (end_amp - start_amp) / (num_samples - 1)
    return [start_amp + i * step for i in range(num_samples)]


def pressure_step_response(client_instance, pd_array, step_time, duration=60.0):
    """Apply step response pressure to selected Arduinos (loops for duration)"""
    start_time = time.time()
    end_time = start_time + duration

    try:
        while (
            client_instance.th1_flag
            and client_instance.th2_flag
            and time.time() < end_time
        ):
            cycle_start = time.time()
            while time.time() - cycle_start < step_time and time.time() < end_time:
                for i in range(len(client_instance.NArs)):
                    if i < len(pd_array):
                        client_instance.pd_array_1[i] = pd_array[i]
                    else:
                        client_instance.pd_array_1[i] = 0.0

                    if i < len(client_instance.comm_manager.client_sockets):
                        client_instance.pm_array_1[i] = client_instance.ard_socket(
                            client_instance.pd_array_1[i],
                            client_instance.comm_manager.client_sockets[i],
                        )
                time.sleep(0.01)
    except KeyboardInterrupt:
        client_instance.th1_flag = False
        client_instance.th2_flag = False


def pressure_ramp_response(
    client_instance, up_rate, down_rate, upper_bound, lower_bound, duration=60.0
):
    """Apply ramp response pressure (loops for duration)"""
    start_time = time.time()
    end_time = start_time + duration
    ramp_up_time = (upper_bound - lower_bound) / up_rate
    ramp_down_time = (upper_bound - lower_bound) / down_rate
    cycle_time = ramp_up_time + ramp_down_time

    try:
        while (
            client_instance.th1_flag
            and client_instance.th2_flag
            and time.time() < end_time
        ):
            cycle_start = time.time()
            while time.time() - cycle_start < cycle_time and time.time() < end_time:
                t = time.time() - cycle_start

                if t <= ramp_up_time:
                    current_pressure = lower_bound + up_rate * t
                else:
                    current_pressure = upper_bound - down_rate * (t - ramp_up_time)

                for i in range(len(client_instance.NArs)):
                    client_instance.pd_array_1[i] = current_pressure
                    if i < len(client_instance.comm_manager.client_sockets):
                        client_instance.pm_array_1[i] = client_instance.ard_socket(
                            client_instance.pd_array_1[i],
                            client_instance.comm_manager.client_sockets[i],
                        )
                time.sleep(0.01)
    except KeyboardInterrupt:
        client_instance.th1_flag = False
        client_instance.th2_flag = False


def pressure_triangular_response(
    client_instance, frequency, upper_bound, lower_bound, duration=60.0
):
    """Generate triangular wave pressure response"""
    period = 1.0 / frequency
    half_period = period / 2.0
    amplitude = upper_bound - lower_bound
    start_time = time.time()
    end_time = start_time + duration

    try:
        while (
            client_instance.th1_flag
            and client_instance.th2_flag
            and time.time() < end_time
        ):
            t_elapsed = time.time() - start_time
            t_cycle = t_elapsed % period

            if t_cycle <= half_period:
                progress = t_cycle / half_period
                current_pressure = lower_bound + (amplitude * progress)
            else:
                progress = (t_cycle - half_period) / half_period
                current_pressure = upper_bound - (amplitude * progress)

            for i in range(len(client_instance.NArs)):
                client_instance.pd_array_1[i] = current_pressure
                if i < len(client_instance.comm_manager.client_sockets):
                    client_instance.pm_array_1[i] = client_instance.ard_socket(
                        client_instance.pd_array_1[i],
                        client_instance.comm_manager.client_sockets[i],
                    )
            time.sleep(0.01)

    except KeyboardInterrupt:
        logger.info("Triangular wave interrupted by user")
        client_instance.th1_flag = False
        client_instance.th2_flag = False


def pressure_sine_wave_response(
    client_instance, amplitude, frequency, offset=0.0, duration=60.0
):
    """Generate sine wave pressure response"""
    import math

    start_time = time.time()
    end_time = start_time + duration

    try:
        while (
            client_instance.th1_flag
            and client_instance.th2_flag
            and time.time() < end_time
        ):
            t_elapsed = time.time() - start_time
            current_pressure = offset + amplitude * math.sin(
                2 * math.pi * frequency * t_elapsed
            )
            current_pressure = max(0.0, current_pressure)

            for i in range(len(client_instance.NArs)):
                client_instance.pd_array_1[i] = current_pressure
                if i < len(client_instance.comm_manager.client_sockets):
                    client_instance.pm_array_1[i] = client_instance.ard_socket(
                        client_instance.pd_array_1[i],
                        client_instance.comm_manager.client_sockets[i],
                    )
            time.sleep(0.01)
    except KeyboardInterrupt:
        client_instance.th1_flag = False
        client_instance.th2_flag = False


def pressure_square_wave_response(
    client_instance, amplitude, frequency, offset=0.0, duration=60.0
):
    """Generate square wave pressure response"""
    period = 1.0 / frequency
    half_period = period / 2.0
    start_time = time.time()
    end_time = start_time + duration

    try:
        while (
            client_instance.th1_flag
            and client_instance.th2_flag
            and time.time() < end_time
        ):
            t_elapsed = time.time() - start_time
            t_cycle = t_elapsed % period

            if t_cycle < half_period:
                current_pressure = offset + amplitude
            else:
                current_pressure = offset - amplitude

            current_pressure = max(0.0, current_pressure)

            for i in range(len(client_instance.NArs)):
                client_instance.pd_array_1[i] = current_pressure
                if i < len(client_instance.comm_manager.client_sockets):
                    client_instance.pm_array_1[i] = client_instance.ard_socket(
                        client_instance.pd_array_1[i],
                        client_instance.comm_manager.client_sockets[i],
                    )
            time.sleep(0.01)
    except KeyboardInterrupt:
        client_instance.th1_flag = False
        client_instance.th2_flag = False


# def pressure_sequential_custom_response(
#     client_instance,
#     initial_delay=2.0,
#     ramp_up_rate=5.0,  # psi per second
#     ramp_down_rate=5.0,  # psi per second
#     hold_time=1.0,  # seconds to hold at target pressure for robot stabilization
#     stabilization_time=1.0,  # Additional time for robot to reach position
#     duration=60.0,  # Total duration to run the wave pattern (seconds)
# ):
#     """
#     Execute a custom sequential pressure pattern using pressures from client_instance.pressure_array:
#     Arduino 4 maintains a constant 5 psi supply throughout the experiment.
#     The pattern will loop continuously until the specified duration elapses:

#     Wave cycle:
#     1. Set Arduino 4 to constant 5 psi (maintained throughout)
#     2. Start ramping up Arduino 7 to pressure_array[1] and wait for robot to reach position
#     3. Start ramping up Arduino 8 to pressure_array[2] and wait for robot to reach position
#     4. Ramp down Arduino 7 to 0 psi and wait for robot to return
#     5. Ramp down Arduino 8 to 0 psi and wait for robot to return
#     6. Repeat cycle while keeping Arduino 4 at 5 psi until duration elapsed

#     The target pressures are automatically taken from client_instance.pressure_array
#     which corresponds to the Arduinos in client_instance.NArs order.

#     Args:
#         client_instance: The client instance with communication methods and pressure_array
#         initial_delay: Time to wait before starting (seconds)
#         ramp_up_rate: Rate of pressure increase (psi/second)
#         ramp_down_rate: Rate of pressure decrease (psi/second)
#         hold_time: Time to hold pressure for robot stabilization (seconds)
#         stabilization_time: Additional time for robot to reach target position (seconds)
#         duration: Total time to run the wave pattern with loops (seconds)
#     """
#     if client_instance.t0_on_trial is None:
#         client_instance.t0_on_trial = time.time()

#     logger.info("Starting sequential custom pressure pattern with looping")
#     logger.info(f"Wave execution start time: {client_instance.t0_on_trial:.6f}")
#     logger.info(f"Total duration: {duration} seconds")

#     # Get target pressures from vtech client's pressure_array
#     target_pressures = {}
#     for i, arduino_id in enumerate(client_instance.NArs):
#         if i < len(client_instance.pressure_array):
#             target_pressures[arduino_id] = client_instance.pressure_array[i]
#         else:
#             target_pressures[arduino_id] = 0.0

#     logger.info(f"Target pressures from vtech pressure_array: {target_pressures}")
#     logger.info(
#         f"Arduino 4 target: {target_pressures.get(4, 'Not found')} psi (constant)"
#     )
#     logger.info(f"Arduino 7 target: {target_pressures.get(7, 'Not found')} psi")
#     logger.info(f"Arduino 8 target: {target_pressures.get(8, 'Not found')} psi")

#     # Store start time for relative timing in wave execution
#     wave_start_time = time.time()
#     experiment_end_time = wave_start_time + duration

#     # Initialize all pressures to 0, except Arduino 4 which gets constant 5 psi
#     for i in range(len(client_instance.NArs)):
#         client_instance.pd_array_1[i] = 0.0

#     # Map Arduino IDs to indices in NArs array
#     arduino_indices = {}
#     for i, arduino_id in enumerate(client_instance.NArs):
#         arduino_indices[arduino_id] = i

#     # One-time ramp-up of Arduino 4 to target pressure (only at start of experiment)
#     arduino_4_idx = arduino_indices.get(4)
#     arduino_4_pressure = target_pressures.get(4)  # Default to 5.0 if not found

#     if (
#         arduino_4_idx is not None
#         and arduino_4_pressure is not None
#         and arduino_4_pressure > 0
#     ):
#         logger.info(f"Initial ramp-up: Arduino 4 to {arduino_4_pressure} psi")
#         target_pressure = arduino_4_pressure
#         ramp_time = target_pressure / ramp_up_rate
#         phase_start = time.time()

#         # Ramp up Arduino 4 once at the beginning
#         while (time.time() - phase_start) < ramp_time:
#             if not client_instance.th1_flag or not client_instance.th2_flag:
#                 return
#             if time.time() >= experiment_end_time:
#                 break

#             elapsed = time.time() - phase_start
#             current_pressure = min(target_pressure, elapsed * ramp_up_rate)
#             client_instance.pd_array_1[arduino_4_idx] = current_pressure
#             _send_pressure_commands(client_instance)
#             time.sleep(0.01)

#         # Hold Arduino 4 at target pressure for initial stabilization
#         logger.info(
#             f"Holding Arduino 4 at {arduino_4_pressure} psi for {stabilization_time}s"
#         )
#         stabilization_start = time.time()
#         while (
#             time.time() - stabilization_start
#         ) < stabilization_time and time.time() < experiment_end_time:
#             if not client_instance.th1_flag or not client_instance.th2_flag:
#                 return
#             client_instance.pd_array_1[arduino_4_idx] = arduino_4_pressure
#             _send_pressure_commands(client_instance)
#             time.sleep(0.01)

#         logger.info(
#             f"Arduino 4 setup complete - will maintain {arduino_4_pressure} psi throughout experiment"
#         )

#     # Calculate single wave cycle duration for timing
#     single_cycle_time = _calculate_wave_cycle_duration(
#         target_pressures,
#         ramp_up_rate,
#         ramp_down_rate,
#         hold_time,
#         initial_delay,
#         stabilization_time,
#     )
#     logger.info(
#         f"Wave cycle duration: {single_cycle_time:.1f}s, positioning time: {stabilization_time}s"
#     )

#     cycle_count = 0

#     try:
#         # Main loop: repeat the wave pattern until duration elapsed
#         while time.time() < experiment_end_time:
#             cycle_count += 1
#             cycle_start_time = time.time()
#             remaining_time = experiment_end_time - cycle_start_time

#             if (
#                 remaining_time < single_cycle_time * 0.3
#             ):  # If less than 30% of cycle time remains
#                 logger.info(
#                     f"Insufficient time for complete cycle {cycle_count}, stopping gracefully"
#                 )
#                 break

#             logger.info(
#                 f"Starting wave cycle {cycle_count}, remaining time: {remaining_time:.1f}s"
#             )

#             # Reset Arduino 7 and 8 pressures to 0, maintain Arduino 4 at target
#             for i in range(len(client_instance.NArs)):
#                 arduino_id = client_instance.NArs[i]
#                 if arduino_id == 4:
#                     client_instance.pd_array_1[i] = (
#                         arduino_4_pressure  # Maintain Arduino 4
#                     )
#                 else:
#                     client_instance.pd_array_1[i] = 0.0

#             # Phase 0: Initial delay (only for first cycle)
#             if cycle_count == 1:
#                 logger.info(f"Phase 0: Initial delay ({initial_delay}s)")
#                 phase_start = time.time()
#                 while (time.time() - phase_start) < initial_delay:
#                     if not client_instance.th1_flag or not client_instance.th2_flag:
#                         return
#                     if time.time() >= experiment_end_time:
#                         break
#                     _send_pressure_commands(client_instance)
#                     time.sleep(0.01)

#             # Phase 1: Start ramping up Arduino 7 to target pressure
#             arduino_7_target = target_pressures.get(7, 0.0)
#             if cycle_count == 1:  # Only log on first cycle
#                 logger.info(f"Ramping Arduino 7 to {arduino_7_target} psi")
#             arduino_7_idx = arduino_indices.get(7)
#             if (
#                 arduino_7_idx is not None
#                 and arduino_7_target > 0
#                 and time.time() < experiment_end_time
#             ):
#                 target_pressure = arduino_7_target
#                 ramp_time = target_pressure / ramp_up_rate
#                 phase_start = time.time()

#                 while (time.time() - phase_start) < ramp_time:
#                     if not client_instance.th1_flag or not client_instance.th2_flag:
#                         return
#                     if time.time() >= experiment_end_time:
#                         break

#                     elapsed = time.time() - phase_start
#                     current_pressure = min(target_pressure, elapsed * ramp_up_rate)
#                     client_instance.pd_array_1[arduino_7_idx] = current_pressure
#                     _send_pressure_commands(client_instance)
#                     time.sleep(0.01)

#                 # Hold Arduino 7 at target pressure for robot stabilization
#                 stabilization_start = time.time()
#                 while (
#                     time.time() - stabilization_start
#                 ) < stabilization_time and time.time() < experiment_end_time:
#                     if not client_instance.th1_flag or not client_instance.th2_flag:
#                         return
#                     client_instance.pd_array_1[arduino_7_idx] = arduino_7_target
#                     _send_pressure_commands(client_instance)
#                     time.sleep(0.01)

#             # Phase 2: Start ramping up Arduino 8 to target pressure
#             arduino_8_target = target_pressures.get(8, 0.0)
#             if cycle_count == 1:  # Only log on first cycle
#                 logger.info(f"Ramping Arduino 8 to {arduino_8_target} psi")
#             arduino_8_idx = arduino_indices.get(8)
#             if (
#                 arduino_8_idx is not None
#                 and arduino_8_target > 0
#                 and time.time() < experiment_end_time
#             ):
#                 target_pressure = arduino_8_target
#                 ramp_time = target_pressure / ramp_up_rate
#                 phase_start = time.time()

#                 while (time.time() - phase_start) < ramp_time:
#                     if not client_instance.th1_flag or not client_instance.th2_flag:
#                         return
#                     if time.time() >= experiment_end_time:
#                         break

#                     elapsed = time.time() - phase_start
#                     current_pressure = min(target_pressure, elapsed * ramp_up_rate)
#                     client_instance.pd_array_1[arduino_8_idx] = current_pressure
#                     _send_pressure_commands(client_instance)
#                     time.sleep(0.01)

#                 # Hold Arduino 8 at target pressure for robot stabilization
#                 stabilization_start = time.time()
#                 while (
#                     time.time() - stabilization_start
#                 ) < stabilization_time and time.time() < experiment_end_time:
#                     if not client_instance.th1_flag or not client_instance.th2_flag:
#                         return
#                     client_instance.pd_array_1[arduino_8_idx] = arduino_8_target
#                     _send_pressure_commands(client_instance)
#                     time.sleep(0.01)

#             # Hold all Arduinos at target pressures for combined stabilization
#             if time.time() < experiment_end_time:
#                 combined_hold_start = time.time()
#                 while (
#                     time.time() - combined_hold_start
#                 ) < hold_time and time.time() < experiment_end_time:
#                     if not client_instance.th1_flag or not client_instance.th2_flag:
#                         return
#                     # Maintain all Arduino pressures during combined hold
#                     if arduino_4_idx is not None:
#                         client_instance.pd_array_1[arduino_4_idx] = arduino_4_pressure
#                     if arduino_7_idx is not None:
#                         client_instance.pd_array_1[arduino_7_idx] = (
#                             target_pressures.get(7, 0.0)
#                         )
#                     if arduino_8_idx is not None:
#                         client_instance.pd_array_1[arduino_8_idx] = (
#                             target_pressures.get(8, 0.0)
#                         )
#                     _send_pressure_commands(client_instance)
#                     time.sleep(0.01)

#             # Phase 3: Ramp down Arduino 7 to 0 psi
#             arduino_7_idx = arduino_indices.get(7)
#             if arduino_7_idx is not None and time.time() < experiment_end_time:
#                 if cycle_count == 1:  # Only log on first cycle
#                     logger.info("Ramping down Arduino 7 to 0 psi")
#                 start_pressure = client_instance.pd_array_1[arduino_7_idx]
#                 if start_pressure > 0:
#                     ramp_time = start_pressure / ramp_down_rate
#                     phase_start = time.time()

#                     while (time.time() - phase_start) < ramp_time:
#                         if not client_instance.th1_flag or not client_instance.th2_flag:
#                             return
#                         if time.time() >= experiment_end_time:
#                             break

#                         elapsed = time.time() - phase_start
#                         current_pressure = max(
#                             0.0, start_pressure - elapsed * ramp_down_rate
#                         )
#                         client_instance.pd_array_1[arduino_7_idx] = current_pressure
#                         _send_pressure_commands(client_instance)
#                         time.sleep(0.01)

#                 # Ensure it's exactly 0
#                 client_instance.pd_array_1[arduino_7_idx] = 0.0

#                 # Hold Arduino 7 at 0 psi for robot return stabilization
#                 return_stabilization_start = time.time()
#                 while (
#                     time.time() - return_stabilization_start
#                 ) < stabilization_time and time.time() < experiment_end_time:
#                     if not client_instance.th1_flag or not client_instance.th2_flag:
#                         return
#                     client_instance.pd_array_1[arduino_7_idx] = 0.0
#                     _send_pressure_commands(client_instance)
#                     time.sleep(0.01)

#             # Phase 4: Ramp down Arduino 8 to 0 psi
#             arduino_8_idx = arduino_indices.get(8)
#             if arduino_8_idx is not None and time.time() < experiment_end_time:
#                 if cycle_count == 1:  # Only log on first cycle
#                     logger.info("Ramping down Arduino 8 to 0 psi")
#                 start_pressure = client_instance.pd_array_1[arduino_8_idx]
#                 if start_pressure > 0:
#                     ramp_time = start_pressure / ramp_down_rate
#                     phase_start = time.time()

#                     while (time.time() - phase_start) < ramp_time:
#                         if not client_instance.th1_flag or not client_instance.th2_flag:
#                             return
#                         if time.time() >= experiment_end_time:
#                             break

#                         elapsed = time.time() - phase_start
#                         current_pressure = max(
#                             0.0, start_pressure - elapsed * ramp_down_rate
#                         )
#                         client_instance.pd_array_1[arduino_8_idx] = current_pressure
#                         _send_pressure_commands(client_instance)
#                         time.sleep(0.01)

#                 # Ensure it's exactly 0
#                 client_instance.pd_array_1[arduino_8_idx] = 0.0

#                 # Hold Arduino 8 at 0 psi for robot return stabilization
#                 return_stabilization_start = time.time()
#                 while (
#                     time.time() - return_stabilization_start
#                 ) < stabilization_time and time.time() < experiment_end_time:
#                     if not client_instance.th1_flag or not client_instance.th2_flag:
#                         return
#                     client_instance.pd_array_1[arduino_8_idx] = 0.0
#                     _send_pressure_commands(client_instance)
#                     time.sleep(0.01)

#             # Check if we should continue the loop
#             if time.time() >= experiment_end_time:
#                 break

#             # Brief pause between cycles for complete system stabilization
#             if (
#                 cycle_count > 1
#             ):  # No delay after first cycle since we already had initial delay
#                 inter_cycle_start = time.time()
#                 while (
#                     time.time() - inter_cycle_start
#                 ) < hold_time and time.time() < experiment_end_time:
#                     if not client_instance.th1_flag or not client_instance.th2_flag:
#                         return
#                     # Maintain Arduino 4 at target pressure, keep Arduino 7 and 8 at 0
#                     for i in range(len(client_instance.NArs)):
#                         arduino_id = client_instance.NArs[i]
#                         if arduino_id == 4:
#                             client_instance.pd_array_1[i] = arduino_4_pressure
#                         elif arduino_id in [7, 8]:
#                             client_instance.pd_array_1[i] = 0.0
#                     _send_pressure_commands(client_instance)
#                     time.sleep(0.01)

#         # Final cleanup - ensure Arduino 7 and 8 are at 0, maintain Arduino 4
#         logger.info(
#             "Setting Arduino 7 and 8 to 0 psi at experiment end, maintaining Arduino 4"
#         )
#         for i in range(len(client_instance.NArs)):
#             arduino_id = client_instance.NArs[i]
#             if arduino_id == 4:
#                 client_instance.pd_array_1[i] = arduino_4_pressure  # Maintain Arduino 4
#             elif arduino_id in [7, 8]:
#                 client_instance.pd_array_1[i] = 0.0
#         _send_pressure_commands(client_instance)

#         # Calculate and log execution timing
#         wave_end_time = time.time()
#         total_wave_duration = wave_end_time - wave_start_time
#         logger.info(
#             f"Pattern completed: {cycle_count} cycles in {total_wave_duration:.1f}s"
#         )

#     except KeyboardInterrupt:
#         logger.info("Sequential custom pressure pattern interrupted by user")
#         client_instance.th1_flag = False
#         client_instance.th2_flag = False

import logging
import time

# Assume logger and _send_pressure_commands are defined elsewhere
# logger = logging.getLogger(__name__)
# def _send_pressure_commands(client_instance):
#     pass
# def _calculate_wave_cycle_duration(*args, **kwargs):
#     return 60.0 # Placeholder


def pressure_sequential_custom_response(
    client_instance,
    initial_delay=2.0,
    ramp_up_rate=5.0,  # psi per second
    ramp_down_rate=5.0,  # psi per second
    hold_time=1.0,  # seconds to hold at target pressure for robot stabilization
    stabilization_time=1.0,  # Additional time for robot to reach position
    duration=60.0,  # Total duration to run the wave pattern (seconds)
):
    """
    Execute a custom sequential pressure pattern using pressures from client_instance.pressure_array:
    Arduino 4 maintains a constant pressure supply throughout the experiment.
    The pattern will loop continuously until the specified duration elapses.

    NOTE: This version skips all actions for Arduino 7.

    Wave cycle:
    1. Set Arduino 4 to a constant pressure (maintained throughout).
    2. Start ramping up Arduino 8 to its target pressure and wait for the robot to reach position.
    3. Hold pressures for a combined stabilization period.
    4. Ramp down Arduino 8 to 0 psi and wait for the robot to return.
    5. Repeat cycle while keeping Arduino 4 at its constant pressure until the duration has elapsed.

    The target pressures are automatically taken from client_instance.pressure_array
    which corresponds to the Arduinos in client_instance.NArs order.

    Args:
        client_instance: The client instance with communication methods and pressure_array.
        initial_delay: Time to wait before starting (seconds).
        ramp_up_rate: Rate of pressure increase (psi/second).
        ramp_down_rate: Rate of pressure decrease (psi/second).
        hold_time: Time to hold pressure for robot stabilization (seconds).
        stabilization_time: Additional time for robot to reach target position (seconds).
        duration: Total time to run the wave pattern with loops (seconds).
    """
    if client_instance.t0_on_trial is None:
        client_instance.t0_on_trial = time.time()

    logger.info("Starting sequential custom pressure pattern with looping")
    logger.info(f"Wave execution start time: {client_instance.t0_on_trial:.6f}")
    logger.info(f"Total duration: {duration} seconds")

    # Get target pressures from vtech client's pressure_array
    target_pressures = {}
    for i, arduino_id in enumerate(client_instance.NArs):
        if i < len(client_instance.pressure_array):
            target_pressures[arduino_id] = client_instance.pressure_array[i]
        else:
            target_pressures[arduino_id] = 0.0

    logger.info(f"Target pressures from vtech pressure_array: {target_pressures}")
    logger.info(
        f"Arduino 4 target: {target_pressures.get(4, 'Not found')} psi (constant)"
    )
    # logger.info(f"Arduino 7 target: {target_pressures.get(7, 'Not found')} psi") # SKIPPED
    logger.info(f"Arduino 8 target: {target_pressures.get(8, 'Not found')} psi")

    # Store start time for relative timing in wave execution
    wave_start_time = time.time()
    experiment_end_time = wave_start_time + duration

    # Initialize all pressures to 0, except Arduino 4 which gets constant 5 psi
    for i in range(len(client_instance.NArs)):
        client_instance.pd_array_1[i] = 0.0

    # Map Arduino IDs to indices in NArs array
    arduino_indices = {}
    for i, arduino_id in enumerate(client_instance.NArs):
        arduino_indices[arduino_id] = i

    # One-time ramp-up of Arduino 4 to target pressure (only at start of experiment)
    arduino_4_idx = arduino_indices.get(4)
    arduino_4_pressure = target_pressures.get(4)

    if (
        arduino_4_idx is not None
        and arduino_4_pressure is not None
        and arduino_4_pressure > 0
    ):
        logger.info(f"Initial ramp-up: Arduino 4 to {arduino_4_pressure} psi")
        target_pressure = arduino_4_pressure
        ramp_time = target_pressure / ramp_up_rate
        phase_start = time.time()

        # Ramp up Arduino 4 once at the beginning
        while (time.time() - phase_start) < ramp_time:
            if not client_instance.th1_flag or not client_instance.th2_flag:
                return
            if time.time() >= experiment_end_time:
                break

            elapsed = time.time() - phase_start
            current_pressure = min(target_pressure, elapsed * ramp_up_rate)
            client_instance.pd_array_1[arduino_4_idx] = current_pressure
            _send_pressure_commands(client_instance)
            time.sleep(0.01)

        # Hold Arduino 4 at target pressure for initial stabilization
        logger.info(
            f"Holding Arduino 4 at {arduino_4_pressure} psi for {stabilization_time}s"
        )
        stabilization_start = time.time()
        while (
            time.time() - stabilization_start
        ) < stabilization_time and time.time() < experiment_end_time:
            if not client_instance.th1_flag or not client_instance.th2_flag:
                return
            client_instance.pd_array_1[arduino_4_idx] = arduino_4_pressure
            _send_pressure_commands(client_instance)
            time.sleep(0.01)

        logger.info(
            f"Arduino 4 setup complete - will maintain {arduino_4_pressure} psi throughout experiment"
        )

    # Calculate single wave cycle duration for timing
    single_cycle_time = _calculate_wave_cycle_duration(
        target_pressures,
        ramp_up_rate,
        ramp_down_rate,
        hold_time,
        initial_delay,
        stabilization_time,
    )
    logger.info(
        f"Wave cycle duration: {single_cycle_time:.1f}s, positioning time: {stabilization_time}s"
    )

    cycle_count = 0

    try:
        # Main loop: repeat the wave pattern until duration elapsed
        while time.time() < experiment_end_time:
            cycle_count += 1
            cycle_start_time = time.time()
            remaining_time = experiment_end_time - cycle_start_time

            if (
                remaining_time < single_cycle_time * 0.3
            ):  # If less than 30% of cycle time remains
                logger.info(
                    f"Insufficient time for complete cycle {cycle_count}, stopping gracefully"
                )
                break

            logger.info(
                f"Starting wave cycle {cycle_count}, remaining time: {remaining_time:.1f}s"
            )

            # Reset Arduino 7 and 8 pressures to 0, maintain Arduino 4 at target
            for i in range(len(client_instance.NArs)):
                arduino_id = client_instance.NArs[i]
                if arduino_id == 4:
                    client_instance.pd_array_1[i] = (
                        arduino_4_pressure  # Maintain Arduino 4
                    )
                else:
                    client_instance.pd_array_1[i] = 0.0

            # Phase 0: Initial delay (only for first cycle)
            if cycle_count == 1:
                logger.info(f"Phase 0: Initial delay ({initial_delay}s)")
                phase_start = time.time()
                while (time.time() - phase_start) < initial_delay:
                    if not client_instance.th1_flag or not client_instance.th2_flag:
                        return
                    if time.time() >= experiment_end_time:
                        break
                    _send_pressure_commands(client_instance)
                    time.sleep(0.01)

            # """
            # # Phase 1: Start ramping up Arduino 7 to target pressure -- SKIPPED
            # arduino_7_target = target_pressures.get(7, 0.0)
            # if cycle_count == 1:  # Only log on first cycle
            #     logger.info(f"Ramping Arduino 7 to {arduino_7_target} psi")
            # arduino_7_idx = arduino_indices.get(7)
            # if (
            #     arduino_7_idx is not None
            #     and arduino_7_target > 0
            #     and time.time() < experiment_end_time
            # ):
            #     target_pressure = arduino_7_target
            #     ramp_time = target_pressure / ramp_up_rate
            #     phase_start = time.time()

            #     while (time.time() - phase_start) < ramp_time:
            #         if not client_instance.th1_flag or not client_instance.th2_flag:
            #             return
            #         if time.time() >= experiment_end_time:
            #             break

            #         elapsed = time.time() - phase_start
            #         current_pressure = min(target_pressure, elapsed * ramp_up_rate)
            #         client_instance.pd_array_1[arduino_7_idx] = current_pressure
            #         _send_pressure_commands(client_instance)
            #         time.sleep(0.01)

            #     # Hold Arduino 7 at target pressure for robot stabilization
            #     stabilization_start = time.time()
            #     while (
            #         time.time() - stabilization_start
            #     ) < stabilization_time and time.time() < experiment_end_time:
            #         if not client_instance.th1_flag or not client_instance.th2_flag:
            #             return
            #         client_instance.pd_array_1[arduino_7_idx] = arduino_7_target
            #         _send_pressure_commands(client_instance)
            #         time.sleep(0.01)
            # """

            # Phase 2: Start ramping up Arduino 8 to target pressure
            arduino_8_target = target_pressures.get(8, 0.0)
            if cycle_count == 1:  # Only log on first cycle
                logger.info(f"Ramping Arduino 8 to {arduino_8_target} psi")
            arduino_8_idx = arduino_indices.get(8)
            if (
                arduino_8_idx is not None
                and arduino_8_target > 0
                and time.time() < experiment_end_time
            ):
                target_pressure = arduino_8_target
                ramp_time = target_pressure / ramp_up_rate
                phase_start = time.time()

                while (time.time() - phase_start) < ramp_time:
                    if not client_instance.th1_flag or not client_instance.th2_flag:
                        return
                    if time.time() >= experiment_end_time:
                        break

                    elapsed = time.time() - phase_start
                    current_pressure = min(target_pressure, elapsed * ramp_up_rate)
                    client_instance.pd_array_1[arduino_8_idx] = current_pressure
                    _send_pressure_commands(client_instance)
                    time.sleep(0.01)

                # Hold Arduino 8 at target pressure for robot stabilization
                stabilization_start = time.time()
                while (
                    time.time() - stabilization_start
                ) < stabilization_time and time.time() < experiment_end_time:
                    if not client_instance.th1_flag or not client_instance.th2_flag:
                        return
                    client_instance.pd_array_1[arduino_8_idx] = arduino_8_target
                    _send_pressure_commands(client_instance)
                    time.sleep(0.01)

            # Hold all Arduinos at target pressures for combined stabilization
            if time.time() < experiment_end_time:
                combined_hold_start = time.time()
                while (
                    time.time() - combined_hold_start
                ) < hold_time and time.time() < experiment_end_time:
                    if not client_instance.th1_flag or not client_instance.th2_flag:
                        return
                    # Maintain all Arduino pressures during combined hold
                    if arduino_4_idx is not None:
                        client_instance.pd_array_1[arduino_4_idx] = arduino_4_pressure
                    # if arduino_7_idx is not None:
                    #     client_instance.pd_array_1[arduino_7_idx] = (
                    #         target_pressures.get(7, 0.0)
                    #     )
                    if arduino_8_idx is not None:
                        client_instance.pd_array_1[arduino_8_idx] = (
                            target_pressures.get(8, 0.0)
                        )
                    _send_pressure_commands(client_instance)
                    time.sleep(0.01)

            # """
            # # Phase 3: Ramp down Arduino 7 to 0 psi -- SKIPPED
            # arduino_7_idx = arduino_indices.get(7)
            # if arduino_7_idx is not None and time.time() < experiment_end_time:
            #     if cycle_count == 1:  # Only log on first cycle
            #         logger.info("Ramping down Arduino 7 to 0 psi")
            #     start_pressure = client_instance.pd_array_1[arduino_7_idx]
            #     if start_pressure > 0:
            #         ramp_time = start_pressure / ramp_down_rate
            #         phase_start = time.time()

            #         while (time.time() - phase_start) < ramp_time:
            #             if not client_instance.th1_flag or not client_instance.th2_flag:
            #                 return
            #             if time.time() >= experiment_end_time:
            #                 break

            #             elapsed = time.time() - phase_start
            #             current_pressure = max(
            #                 0.0, start_pressure - elapsed * ramp_down_rate
            #             )
            #             client_instance.pd_array_1[arduino_7_idx] = current_pressure
            #             _send_pressure_commands(client_instance)
            #             time.sleep(0.01)

            #     # Ensure it's exactly 0
            #     client_instance.pd_array_1[arduino_7_idx] = 0.0

            #     # Hold Arduino 7 at 0 psi for robot return stabilization
            #     return_stabilization_start = time.time()
            #     while (
            #         time.time() - return_stabilization_start
            #     ) < stabilization_time and time.time() < experiment_end_time:
            #         if not client_instance.th1_flag or not client_instance.th2_flag:
            #             return
            #         client_instance.pd_array_1[arduino_7_idx] = 0.0
            #         _send_pressure_commands(client_instance)
            #         time.sleep(0.01)
            # """

            # Phase 4: Ramp down Arduino 8 to 0 psi
            arduino_8_idx = arduino_indices.get(8)
            if arduino_8_idx is not None and time.time() < experiment_end_time:
                if cycle_count == 1:  # Only log on first cycle
                    logger.info("Ramping down Arduino 8 to 0 psi")
                start_pressure = client_instance.pd_array_1[arduino_8_idx]
                if start_pressure > 0:
                    ramp_time = start_pressure / ramp_down_rate
                    phase_start = time.time()

                    while (time.time() - phase_start) < ramp_time:
                        if not client_instance.th1_flag or not client_instance.th2_flag:
                            return
                        if time.time() >= experiment_end_time:
                            break

                        elapsed = time.time() - phase_start
                        current_pressure = max(
                            0.0, start_pressure - elapsed * ramp_down_rate
                        )
                        client_instance.pd_array_1[arduino_8_idx] = current_pressure
                        _send_pressure_commands(client_instance)
                        time.sleep(0.01)

                # Ensure it's exactly 0
                client_instance.pd_array_1[arduino_8_idx] = 0.0

                # Hold Arduino 8 at 0 psi for robot return stabilization
                return_stabilization_start = time.time()
                while (
                    time.time() - return_stabilization_start
                ) < stabilization_time and time.time() < experiment_end_time:
                    if not client_instance.th1_flag or not client_instance.th2_flag:
                        return
                    client_instance.pd_array_1[arduino_8_idx] = 0.0
                    _send_pressure_commands(client_instance)
                    time.sleep(0.01)

            # Check if we should continue the loop
            if time.time() >= experiment_end_time:
                break

            # Brief pause between cycles for complete system stabilization
            if (
                cycle_count > 1
            ):  # No delay after first cycle since we already had initial delay
                inter_cycle_start = time.time()
                while (
                    time.time() - inter_cycle_start
                ) < hold_time and time.time() < experiment_end_time:
                    if not client_instance.th1_flag or not client_instance.th2_flag:
                        return
                    # Maintain Arduino 4 at target pressure, keep Arduino 7 and 8 at 0
                    for i in range(len(client_instance.NArs)):
                        arduino_id = client_instance.NArs[i]
                        if arduino_id == 4:
                            client_instance.pd_array_1[i] = arduino_4_pressure
                        elif arduino_id in [7, 8]:
                            client_instance.pd_array_1[i] = 0.0
                    _send_pressure_commands(client_instance)
                    time.sleep(0.01)

        # Final cleanup - ensure Arduino 7 and 8 are at 0, maintain Arduino 4
        logger.info(
            "Setting Arduino 7 and 8 to 0 psi at experiment end, maintaining Arduino 4"
        )
        for i in range(len(client_instance.NArs)):
            arduino_id = client_instance.NArs[i]
            if arduino_id == 4:
                client_instance.pd_array_1[i] = arduino_4_pressure  # Maintain Arduino 4
            elif arduino_id in [7, 8]:
                client_instance.pd_array_1[i] = 0.0
        _send_pressure_commands(client_instance)

        # Calculate and log execution timing
        wave_end_time = time.time()
        total_wave_duration = wave_end_time - wave_start_time
        logger.info(
            f"Pattern completed: {cycle_count} cycles in {total_wave_duration:.1f}s"
        )

    except KeyboardInterrupt:
        logger.info("Sequential custom pressure pattern interrupted by user")
        client_instance.th1_flag = False
        client_instance.th2_flag = False


def circular(client_instance, freq_hz=0.1, center=6.0, amp=3.0, duration=60.0):
    """
    Drive 4 channels (client_instance.NArs order) with 90°-shifted sinusoids:
      p_k(t) = center + amp * sin(2π f t + φ_k), φ_k in {0, π/2, π, 3π/2}
    Produces a circular trajectory in the robot’s bending plane.
    """
    start = time.time()
    end = start + duration

    # Updated phases for 4 channels (0°, 90°, 180°, 270°)
    phases = [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]

    while client_instance.th1_flag and client_instance.th2_flag and time.time() < end:
        t = time.time() - start
        for i in range(len(client_instance.NArs)):
            # Use i % 4 since there are now 4 phases
            phi = phases[i % 4]
            p = center + amp * math.sin(2.0 * math.pi * freq_hz * t + phi)

            # clamp to valid range
            p = max(0.0, min(100.0, p))
            client_instance.pd_array_1[i] = p

            if i < len(client_instance.comm_manager.client_sockets):
                client_instance.pm_array_1[i] = client_instance.ard_socket(
                    client_instance.pd_array_1[i],
                    client_instance.comm_manager.client_sockets[i],
                )
        time.sleep(0.01)  # ~100 Hz loop


def circular_with_static_pressure(
    client_instance, freq_hz=0.1, center=6.0, amp=3.0, duration=60.0
):
    """
    Drives 3 channels with 120°-shifted sinusoids for circular motion,
    while holding a 4th channel (Arduino 3) at a constant pressure.

    - Arduinos 4, 7, and 8 will follow: p_k(t) = center + amp * sin(2π f t + φ_k)
    - Arduino 3 will be held constant at 2.0 psi.
    """
    start = time.time()
    end = start + duration

    # Phases for a 3-actuator circular motion (0°, 120°, 240°)
    phases = [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]

    # --- Pre-loop setup for Arduino 3 ---
    try:
        # Find the index corresponding to Arduino 3
        arduino_3_idx = client_instance.NArs.index(3)

        # Set the desired constant pressure
        constant_pressure = 2.0
        client_instance.pd_array_1[arduino_3_idx] = constant_pressure

        # Send the command to the physical Arduino once before the loop
        if arduino_3_idx < len(client_instance.comm_manager.client_sockets):
            print(
                f"Setting Arduino 3 (index {arduino_3_idx}) to a constant {constant_pressure} psi."
            )
            client_instance.pm_array_1[arduino_3_idx] = client_instance.ard_socket(
                constant_pressure,
                client_instance.comm_manager.client_sockets[arduino_3_idx],
            )
    except ValueError:
        print(
            "Warning: Arduino with ID 3 not found in client_instance.NArs. Cannot set static pressure."
        )
    # --- End of pre-loop setup ---

    while client_instance.th1_flag and client_instance.th2_flag and time.time() < end:
        t = time.time() - start

        # A separate counter for assigning phases to the moving actuators
        phase_counter = 0

        for i in range(len(client_instance.NArs)):
            # Check if the current Arduino is the one we want to keep static
            if client_instance.NArs[i] == 3:
                continue  # Skip this iteration, leaving its pressure untouched

            # Calculate sinusoidal pressure for the other actuators
            phi = phases[phase_counter % 3]
            p = center + amp * math.sin(2.0 * math.pi * freq_hz * t + phi)

            # Clamp to a valid pressure range
            p = max(0.0, min(100.0, p))
            client_instance.pd_array_1[i] = p

            # Send the updated pressure command
            if i < len(client_instance.comm_manager.client_sockets):
                client_instance.pm_array_1[i] = client_instance.ard_socket(
                    client_instance.pd_array_1[i],
                    client_instance.comm_manager.client_sockets[i],
                )

            # Increment the phase counter only for the actuators that are moving
            phase_counter += 1

        time.sleep(0.01)  # ~100 Hz loop


def elliptical_motion(
    client_instance, freq_hz=0.1, center=6.0, amp_x=4.0, amp_y=2.0, duration=60.0
):
    """
    Drives 4 channels with 90°-shifted sinusoids and different amplitudes
    to produce an elliptical (oval) trajectory.

    This function assumes the actuators are paired for two perpendicular axes.
    - The 1st and 3rd Arduinos in the list control one axis (amplitude `amp_x`).
    - The 2nd and 4th Arduinos in the list control the other axis (amplitude `amp_y`).

    Equation: p_k(t) = center + amp * sin(2π * f * t + φ_k)

    Args:
        client_instance: The client instance with communication methods.
        freq_hz: Frequency of the oval motion in Hz.
        center: The central pressure point around which the wave oscillates (psi).
        amp_x: Pressure amplitude for the first axis (1st and 3rd Arduinos).
        amp_y: Pressure amplitude for the second axis (2nd and 4th Arduinos).
        duration: Total duration of the motion in seconds.
    """
    start = time.time()
    end = start + duration

    # Phases for 4-actuator motion (0°, 90°, 180°, 270°)
    phases = [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]

    while client_instance.th1_flag and client_instance.th2_flag and time.time() < end:
        t = time.time() - start
        for i in range(len(client_instance.NArs)):
            # Assign the correct amplitude based on the actuator's axis
            # This assigns amp_x to i=0,2 and amp_y to i=1,3
            amp = amp_x if i % 2 == 0 else amp_y

            # Get the correct phase for this actuator
            phi = phases[i % 4]
            p = center + amp * math.sin(2.0 * math.pi * freq_hz * t + phi)

            # Clamp pressure to a valid hardware range
            p = max(0.0, min(100.0, p))
            client_instance.pd_array_1[i] = p

            # Send the pressure command to the Arduino
            if i < len(client_instance.comm_manager.client_sockets):
                client_instance.pm_array_1[i] = client_instance.ard_socket(
                    client_instance.pd_array_1[i],
                    client_instance.comm_manager.client_sockets[i],
                )
        time.sleep(0.01)  # Loop at ~100 Hz


def elliptical_with_one_static(
    client_instance, freq_hz=0.1, center=6.0, amp_x=4.0, amp_y=2.0, duration=60.0
):
    """
    Drives 3 channels to produce an elliptical (oval) trajectory, while
    holding a 4th channel (Arduino 4) at a constant pressure.

    - The first two dynamic Arduinos run 180° out of phase on the X-axis (amp_x).
    - The third dynamic Arduino runs 90° out of phase on the Y-axis (amp_y).
    - Arduino 4 is held constant at 2.0 psi throughout.
    """
    start = time.time()
    end = start + duration

    # Phases for a 3-actuator elliptical motion (0°, 180°, 90°)
    phases = [0.0, math.pi, math.pi / 2.0]

    # --- Pre-loop setup for static Arduino 4 ---
    try:
        # Find the index corresponding to Arduino 4
        static_arduino_id = 4
        static_idx = client_instance.NArs.index(static_arduino_id)
        constant_pressure = 2.0

        # Set the constant pressure in the array
        client_instance.pd_array_1[static_idx] = constant_pressure

        # Send the command to the physical Arduino once before the loop
        if static_idx < len(client_instance.comm_manager.client_sockets):
            print(
                f"Setting Arduino {static_arduino_id} (index {static_idx}) to a constant {constant_pressure} psi."
            )
            client_instance.pm_array_1[static_idx] = client_instance.ard_socket(
                constant_pressure,
                client_instance.comm_manager.client_sockets[static_idx],
            )
    except ValueError:
        print(
            f"Warning: Arduino with ID {static_arduino_id} not found. Cannot set static pressure."
        )
    # --- End of pre-loop setup ---

    while client_instance.th1_flag and client_instance.th2_flag and time.time() < end:
        t = time.time() - start

        # A counter for assigning phases to the moving actuators
        phase_counter = 0

        for i in range(len(client_instance.NArs)):
            # Check if the current Arduino is the one we want to keep static
            if client_instance.NArs[i] == static_arduino_id:
                continue  # Skip this iteration, leaving its pressure untouched

            # Assign amplitude: first two moving Arduinos get amp_x, the third gets amp_y
            amp = amp_x if phase_counter < 2 else amp_y

            # Get the correct phase for this actuator
            phi = phases[phase_counter]
            p = center + amp * math.sin(2.0 * math.pi * freq_hz * t + phi)

            # Clamp pressure to a valid hardware range
            p = max(0.0, min(100.0, p))
            client_instance.pd_array_1[i] = p

            # Send the updated pressure command
            if i < len(client_instance.comm_manager.client_sockets):
                client_instance.pm_array_1[i] = client_instance.ard_socket(
                    client_instance.pd_array_1[i],
                    client_instance.comm_manager.client_sockets[i],
                )

            # Increment the phase counter only for the actuators that are moving
            phase_counter += 1

        time.sleep(0.01)  # Loop at ~100 Hz


def pressure_axial_wave(
    client_instance, period=75.0, p_base=6.0, gain=3.0, duration=150.0
):
    import time

    start = time.time()
    end = start + duration

    while client_instance.th1_flag and client_instance.th2_flag and time.time() < end:
        t = time.time() - start
        x, y = axial_target(t, period)
        p_vec = map_xy_to_pressures(x, y, p_base, gain)

        for i in range(len(client_instance.NArs)):
            p = p_vec[i % 3]
            client_instance.pd_array_1[i] = p
            if i < len(client_instance.comm_manager.client_sockets):
                client_instance.pm_array_1[i] = client_instance.ard_socket(
                    p, client_instance.comm_manager.client_sockets[i]
                )
        time.sleep(0.01)
