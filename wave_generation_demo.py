"""
Example usage of the Wave Generation Module

This script demonstrates how to use the new wave generation system
for soft robot pressure control.
"""

import time

import numpy as np

from wave_generation import (
    MultiChannelSignalGenerator,
    SignalParameters,
    SignalType,
    WaveformLibrary,
    create_ramp_response,
    create_sine_wave,
    create_step_response,
)


def demo_basic_signals():
    """Demonstrate basic signal generation"""
    print("=== Basic Signal Generation Demo ===")

    # Create individual signal generators
    step_gen = create_step_response(step_value=5.0, step_time=2.0, duration=10.0)
    ramp_gen = create_ramp_response(
        start=0.0, end=10.0, up_rate=2.0, down_rate=1.0, duration=15.0
    )
    sine_gen = create_sine_wave(amplitude=3.0, frequency=0.5, offset=2.0, duration=10.0)

    # Start generators
    step_gen.start()
    ramp_gen.start()
    sine_gen.start()

    # Sample values over time
    print("Time\tStep\tRamp\tSine")
    print("----\t----\t----\t----")

    for i in range(50):
        step_val = step_gen.get_current_value()
        ramp_val = ramp_gen.get_current_value()
        sine_val = sine_gen.get_current_value()

        print(f"{i * 0.2:.1f}\t{step_val:.2f}\t{ramp_val:.2f}\t{sine_val:.2f}")
        time.sleep(0.2)

        # Check if generators are still running
        if not (step_gen.is_running or ramp_gen.is_running or sine_gen.is_running):
            break


def demo_multi_channel():
    """Demonstrate multi-channel signal generation"""
    print("\n=== Multi-Channel Signal Generation Demo ===")

    # Create a 3-channel system
    multi_gen = MultiChannelSignalGenerator(num_channels=3)

    # Set different signals for each channel
    multi_gen.set_channel_generator(0, create_step_response(5.0, 1.0, 8.0))
    multi_gen.set_channel_generator(1, create_sine_wave(2.0, 1.0, 3.0, duration=8.0))
    multi_gen.set_channel_generator(
        2, create_ramp_response(0.0, 7.0, 3.0, 2.0, duration=8.0)
    )

    # Start all generators
    multi_gen.start_all()

    print("Time\tCh0\tCh1\tCh2")
    print("----\t---\t---\t---")

    for i in range(40):
        values = multi_gen.get_all_values()
        print(f"{i * 0.2:.1f}\t{values[0]:.2f}\t{values[1]:.2f}\t{values[2]:.2f}")
        time.sleep(0.2)

        if not multi_gen.is_running:
            break

    multi_gen.stop_all()


def demo_custom_waveforms():
    """Demonstrate custom waveform library functions"""
    print("\n=== Custom Waveforms Demo ===")

    # Test waveform library functions
    duration = 5.0
    dt = 0.1
    time_points = np.arange(0, duration, dt)

    print("Time\tChirp\tGaussian\tExponential\tSawtooth")
    print("----\t-----\t--------\t-----------\t--------")

    for t in time_points:
        chirp = WaveformLibrary.chirp(t, f0=0.5, f1=2.0, duration=duration)
        gaussian = WaveformLibrary.gaussian_pulse(t, center=2.5, width=0.5)
        exponential = WaveformLibrary.exponential_decay(t, decay_rate=0.5)
        sawtooth = WaveformLibrary.sawtooth(t, frequency=1.0)

        print(
            f"{t:.1f}\t{chirp:.3f}\t{gaussian:.3f}\t\t{exponential:.3f}\t\t{sawtooth:.3f}"
        )


def demo_pressure_control_simulation():
    """Simulate pressure control for soft robot actuators"""
    print("\n=== Pressure Control Simulation ===")

    # Simulate a 3-actuator soft robot
    num_actuators = 3
    controller = MultiChannelSignalGenerator(num_actuators)

    # Setup different control patterns for each actuator
    # Actuator 0: Step input for rapid inflation
    controller.set_channel_generator(
        0, create_step_response(step_value=8.0, step_time=1.0, duration=12.0)
    )

    # Actuator 1: Sinusoidal breathing pattern
    controller.set_channel_generator(
        1, create_sine_wave(amplitude=3.0, frequency=0.3, offset=4.0, duration=12.0)
    )

    # Actuator 2: Ramp for gradual bending
    controller.set_channel_generator(
        2,
        create_ramp_response(
            start=0.0, end=6.0, up_rate=1.0, down_rate=0.5, hold_time=3.0, duration=12.0
        ),
    )

    # Start the control sequence
    controller.start_all()

    print("Simulating soft robot pressure control...")
    print("Time\tActuator_1\tActuator_2\tActuator_3")
    print("----\t----------\t----------\t----------")

    simulation_time = 0
    while controller.is_running and simulation_time < 12.0:
        pressures = controller.get_all_values()
        print(
            f"{simulation_time:.1f}s\t{pressures[0]:.2f} psi\t{pressures[1]:.2f} psi\t{pressures[2]:.2f} psi"
        )

        # Simulate control loop delay
        time.sleep(0.2)
        simulation_time += 0.2

    controller.stop_all()
    print("Simulation complete!")


if __name__ == "__main__":
    print("Wave Generation Module Demo")
    print("===========================")

    try:
        demo_basic_signals()
        demo_multi_channel()
        demo_custom_waveforms()
        demo_pressure_control_simulation()

    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"Error during demo: {e}")

    print("\nDemo finished!")
