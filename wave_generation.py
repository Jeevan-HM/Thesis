"""
Wave Generation Module for Soft Robot Pressure Control

This module provides various signal generation patterns for pressure control,
including step responses, ramp functions, sinusoidal waves, and custom profiles.
"""

import logging
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Callable, List, Optional, Tuple

import numpy as np

# Setup logging
logger = logging.getLogger(__name__)


class SignalType(Enum):
    """Enumeration of available signal types"""

    STEP = "step"
    RAMP = "ramp"
    SINE = "sine"
    SQUARE = "square"
    TRIANGLE = "triangle"
    CUSTOM = "custom"


@dataclass
class SignalParameters:
    """Parameters for signal generation"""

    signal_type: SignalType
    amplitude: float = 1.0
    frequency: float = 1.0  # Hz
    offset: float = 0.0
    phase: float = 0.0  # radians
    duration: float = 10.0  # seconds

    # Step-specific parameters
    step_value: float = 5.0
    step_time: float = 1.0

    # Ramp-specific parameters
    ramp_start: float = 0.0
    ramp_end: float = 10.0
    ramp_up_rate: float = 1.0  # units per second
    ramp_down_rate: float = 1.0  # units per second
    ramp_hold_time: float = 2.0  # seconds at peak

    # Custom function
    custom_function: Optional[Callable[[float], float]] = None


class BaseSignalGenerator(ABC):
    """Abstract base class for signal generators"""

    def __init__(self, parameters: SignalParameters):
        self.parameters = parameters
        self.start_time = None
        self.is_running = False

    @abstractmethod
    def generate(self, t: float) -> float:
        """Generate signal value at time t"""
        pass

    def start(self):
        """Start the signal generation"""
        self.start_time = time.time()
        self.is_running = True
        logger.info(f"Started {self.__class__.__name__} signal generation")

    def stop(self):
        """Stop the signal generation"""
        self.is_running = False
        logger.info(f"Stopped {self.__class__.__name__} signal generation")

    def get_current_value(self) -> float:
        """Get the current signal value"""
        if not self.is_running or self.start_time is None:
            return 0.0

        elapsed_time = time.time() - self.start_time
        if elapsed_time > self.parameters.duration:
            self.stop()
            return self.parameters.offset

        return self.generate(elapsed_time)


class StepSignalGenerator(BaseSignalGenerator):
    """Generates step response signals"""

    def generate(self, t: float) -> float:
        """Generate step signal at time t"""
        if t >= self.parameters.step_time:
            return self.parameters.step_value + self.parameters.offset
        else:
            return self.parameters.offset


class RampSignalGenerator(BaseSignalGenerator):
    """Generates ramp (triangular) signals"""

    def generate(self, t: float) -> float:
        """Generate ramp signal at time t"""
        ramp_up_duration = (
            self.parameters.ramp_end - self.parameters.ramp_start
        ) / self.parameters.ramp_up_rate
        hold_start = ramp_up_duration
        hold_end = hold_start + self.parameters.ramp_hold_time
        ramp_down_duration = (
            self.parameters.ramp_end - self.parameters.ramp_start
        ) / self.parameters.ramp_down_rate

        if t <= ramp_up_duration:
            # Ramp up phase
            value = self.parameters.ramp_start + self.parameters.ramp_up_rate * t
        elif t <= hold_end:
            # Hold phase
            value = self.parameters.ramp_end
        elif t <= hold_end + ramp_down_duration:
            # Ramp down phase
            ramp_down_time = t - hold_end
            value = (
                self.parameters.ramp_end
                - self.parameters.ramp_down_rate * ramp_down_time
            )
        else:
            # Final value
            value = self.parameters.ramp_start

        return value + self.parameters.offset


class SineSignalGenerator(BaseSignalGenerator):
    """Generates sinusoidal signals"""

    def generate(self, t: float) -> float:
        """Generate sine wave at time t"""
        angular_freq = 2 * np.pi * self.parameters.frequency
        value = self.parameters.amplitude * np.sin(
            angular_freq * t + self.parameters.phase
        )
        return value + self.parameters.offset


class SquareSignalGenerator(BaseSignalGenerator):
    """Generates square wave signals"""

    def generate(self, t: float) -> float:
        """Generate square wave at time t"""
        angular_freq = 2 * np.pi * self.parameters.frequency
        sine_value = np.sin(angular_freq * t + self.parameters.phase)
        square_value = self.parameters.amplitude * np.sign(sine_value)
        return square_value + self.parameters.offset


class TriangleSignalGenerator(BaseSignalGenerator):
    """Generates triangle wave signals"""

    def generate(self, t: float) -> float:
        """Generate triangle wave at time t"""
        period = 1.0 / self.parameters.frequency
        t_normalized = (t + self.parameters.phase / (2 * np.pi) * period) % period

        if t_normalized < period / 2:
            # Rising edge
            value = self.parameters.amplitude * (4 * t_normalized / period - 1)
        else:
            # Falling edge
            value = self.parameters.amplitude * (3 - 4 * t_normalized / period)

        return value + self.parameters.offset


class CustomSignalGenerator(BaseSignalGenerator):
    """Generates custom signals using user-defined functions"""

    def generate(self, t: float) -> float:
        """Generate custom signal at time t"""
        if self.parameters.custom_function is None:
            logger.warning("No custom function defined, returning offset")
            return self.parameters.offset

        try:
            value = self.parameters.custom_function(t)
            return self.parameters.amplitude * value + self.parameters.offset
        except Exception as e:
            logger.error(f"Error in custom function: {e}")
            return self.parameters.offset


class MultiChannelSignalGenerator:
    """Manages multiple signal generators for different channels/actuators"""

    def __init__(self, num_channels: int):
        self.num_channels = num_channels
        self.generators: List[Optional[BaseSignalGenerator]] = [None] * num_channels
        self.is_running = False

    def set_channel_generator(self, channel: int, generator: BaseSignalGenerator):
        """Set signal generator for a specific channel"""
        if 0 <= channel < self.num_channels:
            self.generators[channel] = generator
            logger.info(
                f"Set generator for channel {channel}: {generator.__class__.__name__}"
            )
        else:
            raise ValueError(
                f"Channel {channel} out of range (0-{self.num_channels - 1})"
            )

    def start_all(self):
        """Start all signal generators"""
        self.is_running = True
        for i, generator in enumerate(self.generators):
            if generator is not None:
                generator.start()
        logger.info("Started all signal generators")

    def stop_all(self):
        """Stop all signal generators"""
        self.is_running = False
        for generator in self.generators:
            if generator is not None:
                generator.stop()
        logger.info("Stopped all signal generators")

    def get_all_values(self) -> List[float]:
        """Get current values from all channels"""
        values = []
        for i, generator in enumerate(self.generators):
            if generator is not None and generator.is_running:
                values.append(generator.get_current_value())
            else:
                values.append(0.0)
        return values

    def get_channel_value(self, channel: int) -> float:
        """Get current value from a specific channel"""
        if 0 <= channel < self.num_channels and self.generators[channel] is not None:
            return self.generators[channel].get_current_value()
        return 0.0


class WaveformLibrary:
    """Library of common waveform functions"""

    @staticmethod
    def chirp(
        t: float, f0: float = 1.0, f1: float = 10.0, duration: float = 10.0
    ) -> float:
        """Generate chirp signal (frequency sweep)"""
        if t > duration:
            return 0.0
        k = (f1 - f0) / duration
        instantaneous_freq = f0 + k * t
        return np.sin(2 * np.pi * (f0 * t + 0.5 * k * t**2))

    @staticmethod
    def gaussian_pulse(t: float, center: float = 5.0, width: float = 1.0) -> float:
        """Generate Gaussian pulse"""
        return np.exp(-(((t - center) / width) ** 2))

    @staticmethod
    def exponential_decay(t: float, decay_rate: float = 1.0) -> float:
        """Generate exponential decay"""
        return np.exp(-decay_rate * t)

    @staticmethod
    def sawtooth(t: float, frequency: float = 1.0) -> float:
        """Generate sawtooth wave"""
        period = 1.0 / frequency
        t_normalized = t % period
        return 2 * (t_normalized / period) - 1


def create_signal_generator(
    signal_type: SignalType, parameters: SignalParameters
) -> BaseSignalGenerator:
    """Factory function to create signal generators"""
    generators = {
        SignalType.STEP: StepSignalGenerator,
        SignalType.RAMP: RampSignalGenerator,
        SignalType.SINE: SineSignalGenerator,
        SignalType.SQUARE: SquareSignalGenerator,
        SignalType.TRIANGLE: TriangleSignalGenerator,
        SignalType.CUSTOM: CustomSignalGenerator,
    }

    if signal_type not in generators:
        raise ValueError(f"Unknown signal type: {signal_type}")

    return generators[signal_type](parameters)


# Convenience functions for common use cases
def create_step_response(
    step_value: float, step_time: float, duration: float = 10.0
) -> StepSignalGenerator:
    """Create a step response generator"""
    params = SignalParameters(
        signal_type=SignalType.STEP,
        step_value=step_value,
        step_time=step_time,
        duration=duration,
    )
    return StepSignalGenerator(params)


def create_ramp_response(
    start: float,
    end: float,
    up_rate: float,
    down_rate: float,
    hold_time: float = 2.0,
    duration: float = 20.0,
) -> RampSignalGenerator:
    """Create a ramp response generator"""
    params = SignalParameters(
        signal_type=SignalType.RAMP,
        ramp_start=start,
        ramp_end=end,
        ramp_up_rate=up_rate,
        ramp_down_rate=down_rate,
        ramp_hold_time=hold_time,
        duration=duration,
    )
    return RampSignalGenerator(params)


def create_sine_wave(
    amplitude: float,
    frequency: float,
    offset: float = 0.0,
    phase: float = 0.0,
    duration: float = 10.0,
) -> SineSignalGenerator:
    """Create a sine wave generator"""
    params = SignalParameters(
        signal_type=SignalType.SINE,
        amplitude=amplitude,
        frequency=frequency,
        offset=offset,
        phase=phase,
        duration=duration,
    )
    return SineSignalGenerator(params)
