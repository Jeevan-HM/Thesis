# Triangular Wave Pressure Control

## Overview
A new triangular wave function has been added to the pressure control system that generates smooth triangular wave patterns (0 to max psi and back to 0 psi).

## New Function: `pres_single_triangular_response()`

### Parameters:
- **frequency** (float): Wave frequency in Hz (cycles per second)
  - 0.1 Hz = one complete triangle every 10 seconds
  - 0.5 Hz = one complete triangle every 2 seconds
- **upper_bound** (float): Maximum pressure in psi
- **lower_bound** (float): Minimum pressure in psi (usually 0.0)
- **duration** (float): How long to run the wave in seconds (optional)

### Usage Examples:

#### 1. Slow triangular wave (10-second cycles)
```python
# 0 to 5 psi triangular wave, 10-second cycles, run for 60 seconds
controller.pres_single_triangular_response(0.1, 5.0, 0.0, 60.0)
```

#### 2. Fast triangular wave (2-second cycles)
```python
# 0 to 3 psi triangular wave, 2-second cycles, run for 20 seconds
controller.pres_single_triangular_response(0.5, 3.0, 0.0, 20.0)
```

#### 3. Using in the main control loop
Replace or add to the existing ramp cycles in `th_pd_gen()`:

```python
# Instead of:
self.pres_single_ramp_response(up_rate, down_rate, upper_bound, lower_bound)

# Use:
frequency = 0.1  # 0.1 Hz = 10 second cycles
self.pres_single_triangular_response(frequency, upper_bound, lower_bound, duration=20.0)
```

## Demo Script
Run the demo script to test different triangular wave patterns:

```bash
python3 triangular_wave_demo.py
```

The demo provides:
1. **Preset demos** - Three different wave patterns with different frequencies
2. **Custom setup** - Enter your own frequency, pressure range, and duration
3. **Interactive menu** - Easy to use interface

## Key Features:
- ✅ Smooth triangular wave generation
- ✅ Configurable frequency and amplitude
- ✅ Works with all connected Arduino actuators simultaneously
- ✅ Safe thread termination support
- ✅ Automatic pressure reset to 0 after completion
- ✅ Real-time pressure control with 100Hz update rate

## Wave Pattern:
```
Pressure (psi)
     ^
     |     /\      /\      /\
5.0  |    /  \    /  \    /  \
     |   /    \  /    \  /    \
     |  /      \/      \/      \
0.0  | /                       \
     |/                         \
     +----------------------------> Time
     0    5s   10s   15s   20s
     
     One complete cycle = 1/frequency seconds
```

## Integration with Existing Code:
The triangular wave function integrates seamlessly with the existing pressure control system and can be used alongside or instead of the existing ramp and step response functions.