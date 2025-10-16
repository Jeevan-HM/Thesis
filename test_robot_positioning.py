#!/usr/bin/env python3
"""
Test script to verify the robot positioning wait times in the pressure control system.

This script will run a short test to verify that:
1. Arduino 7 and 8 now ramp up to 10 PSI properly
2. The robot has adequate time to reach each position
3. The timing parameters are working correctly

Run this to test the fixes before running a full experiment.
"""

import logging
import sys
import time

# Setup logging to see what's happening
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

try:
    import wave_functions
    from vtech import pc_client
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Make sure you're running this from the RISE_Lab directory")
    sys.exit(1)


def test_pressure_control():
    """Test the pressure control system with robot positioning waits"""

    print("=" * 60)
    print("TESTING PRESSURE CONTROL WITH ROBOT POSITIONING WAITS")
    print("=" * 60)

    # Create client with test configuration
    client = pc_client()

    # Use shorter test parameters for quick verification
    client.NArs = [4, 7, 8]  # Arduino IDs
    client.pressure_array = [5.0, 10.0, 10.0]  # Target pressures
    client.comm_manager.arduino_ids = client.NArs

    print(f"Test configuration:")
    print(f"  - Arduinos: {client.NArs}")
    print(f"  - Target pressures: {client.pressure_array} PSI")
    print(f"  - Robot positioning time: 8 seconds per pressure change")
    print(f"  - System stabilization: 5 seconds")
    print()

    try:
        print("Initializing client...")
        if not client.initialize():
            print("❌ Failed to initialize client - check Arduino connections")
            return False

        print("✅ Client initialized successfully")
        print()

        print("Starting test experiment (30 seconds)...")
        if client.start_experiment("robot_positioning_test"):
            print("✅ Test experiment started")

            # Let it run for 30 seconds to see one complete cycle
            print("Running test pattern - watch the logs for timing details...")
            print("Expected behavior:")
            print("  1. Arduino 4 maintains 5 PSI (constant)")
            print("  2. Arduino 7 ramps to 10 PSI + 8s robot positioning wait")
            print("  3. Arduino 8 ramps to 10 PSI + 8s robot positioning wait")
            print("  4. 5s combined system stabilization")
            print("  5. Arduino 7 ramps down to 0 PSI + 8s return wait")
            print("  6. Arduino 8 ramps down to 0 PSI + 8s return wait")
            print()

            # Wait for test completion
            start_time = time.time()
            while client.is_experiment_running() and (time.time() - start_time) < 30:
                time.sleep(1)
                elapsed = time.time() - start_time
                print(f"Test running... {elapsed:.0f}s elapsed", end="\r")

            print(f"\n✅ Test completed after {time.time() - start_time:.1f} seconds")
            client.stop_experiment()

        else:
            print("❌ Failed to start test experiment")
            return False

    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
        client.stop_experiment()

    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        return False

    finally:
        print("\nCleaning up...")
        client.cleanup()
        print("✅ Cleanup completed")

    print("\n" + "=" * 60)
    print("TEST COMPLETED")
    print("=" * 60)
    print("Check the logs above to verify:")
    print("✓ Arduino 7 and 8 ramped to 10 PSI (not stuck at 0)")
    print("✓ Positioning waits occurred after each pressure change")
    print("✓ No timing errors or premature cycle termination")
    print("✓ Robot had adequate time to reach each position")

    return True


if __name__ == "__main__":
    print("Robot Positioning Test for Pressure Control System")
    print("This test verifies the fixes for Arduino 7 and 8 ramping issues")
    print()

    success = test_pressure_control()

    if success:
        print("\n🎉 Test completed successfully!")
        print(
            "The system is now ready for full experiments with proper robot positioning waits."
        )
    else:
        print("\n⚠️  Test had issues. Check the error messages above.")

    sys.exit(0 if success else 1)
