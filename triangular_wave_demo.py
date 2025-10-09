#!/usr/bin/env python3
"""
Demo script for testing the new triangular wave pressure response

This script demonstrates how to use the new pres_single_triangular_response function
to create smooth triangular wave patterns for pressure control.
"""

import logging
import time

from vtech import pc_client

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def demo_triangular_wave():
    """Demonstrate the triangular wave functionality"""

    print("=" * 60)
    print("    TRIANGULAR WAVE PRESSURE CONTROL DEMO")
    print("=" * 60)
    print()
    print("This demo will create triangular wave pressure patterns:")
    print("- Wave goes from 0 psi to 5 psi and back to 0 psi")
    print("- You can adjust frequency, amplitude, and duration")
    print()

    try:
        # Initialize the pressure controller
        print("Initializing pressure control system...")
        controller = pc_client()

        if not controller.ready:
            print("System not ready. Exiting...")
            return

        print("System initialized successfully!")
        print()

        # Demo 1: Slow triangular wave (10 second cycle)
        print("DEMO 1: Slow triangular wave (0.1 Hz - 10 second cycle)")
        print("Press Enter to start, or 'q' to quit...")
        user_input = input()
        if user_input.lower() == "q":
            return

        controller.test_triangular_wave(
            frequency=0.1,  # 0.1 Hz = 10 second cycle
            upper_bound=5.0,  # 5 psi maximum
            lower_bound=0.0,  # 0 psi minimum
            duration=30.0,  # Run for 30 seconds (3 complete cycles)
        )

        print("\nDemo 1 completed!\n")
        time.sleep(2)

        # Demo 2: Faster triangular wave (5 second cycle)
        print("DEMO 2: Faster triangular wave (0.2 Hz - 5 second cycle)")
        print("Press Enter to start, or 'q' to quit...")
        user_input = input()
        if user_input.lower() == "q":
            return

        controller.test_triangular_wave(
            frequency=0.2,  # 0.2 Hz = 5 second cycle
            upper_bound=3.0,  # 3 psi maximum
            lower_bound=0.0,  # 0 psi minimum
            duration=20.0,  # Run for 20 seconds (4 complete cycles)
        )

        print("\nDemo 2 completed!\n")
        time.sleep(2)

        # Demo 3: High frequency triangular wave (2 second cycle)
        print("DEMO 3: High frequency triangular wave (0.5 Hz - 2 second cycle)")
        print("Press Enter to start, or 'q' to quit...")
        user_input = input()
        if user_input.lower() == "q":
            return

        controller.test_triangular_wave(
            frequency=0.5,  # 0.5 Hz = 2 second cycle
            upper_bound=2.0,  # 2 psi maximum
            lower_bound=0.0,  # 0 psi minimum
            duration=10.0,  # Run for 10 seconds (5 complete cycles)
        )

        print("\nAll demos completed!")

    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        logger.error(f"Error during demo: {e}")
    finally:
        # Cleanup
        try:
            controller.cleanup()
        except (NameError, AttributeError):
            pass
        print("Demo finished. System cleaned up.")


def custom_triangular_wave():
    """Allow user to specify custom triangular wave parameters"""

    print("=" * 60)
    print("    CUSTOM TRIANGULAR WAVE SETUP")
    print("=" * 60)

    try:
        # Get parameters from user
        frequency = float(input("Enter frequency (Hz, e.g., 0.1 for 10-sec cycle): "))
        upper_bound = float(input("Enter maximum pressure (psi, e.g., 5.0): "))
        lower_bound = float(input("Enter minimum pressure (psi, e.g., 0.0): "))
        duration = float(input("Enter duration (seconds, e.g., 30.0): "))

        print("\nSettings:")
        print(f"  Frequency: {frequency} Hz ({1 / frequency:.1f} second cycle)")
        print(f"  Pressure range: {lower_bound} to {upper_bound} psi")
        print(f"  Duration: {duration} seconds")
        print(f"  Number of cycles: {duration * frequency:.1f}")

        confirm = input("\nProceed with these settings? (y/n): ")
        if confirm.lower() != "y":
            print("Cancelled.")
            return

        # Initialize controller and run custom wave
        controller = pc_client()
        if not controller.ready:
            print("System not ready. Exiting...")
            return

        controller.test_triangular_wave(frequency, upper_bound, lower_bound, duration)

    except KeyboardInterrupt:
        print("\nCustom demo interrupted by user")
    except ValueError:
        print("Invalid input. Please enter numeric values.")
    except Exception as e:
        logger.error(f"Error during custom demo: {e}")
    finally:
        try:
            controller.cleanup()
        except (NameError, AttributeError):
            pass


def main():
    """Main menu for triangular wave demos"""

    while True:
        print("\n" + "=" * 60)
        print("    TRIANGULAR WAVE DEMO MENU")
        print("=" * 60)
        print("1. Run preset demos (recommended)")
        print("2. Custom triangular wave")
        print("3. Exit")
        print()

        choice = input("Select option (1-3): ").strip()

        if choice == "1":
            demo_triangular_wave()
        elif choice == "2":
            custom_triangular_wave()
        elif choice == "3":
            print("Goodbye!")
            break
        else:
            print("Invalid choice. Please enter 1, 2, or 3.")


if __name__ == "__main__":
    main()
