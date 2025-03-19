import time
from math import pi, sin

from opensourceleg.time import SoftRealtimeLoop


class SimpleSensorSimulator:
    def __init__(self):
        self.start_time = time.monotonic()

    def read(self, t):
        # Simulate a 0.5 Hz sine wave sensor reading
        return 100 * sin(2 * pi * 0.5 * t)


def sensor_example():
    # Create a 20Hz loop (0.05 second period)
    rt_loop = SoftRealtimeLoop(dt=0.05)

    # Initialize sensor and data storage
    sensor = SimpleSensorSimulator()
    readings = []

    print("Recording simulated sensor data for 2 seconds...")

    for t in rt_loop:
        reading = sensor.read(t)
        readings.append(reading)
        print(f"Time: {t:.2f}s, Sensor value: {reading:.1f}")

        if t > 2.0:  # Stop after 2 seconds
            rt_loop.stop()

    print(f"\nCollected {len(readings)} samples at 20Hz")
    print(f"Average sampling period: {2.0 / len(readings):.3f} seconds")


def fade_example():
    # Create a 10Hz loop with 1-second fade-out
    rt_loop = SoftRealtimeLoop(dt=0.1, fade=1.0)

    print("Running loop with fade-out...")
    print("Press Ctrl+C to test graceful shutdown!")

    try:
        for t in rt_loop:
            fade_value = rt_loop.fade
            print(f"Time: {t:.1f}s, Fade value: {fade_value:.2f}")

            if t > 10.0:  # Auto-stop after 10 seconds if no Ctrl+C
                rt_loop.stop()

    except KeyboardInterrupt:
        print("\nCtrl+C detected - demonstrating fade-out...")
        # The loop will automatically handle the fade-out

    print("\nLoop completed with graceful shutdown!")


if __name__ == "__main__":
    # sensor_example()
    fade_example()
