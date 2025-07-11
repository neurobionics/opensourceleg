import numpy as np

from opensourceleg.generators import DataReplayGenerator

# Optional matplotlib import for plotting
try:
    import matplotlib.pyplot as plt

    PLOTTING_AVAILABLE = True
    print("Matplotlib available - plotting enabled")
except ImportError:
    PLOTTING_AVAILABLE = False
    print("Matplotlib not available - plotting disabled")
    print("Install with: pip install matplotlib")


def demo_replaying_data(generator: DataReplayGenerator):
    """Demonstrate basic data replay functionality."""

    # Generate the replayed sequence
    replayed_time, replayed_values = generator.generate_sequence(duration=2.0, dt=0.02)

    # Plot if matplotlib is available
    if PLOTTING_AVAILABLE:
        plot_signals(
            {"time_points": replayed_time, "values": replayed_values, "name": "Data Replay - Original Sensor Data"},
            "Data Replay - Original Sensor Data",
        )


def plot_signals(data, title):
    """Plot multiple signals on the same figure."""
    if not PLOTTING_AVAILABLE:
        return

    plt.figure(figsize=(12, 8))

    plt.plot(data["time_points"], data["values"], label=data["name"], linewidth=2)

    plt.title(title, fontsize=16, fontweight="bold")
    plt.xlabel("Time (seconds)", fontsize=12)
    plt.ylabel("Amplitude", fontsize=12)
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("signals.png")


if __name__ == "__main__":
    # data can be a list of numerical values or a numpy array
    time_data = np.linspace(0, 1.0, 500)  # sample rate is 500 Hz
    print(time_data)
    position_data = np.sin(2 * np.pi * 1.5 * time_data)

    # create a generator
    generator = DataReplayGenerator(data=position_data, sample_rate=200.0, loop=True)
    # loop=True means that the data will be looped back to the beginning when it reaches the end
    # loop=False means that the data will stop at the end

    demo_replaying_data(generator)
