from opensourceleg.generators import SawtoothGenerator, SignalGenerator
from opensourceleg.utilities.softrealtimeloop import SoftRealtimeLoop

# Optional matplotlib import for plotting
try:
    import matplotlib.pyplot as plt

    PLOTTING_AVAILABLE = True
    print("Matplotlib available - plotting enabled")
except ImportError:
    PLOTTING_AVAILABLE = False
    print("Matplotlib not available - plotting disabled")
    print("Install with: pip install matplotlib")


def demo_generators(
    generator_class: SignalGenerator,
    duration: float = 2.0,
):
    """Demonstrate adding noise to sawtooth signals."""

    # Generate sequences
    time_points, values = generator_class.generate_sequence(duration=duration)

    print(f"Generated {len(values)} samples")

    # Plot comparison if matplotlib is available
    if PLOTTING_AVAILABLE:
        signals_data = {"time_points": time_points, "values": values, "name": generator_class.__class__.__name__}
        plot_signals(signals_data, generator_class.__class__.__name__)


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
    # you can generate signals with or without noise as a sequence
    generator = SawtoothGenerator(
        frequency=10.0, amplitude=1.0, add_noise=True, noise_amplitude=0.01, noise_type="uniform", seed=45
    )
    demo_generators(generator)

    # you can also use the SoftRealtimeLoop to generate signals within your control loop
    loop = SoftRealtimeLoop(dt=0.01)

    for t in loop:
        sawtooth_val = generator.update(t)
        print(f"Time: {t}, Sawtooth: {sawtooth_val}")
