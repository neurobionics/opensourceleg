from opensourceleg.generators import CustomGenerator
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


def demo_expression(generator: CustomGenerator, duration: float = 2.0, **kwargs):
    """Generate a sequence using a CustomGenerator and optionally plot it."""
    time_points, values = generator.generate_sequence(duration=duration, **kwargs)
    print(f"Generated {len(values)} samples")

    if PLOTTING_AVAILABLE:
        data = {
            "time_points": time_points,
            "values": values,
            "name": generator.__class__.__name__,
        }
        plot_signals(data, generator.__class__.__name__)


def plot_signals(data, title):
    """Plot a single signal."""
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
    plt.savefig("custom_expression_signal.png")


if __name__ == "__main__":
    poly_generator = CustomGenerator("a0 + a1*t + a2*t**2", ["a0", "a1", "a2"])
    demo_expression(poly_generator, duration=10.0, a0=0.0, a1=1.0, a2=-0.2)

    # usage in a control loop (optional)
    loop = SoftRealtimeLoop(dt=0.01)
    for t in loop:
        value = poly_generator.update(t, a0=0.0, a1=1.0, a2=-0.2)
        print(f"Time: {t}, Value: {value}")
