"""
Real-time tracer wrapping VizTracer for low-overhead profiling.
"""

import json
from configparser import ConfigParser
from functools import wraps
from pathlib import Path
from typing import Any, Callable, Optional

import numpy as np
from viztracer import VizTracer

__all__ = ["RealtimeTracer", "IterationStats"]

_DEFAULT_CONFIG_PATH = Path(__file__).parent / ".viztracerrc"


def _load_config(config_path: Optional[Path] = None) -> dict:
    """Load configuration from .viztracerrc file."""
    path = config_path or _DEFAULT_CONFIG_PATH

    if not path.exists():
        return {}

    parser = ConfigParser()
    parser.read(path)

    if not parser.has_section("default"):
        return {}

    config = {}
    for key, value in parser.items("default"):
        if value.lower() in ("true", "false"):
            config[key] = value.lower() == "true"
        elif value.isdigit():
            config[key] = int(value)
        else:
            try:
                config[key] = float(value)
            except ValueError:
                config[key] = value

    return config


class IterationStats:
    """Statistics for control loop iterations."""

    def __init__(
        self,
        mean_time: float,
        std_time: float,
        min_time: float,
        max_time: float,
        ideal_time: float,
        frequency: float,
        total_iterations: int,
        iterations_within_tolerance: int,
        tolerance_percentage: float,
        iteration_times: np.ndarray,
        iteration_starts: np.ndarray,
        iteration_ends: np.ndarray,
    ) -> None:
        self.mean_time = mean_time
        self.std_time = std_time
        self.min_time = min_time
        self.max_time = max_time
        self.ideal_time = ideal_time
        self.frequency = frequency
        self.total_iterations = total_iterations
        self.iterations_within_tolerance = iterations_within_tolerance
        self.tolerance_percentage = tolerance_percentage
        self.jitter = std_time
        self.iteration_times = iteration_times
        self.iteration_starts = iteration_starts
        self.iteration_ends = iteration_ends

    def __str__(self) -> str:
        """Format statistics as human-readable string."""
        return (
            f"Target Frequency: {self.frequency:.2f} Hz\n"
            f"Ideal Iteration Time: {self.ideal_time * 1000:.3f} ms\n"
            f"Total Iterations: {self.total_iterations}\n"
            f"\n"
            f"Timing Statistics:\n"
            f"  Mean: {self.mean_time * 1000:.3f} ms\n"
            f"  Std Dev (Jitter): {self.std_time * 1000:.3f} ms\n"
            f"  Min: {self.min_time * 1000:.3f} ms\n"
            f"  Max: {self.max_time * 1000:.3f} ms\n"
            f"\n"
            f"Performance:\n"
            f"  Iterations within 10% tolerance: {self.iterations_within_tolerance}/{self.total_iterations} "
            f"({self.tolerance_percentage:.1f}%)\n"
        )


class RealtimeTracer:
    """
    Low-overhead wrapper around VizTracer optimized for real-time robotics applications.

    Args:
        output_file: Path to save trace JSON (default: "result.json")
        config_file: Path to custom .viztracerrc config file (default: use package default)
        max_stack_depth: Maximum call stack depth to trace (overrides config file)
        ignore_c_function: Skip C functions like numpy internals (overrides config file)
        **kwargs: Additional VizTracer arguments (override config file)

    Examples:
        >>> tracer = RealtimeTracer(output_file="trace.json")
        >>> with tracer:
        ...     for t in loop:
        ...         do_work()
        >>> tracer.analyze(frequency=200, pattern="do_work")
        >>> print(tracer.stats)

        >>> tracer = RealtimeTracer(config_file=".custom_config", max_stack_depth=5)
    """

    def __init__(
        self,
        output_file: str = "result.json",
        config_file: Optional[str | Path] = None,
        max_stack_depth: Optional[int] = None,
        ignore_c_function: Optional[bool] = None,
        **kwargs: Any,
    ) -> None:
        self.output_file = output_file

        config_path = Path(config_file) if config_file else None
        config = _load_config(config_path)

        if max_stack_depth is not None:
            config["max_stack_depth"] = max_stack_depth
        if ignore_c_function is not None:
            config["ignore_c_function"] = ignore_c_function

        config.update(kwargs)

        self._tracer = VizTracer(**config)
        self._running = False

        self.stats: Optional[IterationStats] = None
        self.timeline: Optional[dict] = None
        self.detected_pattern: Optional[str] = None
        self._data: Optional[dict] = None
        self._events: Optional[list] = None

    def start(self) -> None:
        """Start tracing execution."""
        if not self._running:
            self._tracer.start()
            self._running = True

    def stop(self) -> None:
        """Stop tracing execution."""
        if self._running:
            self._tracer.stop()
            self._running = False

    def save(self, output_file: Optional[str] = None) -> None:
        """
        Save trace data to JSON file.

        Args:
            output_file: Override default output file path
        """
        if self._running:
            self.stop()

        file_path = output_file or self.output_file
        self._tracer.save(file_path)

    def __enter__(self) -> "RealtimeTracer":
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, *args: Any) -> None:
        """Context manager exit."""
        self.save()

    def decorate(self, func: Callable) -> Callable:
        """
        Decorator to profile a function.

        Args:
            func: Function to profile

        Returns:
            Wrapped function
        """

        @wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            self.start()
            try:
                result = func(*args, **kwargs)
                return result
            finally:
                self.save()

        return wrapper

    def _load_data(self) -> dict:
        """Load and cache JSON data."""
        if self._data is None:
            with open(self.output_file) as f:
                self._data = json.load(f)
        return self._data

    def _extract_events(self) -> list:
        """Extract trace events from JSON."""
        if self._events is None:
            data = self._load_data()
            self._events = data.get("traceEvents", [])
        return self._events

    def _find_pattern(self, pattern_name: Optional[str] = None) -> list[dict]:
        """Find repeating function call patterns in trace."""
        events = self._extract_events()

        if pattern_name:
            matches = [e for e in events if e.get("ph") in ("B", "E", "X") and pattern_name in e.get("name", "")]
        else:
            matches = [e for e in events if e.get("ph") in ("B", "E", "X")]

        return sorted(matches, key=lambda e: e.get("ts", 0))

    def _detect_most_frequent_pattern(self) -> Optional[str]:
        """Auto-detect the most frequently called user function (excludes framework/stdlib code)."""
        events = self._extract_events()
        function_events = [e for e in events if e.get("ph") in ("B", "E", "X")]

        if not function_events:
            return None

        from collections import Counter

        user_function_names = []
        for e in function_events:
            name = e.get("name", "")
            if not name:
                continue

            if any(excluded in name for excluded in ["/site-packages/", "/lib/python", "/dist-packages/"]):
                continue

            if "/opensourceleg/opensourceleg/" in name:
                continue

            if any(excluded in name.lower() for excluded in ["<", "lambda", "wrapper"]):
                continue

            user_function_names.append(name)

        if not user_function_names:
            function_names = [e.get("name", "") for e in function_events]
            name_counts = Counter(function_names)
        else:
            name_counts = Counter(user_function_names)

        if not name_counts:
            return None

        most_common_name, count = name_counts.most_common(1)[0]

        if count < 2:
            return None

        return most_common_name

    def get_timeline(self) -> dict:
        """
        Extract full timeline including idle periods.

        Returns:
            Dictionary with timeline information
        """
        if self.timeline is not None:
            return self.timeline

        events = self._extract_events()

        if not events:
            self.timeline = {"start_time": 0, "end_time": 0, "duration": 0, "total_events": 0}
            return self.timeline

        timestamps = [e["ts"] for e in events if "ts" in e]
        if not timestamps:
            self.timeline = {"start_time": 0, "end_time": 0, "duration": 0, "total_events": 0}
            return self.timeline

        start_time = min(timestamps) / 1e6
        end_time = max(timestamps) / 1e6
        duration = end_time - start_time

        self.timeline = {
            "start_time": start_time,
            "end_time": end_time,
            "duration": duration,
            "total_events": len(events),
        }
        return self.timeline

    def analyze(
        self,
        frequency: float,
        pattern: Optional[str] = None,
        tolerance: float = 0.1,
    ) -> IterationStats:
        """
        Analyze loop iteration timing from trace data.

        Args:
            frequency: Expected loop frequency in Hz
            pattern: Function name to detect iterations (optional, auto-detects if not provided)
            tolerance: Tolerance for deadline compliance (default 10%)

        Returns:
            IterationStats object with timing analysis

        Examples:
            >>> tracer = RealtimeTracer()
            >>> with tracer:
            ...     # profiled code
            >>> stats = tracer.analyze(frequency=200)  # Auto-detect pattern
            >>> print(f"Detected pattern: {tracer.detected_pattern}")

            >>> stats = tracer.analyze(frequency=200, pattern="my_function")  # Explicit
        """
        ideal_time = 1.0 / frequency

        if pattern is None:
            detected = self._detect_most_frequent_pattern()
            if detected is None:
                raise ValueError("Could not auto-detect iteration pattern. Please specify pattern explicitly.")
            pattern = detected
            self.detected_pattern = pattern
        else:
            self.detected_pattern = pattern

        pattern_events = self._find_pattern(pattern)
        iteration_events = [e for e in pattern_events if e.get("ph") in ("B", "X")]

        if not iteration_events or len(iteration_events) < 2:
            raise ValueError(f"No iteration pattern detected for '{pattern}'")

        timestamps = np.array([e["ts"] for e in iteration_events], dtype=np.float64)
        timestamps_sec = timestamps / 1e6

        starts = timestamps_sec[:-1]
        ends = timestamps_sec[1:]
        iteration_times = np.diff(timestamps_sec)

        mean_time = float(np.mean(iteration_times))
        std_time = float(np.std(iteration_times))
        min_time = float(np.min(iteration_times))
        max_time = float(np.max(iteration_times))

        within_tolerance = np.abs(iteration_times - ideal_time) <= (tolerance * ideal_time)
        iterations_within = int(np.sum(within_tolerance))
        tolerance_pct = 100.0 * iterations_within / len(iteration_times)

        self.stats = IterationStats(
            mean_time=mean_time,
            std_time=std_time,
            min_time=min_time,
            max_time=max_time,
            ideal_time=ideal_time,
            frequency=frequency,
            total_iterations=len(iteration_times),
            iterations_within_tolerance=iterations_within,
            tolerance_percentage=tolerance_pct,
            iteration_times=iteration_times,
            iteration_starts=starts,
            iteration_ends=ends,
        )

        return self.stats

    def summary(self) -> None:
        """
        Print comprehensive analysis summary.

        Call analyze() first to populate stats and timeline.
        """
        timeline = self.get_timeline()
        print(f"\nTrace Timeline")
        print(f"Duration: {timeline['duration']:.3f} seconds")
        print(f"Total Events: {timeline['total_events']}")
        print(f"Start: {timeline['start_time']:.6f} s")
        print(f"End: {timeline['end_time']:.6f} s")

        if self.stats:
            if self.detected_pattern:
                print(f"\nDetected Pattern: {self.detected_pattern}")
            print(f"\n{self.stats}")
        else:
            print("\nNo iteration analysis available. Call analyze() first.")

    def plot_histogram(
        self, bins: int = 50, figsize: tuple = (10, 6), save_path: Optional[str] = None
    ) -> None:
        """
        Plot histogram of iteration times.

        Args:
            bins: Number of histogram bins
            figsize: Figure size (width, height) in inches
            save_path: Optional path to save figure

        Examples:
            >>> tracer.analyze(frequency=200)
            >>> tracer.plot_histogram()
            >>> tracer.plot_histogram(bins=100, save_path="histogram.png")
        """
        if self.stats is None:
            raise ValueError("No analysis results. Call analyze() first.")

        try:
            import matplotlib.pyplot as plt
        except ImportError:
            raise ImportError("matplotlib is required for plotting. Install with: pip install matplotlib")

        fig, ax = plt.subplots(figsize=figsize)

        iteration_times_ms = self.stats.iteration_times * 1000

        ax.hist(iteration_times_ms, bins=bins, color="steelblue", alpha=0.7, edgecolor="black")

        ax.axvline(
            self.stats.ideal_time * 1000,
            color="green",
            linestyle="--",
            linewidth=2,
            label=f"Ideal: {self.stats.ideal_time * 1000:.2f} ms",
        )

        ax.axvline(
            self.stats.mean_time * 1000,
            color="red",
            linestyle="--",
            linewidth=2,
            label=f"Mean: {self.stats.mean_time * 1000:.2f} ms",
        )

        ax.set_xlabel("Iteration Time (ms)", fontsize=12)
        ax.set_ylabel("Frequency", fontsize=12)
        ax.set_title(
            f"Iteration Time Distribution\n"
            f"Mean: {self.stats.mean_time * 1000:.2f} ms, Std: {self.stats.std_time * 1000:.2f} ms",
            fontsize=14,
        )
        ax.legend()
        ax.grid(True, alpha=0.3, axis="y")

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"Histogram saved to: {save_path}")
        else:
            plt.show()
