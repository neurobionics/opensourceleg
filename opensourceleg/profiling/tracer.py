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
_MICROSECONDS_TO_SECONDS = 1e6
_SECONDS_TO_MS = 1000


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
        value_lower = value.lower()
        if value_lower in ("true", "false"):
            config[key] = value_lower == "true"
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
        pattern_data: list[dict],
        tolerance: float = 0.1,
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
        self.tolerance = tolerance
        self.jitter = std_time
        self.iteration_times = iteration_times
        self.iteration_starts = iteration_starts
        self.iteration_ends = iteration_ends
        self.pattern_data = pattern_data

    def __str__(self) -> str:
        """Format statistics as human-readable string."""
        tolerance_pct = self.tolerance * 100
        return (
            f"Target Frequency: {self.frequency:.2f} Hz\n"
            f"Ideal Iteration Time: {self.ideal_time * _SECONDS_TO_MS:.3f} ms\n"
            f"Total Iterations: {self.total_iterations}\n"
            f"\n"
            f"Timing Statistics:\n"
            f"  Mean: {self.mean_time * _SECONDS_TO_MS:.3f} ms\n"
            f"  Std Dev (Jitter): {self.std_time * _SECONDS_TO_MS:.3f} ms\n"
            f"  Min: {self.min_time * _SECONDS_TO_MS:.3f} ms\n"
            f"  Max: {self.max_time * _SECONDS_TO_MS:.3f} ms\n"
            f"\n"
            f"Performance:\n"
            f"  Iterations within {tolerance_pct:.0f}% tolerance: {self.iterations_within_tolerance}/{self.total_iterations} "
            f"({self.tolerance_percentage:.1f}%)\n"
        )


class RealtimeTracer:
    """
    Low-overhead wrapper around VizTracer optimized for real-time robotics applications.

    Use tracer.mark() to explicitly mark iteration boundaries for accurate timing analysis.

    Args:
        output_file: Path to save trace JSON (default: "result.json")
        config_file: Path to custom .viztracerrc config file (default: use package default)
        max_stack_depth: Maximum call stack depth to trace (overrides config file)
        ignore_c_function: Skip C functions like numpy internals (overrides config file)
        **kwargs: Additional VizTracer arguments (override config file)

    Examples:
        Basic usage with iteration markers:
        >>> tracer = RealtimeTracer(output_file="trace.json")
        >>> with tracer:
        ...     for t in loop:
        ...         tracer.mark("iteration")  # Mark iteration boundary
        ...         do_work()
        >>> tracer.analyze(frequency=200, start_marker="iteration")
        >>> print(tracer.stats)
        >>> tracer.plot_timeline()
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
        self.end_marker_used: Optional[str] = None
        self._data: Optional[dict] = None
        self._events: Optional[list] = None

    def start(self) -> None:
        if not self._running:
            self._tracer.start()
            self._running = True

    def stop(self) -> None:
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

    def mark(self, name: str, args: Optional[dict] = None) -> None:
        """
        Mark an iteration boundary or event with a named marker.

        This is the REQUIRED way to mark iteration boundaries for accurate analysis.

        Args:
            name: Name of the marker (e.g., "iteration_start")
            args: Optional dictionary of arguments to log with the marker

        Examples:
            >>> tracer = RealtimeTracer()
            >>> with tracer:
            ...     for i in range(100):
            ...         tracer.mark("iteration")  # Mark iteration boundary
            ...         do_work()
            >>> tracer.analyze(frequency=200, marker="iteration")
        """
        if self._running:
            self._tracer.log_instant(name, args=args, scope="t")

    def __enter__(self) -> "RealtimeTracer":
        self.start()
        return self

    def __exit__(self, *args: Any) -> None:
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
        """
        Find repeating patterns in trace (function calls or instant event markers).

        Supports both function call events (B/E/X) and instant event markers (i).
        Instant markers are recommended for accurate iteration detection.
        """
        events = self._extract_events()

        if pattern_name:
            # Support both function calls and instant event markers
            matches = [
                e for e in events
                if e.get("ph") in ("B", "E", "X", "i") and pattern_name in e.get("name", "")
            ]
        else:
            matches = [e for e in events if e.get("ph") in ("B", "E", "X")]

        return sorted(matches, key=lambda e: e.get("ts", 0))

    def _extract_pattern_durations(self, pattern_name: str) -> list[dict]:
        """
        Extract individual pattern execution details including duration.

        Returns list of dicts with 'start_time', 'end_time', 'duration' for each pattern occurrence.
        """
        events = self._extract_events()

        pattern_events = [
            e for e in events
            if pattern_name in e.get("name", "")
        ]
        pattern_events.sort(key=lambda e: e.get("ts", 0))

        executions = []

        # Handle complete events (X type) - these have duration built-in
        complete_events = [e for e in pattern_events if e.get("ph") == "X"]
        for event in complete_events:
            start_ts = event["ts"] / _MICROSECONDS_TO_SECONDS
            duration = event.get("dur", 0) / _MICROSECONDS_TO_SECONDS
            executions.append({
                "start_time": start_ts,
                "end_time": start_ts + duration,
                "duration": duration,
            })

        # Handle begin/end pairs (B/E types)
        if not executions:
            begin_events = [e for e in pattern_events if e.get("ph") == "B"]
            end_events = [e for e in pattern_events if e.get("ph") == "E"]

            for begin in begin_events:
                tid = begin.get("tid")
                name = begin.get("name")
                start_ts = begin["ts"]

                matching_end = next(
                    (end for end in end_events
                     if end.get("tid") == tid and
                        end.get("name") == name and
                        end["ts"] > start_ts),
                    None
                )

                if matching_end:
                    start_time = start_ts / _MICROSECONDS_TO_SECONDS
                    end_time = matching_end["ts"] / _MICROSECONDS_TO_SECONDS
                    executions.append({
                        "start_time": start_time,
                        "end_time": end_time,
                        "duration": end_time - start_time,
                    })

        executions.sort(key=lambda x: x["start_time"])

        return executions

    def get_timeline(self) -> dict:
        """
        Extract full timeline including idle periods.

        Returns:
            Dictionary with timeline information
        """
        if self.timeline is not None:
            return self.timeline

        events = self._extract_events()
        timestamps = [e["ts"] for e in events if "ts" in e]

        if not timestamps:
            self.timeline = {"start_time": 0, "end_time": 0, "duration": 0, "total_events": 0}
            return self.timeline

        start_time = min(timestamps) / _MICROSECONDS_TO_SECONDS
        end_time = max(timestamps) / _MICROSECONDS_TO_SECONDS

        self.timeline = {
            "start_time": start_time,
            "end_time": end_time,
            "duration": end_time - start_time,
            "total_events": len(events),
        }
        return self.timeline

    def analyze(
        self,
        frequency: float,
        start_marker: str,
        end_marker: Optional[str] = None,
        tolerance: float = 0.1,
    ) -> IterationStats:
        """
        Analyze loop iteration timing from trace data.

        IMPORTANT: Use tracer.mark("marker_name") in your loop to mark iterations.

        Args:
            frequency: Expected loop frequency in Hz
            start_marker: Name of the marker used to detect iteration start
            end_marker: Optional marker for iteration end (analyzes only code between markers)
            tolerance: Tolerance for deadline compliance (default 10%)

        Returns:
            IterationStats object with timing analysis

        Examples:
            Single marker (full iteration):
            >>> tracer = RealtimeTracer()
            >>> with tracer:
            ...     for t in loop:
            ...         tracer.mark("iteration")  # Mark iteration start
            ...         do_work()
            >>> stats = tracer.analyze(frequency=200, start_marker="iteration")

            Dual markers (analyze only code between markers):
            >>> with tracer:
            ...     for t in loop:
            ...         tracer.mark("start")
            ...         do_critical_work()  # Only analyze this
            ...         tracer.mark("end")
            ...         do_other_work()  # Not analyzed
            >>> stats = tracer.analyze(frequency=200, start_marker="start", end_marker="end")
        """
        ideal_time = 1.0 / frequency
        self.detected_pattern = start_marker
        self.end_marker_used = end_marker

        start_events = self._find_pattern(start_marker)
        # Support instant markers (i), function begin (B), and complete events (X)
        start_markers = [e for e in start_events if e.get("ph") in ("B", "X", "i")]

        if not start_markers or len(start_markers) < 2:
            raise ValueError(
                f"No marker '{start_marker}' found in trace data. "
                f"Did you call tracer.mark('{start_marker}') in your loop?"
            )

        # If end_marker is provided, use it to define iteration boundaries
        if end_marker:
            end_events = self._find_pattern(end_marker)
            end_markers = [e for e in end_events if e.get("ph") in ("B", "X", "i")]

            if not end_markers:
                raise ValueError(
                    f"No end marker '{end_marker}' found in trace data. "
                    f"Did you call tracer.mark('{end_marker}') in your loop?"
                )

            if len(end_markers) != len(start_markers):
                raise ValueError(
                    f"Mismatched markers: found {len(start_markers)} '{start_marker}' markers "
                    f"but {len(end_markers)} '{end_marker}' markers. "
                    f"Each iteration must have both start and end markers."
                )

            start_timestamps = np.array([e["ts"] for e in start_markers], dtype=np.float64) / _MICROSECONDS_TO_SECONDS
            end_timestamps = np.array([e["ts"] for e in end_markers], dtype=np.float64) / _MICROSECONDS_TO_SECONDS

            for i, (start, end) in enumerate(zip(start_timestamps, end_timestamps)):
                if end <= start:
                    raise ValueError(
                        f"Invalid marker order at iteration {i}: "
                        f"end marker ('{end_marker}') must come after start marker ('{start_marker}')"
                    )

            starts = start_timestamps[:-1]
            ends = end_timestamps[:-1]
            iteration_times = np.diff(start_timestamps)

        else:
            # Without end_marker: iteration goes from start[i] to start[i+1]
            timestamps = np.array([e["ts"] for e in start_markers], dtype=np.float64) / _MICROSECONDS_TO_SECONDS
            starts = timestamps[:-1]
            ends = timestamps[1:]
            iteration_times = np.diff(timestamps)

        mean_time = float(np.mean(iteration_times))
        std_time = float(np.std(iteration_times))
        min_time = float(np.min(iteration_times))
        max_time = float(np.max(iteration_times))

        within_tolerance = np.abs(iteration_times - ideal_time) <= (tolerance * ideal_time)
        iterations_within = int(np.sum(within_tolerance))
        tolerance_pct = 100.0 * iterations_within / len(iteration_times)

        executions = self._extract_pattern_durations(start_marker)

        pattern_data = []
        for i in range(len(iteration_times)):
            iter_data = {
                "iteration_index": i,
                "start_time": starts[i],
                "next_iteration_start": ends[i],
                "inter_iteration_time": iteration_times[i],
                "within_tolerance": bool(within_tolerance[i]),
            }

            if i < len(executions):
                iter_data["execution_start"] = executions[i]["start_time"]
                iter_data["execution_end"] = executions[i]["end_time"]
                iter_data["execution_duration"] = executions[i]["duration"]

            pattern_data.append(iter_data)

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
            pattern_data=pattern_data,
            tolerance=tolerance,
        )

        return self.stats

    def summary(self) -> None:
        """
        Print comprehensive analysis summary.

        Call analyze() first to populate stats and timeline.
        """
        print(self.stats)

    def plot_histogram(
        self, bins: int = 50, figsize: tuple = (10, 6), colormap: str = "viridis", save_path: Optional[str] = None
    ) -> None:
        """
        Plot histogram of iteration times.

        Args:
            bins: Number of histogram bins
            figsize: Figure size (width, height) in inches
            colormap: Colormap to use for the histogram
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
            import matplotlib.cm as cm
        except ImportError:
            raise ImportError("matplotlib is required for plotting. Install with: pip install matplotlib")

        fig, ax = plt.subplots(figsize=figsize)
        cmap = cm.get_cmap(colormap)

        iteration_times_ms = self.stats.iteration_times * _SECONDS_TO_MS

        ax.hist(iteration_times_ms, bins=bins, color=cmap(0.85), alpha=0.7, edgecolor="black")

        ax.axvline(
            self.stats.ideal_time * _SECONDS_TO_MS,
            color=cmap(0.25),
            linestyle="--",
            linewidth=2,
            label=f"Ideal: {self.stats.ideal_time * _SECONDS_TO_MS:.2f} ms",
        )

        ax.axvline(
            self.stats.mean_time * _SECONDS_TO_MS,
            color=cmap(0.85),
            linestyle="--",
            linewidth=2,
            label=f"Mean: {self.stats.mean_time * _SECONDS_TO_MS:.2f} ms",
        )

        ax.set_xlabel("Iteration Time (ms)", fontsize=12)
        ax.set_ylabel("Frequency", fontsize=12)
        ax.set_title(
            f"Iteration Time Distribution, "
            f"Mean: {self.stats.mean_time * _SECONDS_TO_MS:.2f} ms, Std: {self.stats.std_time * _SECONDS_TO_MS:.2f} ms",
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

    def _extract_function_calls_from_events(
        self, events: list[dict], start_time: float, end_time: float
    ) -> list[dict]:
        """
        Extract function calls with durations from a list of events.

        Args:
            events: List of trace events
            start_time: Iteration start time (seconds)
            end_time: Iteration end time (seconds)

        Returns:
            List of function call dictionaries with name, start, end, duration
        """
        iteration_events = [
            e for e in events
            if "ts" in e and start_time <= (e["ts"] / _MICROSECONDS_TO_SECONDS) < end_time
        ]
        iteration_events.sort(key=lambda e: e["ts"])

        call_stack = []
        function_calls = []

        for event in iteration_events:
            ts = event["ts"] / _MICROSECONDS_TO_SECONDS
            name = event.get("name", "unknown")
            ph = event.get("ph")

            if ph == "B":
                call_stack.append({"name": name, "start": ts, "depth": len(call_stack)})
            elif ph == "E":
                if call_stack:
                    call = call_stack.pop()
                    if call["name"] == name:
                        function_calls.append({
                            "name": name,
                            "start": call["start"],
                            "end": ts,
                            "duration": ts - call["start"],
                            "depth": call.get("depth", 0),
                        })
            elif ph == "X":
                dur = event.get("dur", 0) / _MICROSECONDS_TO_SECONDS
                function_calls.append({
                    "name": name,
                    "start": ts,
                    "end": ts + dur,
                    "duration": dur,
                    "depth": 0,
                })

        return function_calls

    def _extract_function_name(self, full_name: str) -> str:
        """
        Extract readable function name from VizTracer event name.

        VizTracer can store names in various formats:
        - "function_name (path/to/file.py:line)"
        - "Class.method (path/to/file.py:line)"
        - "path/to/file.py:line:function_name"
        - Just "function_name"

        Args:
            full_name: The full event name from VizTracer

        Returns:
            Cleaned function name
        """
        if " (" in full_name:
            return full_name.split(" (")[0].strip()

        if ":" in full_name:
            parts = full_name.split(":")
            if len(parts) >= 3:
                return parts[-1].strip()
            if len(parts) == 2:
                return parts[0].split("/")[-1]

        if "/" in full_name:
            return full_name.split("/")[-1]

        return full_name

    def plot_timeline(
        self,
        figsize: tuple = (14, 8),
        save_path: Optional[str] = None,
        max_iterations: Optional[int] = None,
        colormap: str = "viridis",
        dpi: int = 150,
        error_threshold_ms: float = 0.01,
        color_range: tuple = (0.15, 0.85),
    ) -> None:
        """
        Plot execution pattern aggregated across all iterations with error bars.

        Shows mean execution time and position for each function with error bars
        indicating variation (min/max) across all iterations.

        Args:
            figsize: Figure size (width, height) in inches
            save_path: Optional path to save figure
            max_iterations: Maximum number of iterations to analyze (None = all)
            colormap: Matplotlib colormap name (default: "inferno")
            dpi: DPI for saved figure (default: 150)
            error_threshold_ms: Minimum error to display annotation in ms (default: 0.01)
            color_range: Tuple of (min, max) for colormap sampling (default: (0.15, 0.85))

        Examples:
            >>> tracer.analyze(frequency=200)
            >>> tracer.plot_timeline()
            >>> tracer.plot_timeline(max_iterations=100, save_path="all_iters.png")
            >>> tracer.plot_timeline(colormap="viridis", dpi=300)
        """
        if self.stats is None:
            raise ValueError("No analysis results. Call analyze() first.")

        try:
            import matplotlib.pyplot as plt
            import matplotlib.cm as cm
        except ImportError:
            raise ImportError("matplotlib is required for plotting. Install with: pip install matplotlib")

        pattern_data = self.stats.pattern_data
        if max_iterations:
            pattern_data = pattern_data[:max_iterations]

        events = self._extract_events()

        # Structure: {function_name: [{"start_rel": ..., "duration": ..., "iter": ...}, ...]}
        function_executions = {}

        for iter_idx, iter_info in enumerate(pattern_data):
            start_time = iter_info["start_time"]
            end_time = iter_info["next_iteration_start"]

            function_calls = self._extract_function_calls_from_events(events, start_time, end_time)

            # Filter out instant event markers and the mark() method itself (metadata/instrumentation)
            function_calls = [
                call for call in function_calls
                if call.get("duration", 0) > 0
                and not self._extract_function_name(call["name"]).endswith(".mark")
            ]

            call_counts = {}
            for call in function_calls:
                display_name = self._extract_function_name(call["name"])
                call_counts[display_name] = call_counts.get(display_name, 0) + 1

            call_sequence = {}

            for call in function_calls:
                start_rel = (call["start"] - start_time) * _SECONDS_TO_MS
                duration = call["duration"] * _SECONDS_TO_MS
                end_rel = start_rel + duration
                display_name = self._extract_function_name(call["name"])

                if display_name not in call_sequence:
                    call_sequence[display_name] = 0
                call_sequence[display_name] += 1
                seq_num = call_sequence[display_name]

                # If function appears multiple times in this iteration, add sequence number
                if call_counts[display_name] > 1:
                    unique_key = f"{display_name} #{seq_num}"
                else:
                    unique_key = display_name

                if unique_key not in function_executions:
                    function_executions[unique_key] = []

                function_executions[unique_key].append({
                    "start_rel": start_rel,
                    "end_rel": end_rel,
                    "duration": duration,
                    "iter": iter_idx,
                    "seq_num": seq_num,
                })

        function_stats = {}
        for func_name, executions in function_executions.items():
            starts = np.array([e["start_rel"] for e in executions])
            ends = np.array([e["end_rel"] for e in executions])
            durations = np.array([e["duration"] for e in executions])

            function_stats[func_name] = {
                "mean_start": np.mean(starts),
                "min_start": np.min(starts),
                "max_start": np.max(starts),
                "std_start": np.std(starts),
                "mean_end": np.mean(ends),
                "min_end": np.min(ends),
                "max_end": np.max(ends),
                "std_end": np.std(ends),
                "mean_duration": np.mean(durations),
                "min_duration": np.min(durations),
                "max_duration": np.max(durations),
                "std_duration": np.std(durations),
                "count": len(executions),
            }

        ordered_functions = sorted(function_stats.keys(), key=lambda f: function_stats[f]["mean_start"])
        name_to_row = {name: idx for idx, name in enumerate(ordered_functions)}

        fig, ax = plt.subplots(figsize=figsize)
        cmap = cm.get_cmap(colormap)

        n_funcs = len(ordered_functions)
        color_indices = np.linspace(color_range[0], color_range[1], n_funcs)
        color_map = {name: cmap(color_indices[i])
                     for i, name in enumerate(ordered_functions)}

        for func_name in ordered_functions:
            stats = function_stats[func_name]
            y_pos = name_to_row[func_name]

            ax.barh(
                y_pos,
                stats["mean_duration"],
                left=stats["mean_start"],
                height=0.6,
                color=color_map[func_name],
                alpha=0.8,
                edgecolor="black",
                linewidth=1.0,
                label=func_name,
            )

            full_range = stats["max_end"] - stats["min_start"]
            ax.barh(
                y_pos,
                full_range,
                left=stats["min_start"],
                height=0.3,
                color=color_map[func_name],
                alpha=0.2,
                edgecolor="none",
            )

            ax.errorbar(
                stats["mean_start"],
                y_pos,
                xerr=[[stats["mean_start"] - stats["min_start"]],
                      [stats["max_start"] - stats["mean_start"]]],
                fmt="o",
                ecolor="black",
                markersize=4,
                capsize=4,
                capthick=1.5,
                alpha=0.7,
                markerfacecolor="black",
            )

            ax.errorbar(
                stats["mean_end"],
                y_pos,
                xerr=[[stats["mean_end"] - stats["min_end"]],
                      [stats["max_end"] - stats["mean_end"]]],
                fmt="o",
                ecolor="black",
                markersize=4,
                capsize=4,
                capthick=1.5,
                alpha=0.7,
                markerfacecolor="black",
            )

            ax.text(
                stats["mean_start"] + stats["mean_duration"] / 2,
                y_pos,
                f"{stats['mean_duration']:.2f}ms",
                ha="center",
                va="center",
                fontsize=9,
                color="white",
                weight="bold",
            )

            # Calculate asymmetric error values
            start_lower_err = stats["mean_start"] - stats["min_start"]
            start_upper_err = stats["max_start"] - stats["mean_start"]

            if max(start_lower_err, start_upper_err) > error_threshold_ms:
                ax.text(
                    stats["mean_start"],
                    y_pos + 0.35,
                    f"[-{start_lower_err:.2f}, +{start_upper_err:.2f}]",
                    ha="center",
                    va="bottom",
                    fontsize=7,
                    color="black",
                    alpha=0.7,
                )

            # Calculate asymmetric error values
            end_lower_err = stats["mean_end"] - stats["min_end"]
            end_upper_err = stats["max_end"] - stats["mean_end"]

            if max(end_lower_err, end_upper_err) > error_threshold_ms:
                ax.text(
                    stats["mean_end"],
                    y_pos + 0.35,
                    f"[-{end_lower_err:.2f}, +{end_upper_err:.2f}]",
                    ha="center",
                    va="bottom",
                    fontsize=7,
                    color="black",
                    alpha=0.7,
                )

        ax.set_xlabel("Time (ms)", fontsize=12)
        ax.set_ylabel("Function", fontsize=12)
        ax.set_title(
            f"Execution Timeline Across {len(pattern_data)} Iterations, "
            f"Mean: {self.stats.mean_time*_SECONDS_TO_MS:.2f} ms, "
            f"Std: {self.stats.std_time*_SECONDS_TO_MS:.2f} ms",
            fontsize=14,
        )
        ax.grid(True, alpha=0.3, axis="x")

        ax.set_yticks(range(len(ordered_functions)))
        ax.set_yticklabels(ordered_functions)
        ax.set_ylim(-0.5, len(ordered_functions) - 0.5)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=dpi, bbox_inches="tight")
            print(f"Timeline saved to: {save_path}")
        else:
            plt.show()
