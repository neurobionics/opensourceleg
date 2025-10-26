"""
CLI entry point for opensourceleg.profile

Usage:
    python -m opensourceleg.profile script.py [options]
"""

import argparse
import sys
from pathlib import Path

from opensourceleg.profile.tracer import RealtimeTracer


def main() -> None:
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Profile Python scripts with low-overhead real-time tracing",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python -m opensourceleg.profile script.py
  python -m opensourceleg.profile script.py --frequency 200 --pattern crazy_math --analyze
  python -m opensourceleg.profile script.py --max-stack-depth 5 --output trace.json
        """,
    )

    parser.add_argument("script", type=str, help="Python script to profile")

    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="result.json",
        help="Output JSON file (default: result.json)",
    )

    parser.add_argument(
        "--config",
        type=str,
        help="Path to custom .viztracerrc config file",
    )

    parser.add_argument(
        "--max-stack-depth",
        type=int,
        help="Maximum call stack depth to trace (overrides config)",
    )

    parser.add_argument(
        "--ignore-c-function",
        action="store_true",
        help="Ignore C functions (overrides config)",
    )

    parser.add_argument(
        "--frequency",
        type=float,
        help="Expected loop frequency in Hz (for analysis)",
    )

    parser.add_argument(
        "--analyze",
        action="store_true",
        help="Automatically analyze results after profiling",
    )

    parser.add_argument(
        "--pattern",
        type=str,
        help="Function name pattern to detect iterations (optional, auto-detects most frequent if not provided)",
    )

    args = parser.parse_args()

    script_path = Path(args.script)
    if not script_path.exists():
        print(f"Error: Script not found: {script_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Profiling {script_path}...")
    print(f"Output: {args.output}")

    kwargs = {}
    if args.config:
        kwargs["config_file"] = args.config
    if args.max_stack_depth is not None:
        kwargs["max_stack_depth"] = args.max_stack_depth
    if args.ignore_c_function:
        kwargs["ignore_c_function"] = args.ignore_c_function

    tracer = RealtimeTracer(output_file=args.output, **kwargs)

    sys.argv = [str(script_path)]

    with tracer:
        script_globals = {
            "__name__": "__main__",
            "__file__": str(script_path),
        }
        with open(script_path) as f:
            code = compile(f.read(), str(script_path), "exec")
            exec(code, script_globals)

    print(f"\nProfiling complete: {args.output}")

    if args.analyze:
        if not args.frequency:
            print("Error: --frequency is required for analysis", file=sys.stderr)
            sys.exit(1)

        print("\nAnalyzing results...")
        tracer.analyze(frequency=args.frequency, pattern=args.pattern)
        tracer.summary()


if __name__ == "__main__":
    main()
