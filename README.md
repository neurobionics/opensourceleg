# Fast Logger for Open Source Leg
This project aims to improve the performance of the logging module for the Open-Source Leg
library.

## Goals
- Benchmark and profile the current Python logger used in OSL
- Identify and eliminate bottlenecks
- Provide a drop-in replacement logger with reduced latency

## Benchmark Results

### Logging by Number of Messages
| # of Messages | Rust Logger (sec) | Python Logger (sec) |
| ------------- | ----------------- | ------------------- |
| 1,000         | 0.00256801        | 0.0582547           |
| 10,000        | 0.0232134         | 0.448561            |
| 100,000       | 0.227412          | 4.32093             |
| 1,000,000     | 2.35191           | 44.6831             |
| 10,000,000    | 22.7011           | 436.589             |

> **Rust is ~20x faster** on average for high volume logging
---

### Logging by Message Size
| Message Size | Rust Logger (sec) | Python Logger (sec) |
| ------------ | ----------------- | ------------------- |
| 10 KB        | 0.0000591         | 0.0000908           |
| 10 MB        | 0.0381405         | 0.0489812          |

> Both loggers perform similarly irrespective of the size of the payload

---

### Logging Performance: Objects & Primitives
| Test Case              | Rust Logger (sec) | Python Logger (sec) |
| ---------------------- | ----------------- | ------------------- |
|  Track objects (1M messages) | 2.62991           | 2.24524             |
| Track primitives (1M messages)       | 0.928524          | 2.61603             |

> Tracking objects in Rust requires calling Python’s __str__ method, which adds overhead from Python function calls and interop. Still, Rust maintains a performance advantage for primitive logging.

---

## Usage

Clone the repo and run the benchmarks:
```bash
python benchmark/bench.py       # Python logger
python benchmark/bench_rust.py  # Rust logger
python benchmark/bench_object_logging.py # Object logging
```

### To build the logger for local development

1. Create a virtual environment
2. Install dependencies: `pip install -r requirements.txt`
3. Run `maturin develop -r`
4. You can now use the Rust logger in Python!

### To build the logger for distribution
Run `maturin build -r` after step 2 of the instructions above, this will generate distributable wheel.

