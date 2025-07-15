# Fast Logger & Profiler for Open Source Leg
This project aims to improve the performance of the logging & profiling module for the Open-Source Leg
library.

## Goals
- Benchmark and profile the current Python logger used in OSL
- Identify and eliminate bottlenecks
- Provide a drop-in replacement logger with reduced latency

## Benchmark Results

### Logging by Number of Messages
| # of Messages | Rust Logger (sec) | Python Logger (sec) |
| ------------- | ----------------- | ------------------- |
| 1,000         | 0.00151491        | 0.0582547           |
| 10,000        | 0.014786         | 0.448561            |
| 100,000       | 0.15254          | 4.32093             |
| 1,000,000     | 1.56321           | 44.6831             |
| 10,000,000    | 15.4325           | 436.589             |

> **Rust is ~30x faster** on average for high volume logging
---

### Logging by Message Size
| Message Size | Rust Logger (sec) | Python Logger (sec) |
| ------------ | ----------------- | ------------------- |
| 10 KB        | 0.0000591         | 0.0000908           |
| 10 MB        | 0.0381405         | 0.0489812          |

> Both loggers perform similarly irrespective of the size of the payload

### Logging Performance: Objects & Primitives
| Test Case              | Rust Logger (sec) | Python Logger (sec) |
| ---------------------- | ----------------- | ------------------- |
|  Track objects (1M messages) | 1.96           | 2.13             |
| Track primitives (1M messages)       | 2.64          | 2.5             |

> Tracking objects in Rust requires calling Python’s __str__ method, which adds overhead from Python function calls and interop. Still, Rust maintains a performance advantage for object logging.

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
Run `maturin build -r` after step 2 of the instructions above, this will generate a distributable wheel.

### Log Observability
Since many applications built upon the Open-Source Leg 
can produce extensive logs, this module also provides a `grafana` dashboard for realtime log observability using loki and alloy.

To use the dashboard:
  
1. Install Docker Desktop
2. Navigate to the directory containing `docker-compose.yml`
3. Run `docker-compose up -d`
4. Open your browser and go to [http://localhost:3000](http://localhost:3000)
5. Configure the data source:
   - Go to **Add data source**
   - Select **Loki**
   - Set the URL to `http://loki:3100`
   - Click **Save & Test**
6. Create your own dashboard using the Explore tab or panel editor
7. Run your application, logs will stream into Grafana automatically
