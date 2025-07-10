# Logger
This project aims to improve the performance of the logging module for the Open-Source Leg
library.

## Goals
- Benchmark and profile the current Python logger used in OSL
- Identify and eliminate bottlenecks
- Provide a drop-in replacement logger with reduced latency

### Logging by Number of Messages
| # of Messages | Rust Logger (sec) | Python Logger (sec) |
| ------------- | ----------------- | ------------------- |
| 1,000         | 0.00256801        | 0.0582547           |
| 10,000        | 0.0232134         | 0.448561            |
| 100,000       | 0.227412          | 4.32093             |
| 1,000,000     | 2.35191           | 44.6831             |
| 10,000,000    | 22.7011           | 436.589             |

### Logging by Message Size
| Message Size | Rust Logger (sec) | Python Logger (sec) |
| ------------ | ----------------- | ------------------- |
| 10 KB        | 0.0000591         | 0.0000908           |
| 10 MB        | 0.0381405         | 0.0489812          |

### Logging Performance: Objects & Primitives
| Test Case              | Rust Logger (sec) | Python Logger (sec) |
| ---------------------- | ----------------- | ------------------- |
|  Track objects (1M messages) | 2.62991           | 2.24524             |
| Track primitives (1M messages)       | 0.928524          | 2.61603             |
