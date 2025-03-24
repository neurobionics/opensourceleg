# Basic Motion Example
## Overview
This example demonstrates how to create basic sinusoidal motions for the knee and ankle joints of the OSL using `DephyActuator`s in position control mode. The reference positions are updated in real-time using an instance of `SoftRealtimeLoop`. The loop runs continuously until interrupted with `Ctrl+C`.

> **Note**:  
> This script is not intended for walking or load-bearing use. Ensure the OSL is in a safe configuration, such as being securely fixed to a benchtop, allowing the joints to move freely.

## Full Code for This Example

```python
--8<-- "examples/basic_motion.py"
```
