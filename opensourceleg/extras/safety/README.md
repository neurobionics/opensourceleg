# Safety Library Guide

## Introduction

The safety module provides a comprehensive framework for adding safety checks and monitoring to actuator systems in the opensourceleg library. It includes decorators for value validation, exception handling for thermal and electrical limits, and a centralized safety management system.

## Customization Guide of Safety Module

1. Import safety module

   ```python
   import opensourceleg.safety as safety

   ```

2. Configure safety decorators for your actuator properties

   `is_within_range`, `is_positive`, `is_zero`, `is_greater_than`, `is_less_than`, and `custom_criteria` are predefined for common use.

   For example, when adding temperature monitoring:

   ```python
   from opensourceleg.safety import SafetyManager, SafetyDecorators

   # Initialize safety manager
   safety_manager = SafetyManager()

   # Add temperature monitoring
   safety_manager.add_safety(
       actuator,
       "case_temperature",
       SafetyDecorators.is_less_than(80, clamp=False)
   )
   ```

   Or using custom criteria for specialized checks:

   ```python
   def is_reasonable_velocity(velocity):
   """Custom check for reasonable velocity values"""
   return abs(velocity) < 10.0  # rad/s

   safety_manager.add_safety(
       actuator,
       "output_velocity",
       SafetyDecorators.custom_criteria(is_reasonable_velocity)
   )
   ```

3. Apply safety monitoring to your actuator

   ```python
   from opensourceleg.actuators.dephy import DephyActuator
   from opensourceleg.safety import SafetyManager, SafetyDecorators
   from opensourceleg.safety import ThermalLimitException, I2tLimitException

   def setup_actuator_with_safety(actuator):
       safety_manager = SafetyManager()

       # Temperature monitoring
       safety_manager.add_safety(
           actuator, "case_temperature",
           SafetyDecorators.is_less_than(75, clamp=False)
       )

       # Current limits
       safety_manager.add_safety(
           actuator, "motor_current",
           SafetyDecorators.is_within_range(-8000, 8000, clamp=True)
       )

       # Position monitoring
       safety_manager.add_safety(
           actuator, "output_position",
           SafetyDecorators.is_within_range(-3.14, 3.14, clamp=False)
       )

       return safety_manager

   # Usage
   actuator = DephyActuator(port='/dev/ttyACM0')
   safety_manager = setup_actuator_with_safety(actuator)

   try:
       with actuator:
           safety_manager.start()

           while True:
               actuator.update()
               safety_manager.update()
               # Your control logic here

   except ThermalLimitException as e:
       print(f"Thermal limit exceeded: {e}")
       # Handle thermal emergency
   except I2tLimitException as e:
       print(f"Current limit exceeded: {e}")
       # Handle current emergency
   ```

4. Link safety to your actuator update cycle

   For example:

   ```python
   class MyActuator(base.ActuatorBase):

    def __init__(self, *args, **kwargs):  # ← Now properly indented
        super().__init__(*args, **kwargs)
        self.safety_manager = SafetyManager()
        # Configure safety checks
        self._setup_safety()

    def _setup_safety(self):  # ← Also indented
        # Temperature monitoring
        self.safety_manager.add_safety(
            self, "case_temperature",
            SafetyDecorators.is_less_than(80, clamp=False)
        )
        # Should implement your own safety checks below

    def update(self) -> None:  # ← Also indented
        super().update()

        # Run safety checks
        self.safety_manager.update()

        # Should implement your own update steps below
   ```

## Reference

### Safety Execption Classes

- **`ThermalLimitException`**: Raised when software thermal limits are exceeded
- **`I2tLimitException`**: Raised when I²t (current-time) limits are exceeded

### Safety Decorators Reference

| Decorator                                                    | Purpose                        | Parameters                            |
| ------------------------------------------------------------ | ------------------------------ | ------------------------------------- |
| `is_within_range(min_val, max_val, clamp=False)`             | Validates values within bounds | min_val, max_val, clamp               |
| `is_positive(clamp=False)`                                   | Ensures positive values        | clamp                                 |
| `is_negative(clamp=False)`                                   | Ensures negative values        | clamp                                 |
| `is_zero(clamp=False)`                                       | Ensures zero values            | clamp                                 |
| `is_greater_than(threshold, clamp=False, equality=False)`    | Validates above threshold      | threshold, clamp, equality            |
| `is_less_than(threshold, clamp=False, equality=False)`       | Validates below threshold      | threshold, clamp, equality            |
| `is_changing(attribute_name, max_points=10, threshold=1e-6)` | Monitors value changes         | attribute_name, max_points, threshold |
| `custom_criteria(criteria)`                                  | Custom validation function     | criteria                              |

### SafetyManager Methods

| Method                                       | Purpose                         | Parameters                     |
| -------------------------------------------- | ------------------------------- | ------------------------------ |
| `add_safety(instance, attribute, decorator)` | Add safety check to property    | instance, attribute, decorator |
| `start()`                                    | Apply all configured decorators | None                           |
| `update()`                                   | Execute safety checks           | None                           |

### Best Practices

1. **Configure safety limits before starting** actuator operation
2. **Handle exceptions gracefully** with proper cleanup and logging
3. **Test safety systems** thoroughly before deployment
4. **Set reasonable thresholds** based on your hardware specifications

### Error Handling Example

```python
try:
    # Your actuator control code
    while True:
        actuator.update()
        safety_manager.update()
        # Control logic here

except ThermalLimitException:
    actuator.stop()
    print("System overheated - emergency shutdown")

except I2tLimitException:
    actuator.stop()
    print("Overcurrent detected - emergency stop")

except ValueError as e:
    print(f"Safety violation: {e}")
    # Implement appropriate response

finally:
    actuator.stop()
```
