import pytest
from opensourceleg.actuators.dephy import DephyActuator, DephyLegacyActuator

def test_dephy_actuator_offline_mode():
    actuator = DephyActuator(offline=True)
    # Should not raise when calling start/stop/update/home
    actuator.start()
    actuator.stop()
    actuator.update()
    actuator.home()
    # Should not raise when setting commands
    actuator.set_motor_voltage(1)
    actuator.set_motor_current(1)
    actuator.set_motor_torque(1)
    actuator.set_output_torque(1)
    actuator.set_position_gains(1, 1, 1, 1)
    actuator.set_current_gains(1, 1, 1, 1)
    actuator.set_impedance_gains(1, 1, 1, 1, 1, 1)
    actuator.set_motor_position(1)
    actuator.set_output_impedance(1, 1, 1, 1, 1, 1)
    actuator.set_motor_impedance(1, 1, 1, 1, 1, 1)
    # Properties should return zeros or safe values
    assert actuator.motor_position == 0.0
    assert actuator.motor_velocity == 0.0
    assert actuator.motor_voltage == 0.0
    assert actuator.motor_current == 0.0
    assert actuator.motor_torque == 0.0
    assert actuator.case_temperature == 0.0
    assert actuator.winding_temperature == 0.0
    assert actuator.output_torque == 0.0
    assert actuator.battery_voltage == 0.0
    assert actuator.battery_current == 0.0
    assert actuator.accelx == 0.0
    assert actuator.accely == 0.0
    assert actuator.accelz == 0.0
    assert actuator.gyrox == 0.0
    assert actuator.gyroy == 0.0
    assert actuator.gyroz == 0.0
    assert actuator.thermal_scaling_factor == 1.0 or actuator.thermal_scaling_factor == 0.0
    # Should not raise when accessing genvars
    _ = actuator.genvars

def test_dephy_legacy_actuator_offline_mode():
    actuator = DephyLegacyActuator(offline=True)
    actuator.start()
    actuator.stop()
    actuator.update()
    actuator.home()
    actuator.set_motor_voltage(1)
    actuator.set_motor_current(1)
    actuator.set_motor_torque(1)
    actuator.set_output_torque(1)
    actuator.set_position_gains(1, 1, 1, 1)
    actuator.set_current_gains(1, 1, 1, 1)
    actuator.set_impedance_gains(1, 1, 1, 1, 1, 1)
    actuator.set_motor_position(1)
    actuator.set_output_impedance(1, 1, 1, 1, 1, 1)
    actuator.set_motor_impedance(1, 1, 1, 1, 1, 1)
    assert actuator.motor_position == 0.0
    assert actuator.motor_velocity == 0.0
    assert actuator.motor_voltage == 0.0
    assert actuator.motor_current == 0.0
    assert actuator.motor_torque == 0.0
    assert actuator.case_temperature == 0.0
    assert actuator.winding_temperature == 0.0
    assert actuator.output_torque == 0.0
    assert actuator.battery_voltage == 0.0
    assert actuator.battery_current == 0.0
    assert actuator.accelx == 0.0
    assert actuator.accely == 0.0
    assert actuator.accelz == 0.0
    assert actuator.gyrox == 0.0
    assert actuator.gyroy == 0.0
    assert actuator.gyroz == 0.0
    assert actuator.thermal_scaling_factor == 1.0 or actuator.thermal_scaling_factor == 0.0
    _ = actuator.genvars
