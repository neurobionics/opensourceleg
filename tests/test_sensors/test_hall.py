from unittest.mock import patch

import pytest

from opensourceleg.sensors import hall


@pytest.fixture
def sensor():
    return hall.DRV5056()


# Test DRV5056 init
def test_DRV5056_init():
    sensor = hall.DRV5056()
    assert sensor.tag == "DRV5056A1"
    assert sensor._sensor_num == "A1"
    assert sensor._t_a == 23
    assert sensor._supply_voltage == 5

    custom = hall.DRV5056(tag="MyHall", sensor_num="A3", t_a=25, supply_voltage=3.3)
    assert custom.tag == "MyHall"
    assert custom._sensor_num == "A3"
    assert custom._t_a == 25
    assert custom._supply_voltage == 3.3

    # offline mode is not supported and should exit
    with pytest.raises(SystemExit):
        hall.DRV5056(offline=True)


# Test DRV5056 repr
def test_DRV5056_repr(sensor: hall.DRV5056):
    assert sensor.__repr__() == "DRV5056"


# Test DRV5056 configure with default 5V supply
def test_DRV5056_configure_default(sensor: hall.DRV5056):
    sensor.configure()
    assert sensor.base_sensitivity == 200
    assert sensor.lower_volt_sensitivity == pytest.approx(0.6 * 200)
    assert sensor.base_range == 20
    assert sensor.lower_volt_range == 19
    assert sensor._s_tc == 0.0012
    assert sensor.range == sensor.base_range
    assert sensor._sensitivity == sensor.base_sensitivity


# Test DRV5056 configure with 3.3V supply
def test_DRV5056_configure_3_3V():
    sensor = hall.DRV5056(sensor_num="A1", supply_voltage=3.3)
    sensor.configure()
    assert sensor.range == sensor.lower_volt_range
    assert sensor._sensitivity == pytest.approx(sensor.lower_volt_sensitivity)


# Test DRV5056 configure with a supply voltage in the 4.5-5.5V range but not exactly 5V
def test_DRV5056_configure_near_5V():
    sensor = hall.DRV5056(sensor_num="A1", supply_voltage=4.8)
    sensor.configure()
    assert sensor.range == sensor.base_range
    assert sensor._sensitivity == pytest.approx(sensor.base_sensitivity * 4.8 / 5)


# Test DRV5056 configure with a supply voltage in the 3-3.6V range but not exactly 3.3V
def test_DRV5056_configure_near_3_3V():
    sensor = hall.DRV5056(sensor_num="A1", supply_voltage=3.5)
    sensor.configure()
    assert sensor.range == sensor.lower_volt_range
    assert sensor._sensitivity == pytest.approx(sensor.lower_volt_sensitivity * 3.5 / 3.3)


# Test DRV5056 configure with an out-of-range supply voltage
def test_DRV5056_configure_voltage_out_of_range():
    sensor = hall.DRV5056(sensor_num="A1", supply_voltage=4.0)
    with pytest.raises(ValueError):
        sensor.configure()


# Test DRV5056 configure with an unsupported sensor number
def test_DRV5056_configure_invalid_sensor_num():
    sensor = hall.DRV5056(sensor_num="B1")
    with pytest.raises(ValueError):
        sensor.configure()


# Test DRV5056 start and stop
def test_DRV5056_start_stop(sensor: hall.DRV5056):
    sensor.start()
    assert sensor.is_streaming is True

    sensor.stop()
    assert sensor.is_streaming is False


# Test DRV5056 voltage property
def test_DRV5056_voltage(sensor: hall.DRV5056):
    assert sensor.voltage == 0.0


# Test DRV5056 update
def test_DRV5056_update(sensor: hall.DRV5056):
    sensor.configure()
    sensor.update()

    expected = (sensor.voltage * sensor._V_TO_MV - sensor._QUIESCENT_OFFSET) / (
        sensor._sensitivity * (1 + (sensor._s_tc * (sensor._t_a - 25)))
    )
    assert sensor.field_strength == pytest.approx(expected)


# Test DRV5056 update logs an error when the field strength reaches the sensor's range
@patch("opensourceleg.logging.logger.LOGGER.error")
def test_DRV5056_update_out_of_range_logs_error(mock_error, sensor: hall.DRV5056):
    sensor.configure()
    sensor.update()
    sensor.range = sensor.field_strength

    sensor.update()
    mock_error.assert_called_once()


# Test DRV5056 field_mT property
def test_DRV5056_field_mT(sensor: hall.DRV5056):
    sensor.configure()
    field = sensor.field_mT
    assert field == sensor.field_strength


# Test DRV5056 data property
def test_DRV5056_data(sensor: hall.DRV5056):
    with pytest.raises(NotImplementedError):
        _ = sensor.data
