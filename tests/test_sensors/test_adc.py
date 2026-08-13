import sys
from unittest.mock import MagicMock, Mock, patch

import pytest

from opensourceleg.sensors import adc as adc_module
from opensourceleg.sensors.adc import ADS114S0x, ChannelConfig


# inmport fake spidev
@pytest.fixture(autouse=True)
def fake_spidev(monkeypatch):
    module = MagicMock()
    monkeypatch.setitem(sys.modules, "spidev", module)
    return module


@pytest.fixture(autouse=True)
def reset_class_state():
    """
    _register_map and _crc_lookup_table are CLASS attributes, so mutations leak
    between tests (and between real instances -- see notes). Snapshot/restore.
    """
    crc = list(ADS114S0x._crc_lookup_table)
    initialized = ADS114S0x._initialized
    yield
    ADS114S0x._crc_lookup_table[:] = crc
    ADS114S0x._initialized = initialized


@pytest.fixture
def mock_spi():
    spi = Mock()
    spi.xfer2 = Mock(return_value=[0, 0, 0])
    return spi


@pytest.fixture
def adc(mock_spi):
    """A constructed ADS114S0x with SPI and DRDY replaced by mocks."""
    with patch.object(adc_module, "DigitalInputDevice") as mock_drdy:
        device = ADS114S0x(tag="TestADC", spi_bus=0, spi_cs=0, data_rate=1000, pga_gain=1, drdy=16)
    device._spi = mock_spi
    device._drdy_mock = mock_drdy
    return device


# Construction / basic properties
def test_init_defaults(mock_spi):
    with patch.object(adc_module, "DigitalInputDevice") as mock_drdy:
        device = ADS114S0x()

    assert device.tag == "ADS114S08"
    assert device._spi_bus == 0
    assert device._spi_cs == 0
    assert device._data_rate == 400
    assert device._pga_gain == 1
    assert device._voltage_reference == pytest.approx(2.5)
    assert device._streaming is False
    assert device._channels == {}
    mock_drdy.assert_called_once_with(16, pull_up=False)


def test_repr(adc):
    assert repr(adc) == "ADS114S0x"


def test_is_streaming_reflects_device_state(adc):
    assert adc.is_streaming is False
    adc._set_device_state(1)
    assert adc.is_streaming is True
    adc._set_device_state(0)
    assert adc.is_streaming is False


def test_set_device_state_ignores_unknown_values(adc):
    adc._set_device_state(1)
    adc._set_device_state(7)  # not 0 or 1 -> no change
    assert adc.is_streaming is True


def test_init_exits_when_spidev_missing(monkeypatch):
    monkeypatch.delitem(sys.modules, "spidev", raising=False)
    real_import = __builtins__["__import__"] if isinstance(__builtins__, dict) else __builtins__.__import__

    def blocked_import(name, *args, **kwargs):
        if name == "spidev":
            raise ImportError("no spidev")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr("builtins.__import__", blocked_import)
    with patch.object(adc_module, "DigitalInputDevice"), pytest.raises(SystemExit):
        ADS114S0x(offline=False)


# Register map helpers
def test_restore_register_defaults(adc):
    adc._register_map[:] = [0xFF] * ADS114S0x._NUM_REGISTERS
    adc.restore_register_defaults()

    assert adc.get_register_value(ADS114S0x._REG_ADDR_ID) == ADS114S0x._ID_DEFAULT
    assert adc.get_register_value(ADS114S0x._REG_ADDR_STATUS) == ADS114S0x._STATUS_DEFAULT
    assert adc.get_register_value(ADS114S0x._REG_ADDR_INPMUX) == ADS114S0x._INPMUX_DEFAULT
    assert adc.get_register_value(ADS114S0x._REG_ADDR_PGA) == ADS114S0x._PGA_DEFAULT
    assert adc.get_register_value(ADS114S0x._REG_ADDR_DATARATE) == ADS114S0x._DATARATE_DEFAULT
    assert adc.get_register_value(ADS114S0x._REG_ADDR_REF) == ADS114S0x._REF_DEFAULT
    assert adc.get_register_value(ADS114S0x._REG_ADDR_FSCAL1) == ADS114S0x._FSCAL1_DEFAULT
    assert adc.get_register_value(ADS114S0x._REG_ADDR_GPIOCON) == ADS114S0x._GPIOCON_DEFAULT


def test_get_register_value_out_of_range(adc):
    with pytest.raises(ValueError):
        adc.get_register_value(ADS114S0x._NUM_REGISTERS + 1)


def test_is_sendstat_and_is_crc_set(adc):
    adc._register_map[ADS114S0x._REG_ADDR_SYS] = 0x00
    assert adc.is_sendstat_set() is False
    assert adc.is_crc_set() is False

    adc._register_map[ADS114S0x._REG_ADDR_SYS] = ADS114S0x._ADS_SENDSTATUS_MASK
    assert adc.is_sendstat_set() is True
    assert adc.is_crc_set() is False

    adc._register_map[ADS114S0x._REG_ADDR_SYS] = ADS114S0x._ADS_CRC_MASK
    assert adc.is_sendstat_set() is False
    assert adc.is_crc_set() is True


# Single / multiple register reads and writes
def test_read_single_register(adc, mock_spi):
    mock_spi.xfer2.return_value = [0x00, 0x00, 0x42]

    value = adc.read_single_register(ADS114S0x._REG_ADDR_INPMUX)

    assert value == 0x42
    assert adc.get_register_value(ADS114S0x._REG_ADDR_INPMUX) == 0x42
    mock_spi.xfer2.assert_called_once_with([ADS114S0x._OPCODE_RREG | ADS114S0x._REG_ADDR_INPMUX, 0, 0])


def test_read_single_register_out_of_range(adc):
    with pytest.raises(ValueError):
        adc.read_single_register(ADS114S0x._NUM_REGISTERS + 1)


def test_write_single_register(adc, mock_spi):
    adc.write_single_register(ADS114S0x._REG_ADDR_PGA, 0x1AB)  # >8 bits, should be masked

    assert adc.get_register_value(ADS114S0x._REG_ADDR_PGA) == 0xAB
    mock_spi.xfer2.assert_called_once_with([ADS114S0x._OPCODE_WREG | ADS114S0x._REG_ADDR_PGA, 0, 0xAB])


def test_write_single_register_out_of_range(adc):
    with pytest.raises(ValueError):
        adc.write_single_register(ADS114S0x._NUM_REGISTERS + 1, 0x00)


def test_read_multiple_registers(adc, mock_spi):
    # 2 command bytes then 3 data bytes
    mock_spi.xfer2.return_value = [0x00, 0x00, 0x11, 0x22, 0x33]

    adc.read_multiple_registers(start_address=0x02, count=3)

    assert adc.get_register_value(0x02) == 0x11
    assert adc.get_register_value(0x03) == 0x22
    assert adc.get_register_value(0x04) == 0x33
    mock_spi.xfer2.assert_called_once_with([ADS114S0x._OPCODE_RREG | 0x02, 2, 0, 0, 0])


def test_read_multiple_registers_out_of_range(adc):
    with pytest.raises(ValueError):
        adc.read_multiple_registers(start_address=0x10, count=10)


def test_write_multiple_registers(adc, mock_spi):
    adc.write_multiple_registers(start_address=0x02, count=3, reg_data=[0x11, 0x22, 0x133])

    assert adc.get_register_value(0x02) == 0x11
    assert adc.get_register_value(0x03) == 0x22
    assert adc.get_register_value(0x04) == 0x33
    mock_spi.xfer2.assert_called_once_with([ADS114S0x._OPCODE_WREG | 0x02, 2, 0x11, 0x22, 0x33])


def test_write_multiple_registers_out_of_range(adc):
    with pytest.raises(ValueError):
        adc.write_multiple_registers(start_address=0x10, count=10, reg_data=[0] * 10)


def test_write_multiple_registers_none_data(adc):
    with pytest.raises(ValueError):
        adc.write_multiple_registers(start_address=0x00, count=1, reg_data=None)


def test_register_map_is_isolated_between_instances(mock_spi):
    with patch.object(adc_module, "DigitalInputDevice"):
        a = ADS114S0x(tag="A")
        b = ADS114S0x(tag="B")
    a._spi = mock_spi
    a.write_single_register(ADS114S0x._REG_ADDR_PGA, 0x07)
    assert b.get_register_value(ADS114S0x._REG_ADDR_PGA) == 0x00


# Commands
@pytest.mark.parametrize("opcode", [ADS114S0x._OPCODE_RREG, ADS114S0x._OPCODE_WREG])
def test_send_command_rejects_register_opcodes(adc, opcode):
    with pytest.raises(ValueError):
        adc.send_command(opcode)


def test_send_command_writes_byte(adc, mock_spi):
    mock_spi.xfer2.return_value = [0x7F]
    adc.send_command(ADS114S0x._OPCODE_START)
    mock_spi.xfer2.assert_called_once_with([ADS114S0x._OPCODE_START])


def test_send_command_reset_restores_defaults(adc, mock_spi):
    mock_spi.xfer2.return_value = [0x00]
    adc._register_map[:] = [0xFF] * ADS114S0x._NUM_REGISTERS

    with patch.object(adc, "delay_us") as mock_delay:
        adc.send_command(ADS114S0x._OPCODE_RESET)

    mock_delay.assert_called_once_with(ADS114S0x._DELAY_4096TCLK)
    assert adc.get_register_value(ADS114S0x._REG_ADDR_STATUS) == ADS114S0x._STATUS_DEFAULT


@pytest.mark.parametrize(
    ("method", "opcode"),
    [
        ("send_start", ADS114S0x._OPCODE_START),
        ("send_stop", ADS114S0x._OPCODE_STOP),
        ("send_wakeup", ADS114S0x._OPCODE_WAKEUP),
        ("send_powerdown", ADS114S0x._OPCODE_POWERDOWN),
        ("reset", ADS114S0x._OPCODE_RESET),
    ],
)
def test_command_wrappers(adc, method, opcode):
    with patch.object(adc, "send_command") as mock_send:
        getattr(adc, method)()
    mock_send.assert_called_once_with(opcode)


def test_start_conversions_wakes_then_starts(adc):
    with patch.object(adc, "send_wakeup") as wake, patch.object(adc, "send_start") as start:
        adc.start_conversions()
    wake.assert_called_once()
    start.assert_called_once()


# SPI plumbing
def test_init_spi_configures_port(adc, mock_spi):
    adc.init_spi()
    mock_spi.open.assert_called_once_with(0, 0)
    assert mock_spi.max_speed_hz == ADS114S0x._SPI_SPEED
    assert mock_spi.mode == 0b01
    assert mock_spi.bits_per_word == 8


def test_init_spi_raises_without_spi(adc):
    adc._spi = None
    with pytest.raises(RuntimeError):
        adc.init_spi()


def test_spi_send_receive_arrays_truncates_to_byte_length(adc, mock_spi):
    mock_spi.xfer2.return_value = [1, 2]
    result = adc.spi_send_receive_arrays([1, 2, 3, 4], 2)
    mock_spi.xfer2.assert_called_once_with([1, 2])
    assert result == [1, 2]


def test_spi_send_receive_arrays_raises_without_spi(adc):
    adc._spi = None
    with pytest.raises(RuntimeError):
        adc.spi_send_receive_arrays([0], 1)


def test_spi_send_receive_byte_masks_input(adc, mock_spi):
    mock_spi.xfer2.return_value = [0x5A]
    assert adc.spi_send_receive_byte(0x1FF) == 0x5A
    mock_spi.xfer2.assert_called_once_with([0xFF])


def test_spi_send_receive_byte_raises_without_spi(adc):
    adc._spi = None
    with pytest.raises(RuntimeError):
        adc.spi_send_receive_byte(0x00)


def test_cleanup_closes_and_clears_spi(adc, mock_spi):
    adc.cleanup()
    mock_spi.close.assert_called_once()
    assert adc._spi is None
    adc.cleanup()  # idempotent


def test_delay_us(adc):
    with patch.object(adc_module, "sleep") as mock_sleep:
        adc.delay_us(2500)
    mock_sleep.assert_called_once_with(pytest.approx(0.0025))


def test_wait_for_drdy_htol_sleeps_one_conversion_period(adc):
    with patch.object(adc_module, "sleep") as mock_sleep:
        assert adc.wait_for_drdy_htol(timeout_ms=200) is True
    mock_sleep.assert_called_once_with(pytest.approx(1.5 / adc._data_rate))


# start / stop / update
def test_start_sequence(adc):
    with (
        patch.object(adc, "init_spi") as init_spi,
        patch.object(adc, "delay_us") as delay,
        patch.object(adc, "reset") as reset,
        patch.object(adc, "restore_register_defaults") as restore,
        patch.object(adc, "write_single_register") as write,
    ):
        adc.start()

    init_spi.assert_called_once()
    delay.assert_called_once_with(ADS114S0x._DELAY_2p2MS)
    reset.assert_called_once()
    restore.assert_called_once()
    write.assert_called_once_with(ADS114S0x._REG_ADDR_STATUS, 0x00)
    assert adc.is_streaming is True


def test_stop_sequence(adc):
    adc._set_device_state(1)
    with patch.object(adc, "send_stop") as stop, patch.object(adc, "cleanup") as cleanup:
        adc.stop()
    stop.assert_called_once()
    cleanup.assert_called_once()


def test_update_stores_millivolts(adc):
    assert adc.data is None
    with (
        patch.object(adc, "_ready_to_read", return_value=True),
        patch.object(adc, "_read_data_millivolts", return_value=[1.0, 2.0]),
    ):
        adc.update()
    assert adc.data == [1.0, 2.0]


def test_update_raises_when_never_ready(adc):
    with (
        patch.object(adc, "_ready_to_read", return_value=False),
        patch.object(adc_module, "sleep"),
        pytest.raises(RuntimeError),
    ):
        adc.update()


def test_ready_to_read(adc):
    with patch.object(adc, "read_single_register", return_value=0x00):
        assert adc._ready_to_read() is True
    with patch.object(adc, "read_single_register", return_value=ADS114S0x._ADS_nRDY_MASK):
        assert adc._ready_to_read() is False


def test_read_data_millivolts_without_channels(adc):
    assert adc._read_data_millivolts() is None


def test_read_data_millivolts_reads_each_channel(adc):
    adc._channels = {
        "a": ChannelConfig(name="a", ain_pos_code=ADS114S0x._ADS_P_AIN0),
        "b": ChannelConfig(name="b", ain_pos_code=ADS114S0x._ADS_P_AIN3, postprocess=lambda mv: mv * 2),
    }

    with (
        patch.object(adc, "set_mux_single_ended") as set_mux,
        patch.object(adc, "discard_settling_reads") as discard,
        patch.object(adc, "start_conversions") as start_conv,
        patch.object(adc, "wait_and_read_code16", return_value=(3277, None)),
    ):
        row = adc._read_data_millivolts()

    expected_mv = (3277 * 2.5 / 32768.0) * 1000
    assert row[0] == pytest.approx(expected_mv)
    assert row[1] == pytest.approx(expected_mv * 2)  # postprocess applied
    assert set_mux.call_count == 2
    assert discard.call_count == 2
    assert start_conv.call_count == 2


# Conversion data parsing
def _configure_plain_read(adc):
    """No STATUS byte, no CRC."""
    adc._register_map[ADS114S0x._REG_ADDR_SYS] = 0x00


def test_read_converted_data_positive(adc, mock_spi):
    _configure_plain_read(adc)
    mock_spi.xfer2.return_value = [0x12, 0x34, 0x00]

    code16, status = adc.read_converted_data()

    assert code16 == 0x1234
    assert status is None
    mock_spi.xfer2.assert_called_once_with([0, 0, 0])


def test_read_converted_data_negative_sign_extends(adc, mock_spi):
    _configure_plain_read(adc)
    mock_spi.xfer2.return_value = [0xFF, 0xFF, 0x00]

    code16, _ = adc.read_converted_data()

    assert code16 == -1


def test_read_converted_data_full_scale(adc, mock_spi):
    _configure_plain_read(adc)
    mock_spi.xfer2.return_value = [0x80, 0x00, 0x00]

    code16, _ = adc.read_converted_data()

    assert code16 == -32768


def test_read_converted_data_command_mode_sends_rdata(adc, mock_spi):
    _configure_plain_read(adc)
    mock_spi.xfer2.return_value = [0x00, 0x12, 0x34, 0x00]

    code16, status = adc.read_converted_data(mode=ADS114S0x.ReadMode.COMMAND)

    assert mock_spi.xfer2.call_args[0][0][0] == ADS114S0x._OPCODE_RDATA
    assert code16 == 0x1234
    assert status is None


def test_read_converted_data_with_status_byte(adc, mock_spi):
    adc._register_map[ADS114S0x._REG_ADDR_SYS] = ADS114S0x._ADS_SENDSTATUS_MASK
    mock_spi.xfer2.return_value = [0x80, 0x12, 0x34, 0x00]

    code16, status = adc.read_converted_data()

    assert status == 0x80
    assert code16 == 0x1234


def test_read_converted_data_crc_error_raises(adc, mock_spi):
    adc._register_map[ADS114S0x._REG_ADDR_SYS] = ADS114S0x._ADS_CRC_MASK
    mock_spi.xfer2.return_value = [0x12, 0x34, 0x00, 0xFF]

    with patch.object(adc, "get_crc", return_value=1), pytest.raises(ValueError):
        adc.read_converted_data()


def test_wait_and_read_code16_success(adc):
    with (
        patch.object(adc, "wait_for_drdy_htol", return_value=True),
        patch.object(adc, "read_converted_data", return_value=(1234, None)) as read,
    ):
        code16, status = adc.wait_and_read_code16(timeout_ms=50)

    assert (code16, status) == (1234, None)
    read.assert_called_once_with(mode=ADS114S0x.ReadMode.DIRECT)


def test_wait_and_read_code16_timeout(adc):
    with patch.object(adc, "wait_for_drdy_htol", return_value=False), pytest.raises(TimeoutError):
        adc.wait_and_read_code16(timeout_ms=50)


def test_discard_settling_reads(adc):
    with patch.object(adc, "send_start") as start, patch.object(adc, "wait_and_read_code16") as read:
        adc.discard_settling_reads(n=3, timeout_ms=100)
    assert start.call_count == 3
    assert read.call_count == 3


def test_discard_settling_reads_negative_n_is_noop(adc):
    with patch.object(adc, "send_start") as start:
        adc.discard_settling_reads(n=-5)
    start.assert_not_called()


# Voltage conversion
@pytest.mark.parametrize(
    ("code16", "gain", "expected"),
    [
        (0, 1, 0.0),
        (32767, 1, 32767 * 2.5 / 32768.0),
        (-32768, 1, -2.5),
        (16384, 1, 1.25),
        (16384, 2, 0.625),
        (16384, 128, 1.25 / 128),
    ],
)
def test_code16_to_volts(adc, code16, gain, expected):
    adc._pga_gain = gain
    assert adc.code16_to_volts(code16) == pytest.approx(expected)


def test_code16_to_volts_uses_voltage_reference(adc):
    adc._voltage_reference = 5.0
    assert adc.code16_to_volts(16384) == pytest.approx(2.5)


# MUX
def test_set_mux_single_ended_defaults_to_aincom(adc):
    with patch.object(adc, "write_single_register") as write:
        adc.set_mux_single_ended(ADS114S0x._ADS_P_AIN3)
    write.assert_called_once_with(ADS114S0x._REG_ADDR_INPMUX, 0x30 | ADS114S0x._ADS_N_AINCOM)


def test_set_mux_single_ended_with_negative_code(adc):
    with patch.object(adc, "write_single_register") as write:
        adc.set_mux_single_ended(ADS114S0x._ADS_P_AIN5, ADS114S0x._ADS_N_AIN2)
    write.assert_called_once_with(ADS114S0x._REG_ADDR_INPMUX, 0x52)


# CRC
def test_calculate_crc_of_zero_is_zero(adc):
    assert adc._calculate_crc([0x00], 1) == 0x00


def test_lookup_matches_calculate(adc):
    adc.init_crc()
    for message in ([0x01], [0xFF, 0x00], [0x12, 0x34, 0x56], [0xDE, 0xAD, 0xBE, 0xEF]):
        assert adc._lookup_crc(message, len(message)) == adc._calculate_crc(message, len(message))


def test_get_crc_detects_no_error_when_crc_appended(adc):
    message = [0x12, 0x34, 0x56]
    crc = adc.get_crc(message, len(message))
    assert adc.get_crc([*message, crc], len(message) + 1) == 0


def test_get_crc_detects_corruption(adc):
    message = [0x12, 0x34, 0x56]
    crc = adc.get_crc(message, len(message))
    corrupted = [0x12, 0x35, 0x56, crc]
    assert adc.get_crc(corrupted, 4) != 0


def test_init_crc_builds_lookup_table(adc):
    assert adc._initialized is False  # reads through to the class attr
    adc.init_crc()
    assert adc._initialized is True
    assert "_initialized" in adc.__dict__  # instance attr now shadows the class one
    assert len(ADS114S0x._crc_lookup_table) == 256
    assert ADS114S0x._crc_lookup_table[0] == 0x00


def test_get_crc_initializes_table_lazily(adc):
    ADS114S0x._initialized = False
    ADS114S0x._crc_lookup_table[:] = [0] * 256
    result = adc.get_crc([0x12, 0x34], 2)
    assert result == adc._calculate_crc([0x12, 0x34], 2)


# adc_configure_common
def test_adc_configure_common_bypasses_pga_at_gain_1(adc):
    adc._pga_gain = 1
    adc._data_rate = 1000
    with patch.object(adc, "write_single_register") as write:
        adc.adc_configure_common()

    write.assert_any_call(ADS114S0x._REG_ADDR_PGA, ADS114S0x._ADS_PGA_BYPASS | ADS114S0x._ADS_GAIN_1)


def test_adc_configure_common_enables_pga(adc):
    adc._pga_gain = 32
    adc._data_rate = 1000
    with patch.object(adc, "write_single_register") as write:
        adc.adc_configure_common()

    write.assert_any_call(ADS114S0x._REG_ADDR_PGA, ADS114S0x._ADS_PGA_ENABLED | ADS114S0x._ADS_GAIN_32)


def test_adc_configure_common_datarate_register(adc):
    adc._data_rate = 1000
    with patch.object(adc, "write_single_register") as write:
        adc.adc_configure_common(single_shot=True, filter_low_latency=True)

    expected = ADS114S0x._ADS_CONVMODE_SS | ADS114S0x._ADS_FILTERTYPE_LL | ADS114S0x._ADS_DR_1000
    write.assert_any_call(ADS114S0x._REG_ADDR_DATARATE, expected)


def test_adc_configure_common_continuous_mode(adc):
    adc._data_rate = 100
    with patch.object(adc, "write_single_register") as write:
        adc.adc_configure_common(single_shot=False, filter_low_latency=False)

    write.assert_any_call(ADS114S0x._REG_ADDR_DATARATE, ADS114S0x._ADS_CONVMODE_CONT | ADS114S0x._ADS_DR_100)


def test_adc_configure_common_sys_register(adc):
    adc._data_rate = 1000
    with patch.object(adc, "write_single_register") as write, patch.object(adc, "init_crc") as init_crc:
        adc.adc_configure_common(enable_crc=True, enable_status_byte=True)

    expected = ADS114S0x._ADS_SYS_MON_OFF | ADS114S0x._ADS_CRC_ENABLE | ADS114S0x._ADS_SENDSTATUS_ENABLE
    write.assert_any_call(ADS114S0x._REG_ADDR_SYS, expected)
    init_crc.assert_called_once()


def test_adc_configure_common_no_crc_skips_init(adc):
    adc._data_rate = 1000
    with patch.object(adc, "write_single_register"), patch.object(adc, "init_crc") as init_crc:
        adc.adc_configure_common(enable_crc=False)
    init_crc.assert_not_called()


def test_adc_configure_common_rejects_bad_gain(adc):
    adc._pga_gain = 3
    adc._data_rate = 1000
    with patch.object(adc, "write_single_register"), pytest.raises(ValueError, match="Unsupported gain"):
        adc.adc_configure_common()


def test_adc_configure_common_rejects_bad_data_rate(adc):
    adc._data_rate = 500  # NOTE: this is the constructor default -- see notes
    with patch.object(adc, "write_single_register"), pytest.raises(ValueError, match="Unsupported frequency"):
        adc.adc_configure_common()


def test_adc_configure_common_writes_ref_register(adc):
    adc._data_rate = 1000
    with patch.object(adc, "write_single_register") as write:
        adc.adc_configure_common(vref_select_reg=ADS114S0x._ADS_REFSEL_INT)
    write.assert_any_call(ADS114S0x._REG_ADDR_REF, ADS114S0x._ADS_REFSEL_INT)


# ChannelConfig
def test_channel_config_defaults():
    ch = ChannelConfig(name="knee")
    assert ch.name == "knee"
    assert ch.ain_pos_code == ADS114S0x._ADS_P_AIN0
    assert ch.ain_neg_code == ADS114S0x._ADS_N_AINCOM
    assert ch.postprocess is None
    assert ch.units == "V"


def test_channel_config_postprocess_is_callable():
    ch = ChannelConfig(name="scaled", postprocess=lambda mv: mv / 10)
    assert ch.postprocess(100.0) == pytest.approx(10.0)
