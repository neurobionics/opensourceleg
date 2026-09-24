from unittest.mock import MagicMock, patch

import pytest

from opensourceleg.sensors.encoderCounter import LS7366R


@pytest.fixture
def mock_spi():
    """
    Patch spidev.SpiDev and the init-time sleep so an LS7366R can be built and
    exercised without any real SPI bus. Yields the mocked SpiDev instance.
    """
    with (
        patch("opensourceleg.sensors.encoderCounter.spidev.SpiDev") as mock_spidev_cls,
        patch("opensourceleg.sensors.encoderCounter.sleep"),
    ):
        spi_instance = MagicMock()
        mock_spidev_cls.return_value = spi_instance
        yield spi_instance


@pytest.fixture
def encoder_counter(mock_spi):
    """A default LS7366R (4-byte mode) backed by the mocked SPI device."""
    return LS7366R()


# test initialize/configuration
def test_ls7366r_init_defaults(encoder_counter: LS7366R, mock_spi: MagicMock):
    assert encoder_counter.counter_size == 4
    assert encoder_counter.max_val == 4294967295
    # SPI device opened on the default bus/chip-select and clocked correctly.
    mock_spi.open.assert_called_once_with(0, 0)
    assert mock_spi.max_speed_hz == 1000000


def test_ls7366r_init_custom_params(mock_spi: MagicMock):
    enc = LS7366R(csx=1, clk=500000, byte_mode=2, spi_bus=0, max_val=65535)
    assert enc.counter_size == 2
    assert enc.max_val == 65535
    # spi.open is called as open(spi_bus, csx).
    mock_spi.open.assert_called_once_with(0, 1)
    assert mock_spi.max_speed_hz == 500000


def test_ls7366r_init_configures_modes(mock_spi: MagicMock):
    LS7366R(byte_mode=4)
    # Mode 0: quadrature count mode write.
    mock_spi.xfer2.assert_any_call([LS7366R.WRITE_MODE0, LS7366R.QUADRATURE_COUNT_MODE])
    # Mode 1: byte-mode write (4-byte -> FOURBYTE_COUNTER == 0x00).
    mock_spi.xfer2.assert_any_call([LS7366R.WRITE_MODE1, LS7366R.CounterConfig.FOURBYTE_COUNTER])


# test clear_counter
def test_clear_counter(encoder_counter: LS7366R, mock_spi: MagicMock):
    mock_spi.reset_mock()  # discard the init-time transactions
    result = encoder_counter.clear_counter()
    assert result == "[DONE]"
    mock_spi.xfer2.assert_called_once_with([LS7366R.CLEAR_COUNTER])


# test clear_status
def test_clear_status(encoder_counter: LS7366R, mock_spi: MagicMock):
    mock_spi.reset_mock()
    result = encoder_counter.clear_status()
    assert result == "[DONE]"
    mock_spi.xfer2.assert_called_once_with([LS7366R.CLEAR_STATUS])


# test read_counter
def test_read_counter_positive(encoder_counter: LS7366R, mock_spi: MagicMock):
    # 4-byte payload -> 0x0000012C == 300; leading count byte != 255 -> positive.
    mock_spi.xfer2.return_value = [0x00, 0x00, 0x00, 0x01, 0x2C]
    assert encoder_counter.read_counter() == 300
    assert encoder_counter.encoder_count == 300


def test_read_counter_negative(encoder_counter: LS7366R, mock_spi: MagicMock):
    # Leading count byte == 255 triggers the signed wrap: 0xFFFFFFFF -> -1.
    mock_spi.xfer2.return_value = [0x00, 0xFF, 0xFF, 0xFF, 0xFF]
    assert encoder_counter.read_counter() == -1


def test_read_counter_two_byte_mode(mock_spi: MagicMock):
    enc = LS7366R(byte_mode=2)
    # 2-byte payload -> 0x012C == 300.
    mock_spi.xfer2.return_value = [0x00, 0x01, 0x2C]
    assert enc.read_counter() == 300


def test_read_counter_transaction_length(encoder_counter: LS7366R, mock_spi: MagicMock):
    mock_spi.reset_mock()
    mock_spi.xfer2.return_value = [0x00, 0x00, 0x00, 0x00, 0x00]
    encoder_counter.read_counter()
    # READ_COUNTER command byte followed by counter_size zero placeholders.
    mock_spi.xfer2.assert_called_once_with([LS7366R.READ_COUNTER, 0, 0, 0, 0])


# test read_status
def test_read_status(encoder_counter: LS7366R, mock_spi: MagicMock):
    mock_spi.reset_mock()
    mock_spi.xfer2.return_value = [0x00, 0x42]
    assert encoder_counter.read_status() == 0x42
    mock_spi.xfer2.assert_called_once_with([LS7366R.READ_STATUS, 0xFF])


# test start
def test_start_is_noop(encoder_counter: LS7366R):
    assert encoder_counter.start() is None


def test_update_reads_counter(encoder_counter: LS7366R, mock_spi: MagicMock):
    mock_spi.xfer2.return_value = [0x00, 0x00, 0x00, 0x00, 0x2A]
    encoder_counter.update()
    assert encoder_counter.encoder_count == 0x2A


def test_count_property(encoder_counter: LS7366R, mock_spi: MagicMock):
    mock_spi.xfer2.return_value = [0x00, 0x00, 0x00, 0x01, 0x00]
    assert encoder_counter.count == 256


def test_close(encoder_counter: LS7366R, mock_spi: MagicMock):
    encoder_counter.close()
    mock_spi.close.assert_called_once()
    assert encoder_counter.spi is None


def test_stop_closes_connection(encoder_counter: LS7366R, mock_spi: MagicMock):
    encoder_counter.stop()
    mock_spi.close.assert_called_once()
    assert encoder_counter.spi is None


# Unsupported properties
def test_data_not_implemented(encoder_counter: LS7366R):
    with pytest.raises(NotImplementedError):
        _ = encoder_counter.data


def test_is_streaming_not_implemented(encoder_counter: LS7366R):
    with pytest.raises(NotImplementedError):
        _ = encoder_counter.is_streaming


# test repr (inherited from Encoder_counterCounterBase)
def test_repr(encoder_counter: LS7366R):
    assert repr(encoder_counter) == "EncoderCounterBase"
