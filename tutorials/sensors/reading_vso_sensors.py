import time

import numpy as np

from opensourceleg.actuators.brushed import MaxonActuator
from opensourceleg.logging import LOGGER
from opensourceleg.logging.logger import Logger
from opensourceleg.robots.vso import VSO
from opensourceleg.sensors.adc import ADS114S0x, ChannelConfig
from opensourceleg.sensors.base import SensorBase
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.sensors.hall import DRV5056
from opensourceleg.utilities.softrealtimeloop import SoftRealtimeLoop

FREQUENCY = 200  # control/read loop rate (Hz)
READ_FREQUENCY = 10  # console readout rate (Hz)
OFFLINE = False
SIDE = -1  # 1: left-leg lateral encoder (-1 if medial); -1: right-leg lateral (1 if medial)
ENCODER_ALPHA = 0.15  # EMA smoothing factor (~5 Hz cutoff at 200 Hz)


def _hall_voltage(adc: ADS114S0x) -> float:
    """Return the first ADC channel reading in volts, or 0.0 if unavailable.

    Args:
        adc: The ADS114S0x instance whose latest data to read.

    Returns:
        float: Hall channel voltage in volts (ADC data is in millivolts).
    """
    data = getattr(adc, "_data", None)
    if not data:
        return 0.0
    return data[0] / 1000.0


def read_sensors(data_logger: Logger) -> None:
    """Configure the VSO sensors and stream their readings to the data logger.

    Args:
        data_logger: Logger used to record the tracked sensor values to CSV.
    """
    # Define the VSO robot (sensors only)
    vso = VSO[MaxonActuator, SensorBase](
        tag="variableStiffnessOrthosis",
        actuators={},
        sensors={
            "adc": ADS114S0x(offline=OFFLINE, tag="adc", spi_bus=1, data_rate=2000, drdy=16, voltage_reference=1.65),
            "hallEffect_sensor": DRV5056(
                offline=OFFLINE, tag="hall_effect", sensor_num="A1", t_a=23, supply_voltage=3.3
            ),
            "ankle_encoder": AS5048B(
                offline=OFFLINE,
                tag="joint_encoder_ankle",
                bus="/dev/i2c-3",
                A1_adr_pin=False,
                A2_adr_pin=True,
                zero_position=0,
                enable_diagnostics=False,
            ),
        },
    )
    LOGGER.info("Finished setting up VSO.")

    # ADC and Hall channel configuration
    adc = vso.sensors.get("adc")
    if adc is None:
        LOGGER.error("No ADC found — cannot read the Hall effect sensor. Aborting.")
        return

    adc.adc_configure_common(single_shot=True, filter_low_latency=True)

    hall = vso.sensors.get("hallEffect_sensor")
    if hall is not None:
        hall.configure()
        adc._channels["hall_drv5056_ain3"] = ChannelConfig(
            name="hall_drv5056_ain3",
            ain_pos_code=adc._ADS_P_AIN3,  # set to the AIN pin the Hall output is wired to
            postprocess=None,
            units="V",
        )

    # Ankle encoder offset (referenced to the unloaded equilibrium angle)
    calib_offset = 0.0
    ankle_sensor = vso.sensors.get("ankle_encoder")
    if ankle_sensor is not None:
        LOGGER.info("Capturing unloaded equilibrium angle. Waiting for ankle encoder warmup.")
        time.sleep(2)  # warmup period for the ankle encoder to stabilize
        ankle_sensor.update()
        calib_offset = SIDE * np.rad2deg(ankle_sensor.position)
        LOGGER.info(f"Ankle encoder offset calibrated. calib_offset={calib_offset:.4f} deg")
    else:
        LOGGER.warning("No ankle encoder found. Angle will not be captured.")

    # Logged values
    angle = 0.0
    data_logger.track_function(lambda: angle, name="ankleEncoderPos")
    data_logger.track_function(lambda: _hall_voltage(adc), name="hallEffect_data")

    readout_interval = FREQUENCY // READ_FREQUENCY

    with vso:
        loop = SoftRealtimeLoop(dt=1 / FREQUENCY)
        loop_count = 0
        t_start = time.monotonic()

        for loop_count in enumerate(loop):
            vso.update()

            # Ankle angle: raw reading, referenced to calib_offset, then EMA-smoothed.
            angle_raw = SIDE * np.rad2deg(vso.sensors["ankle_encoder"].position) - calib_offset
            angle = angle_raw if loop_count == 0 else ENCODER_ALPHA * angle_raw + (1 - ENCODER_ALPHA) * angle

            hall_data = _hall_voltage(adc)

            data_logger.update()
            data_logger.flush_buffer()

            if loop_count % readout_interval == 0:
                elapsed = time.monotonic() - t_start
                print(
                    f"\r  t={elapsed:6.1f}s" f"  angle={angle:+7.2f}°" f"  hall={hall_data:+.4f} V",
                    end="",
                    flush=True,
                )


if __name__ == "__main__":
    data_logger = Logger(
        log_path="./logs",
        file_name="read_sensor",
    )
    read_sensors(data_logger)
