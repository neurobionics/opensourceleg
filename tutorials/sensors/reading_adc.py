from opensourceleg_rs import Logger

from opensourceleg.sensors.adc import ADS131M0x
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 500
DT = 1 / FREQUENCY

if __name__ == "__main__":
    Logger.update_log_file_configuration(
        log_directory="./logs",
        log_name="reading_adc_data.log",
    )
    clock = SoftRealtimeLoop(dt=DT)
    adc = ADS131M0x(
        tag="ADS131M0x",
        spi_bus=0,
        spi_cs=0,
        data_rate=FREQUENCY,
        clock_freq=8192000,
        num_channels=6,
        gains=[1] * 6,
        voltage_reference=1.2,
        gain_error=[0] * 6,
        offline=False,
    )
    Logger.track_functions({"Ch_Voltages": lambda: adc.data})

    with adc:
        adc.calibrate()
        for t in clock:
            adc.update()
            Logger.info(f"Time: {t}; Ch Voltages = {adc.data}")
            Logger.record()
