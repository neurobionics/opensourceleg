from opensourceleg_rs import Logger

from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 1000
DT = 1 / FREQUENCY

if __name__ == "__main__":
    Logger.update_log_file_configuration(
        log_directory="./logs",
        log_name="reading_encoder_data.log",
    )
    clock = SoftRealtimeLoop(dt=DT)
    encoder = AS5048B(
        tag="encoder1",
        bus="/dev/i2c-1",
        A1_adr_pin=False,
        A2_adr_pin=True,
        zero_position=0,
        enable_diagnostics=False,
    )
    Logger.track_functions(Encoder_Position=lambda: encoder.position)

    with encoder:
        for t in clock:
            encoder.update()
            Logger.info(f"Time: {t}; Encoder Angle: {encoder.position};")
            Logger.record()
