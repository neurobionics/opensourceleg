from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.time import SoftRealtimeLoop

FREQUENCY = 1000
DT = 1 / FREQUENCY

if __name__ == "__main__":
    encoder_logger = Logger(
        log_path="./logs",
        file_name="reading_encoder_data",
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
    encoder_logger.track_variable(lambda: encoder.position, "Encoder Position")

    with encoder:
        for t in clock:
            encoder.update()
            encoder_logger.info(f"Time: {t}; Encoder Angle: {encoder.position};")
            encoder_logger.update()
