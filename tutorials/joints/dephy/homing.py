import time

import opensourceleg.actuators.dephy as Dephy
from opensourceleg.logging.logger import LOGGER

actpack = Dephy.DephyActpack(
    port="/dev/ttyACM0",
    gear_ratio=9.0,
)
with actpack:
    try:
        actpack.home()
        actpack.make_encoder_map(overwrite=True)
    except KeyboardInterrupt:
        exit()
