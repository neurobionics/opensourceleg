import opensourceleg.actuators.dephy_legacy as Dephy
from opensourceleg.logging.logger import LOGGER
import time

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