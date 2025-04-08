import asyncio

import numpy as np
import pandas as pd

from opensourceleg.actuators.moteus import MoteusActuator
from opensourceleg.logging.logger import LOGGER
from opensourceleg.utilities import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY


async def main():
    mc1 = MoteusActuator(
        tag="MC1",
        frequency=FREQUENCY,
        servo_id=42,
        bus_id=3,
        gear_ratio=9.0,
    )

    position_data = pd.DataFrame({
        "Time": [],
        "Output_Position": [],
        "Command_Position": [],
    })

    clock = SoftRealtimeLoop(dt=DT)

    try:
        await mc1.start()
        await mc1.update()
        mc1.set_control_mode(mode=mc1.CONTROL_MODES.POSITION)
        await mc1.set_position_gains()

        position = mc1.output_position
        # start_time = time.monotonic()

        await mc1.update()

        for t in clock:
            # current_time = time.monotonic()

            if t > TIME_TO_STEP:
                command_position = position + np.pi
                mc1.set_motor_position(
                    value=command_position,
                )
                await mc1.update()
            else:
                command_position = position

            print("######")
            LOGGER.info("".join(f"Motor Position: {mc1.output_position}\t" + f"Command_Position: {command_position}\t"))

            position_data = pd.concat(
                [
                    position_data,
                    pd.DataFrame({
                        "Time": [t],
                        "Output_Position": [mc1.output_position],
                        "Command_Position": [command_position],
                    }),
                ],
                ignore_index=True,
            )

            print("------")
            await asyncio.sleep(DT)

    finally:
        position_data.to_csv("position_data_moteus.csv", index=False)
        await mc1.stop()


if __name__ == "__main__":
    asyncio.run(main())
