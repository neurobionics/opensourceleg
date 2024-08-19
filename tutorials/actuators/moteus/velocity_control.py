import asyncio

import numpy as np
import pandas as pd
from moteus import Register

from opensourceleg.actuators.moteus import MoteusActuator
from opensourceleg.logging.logger import LOGGER

# import time
from opensourceleg.time import SoftRealtimeLoop

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
    velocity_data = pd.DataFrame(
        {
            "Time": [],
            "Output_Velocity": [],
            "Command_Velocity": [],
        }
    )
    clock = SoftRealtimeLoop(dt=DT)

    try:
        await mc1.start()

        mc1.set_control_mode(mode=mc1.CONTROL_MODES.VELOCITY)

        await mc1.set_velocity_gains()

        # start_time = time.monotonic()

        await mc1.update()

        for t in clock:

            # current_time = time.monotonic()

            if t > TIME_TO_STEP:

                mc1.set_motor_velocity(
                    value=np.pi * 2,
                )
                await mc1.update()

            print(f"######")
            LOGGER.info(
                "".join(
                    f"Motor Velocity: {mc1.motor_velocity}\t"
                    + f"Motor Velocity Command: {mc1._data[0].values[Register.COMMAND_VELOCITY] * 2 * np.pi / mc1.gear_ratio}\t"
                )
            )
            velocity_data = pd.concat(
                [
                    velocity_data,
                    pd.DataFrame(
                        {
                            "Time": [t],
                            "Output_Velocity": [mc1.motor_velocity],
                            "Command_Velocity": [
                                mc1._data[0].values[Register.COMMAND_VELOCITY]
                                * 2
                                * np.pi
                                / mc1.gear_ratio
                            ],
                        }
                    ),
                ],
                ignore_index=True,
            )

            print(f"------")
            await asyncio.sleep(DT)

    finally:
        velocity_data.to_csv("velocity_data.csv", index=False)
        await mc1.stop()


if __name__ == "__main__":
    asyncio.run(main())
