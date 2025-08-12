import asyncio

import pandas as pd
from moteus import Register
from opensourceleg_rs import Logger

from opensourceleg.actuators.moteus import MoteusActuator
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
    torque_data = pd.DataFrame({
        "Time": [],
        "Output_Torque": [],
        "Command_Torque": [],
    })

    clock = SoftRealtimeLoop(dt=DT)

    try:
        await mc1.start()

        mc1.set_control_mode(mode=mc1.CONTROL_MODES.TORQUE)

        mc1.set_torque_gains()

        # start_time = time.monotonic()

        await mc1.update()

        for t in clock:
            # current_time = time.monotonic()
            if t > TIME_TO_STEP:
                mc1.set_motor_torque(
                    value=0.27225,
                )
                await mc1.update()

            print("######")
            Logger.info("".join(f"Output Torque: {mc1._data[0].values[Register.TORQUE] * mc1.gear_ratio}\t"))

            print("------")
            torque_data = pd.concat(
                [
                    torque_data,
                    pd.DataFrame({
                        "Time": [t],
                        "Output_Torque": [mc1._data[0].values[Register.TORQUE] * mc1.gear_ratio],
                        "Command_Torque": [mc1._data[0].values[Register.COMMAND_FEEDFORWARD_TORQUE] * mc1.gear_ratio],
                    }),
                ],
                ignore_index=True,
            )

            await asyncio.sleep(DT)

    finally:
        torque_data.to_csv("torque_data.csv", index=False)
        await mc1.stop()


if __name__ == "__main__":
    asyncio.run(main())
