from opensourceleg import OpenSourceLeg

osl = OpenSourceLeg(frequency=100)

osl.add_joint(
    name="knee",
    port="/dev/ttyACM1",
    baud_rate=230400,
    gear_ratio=9.0,
)

osl.add_tui(frequency=30)

with osl:
    osl.run_tui()
