from osl import OpenSourceLeg
from state_machine import Event, State

BODY_WEIGHT = 60.0  # kg

def estance_to_lstance(osl: OpenSourceLeg):
    if osl.loadcell.fz < -0.0167 * BODY_WEIGHT:
        return True
    else:
        return False


def lstance_to_eswing(osl: OpenSourceLeg):
    if osl.loadcell.fz > -0.00267 * BODY_WEIGHT:
        return True
    else:
        return False


def eswing_to_lswing(osl: OpenSourceLeg):
    if (
        osl.knee.output_position > 60
        and osl.knee.output_velocity < 0.135
    ):
        return True
    else:
        return False


def eswing_to_estance(osl: OpenSourceLeg):
    if osl.loadcell.fz < -0.02 * BODY_WEIGHT:
        return True
    else:
        False


def lswing_to_estance(osl: OpenSourceLeg):
    if osl.loadcell.fz < -0.02 * BODY_WEIGHT or osl.knee.output_position < 30:
        return True
    else:
        return False


def main():
    osl = OpenSourceLeg(frequency=200)
    osl.units["position"] = "deg"

    osl.add_joint(
        name="knee",
        gear_ratio=41.4999,
    )

    osl.add_loadcell(
        dephy_mode=False,
    )

    osl.add_state_machine()

    early_stance = State("e_stance", 5, 130, 100)
    late_stance = State("l_stance", 5, 175, 0)
    early_swing = State("e_swing", 62, 40, 40)
    late_swing = State("l_swing", 30, 60, 200)

    foot_flat = Event("foot_flat")
    heel_off = Event("heel_off")
    toe_off = Event("toe_off")
    pre_heel_strike = Event("pre_heel_strike")
    heel_strike = Event("heel_strike")
    misc = Event("misc")

    osl.state_machine.add_state(early_stance, initial_state=True)
    osl.state_machine.add_state(late_stance)
    osl.state_machine.add_state(early_swing)
    osl.state_machine.add_state(late_swing)

    osl.state_machine.add_event(foot_flat)
    osl.state_machine.add_event(heel_off)
    osl.state_machine.add_event(toe_off)
    osl.state_machine.add_event(pre_heel_strike)
    osl.state_machine.add_event(heel_strike)
    osl.state_machine.add_event(misc)

    osl.state_machine.add_transition(
        early_stance,
        late_stance,
        foot_flat,
        estance_to_lstance,
    )

    osl.state_machine.add_transition(
        late_stance,
        early_swing,
        heel_off,
        lstance_to_eswing,
    )

    osl.state_machine.add_transition(
        early_swing,
        late_swing,
        toe_off,
        eswing_to_lswing,
    )

    osl.state_machine.add_transition(
        late_swing,
        early_stance,
        heel_strike,
        lswing_to_estance,
    )

    osl.add_tui()

    with osl:
        osl.run(set_state_machine_parameters=True)

if __name__ == "__main__":
    main()
