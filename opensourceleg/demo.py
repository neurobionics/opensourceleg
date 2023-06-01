from opensourceleg.osl import OpenSourceLeg
from opensourceleg.state_machine import Event, State

BODY_WEIGHT = 60.0  # kg


def estance_to_lstance(osl: OpenSourceLeg) -> bool:
    """
    Transition from early stance to late stance when the loadcell
    reads a force greater than a threshold.
    """
    assert osl.loadcell is not None
    if osl.loadcell.fz < -0.0167 * BODY_WEIGHT:
        return True
    else:
        return False


def lstance_to_eswing(osl: OpenSourceLeg) -> bool:
    """
    Transition from late stance to early swing when the loadcell
    reads a force less than a threshold.
    """
    assert osl.loadcell is not None
    if osl.loadcell.fz > -0.00267 * BODY_WEIGHT:
        return True
    else:
        return False


def eswing_to_lswing(osl: OpenSourceLeg) -> bool:
    """
    Transition from early swing to late swing when the knee angle
    is greater than a threshold and the knee velocity is less than
    a threshold.
    """
    assert osl.knee is not None
    if osl.knee.output_position > 60 and osl.knee.output_velocity < 0.135:
        return True
    else:
        return False


def eswing_to_estance(osl: OpenSourceLeg) -> bool:
    """
    Transition from early swing to early stance when the loadcell
    reads a force greater than a threshold.
    """
    assert osl.loadcell is not None
    if osl.loadcell.fz < -0.02 * BODY_WEIGHT:
        return True
    else:
        return False


def lswing_to_estance(osl: OpenSourceLeg) -> bool:
    """
    Transition from late swing to early stance when the loadcell
    reads a force greater than a threshold or the knee angle is
    less than a threshold.
    """
    assert osl.knee is not None and osl.loadcell is not None
    if osl.loadcell.fz < -0.02 * BODY_WEIGHT or osl.knee.output_position < 30:
        return True
    else:
        return False


def main():
    osl = OpenSourceLeg(frequency=200)
    osl.units["position"] = "deg"  # type: ignore

    osl.add_joint(
        name="knee",
        gear_ratio=41.4999,
    )

    osl.add_loadcell(
        dephy_mode=False,
    )

    osl.add_state_machine()

    early_stance = State(name="e_stance", theta=5, k=130, b=450)
    late_stance = State(name="l_stance", theta=5, k=175, b=200)
    early_swing = State(name="e_swing", theta=62, k=40, b=40)
    late_swing = State(name="l_swing", theta=30, k=100, b=200)

    foot_flat = Event(name="foot_flat")
    heel_off = Event(name="heel_off")
    toe_off = Event(name="toe_off")
    pre_heel_strike = Event(name="pre_heel_strike")
    heel_strike = Event(name="heel_strike")
    misc = Event(name="misc")

    assert osl.state_machine is not None
    osl.state_machine.add_state(state=early_stance, initial_state=True)
    osl.state_machine.add_state(state=late_stance)
    osl.state_machine.add_state(state=early_swing)
    osl.state_machine.add_state(state=late_swing)

    osl.state_machine.add_event(event=foot_flat)
    osl.state_machine.add_event(event=heel_off)
    osl.state_machine.add_event(event=toe_off)
    osl.state_machine.add_event(event=pre_heel_strike)
    osl.state_machine.add_event(event=heel_strike)
    osl.state_machine.add_event(event=misc)

    osl.state_machine.add_transition(
        source=early_stance,
        destination=late_stance,
        event=foot_flat,
        callback=estance_to_lstance,
    )

    osl.state_machine.add_transition(
        source=late_stance,
        destination=early_swing,
        event=heel_off,
        callback=lstance_to_eswing,
    )

    osl.state_machine.add_transition(
        source=early_swing,
        destination=late_swing,
        event=toe_off,
        callback=eswing_to_lswing,
    )

    osl.state_machine.add_transition(
        source=late_swing,
        destination=early_stance,
        event=heel_strike,
        callback=lswing_to_estance,
    )

    osl.add_tui()

    with osl:
        osl.run(set_state_machine_parameters=True)


if __name__ == "__main__":
    main()
