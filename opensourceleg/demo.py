from osl import OpenSourceLeg
from state_machine import State, Event
import numpy as np

BODY_WEIGHT = 60.0 # kg
FZ_LSTANCE = -0.02 * BODY_WEIGHT
FZ_ESWING = -0.01 * BODY_WEIGHT
FZ_ESTANCE = -0.02 * BODY_WEIGHT
VELOCITY_LSWING = np.deg2rad(0.135)
POSITION_LSWING = np.deg2rad(60)
POSITION_ESTANCE = np.deg2rad(30)

def estance_to_lstance(osl: OpenSourceLeg):
    if osl.loadcell.fz < -1.0:
        return True
    else:
        return False

def lstance_to_eswing(osl: OpenSourceLeg):
    if osl.loadcell.fz > -0.1:
        return True
    else:
        return False

def eswing_to_lswing(osl: OpenSourceLeg):
    if (
        osl.knee.output_position > POSITION_LSWING
        and osl.knee.output_velocity < VELOCITY_LSWING
    ):
        return True
    else:
        return False

def eswing_to_estance(osl: OpenSourceLeg):
    if osl.loadcell.fz < FZ_ESTANCE:
        return True
    else:
        False

def lswing_to_estance(osl: OpenSourceLeg):
    if (
        osl.loadcell.fz < FZ_ESTANCE or osl.knee.output_position < POSITION_ESTANCE
    ):
        return True
    else:
        return False

def main():
    osl = OpenSourceLeg(frequency=200)

    osl.add_joint(
        name="knee",
        port="/dev/ttyACM0",
        baud_rate=230400,
        gear_ratio=41.4999,
    )

    osl.add_loadcell(
        dephy_mode=False,
    )

    osl.add_state_machine()

    early_stance = State(
        "EStance", 
        np.deg2rad(5), 
        130, 
        100,
    )

    late_stance = State(
        "LStance", 
        np.deg2rad(5), 
        175, 
        0,
    )

    early_swing = State(
        "ESwing", 
        np.deg2rad(62), 
        40, 
        40,
    )

    late_swing = State(
        "LSwing", 
        np.deg2rad(30), 
        60, 
        200,
    )


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

    with osl:
        osl.state_machine.start(data=None)
        osl.knee.set_mode("impedance")
        osl.knee.set_impedance_gains() 

        for t in osl.clock:
            osl.update(current_limit=8000)
            osl.state_machine.on_event([t])
            osl.log.info("[OSL] State: {}".format(osl.state_machine.current_state.name))

            osl.knee.set_impedance_gains(
                K = osl.state_machine.current_state.stiffness,
                B = osl.state_machine.current_state.damping,
            )

            osl.knee.set_output_position(osl.state_machine.current_state.equilibrium_angle)


if __name__ == "__main__":
    main()
    