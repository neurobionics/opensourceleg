import numpy as np

import opensourceleg.tools.units as units
from opensourceleg.control.state_machine import Event, State, StateMachine
from opensourceleg.osl import OpenSourceLeg

offline_mode = False  # Set to true for debugging without hardware

# ------------- TUNABLE FSM PARAMETERS ---------------- #
BODY_WEIGHT = 60 * 9.8

# STATE 1: EARLY STANCE
KNEE_K_ESTANCE = 99.372
KNEE_B_ESTANCE = 3.180
KNEE_THETA_ESTANCE = 5
ANKLE_K_ESTANCE = 19.874
ANKLE_B_ESTANCE = 0
ANKLE_THETA_ESTANCE = -2
LOAD_LSTANCE: float = -1.0 * BODY_WEIGHT * 0.25
ANKLE_THETA_ESTANCE_TO_LSTANCE = 6.0

# STATE 2: LATE STANCE
KNEE_K_LSTANCE = 99.372
KNEE_B_LSTANCE = 1.272
KNEE_THETA_LSTANCE = 8
ANKLE_K_LSTANCE = 79.498
ANKLE_B_LSTANCE = 0.063
ANKLE_THETA_LSTANCE = -20
LOAD_ESWING: float = -1.0 * BODY_WEIGHT * 0.15

# STATE 3: EARLY SWING
KNEE_K_ESWING = 39.749
KNEE_B_ESWING = 0.063
KNEE_THETA_ESWING = 60
ANKLE_K_ESWING = 7.949
ANKLE_B_ESWING = 0.0
ANKLE_THETA_ESWING = 25
KNEE_THETA_ESWING_TO_LSWING = 50
KNEE_DTHETA_ESWING_TO_LSWING = 3

# STATE 4: LATE SWING
KNEE_K_LSWING = 15.899
KNEE_B_LSWING = 3.816
KNEE_THETA_LSWING = 5
ANKLE_K_LSWING = 7.949
ANKLE_B_LSWING = 0.0
ANKLE_THETA_LSWING = 15
LOAD_ESTANCE: float = -1.0 * BODY_WEIGHT * 0.4
KNEE_THETA_LSWING_TO_ESTANCE = 30
# ---------------------------------------------------- #


def run_FSM_controller():
    """
    This is the main function for this script.
    It creates an OSL object and builds a state machine with 4 states.
    It runs a main loop that updates the state machine based on the
    hardware information and sends updated commands to the motors.
    """
    osl = OpenSourceLeg(frequency=200)
    osl.add_joint(name="knee", gear_ratio=41.4999, offline_mode=offline_mode)
    osl.add_joint(name="ankle", gear_ratio=41.4999, offline_mode=offline_mode)
    LOADCELL_MATRIX = np.array(
        [
            (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
            (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
            (-1047.16800, 8.63900, -1047.28200, -20.70000, -1073.08800, -8.92300),
            (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
            (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
            (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
        ]
    )
    osl.add_loadcell(
        dephy_mode=False,
        offline_mode=offline_mode,
        loadcell_matrix=LOADCELL_MATRIX,
    )

    fsm = build_4_state_FSM(osl)

    osl.log.add_attributes(container=osl, attributes=["timestamp"])
    osl.log.add_attributes(
        container=osl.knee,
        attributes=[
            "output_position",
            "motor_current",
            "joint_torque",
            "motor_voltage",
            "accelx",
        ],
    )
    osl.log.add_attributes(
        container=osl.ankle,
        attributes=[
            "output_position",
            "motor_current",
            "joint_torque",
            "motor_voltage",
            "accelx",
        ],
    )
    osl.log.add_attributes(container=osl.loadcell, attributes=["fz"])
    osl.log.add_attributes(container=fsm.current_state, attributes=["name"])

    with osl:
        osl.home()
        fsm.start()

        for t in osl.clock:
            osl.update()
            fsm.update()

            if osl.knee.mode != osl.knee.control_modes.impedance:
                osl.knee.set_mode(mode=osl.knee.control_modes.impedance)
                osl.knee.set_impedance_gains()
            osl.knee.set_joint_impedance(
                K=units.convert_to_default(
                    fsm.current_state.knee_stiffness,
                    units.stiffness.N_m_per_rad,
                ),
                B=units.convert_to_default(
                    fsm.current_state.knee_damping,
                    units.damping.N_m_per_rad_per_s,
                ),
            )
            osl.knee.set_output_position(
                position=units.convert_to_default(
                    fsm.current_state.knee_theta, units.position.deg
                ),
            )

            if osl.ankle.mode != osl.ankle.control_modes.impedance:
                osl.ankle.set_mode(osl.ankle.control_modes.impedance)
                osl.ankle.set_impedance_gains()
            osl.ankle.set_joint_impedance(
                K=units.convert_to_default(
                    fsm.current_state.ankle_stiffness,
                    units.stiffness.N_m_per_rad,
                ),
                B=units.convert_to_default(
                    fsm.current_state.ankle_damping,
                    units.damping.N_m_per_rad_per_s,
                ),
            )
            osl.ankle.set_output_position(
                position=units.convert_to_default(
                    fsm.current_state.ankle_theta, units.position.deg
                ),
            )
            print(
                "Current time in state {}: {:.2f} seconds, Knee Eq {:.2f}, Ankle Eq {:.2f}, Fz {:.2f}".format(
                    fsm.current_state.name,
                    fsm.current_state.current_time_in_state,
                    fsm.current_state.knee_theta,
                    fsm.current_state.ankle_theta,
                    osl.loadcell.fz,
                ),
                end="\r",
            )

        fsm.stop()
        print("")


def build_4_state_FSM(osl: OpenSourceLeg) -> StateMachine:
    """This method builds a state machine with 4 states.
    The states are early stance, late stance, early swing, and late swing.
    It uses the impedance parameters and transition criteria above.

    Inputs:
        OSL instance
    Returns:
        FSM object"""

    def estance_to_lstance(osl: OpenSourceLeg) -> bool:
        """
        Transition from early stance to late stance when the loadcell
        reads a force greater than a threshold.
        """
        assert osl.loadcell is not None
        return bool(
            osl.loadcell.fz < LOAD_LSTANCE
            and osl.ankle.output_position > ANKLE_THETA_ESTANCE_TO_LSTANCE
        )

    def lstance_to_eswing(osl: OpenSourceLeg) -> bool:
        """
        Transition from late stance to early swing when the loadcell
        reads a force less than a threshold.
        """
        assert osl.loadcell is not None
        return bool(osl.loadcell.fz > LOAD_ESWING)

    def eswing_to_lswing(osl: OpenSourceLeg) -> bool:
        """
        Transition from early swing to late swing when the knee angle
        is greater than a threshold and the knee velocity is less than
        a threshold.
        """
        assert osl.knee is not None
        return bool(
            osl.knee.output_position > KNEE_THETA_ESWING_TO_LSWING
            and osl.knee.output_velocity < KNEE_DTHETA_ESWING_TO_LSWING
        )

    def lswing_to_estance(osl: OpenSourceLeg) -> bool:
        """
        Transition from late swing to early stance when the loadcell
        reads a force greater than a threshold or the knee angle is
        less than a threshold.
        """
        assert osl.knee is not None and osl.loadcell is not None
        return bool(
            osl.loadcell.fz < LOAD_ESTANCE
            or osl.knee.output_position < KNEE_THETA_LSWING_TO_ESTANCE
        )

    early_stance = State(name="e_stance")
    early_stance.set_knee_impedance_paramters(
        theta=KNEE_THETA_ESTANCE, k=KNEE_K_ESTANCE, b=KNEE_B_ESTANCE
    )
    early_stance.make_knee_active()
    early_stance.set_ankle_impedance_paramters(
        theta=ANKLE_THETA_ESTANCE, k=ANKLE_K_ESTANCE, b=ANKLE_B_ESTANCE
    )
    early_stance.make_ankle_active()

    late_stance = State(name="l_stance")
    late_stance.set_knee_impedance_paramters(
        theta=KNEE_THETA_LSTANCE, k=KNEE_K_LSTANCE, b=KNEE_B_LSTANCE
    )
    late_stance.make_knee_active()
    late_stance.set_ankle_impedance_paramters(
        theta=ANKLE_THETA_LSTANCE, k=ANKLE_K_LSTANCE, b=ANKLE_B_LSTANCE
    )
    late_stance.make_ankle_active()

    early_swing = State(name="e_swing")
    early_swing.set_knee_impedance_paramters(
        theta=KNEE_THETA_ESWING, k=KNEE_K_ESWING, b=KNEE_B_ESWING
    )
    early_swing.make_knee_active()
    early_swing.set_ankle_impedance_paramters(
        theta=ANKLE_THETA_ESWING, k=ANKLE_K_ESWING, b=ANKLE_B_ESWING
    )
    early_swing.make_ankle_active()

    late_swing = State(name="l_swing")
    late_swing.set_knee_impedance_paramters(
        theta=KNEE_THETA_LSWING, k=KNEE_K_LSWING, b=KNEE_B_LSWING
    )
    late_swing.make_knee_active()
    late_swing.set_ankle_impedance_paramters(
        theta=ANKLE_THETA_LSWING, k=ANKLE_K_LSWING, b=ANKLE_B_LSWING
    )
    late_swing.make_ankle_active()

    foot_flat = Event(name="foot_flat")
    heel_off = Event(name="heel_off")
    toe_off = Event(name="toe_off")
    pre_heel_strike = Event(name="pre_heel_strike")
    heel_strike = Event(name="heel_strike")
    misc = Event(name="misc")

    fsm = StateMachine(osl=osl, spoof=offline_mode)

    fsm.add_state(state=early_stance, initial_state=True)
    fsm.add_state(state=late_stance)
    fsm.add_state(state=early_swing)
    fsm.add_state(state=late_swing)

    fsm.add_event(event=foot_flat)
    fsm.add_event(event=heel_off)
    fsm.add_event(event=toe_off)
    fsm.add_event(event=pre_heel_strike)
    fsm.add_event(event=heel_strike)
    fsm.add_event(event=misc)

    fsm.add_transition(
        source=early_stance,
        destination=late_stance,
        event=foot_flat,
        callback=estance_to_lstance,
    )
    fsm.add_transition(
        source=late_stance,
        destination=early_swing,
        event=heel_off,
        callback=lstance_to_eswing,
    )
    fsm.add_transition(
        source=early_swing,
        destination=late_swing,
        event=toe_off,
        callback=eswing_to_lswing,
    )
    fsm.add_transition(
        source=late_swing,
        destination=early_stance,
        event=heel_strike,
        callback=lswing_to_estance,
    )
    return fsm


if __name__ == "__main__":
    run_FSM_controller()
