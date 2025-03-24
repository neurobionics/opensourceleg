from opensourceleg.control.fsm import State, StateMachine
from opensourceleg.robots.osl import OpenSourceLeg
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.sensors.loadcell import DephyLoadcellAmplifier
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.time import SoftRealtimeLoop
from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.logging.logger import Logger

import numpy as np

GEAR_RATIO = 9 * (83/18)
FREQUENCY = 200
LOADCELL_CALIBRATION_MATRIX = np.array(
    [
        (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
        (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
        (-1047.16800, 8.63900, -1047.28200, -20.70000, -1073.08800, -8.92300),
        (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
        (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
        (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
    ]
)

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
ANKLE_THETA_ESTANCE_TO_LSTANCE = np.deg2rad(6.0)

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
KNEE_THETA_ESWING_TO_LSWING = np.deg2rad(50)
KNEE_DTHETA_ESWING_TO_LSWING = 3

# STATE 4: LATE SWING
KNEE_K_LSWING = 15.899
KNEE_B_LSWING = 3.816
KNEE_THETA_LSWING = 5
ANKLE_K_LSWING = 7.949
ANKLE_B_LSWING = 0.0
ANKLE_THETA_LSWING = 15
LOAD_ESTANCE: float = -1.0 * BODY_WEIGHT * 0.4
KNEE_THETA_LSWING_TO_ESTANCE = np.deg2rad(30)

# ---------------------------------------------------- #

def create_simple_walking_fsm(osl: OpenSourceLeg) -> StateMachine:

    e_stance = State(
        name="e_stance",
        knee_theta=KNEE_THETA_ESTANCE,
        knee_stiffness=KNEE_K_ESTANCE,
        knee_damping=KNEE_B_ESTANCE,
        ankle_theta=ANKLE_THETA_ESTANCE,
        ankle_stiffness=ANKLE_K_ESTANCE,
        ankle_damping=ANKLE_B_ESTANCE,
    )

    l_stance = State(
        name="l_stance",
        knee_theta=KNEE_THETA_LSTANCE,
        knee_stiffness=KNEE_K_LSTANCE,
        knee_damping=KNEE_B_LSTANCE,
        ankle_theta=ANKLE_THETA_LSTANCE,
        ankle_stiffness=ANKLE_K_LSTANCE,
        ankle_damping=ANKLE_B_LSTANCE,
    )

    e_swing = State(
        name="e_swing",
        knee_theta=KNEE_THETA_ESWING,
        knee_stiffness=KNEE_K_ESWING,
        knee_damping=KNEE_B_ESWING,
        ankle_theta=ANKLE_THETA_ESWING,
        ankle_stiffness=ANKLE_K_ESWING,
        ankle_damping=ANKLE_B_ESWING,
    )

    l_swing = State(
        name="l_swing",
        knee_theta=KNEE_THETA_LSWING,
        knee_stiffness=KNEE_K_LSWING,
        knee_damping=KNEE_B_LSWING,
        ankle_theta=ANKLE_THETA_LSWING,
        ankle_stiffness=ANKLE_K_LSWING,
        ankle_damping=ANKLE_B_LSWING,
    )

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

    fsm = StateMachine(
        states=[
            e_stance,
            l_stance,
            e_swing,
            l_swing,
        ],
        initial_state_name="e_stance",
    )

    fsm.add_transition(
        source=e_stance,
        destination=l_stance,
        event_name="foot_flat",
        criteria=estance_to_lstance,
    )
    fsm.add_transition(
        source=l_stance,
        destination=e_swing,
        event_name="heel_off",
        criteria=lstance_to_eswing,
    )
    fsm.add_transition(
        source=e_swing,
        destination=l_swing,
        event_name="toe_off",
        criteria=eswing_to_lswing,
    )
    fsm.add_transition(
        source=l_swing,
        destination=e_stance,
        event_name="heel_strike",
        criteria=lswing_to_estance,
    )
    return fsm

if __name__ == "__main__":
    actuators = {
        "knee": DephyActuator(
            port="/dev/ttyACM0",
            gear_ratio=GEAR_RATIO,
            frequency=FREQUENCY,
            debug_level=0,
            dephy_log=False,
        ),
        "ankle": DephyActuator(
            port="/dev/ttyACM1",
            gear_ratio=GEAR_RATIO,
            frequency=FREQUENCY,
            debug_level=0,
            dephy_log=False,
        ),
    }

    sensors = {
        "loadcell": DephyLoadcellAmplifier(
            calibration_matrix=LOADCELL_CALIBRATION_MATRIX,
        ),
        "joint_encoder_knee": AS5048B(
            bus=1,
            A1_adr_pin=True,
            A2_adr_pin=False,
            name="knee_encoder",
            zero_position=0,
            enable_diagnostics=False,
        ),
        "joint_encoder_ankle": AS5048B(
            bus=1,
            A1_adr_pin=False,
            A2_adr_pin=True,
            name="ankle_encoder",
            zero_position=0,
            enable_diagnostics=False,
        ),
    }

    clock = SoftRealtimeLoop(dt=1/FREQUENCY)
    fsm_logger = Logger(
        log_path="./logs",
        file_name="fsm.log",
    )

    osl = OpenSourceLeg(
        tag="osl",
        actuators=actuators,
        sensors=sensors,
    )

    osl_fsm = create_simple_walking_fsm(osl)

    with osl, osl_fsm:
        osl.update()
        osl.home()

        input("Press Enter to continue...")
        
        #knee
        osl.knee.set_control_mode(mode=CONTROL_MODES.IMPEDANCE)
        osl.knee.set_impedance_gains()

        #ankle
        osl.ankle.set_control_mode(mode=CONTROL_MODES.IMPEDANCE)
        osl.ankle.set_impedance_gains()

        for t in clock:
            osl.update()
            osl_fsm.update(osl=osl)
            osl.knee.set_output_impedance(
                k=osl_fsm.current_state.knee_stiffness,
                b=osl_fsm.current_state.knee_damping,
            )
            osl.ankle.set_output_impedance(
                k=osl_fsm.current_state.ankle_stiffness,
                b=osl_fsm.current_state.ankle_damping,
            )

            osl.knee.set_output_position(np.deg2rad(osl_fsm.current_state.knee_theta))
            osl.ankle.set_output_position(np.deg2rad(osl_fsm.current_state.ankle_theta))

            fsm_logger.info(
                f"T: {t:.3f}s, "
                f"Current state: {osl_fsm.current_state.name}; "
                f"Loadcell Fz: {osl.loadcell.fz:.3f} N; "
                f"Knee theta: {np.rad2deg(osl.knee.output_position):.3f} deg; "
                f"Ankle theta: {np.rad2deg(osl.ankle.output_position):.3f} deg; "
                f"Knee winding temperature: {osl.knee.winding_temperature:.3f} deg; "
                f"Ankle winding temperature: {osl.ankle.winding_temperature:.3f} deg; "
            )




    