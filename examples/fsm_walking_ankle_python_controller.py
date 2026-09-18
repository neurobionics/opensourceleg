"""
A finite state machine to control the OSL ankle with impedance control.
This script can be helpful to test the OSL ankle using the load cell.
The fsm runs with the motor encoder, but the joint encoder can be also used.

Matteo Crotti
Soft Robotics for Human Cooperation and Rehabilitation Lab/Neurobionics Lab
Istituto Italiano di Tecnologia/University of Michigan
June 6, 2025
"""

import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.control.fsm import State, StateMachine
from opensourceleg.logging.logger import Logger
from opensourceleg.robots.osl import OpenSourceLeg
from opensourceleg.sensors.loadcell import NBLoadcellDAQ
from opensourceleg.utilities import SoftRealtimeLoop

GEAR_RATIO = 9 * (83 / 18)
FREQUENCY = 200

LOADCELL_MATRIX = np.array([
    (-9.817417, -1895.974287, 14.899174, -4.333117, -40.684438, 1900.65478),
    (-32.435533, 1129.587599, -5.693634, -2206.916117, 14.346322, 1086.297672),
    (-963.721193, -4.911412, -965.891326, -1.404768, -981.000498, 8.693936),
    (19.854122, -0.21849, 0.25324, 0.734436, -20.471398, -0.232788),
    (-11.530613, -0.964467, 22.849398, -0.011803, -11.785854, 1.015088),
    (-0.296284, -26.299567, -0.138142, -26.753995, 0.385682, -25.052914),
])

# ------------- TUNABLE FSM PARAMETERS ---------------- #
BODY_WEIGHT = 20 * 9.8

# STATE 1: EARLY STANCE
ANKLE_K_ESTANCE = 19.874
ANKLE_B_ESTANCE = 0
ANKLE_THETA_ESTANCE = -2
LOAD_LSTANCE: float = 1.0 * BODY_WEIGHT * 0.25
ANKLE_THETA_ESTANCE_TO_LSTANCE = np.deg2rad(8.0)

# STATE 2: LATE STANCE
ANKLE_K_LSTANCE = 81  # 79.498
ANKLE_B_LSTANCE = 0.063
ANKLE_THETA_LSTANCE = -20
LOAD_ESWING: float = 1.0 * BODY_WEIGHT * 0.15

# STATE 3: EARLY SWING
ANKLE_K_ESWING = 7.949
ANKLE_B_ESWING = 0.0
ANKLE_THETA_ESWING = 25
ANKLE_DTHETA_ESWING_TO_LSWING = 3
ANKLE_THETA_ESWING_TO_LSWING = np.deg2rad(15.0)

# STATE 4: LATE SWING
ANKLE_K_LSWING = 7.949
ANKLE_B_LSWING = 0.06  # 0.0
ANKLE_THETA_LSWING = 15
LOAD_ESTANCE: float = 1.0 * BODY_WEIGHT * 0.4
ANKLE_THETA_LSWING_TO_ESTANCE = np.deg2rad(16.0)


# ---------------------------------------------------- #
def estance_to_lstance(osl: OpenSourceLeg) -> bool:
    """
    Transition from early stance to late stance when the loadcell
    reads a force greater than a threshold and the ankle position
    is greater than a threshold.
    """
    if osl.loadcell is None:
        raise ValueError("Loadcell is not connected")
    if osl.ankle is None:
        raise ValueError("Ankle is not connected")
    return bool(-(osl.loadcell.fz) > LOAD_LSTANCE and osl.ankle.output_position > ANKLE_THETA_ESTANCE_TO_LSTANCE)


def lstance_to_eswing(osl: OpenSourceLeg) -> bool:
    """
    Transition from late stance to early swing when the loadcell
    reads a force less than a threshold.
    """
    if osl.loadcell is None:
        raise ValueError("Loadcell is not connected")
    return bool(-(osl.loadcell.fz) < LOAD_ESWING)


def eswing_to_lswing(osl: OpenSourceLeg) -> bool:
    """
    Transition from early swing to late swing when the ankle angle
    is greater than a threshold and the ankle velocity is less than
    a threshold.
    """
    if osl.ankle is None:
        raise ValueError("Ankle is not connected")

    return bool(
        osl.ankle.output_position > ANKLE_THETA_ESWING_TO_LSWING
        and osl.ankle.output_velocity < ANKLE_DTHETA_ESWING_TO_LSWING
    )


def lswing_to_estance(osl: OpenSourceLeg) -> bool:
    """
    Transition from late swing to early stance when the loadcell
    reads a force greater than a threshold or the ankle angle is
    less than a threshold.
    """
    if osl.loadcell is None:
        raise ValueError("Loadcell is not connected")
    if osl.ankle is None:
        raise ValueError("Ankle is not connected")
    return bool(-osl.loadcell.fz > LOAD_ESTANCE or osl.ankle.output_position < ANKLE_THETA_LSWING_TO_ESTANCE)


def create_simple_walking_fsm(osl: OpenSourceLeg) -> StateMachine:
    e_stance = State(
        name="e_stance",
        ankle_theta=ANKLE_THETA_ESTANCE,
        ankle_stiffness=ANKLE_K_ESTANCE,
        ankle_damping=ANKLE_B_ESTANCE,
    )

    l_stance = State(
        name="l_stance",
        ankle_theta=ANKLE_THETA_LSTANCE,
        ankle_stiffness=ANKLE_K_LSTANCE,
        ankle_damping=ANKLE_B_LSTANCE,
    )

    e_swing = State(
        name="e_swing",
        ankle_theta=ANKLE_THETA_ESWING,
        ankle_stiffness=ANKLE_K_ESWING,
        ankle_damping=ANKLE_B_ESWING,
    )

    l_swing = State(
        name="l_swing",
        ankle_theta=ANKLE_THETA_LSWING,
        ankle_stiffness=ANKLE_K_LSWING,
        ankle_damping=ANKLE_B_LSWING,
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
        "ankle": DephyActuator(
            tag="ankle",
            port="/dev/ttyACM0",
            gear_ratio=GEAR_RATIO,
            frequency=FREQUENCY,
            debug_level=0,
            torque_constant=0.145,
            dephy_log=False,
        ),
    }

    sensors = {
        "loadcell": NBLoadcellDAQ(
            LOADCELL_MATRIX,
            tag="loadcell",
            excitation_voltage=5.0,
            amp_gain=[34] * 3 + [151] * 3,
            spi_bus=1,
        ),
        # "joint_encoder_ankle": AS5048B(
        #     tag="joint_encoder_ankle",
        #     bus="/dev/i2c-2",
        #     A1_adr_pin=False,
        #     A2_adr_pin=False,
        #     zero_position=0,
        #     enable_diagnostics=False,
        # ),
    }

    clock = SoftRealtimeLoop(dt=1 / FREQUENCY)
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

    # Zeroing the joint encoders
    def knee_homing_complete():
        osl.joint_encoder_knee.update()
        osl.joint_encoder_knee.zero_position = osl.joint_encoder_knee.counts
        print("Knee homing complete!")

    def ankle_homing_complete():
        osl.joint_encoder_ankle.update()
        # The hard stop for ankle is at 30 deg from the zero position
        osl.joint_encoder_ankle.zero_position = osl.joint_encoder_ankle.counts - osl.joint_encoder_ankle.deg_to_counts(
            30
        )
        print("Ankle homing complete!")

    callbacks = {"knee": knee_homing_complete, "ankle": ankle_homing_complete}

    with fsm_logger, osl, osl_fsm:
        osl.update()
        osl.home()

        input("Press Enter to start walking...")

        # ankle
        osl.ankle.set_control_mode(mode=CONTROL_MODES.IMPEDANCE)
        osl.ankle.set_impedance_cc_pidf_gains()
        osl.ankle.set_output_impedance()

        for t in clock:
            osl.update()
            osl_fsm.update(osl=osl)
            osl.ankle.set_output_impedance(
                k=osl_fsm.current_state.ankle_stiffness,
                b=osl_fsm.current_state.ankle_damping,
            )

            osl.ankle.set_output_position(np.deg2rad(osl_fsm.current_state.ankle_theta))

            fsm_logger.info(
                f"T: {t:.3f}s, "
                f"Current state: {osl_fsm.current_state.name}; "
                f"Loadcell Fz: {osl.loadcell.fz:.3f} N; "
                f"Ankle theta: {np.rad2deg(osl.ankle.output_position):.3f} deg; "
                f"Ankle winding temperature: {osl.ankle.winding_temperature:.3f} c; "
            )
