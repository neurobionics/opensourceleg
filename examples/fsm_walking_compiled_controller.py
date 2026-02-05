"""
Example walking controller for the OSL.

Senthur Raj Ayyappan, Kevin Best
Neurobionics Lab
Robotics Department
University of Michigan
October 9, 2023
"""

import inspect
import os

import numpy as np

from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.control.compiled import CompiledController
from opensourceleg.sensors.loadcell import DephyLoadcellAmplifier
from opensourceleg.utilities import SoftRealtimeLoop, units

use_offline_mode = False
FREQUENCY = 200
clock = SoftRealtimeLoop(dt=1 / FREQUENCY)

knee = DephyActuator(
    tag="knee",
    firmware_version="7.2.0",
    port="/dev/ttyACM0",
    gear_ratio=9 * 83 / 18,
    frequency=FREQUENCY,
)

ankle = DephyActuator(
    tag="ankle",
    firmware_version="7.2.0",
    port="/dev/ttyACM1",
    gear_ratio=9 * 83 / 18,
    frequency=FREQUENCY,
)

actuators = [knee, ankle]


LOADCELL_MATRIX = np.array([
    (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
    (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
    (-1047.16800, 8.63900, -1047.28200, -20.70000, -1073.08800, -8.92300),
    (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
    (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
    (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
])

loadcell = DephyLoadcellAmplifier(
    calibration_matrix=LOADCELL_MATRIX,
    bus=1,
)

sensors = [loadcell]

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
controller = CompiledController(
    library_name="FSMController",
    library_path=currentdir,
    main_function_name="FSMController",
    initialization_function_name="FSMController_initialize",
    cleanup_function_name="FSMController_terminate",
)

controller.define_type(
    "impedance_param_type",
    [
        ("stiffness", controller.types.c_double),
        ("damping", controller.types.c_double),
        ("eq_angle", controller.types.c_double),
    ],
)
controller.define_type(
    "joint_impedance_set",
    [
        ("early_stance", controller.types.impedance_param_type),
        ("late_stance", controller.types.impedance_param_type),
        ("early_swing", controller.types.impedance_param_type),
        ("late_swing", controller.types.impedance_param_type),
    ],
)
controller.define_type(
    "transition_parameters",
    [
        ("min_time_in_state", controller.types.c_double),
        ("loadLStance", controller.types.c_double),
        ("ankleThetaEstanceToLstance", controller.types.c_double),
        ("kneeThetaESwingToLSwing", controller.types.c_double),
        ("kneeDthetaESwingToLSwing", controller.types.c_double),
        ("loadESwing", controller.types.c_double),
        ("loadEStance", controller.types.c_double),
        ("kneeThetaLSwingToEStance", controller.types.c_double),
    ],
)
controller.define_type(
    "UserParameters",
    [
        ("body_weight", controller.types.c_double),
        ("knee_impedance", controller.types.joint_impedance_set),
        ("ankle_impedance", controller.types.joint_impedance_set),
        ("transition_parameters", controller.types.transition_parameters),
    ],
)
controller.define_type("sensors", controller.DEFAULT_SENSOR_LIST)

controller.define_inputs(
    input_list=[
        ("parameters", controller.types.UserParameters),
        ("sensors", controller.types.sensors),
        ("time", controller.types.c_double),
    ]
)
controller.define_outputs(
    output_list=[
        ("current_state", controller.types.c_int),
        ("time_in_current_state", controller.types.c_double),
        ("knee_impedance", controller.types.impedance_param_type),
        ("ankle_impedance", controller.types.impedance_param_type),
    ]
)

# Populate Controller inputs as needed
controller.inputs.parameters.knee_impedance.early_stance.stiffness = 99.372
controller.inputs.parameters.knee_impedance.early_stance.damping = 3.180
controller.inputs.parameters.knee_impedance.early_stance.eq_angle = 5
controller.inputs.parameters.knee_impedance.late_stance.stiffness = 99.372
controller.inputs.parameters.knee_impedance.late_stance.damping = 1.272
controller.inputs.parameters.knee_impedance.late_stance.eq_angle = 8
controller.inputs.parameters.knee_impedance.early_swing.stiffness = 39.746
controller.inputs.parameters.knee_impedance.early_swing.damping = 0.063
controller.inputs.parameters.knee_impedance.early_swing.eq_angle = 60
controller.inputs.parameters.knee_impedance.late_swing.stiffness = 15.899
controller.inputs.parameters.knee_impedance.late_swing.damping = 3.186
controller.inputs.parameters.knee_impedance.late_swing.eq_angle = 5
controller.inputs.parameters.ankle_impedance.early_stance.stiffness = 19.874
controller.inputs.parameters.ankle_impedance.early_stance.damping = 0
controller.inputs.parameters.ankle_impedance.early_stance.eq_angle = -2
controller.inputs.parameters.ankle_impedance.late_stance.stiffness = 79.498
controller.inputs.parameters.ankle_impedance.late_stance.damping = 0.063
controller.inputs.parameters.ankle_impedance.late_stance.eq_angle = -20
controller.inputs.parameters.ankle_impedance.early_swing.stiffness = 7.949
controller.inputs.parameters.ankle_impedance.early_swing.damping = 0
controller.inputs.parameters.ankle_impedance.early_swing.eq_angle = 25
controller.inputs.parameters.ankle_impedance.late_swing.stiffness = 7.949
controller.inputs.parameters.ankle_impedance.late_swing.damping = 0.0
controller.inputs.parameters.ankle_impedance.late_swing.eq_angle = 15

# Configure state machine
body_weight = 82  # kg
controller.inputs.parameters.body_weight = body_weight
controller.inputs.parameters.transition_parameters.min_time_in_state = 0.20
controller.inputs.parameters.transition_parameters.loadLStance = -body_weight * 0.25
controller.inputs.parameters.transition_parameters.ankleThetaEstanceToLstance = 6.0
controller.inputs.parameters.transition_parameters.loadESwing = -body_weight * 0.15
controller.inputs.parameters.transition_parameters.kneeThetaESwingToLSwing = 50
controller.inputs.parameters.transition_parameters.kneeDthetaESwingToLSwing = 3
controller.inputs.parameters.transition_parameters.loadEStance = -body_weight * 0.4
controller.inputs.parameters.transition_parameters.kneeThetaLSwingToEStance = 30

with knee, ankle, loadcell:
    knee.home()
    ankle.home()

    knee.set_control_mode(knee.CONTROL_MODES.IMPEDANCE)
    ankle.set_control_mode(ankle.CONTROL_MODES.IMPEDANCE)

    knee.set_impedance_cc_pidf_gains()
    ankle.set_impedance_cc_pidf_gains()
    # Main Loop
    for t in clock:
        knee.update()
        ankle.update()
        loadcell.update()

        controller.inputs.sensors.knee_angle = units.convert_from_default(knee.output_position, units.Position.deg)
        controller.inputs.sensors.ankle_angle = units.convert_from_default(ankle.output_position, units.Position.deg)
        controller.inputs.sensors.knee_velocity = units.convert_from_default(
            knee.output_velocity, units.Velocity.deg_per_s
        )
        controller.inputs.sensors.ankle_velocity = units.convert_from_default(
            ankle.output_velocity, units.Velocity.deg_per_s
        )
        controller.inputs.sensors.Fz = loadcell.fz

        # Update any control inputs that change every loop
        controller.inputs.time = t

        # Call the controller
        outputs = controller.run()

        # Test print to ensure external library call works
        print(
            f"Current time in state {outputs.current_state}: {outputs.time_in_current_state:.2f} seconds, \
                Knee Eq {outputs.knee_impedance.eq_angle:.2f}, \
                Ankle Eq {outputs.ankle_impedance.eq_angle:.2f}, \
                Fz {loadcell.fz:.2f}",
            end="\r",
        )

        # Write to the hardware
        knee.set_output_impedance(
            k=units.convert_to_default(outputs.knee_impedance.stiffness, units.Stiffness.N_m_per_rad),
            b=units.convert_to_default(outputs.knee_impedance.damping, units.Damping.N_m_per_rad_per_s),
        )
        knee.set_output_position(value=units.convert_to_default(outputs.knee_impedance.eq_angle, units.Position.deg))
        ankle.set_output_impedance(
            k=units.convert_to_default(outputs.ankle_impedance.stiffness, units.Stiffness.N_m_per_rad),
            b=units.convert_to_default(outputs.ankle_impedance.damping, units.Damping.N_m_per_rad_per_s),
        )
        ankle.set_output_position(value=units.convert_to_default(outputs.ankle_impedance.eq_angle, units.Position.deg))

    print("\n")
