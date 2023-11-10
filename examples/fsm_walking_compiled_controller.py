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

from opensourceleg.control.compiled_controller import CompiledController
from opensourceleg.osl import OpenSourceLeg
from opensourceleg.tools import units

osl = OpenSourceLeg(frequency=200)
use_offline_mode = False
osl.add_joint("knee", gear_ratio=9 * 83 / 18, offline_mode=use_offline_mode)
osl.add_joint("ankle", gear_ratio=9 * 83 / 18, offline_mode=use_offline_mode)
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
    joint=osl.knee, offline_mode=use_offline_mode, loadcell_matrix=LOADCELL_MATRIX
)

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))  # type: ignore
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
    [
        ("parameters", controller.types.UserParameters),
        ("sensors", controller.types.sensors),
        ("time", controller.types.c_double),
    ]
)
controller.define_outputs(
    [
        ("current_state", controller.types.c_int),
        ("time_in_current_state", controller.types.c_double),
        ("knee_impedance", controller.types.impedance_param_type),
        ("ankle_impedance", controller.types.impedance_param_type),
    ]
)

# Populate Controller inputs as needed
controller.inputs.parameters.knee_impedance.early_stance.stiffness = 99.372  # type: ignore
controller.inputs.parameters.knee_impedance.early_stance.damping = 3.180  # type: ignore
controller.inputs.parameters.knee_impedance.early_stance.eq_angle = 5  # type: ignore
controller.inputs.parameters.knee_impedance.late_stance.stiffness = 99.372  # type: ignore
controller.inputs.parameters.knee_impedance.late_stance.damping = 1.272  # type: ignore
controller.inputs.parameters.knee_impedance.late_stance.eq_angle = 8  # type: ignore
controller.inputs.parameters.knee_impedance.early_swing.stiffness = 39.746  # type: ignore
controller.inputs.parameters.knee_impedance.early_swing.damping = 0.063  # type: ignore
controller.inputs.parameters.knee_impedance.early_swing.eq_angle = 60  # type: ignore
controller.inputs.parameters.knee_impedance.late_swing.stiffness = 15.899  # type: ignore
controller.inputs.parameters.knee_impedance.late_swing.damping = 3.186  # type: ignore
controller.inputs.parameters.knee_impedance.late_swing.eq_angle = 5  # type: ignore
controller.inputs.parameters.ankle_impedance.early_stance.stiffness = 19.874  # type: ignore
controller.inputs.parameters.ankle_impedance.early_stance.damping = 0  # type: ignore
controller.inputs.parameters.ankle_impedance.early_stance.eq_angle = -2  # type: ignore
controller.inputs.parameters.ankle_impedance.late_stance.stiffness = 79.498  # type: ignore
controller.inputs.parameters.ankle_impedance.late_stance.damping = 0.063  # type: ignore
controller.inputs.parameters.ankle_impedance.late_stance.eq_angle = -20  # type: ignore
controller.inputs.parameters.ankle_impedance.early_swing.stiffness = 7.949  # type: ignore
controller.inputs.parameters.ankle_impedance.early_swing.damping = 0  # type: ignore
controller.inputs.parameters.ankle_impedance.early_swing.eq_angle = 25  # type: ignore
controller.inputs.parameters.ankle_impedance.late_swing.stiffness = 7.949  # type: ignore
controller.inputs.parameters.ankle_impedance.late_swing.damping = 0.0  # type: ignore
controller.inputs.parameters.ankle_impedance.late_swing.eq_angle = 15  # type: ignore

# Configure state machine
body_weight = 82  # kg
controller.inputs.parameters.body_weight = body_weight  # type: ignore
controller.inputs.parameters.transition_parameters.min_time_in_state = 0.20  # type: ignore
controller.inputs.parameters.transition_parameters.loadLStance = -body_weight * 0.25  # type: ignore
controller.inputs.parameters.transition_parameters.ankleThetaEstanceToLstance = 6.0  # type: ignore
controller.inputs.parameters.transition_parameters.loadESwing = -body_weight * 0.15  # type: ignore
controller.inputs.parameters.transition_parameters.kneeThetaESwingToLSwing = 50  # type: ignore
controller.inputs.parameters.transition_parameters.kneeDthetaESwingToLSwing = 3  # type: ignore
controller.inputs.parameters.transition_parameters.loadEStance = -body_weight * 0.4  # type: ignore
controller.inputs.parameters.transition_parameters.kneeThetaLSwingToEStance = 30  # type: ignore

with osl:
    osl.home()
    osl.update()
    osl.knee.set_mode(osl.knee.control_modes.impedance)
    osl.ankle.set_mode(osl.ankle.control_modes.impedance)

    # Main Loop
    for t in osl.clock:
        osl.update()

        controller.inputs.sensors.knee_angle = (  # type: ignore
            units.convert_from_default(osl.knee.output_position, units.position.deg)
        )
        controller.inputs.sensors.ankle_angle = units.convert_from_default(osl.ankle.output_position, units.position.deg)  # type: ignore
        controller.inputs.sensors.knee_velocity = units.convert_from_default(osl.knee.output_velocity, units.velocity.deg_per_s)  # type: ignore
        controller.inputs.sensors.ankle_velocity = units.convert_from_default(osl.ankle.output_velocity, units.velocity.deg_per_s)  # type: ignore
        controller.inputs.sensors.Fz = osl.loadcell.fz  # type: ignore

        # Update any control inputs that change every loop
        controller.inputs.time = t  # type: ignore

        # Call the controller
        outputs = controller.run()

        # Test print to ensure external library call works
        print(
            "Current time in state {}: {:.2f} seconds, Knee Eq {:.2f}, Ankle Eq {:.2f}, Fz {:.2f}".format(
                outputs.current_state,
                outputs.time_in_current_state,
                outputs.knee_impedance.eq_angle,
                outputs.ankle_impedance.eq_angle,
                osl.loadcell.fz,
            ),
            end="\r",
        )

        # Write to the hardware
        osl.knee.set_joint_impedance(
            K=units.convert_to_default(
                outputs.knee_impedance.stiffness, units.stiffness.N_m_per_rad
            ),
            B=units.convert_to_default(
                outputs.knee_impedance.damping, units.damping.N_m_per_rad_per_s
            ),
        )
        osl.knee.set_output_position(
            position=units.convert_to_default(
                outputs.knee_impedance.eq_angle, units.position.deg
            )
        )
        osl.ankle.set_joint_impedance(
            K=units.convert_to_default(
                outputs.ankle_impedance.stiffness, units.stiffness.N_m_per_rad
            ),
            B=units.convert_to_default(
                outputs.ankle_impedance.damping, units.damping.N_m_per_rad_per_s
            ),
        )
        osl.ankle.set_output_position(
            position=units.convert_to_default(
                outputs.ankle_impedance.eq_angle, units.position.deg
            )
        )

    print("\n")
