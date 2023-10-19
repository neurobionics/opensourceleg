"""
Example walking controller for the OSL. 

Senthur Raj Ayyappan, Kevin Best
Neurobionics Lab
Robotics Department
University of Michigan
October 9, 2023
"""

import inspect

# Imports
import os
import sys

import numpy as np

# Path fixes to handle running directly from cloned repo instead of pip installed.
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

import opensourceleg.control as control

# Import OSL packages
from opensourceleg.osl import OpenSourceLeg

# Initialization
osl = OpenSourceLeg(frequency=200)
use_offline_mode = True
osl.add_joint("knee", gear_ratio=9 * 83 / 18, port=None, offline_mode=use_offline_mode)
osl.add_joint(
    "ankle", gear_ratio=9 * 83 / 18, port="todo", offline_mode=use_offline_mode
)
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

# Instantiate a compiled controller wrapper object
controller = control.CompiledController(
    library_name="FSM_WalkingController",
    library_path=currentdir,
    main_function_name="FSMController",
    initialization_function_name="FSMController_initialize",
    cleanup_function_name="FSMController_terminate",
)

# Define custom types for parameters and outputs as list of tuples
controller.Define_Type(
    "impedance_param_type",
    [
        ("stiffness", controller.types.c_double),
        ("damping", controller.types.c_double),
        ("eq_angle", controller.types.c_double),
    ],
)
controller.Define_Type(
    "joint_impedance_set",
    [
        ("early_stance", controller.types.impedance_param_type),
        ("late_stance", controller.types.impedance_param_type),
        ("early_swing", controller.types.impedance_param_type),
        ("late_swing", controller.types.impedance_param_type),
    ],
)
controller.Define_Type(
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
controller.Define_Type(
    "UserParameters",
    [
        ("body_weight", controller.types.c_double),
        ("knee_impedance", controller.types.joint_impedance_set),
        ("ankle_impedance", controller.types.joint_impedance_set),
        ("transition_parameters", controller.types.transition_parameters),
    ],
)
controller.Define_Type("sensors", controller.DEFAULT_SENSOR_LIST)

# Define the function interface of the external library
controller.Define_Inputs(
    [
        ("parameters", controller.types.UserParameters),
        ("sensors", controller.types.sensors),
        ("time", controller.types.c_double),
    ]
)
controller.Define_Outputs(
    [
        ("current_state", controller.types.c_uint8),
        ("time_in_current_state", controller.types.c_double),
        ("knee_impedance", controller.types.impedance_param_type),
        ("ankle_impedance", controller.types.impedance_param_type),
    ]
)

# Populate Controller inputs as needed
controller.inputs.parameters.knee_impedance.early_stance.stiffness = 5
# TODO: Finish defining imipedance parameters for each state

# Configure state machine
controller.inputs.parameters.transition_parameters.min_time_in_state = 2.0
# TODO: Finish defining transition parameters

with osl:
    osl.home()
    osl.calibrate_loadcell()
    osl.knee.set_mode(osl.knee.control_modes.impedance)
    osl.ankle.set_mode(osl.knee.control_modes.impedance)

    # Main Loop
    for t in osl.clock:
        # Read from the hardware and update the inputs object
        osl.update()

        # TODO: Make this automated for standard inputs list
        controller.inputs.sensors.knee_angle = (
            osl.knee.output_position
        )  # Default is in radians
        controller.inputs.sensors.ankle_angle = osl.ankle.output_position
        controller.inputs.sensors.knee_velocity = osl.knee.output_velocity
        controller.inputs.sensors.ankle_velocity = osl.ankle.output_velocity
        controller.inputs.sensors.Fz = osl.loadcell.fz  # Newtons?

        # Update any control inputs that change every loop
        outputs = controller.inputs.time = t

        # Call the controller
        outputs = controller.run()

        # Write to the hardware
        osl.knee.set_joint_impedance(
            K=outputs.knee_impedance.stiffness, B=outputs.knee_impedance.damping
        )
        osl.knee.set_output_position(position=outputs.knee_impedance.eq_angle)
        osl.ankle.set_joint_impedance(
            K=outputs.ankle_impedance.stiffness, B=outputs.ankle_impedance.damping
        )
        osl.ankle.set_output_position(position=outputs.ankle_impedance.eq_angle)
