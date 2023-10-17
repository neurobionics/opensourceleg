"""
Example walking controller for the OSL. 

Senthur Raj Ayyappan, Kevin Best
Neurobionics Lab
Robotics Department
University of Michigan
October 9, 2023
"""

# Imports
import os
import sys
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir) 
from opensourceleg.osl import OpenSourceLeg
import opensourceleg.control as control

# Initialization
osl = OpenSourceLeg(frequency=200)
use_offline_mode = True
osl.add_joint('knee', gear_ratio=9*83/18, port=None, use_offline_mode=True)
osl.add_joint('ankle',gear_ratio=9*83/18, port=None, use_offline_mode=True)
osl.add_loadcell(joint=osl.knee, use_offline_mode=True)
osl.units["position"] = "deg"  # type: ignore
osl.units["velocity"] = "deg/s"  # type: ignore

# Instantiate a compiled controller wrapper object
controller = control.CompiledController(main_function_name = 'FSM_Walking_Controller.so',
                                        controller_path = currentdir, 
                                        initialization_function_name = None,
                                        cleanup_function_name = None)


# Define custom types for parameters and outputs as list of tuples
controller.Define_Type('impedance_param_type',[('stiffness',controller.types.c_double),
                        ('damping',controller.types.c_double),
                        ('eq_angle',controller.types.c_double)])
controller.Define_Type('joint_impedances', [('early_stance', controller.types.impedance_param_type), 
                    ('late_stance', controller.types.impedance_param_type), 
                    ('early_swing', controller.types.impedance_param_type), 
                    ('late_swing', controller.types.impedance_param_type)])
controller.Define_Type('UserParameters', [('knee_impedance', controller.types.joint_impedances),('ankle_impedance', controller.types.joint_impedances)])
controller.Define_Type('sensors', controller.DEFAULT_SENSOR_LIST)

controller.Define_Inputs([('parameters',controller.types.UserParameters), ('sensors',controller.types.sensors)])
controller.Define_Outputs([('knee_impedance', controller.types.impedance_param_type), ('ankle_impedance', controller.types.impedance_param_type)])

# Populate Controller inputs as needed
controller.inputs.parameters.knee_impedance.early_stance.stiffness = 5

# Configure state machine

with osl:
    osl.home()
    osl.calibrate_loadcell()
    osl.knee.set_mode('impedance')
    osl.ankle.set_mode('impedance')

    # Main Loop
    for t in osl.clock:
        # Read from the hardware
        osl.update()

        # sensors = 

        # Call the controller
        # outputs = controller.run(sensors, params)

        # Write to the hardware
        osl.knee.set_joint_impedance(K = outputs.knee_impedance.stiffness,
                                     B = outputs.knee_impedance.damping)
        osl.knee.set_output_position(position = outputs.knee_impedance.eq_angle)
        osl.ankle.set_joint_impedance(K = outputs.ankle_impedance.stiffness,
                                      B = outputs.ankle_impedance.damping)
        osl.ankle.set_output_position(position = outputs.ankle_impedance.eq_angle)
