"""
Example walking controller for the OSL. 

Senthur Raj Ayyappan, Kevin Best
University of Michigan
October 9, 2023
"""

# Imports
import sys
sys.path.append("../")
from opensourceleg.osl import OpenSourceLeg
import ctypes

# Initialization
osl = OpenSourceLeg(frequency=200)
osl.add_joint('knee', 9*83/18)
osl.add_joint('ankle',9*83/18)
osl.add_loadcell(joint=osl.knee)
osl.units["position"] = "deg"  # type: ignore
osl.units["velocity"] = "deg/s"  # type: ignore

# Define function interface
sensors = osl.control.default_sensors

# Define custom types for parameters and outputsas list of tuples
impedance_param_type = [('stiffness',ctypes.c_double), 
                        ('damping',ctypes.c_double), 
                        ('eq_angle',ctypes.c_double)]
joint_impedances = [('early_stance_impedance', impedance_param_type), 
                    ('late_stance_impedance', impedance_param_type), 
                    ('early_swing_impedance', impedance_param_type), 
                    ('late_swing_impedance', impedance_param_type)]
params = [('knee_impedance', joint_impedances), ('ankle_impedance', joint_impedances)]
outputs = [('knee_impedance', impedance_param_type), ('ankle_impedance', impedance_param_type)]

# Instantiate controller
controller = osl.control.load_library_controller('MATLAB_FMS_walking_controller.so', sensors, params, outputs)

with osl:
    osl.home()
    osl.calibrate_loadcell()

    # Main Loop
    for t in osl.clock:
        osl.update()
        outputs = controller.run(sensors, params)
