import ctypes
from dataclasses import dataclass

import flexsea.fx_enums as fxe
import numpy.ctypeslib as ctl


@dataclass
class ControlModes:
    """
    Control modes for the Dephy Actpack.

    Available modes are Voltage, Current, Position, Impedance.
    """

    voltage: ctypes.c_int = fxe.FX_VOLTAGE
    current: ctypes.c_int = fxe.FX_CURRENT
    position: ctypes.c_int = fxe.FX_POSITION
    impedance: ctypes.c_int = fxe.FX_IMPEDANCE


CONTROL_MODE = ControlModes()


@dataclass
class Gains:
    """
    Dataclass for controller gains

    Args:
        kp (int): Proportional gain
        ki (int): Integral gain
        kd (int): Derivative gain
        K (int): Stiffness of the impedance controller
        B (int): Damping of the impedance controller
        ff (int): Feedforward gain
    """

    kp: int = 0
    ki: int = 0
    kd: int = 0
    K: int = 0
    B: int = 0
    ff: int = 0

    def __repr__(self) -> str:
        return f"kp={self.kp}, ki={self.ki}, kd={self.kd}, K={self.K}, B={self.B}, ff={self.ff}"

class CompiledController:
    """
    Controller class to handle using compiled controllers.
    This class expects that your function has the form:
        myFunction(*inputs, *outputs)
    where *inputs is a pointer to an inputs structure and 
    *outputs is a pointer to an outputs structure. 
    """

    def __init__(self, library_name , library_path, main_function_name, initialization_function_name, cleanup_function_name) -> None:
        """
        Initialize the Controller class.

        Parameters
        ----------
        main_function_name : string
            Name of the main function to call within the library
        controller_path : string
            Full path to the *.so object. 
        """
        # Load functions
        self.lib = ctl.load_library(library_name, library_path)
        self.cleanup_func = self._load_function(cleanup_function_name)
        self.main_func = self._load_function(main_function_name)
        self.init_func = self._load_function(initialization_function_name)
        
        # Call init function if there is one
        if not self.init_func == None:
            self.init_func()

        # Alias the ctypes class as member of this class
        self.types = ctypes

        # Define default sensor list
        self.DEFAULT_SENSOR_LIST = [('knee_angle',self.types.c_double),
                                    ('ankle_angle',self.types.c_double), 
                                    ('knee_velocity', self.types.c_double),
                                    ('ankle_velocity', self.types.c_double),
                                    ('Fz', self.types.c_double)]
        
        # Initialize input and output lists
        self._input_type = None
        self.inputs = None
        self._output_type = None
        self.outputs = None

    def __del__(self):
        if not self.cleanup_func == None:
            self.cleanup_func()

    def _load_function(self, function_name):
        if function_name == None:
            return None
        else:
            return getattr(self.lib, function_name)

    def Define_Inputs(self, input_list):
        self._input_type = self.Define_Type('inputs', input_list)
        self.inputs = self._input_type()

    def Define_Outputs(self, output_list): 
        self._output_type = self.Define_Type('outputs',output_list)
        self.outputs = self._output_type()

    def Define_Type(self, type_name, parameter_list):
        slots = []
        for param in parameter_list:
                slots.append(param[0])
        class ctypes_structure(ctypes.Structure):
            _fields_ = parameter_list
            __slots__ = slots
        setattr(self.types, type_name, ctypes_structure)
        return getattr(self.types, type_name)
    
    def run(self):
        if self.inputs is None:
            raise ValueError('Must define input type before calling controller.run(). Use Define_Inputs() method.')
        if self.outputs is None:
            raise ValueError('Must define output type before calling controller.run(). Use Define_Outputs() method.')
        self.main_func(ctypes.byref(self.inputs), ctypes.byref(self.outputs))
        return self.outputs

if __name__ == "__main__":

    pass
