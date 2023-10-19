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
    You can define these input and output structures however you please.
    See examples folder of repo for examples.

    Kevin Best, Senthur Raj Ayyappan
    Neurobionics Lab
    Robotics Department
    University of Michigan
    October 2023
    """

    def __init__(
        self,
        library_name,
        library_path,
        main_function_name,
        initialization_function_name,
        cleanup_function_name,
    ) -> None:
        """
        Initialize the Controller class.

        Parameters
        ----------
        library_name : string
            The name of the compiled library file, without the *.so
        library_path : string
            The path to the directory containing the library.
            See examples for how to get working directory of parent script.
        main_function_name : string
            Name of the main function to call within the library.
            This is the function that will get called via the run() method
        initialization_function_name : string
            Name of an initialization function for your library. This gets
            called only once when the library is loaded.
            If you don't have an initialization function, pass None.
        cleanup_function_name : string
            Name of a cleanup function for your library. This gets called when
            the CompiledController class has gone out of scope and is
            garbage collected. Again, pass None if you don't need this functionality.
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
        self.DEFAULT_SENSOR_LIST = [
            ("knee_angle", self.types.c_double),
            ("ankle_angle", self.types.c_double),
            ("knee_velocity", self.types.c_double),
            ("ankle_velocity", self.types.c_double),
            ("Fz", self.types.c_double),
        ]

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

    def Define_Inputs(self, input_list: list) -> None:
        """
        This method defines the input structure to your function.
        See example folder and tutorials for help on using this method.

        Parameters
        ------------
        input_list: Input parameters given as a list of [('field_name', field_type)...]
            field_name is a string you choose as the title of the field.
            field_type is a type either given by a native c_types value or
                a custom type defined via the Define_Type() method.
                All types can be accessed as CompiledController.types.(type_name)
        """
        self._input_type = self.Define_Type("inputs", input_list)
        self.inputs = self._input_type()

    def Define_Outputs(self, output_list: list) -> None:
        """
        This method defines the output structure to your function.
        See example folder and tutorials for help on using this method.

        Parameters
        ------------
        output_list: Output parameters given as a list of [('field_name', field_type)...]
            field_name is a string you choose as the title of the field.
            field_type is a type either given by a native c_types value or
                a custom type defined via the Define_Type() method.
                All types can be accessed as CompiledController.types.(type_name)
        """
        self._output_type = self.Define_Type("outputs", output_list)
        self.outputs = self._output_type()

    def Define_Type(self, type_name: str, parameter_list: list):
        """
        This method defines a new type to be used in the compiled controller.
        After calling this method, the datatype with name type_name will be
        available in my_controller.types.type_name for use.
        See example folder and tutorials for help on using this method.

        Parameters
        ------------
        type_name : A string defining the name of your new datatype
        parameter_list: A list of [('field_name', field_type)...]
            field_name is a string you choose as the title of the field.
            field_type is a type either given by a native c_types value or
                a custom type defined via the Define_Type() method.
                All types can be accessed as CompiledController.types.(type_name)

        Example Usage:
            my_controller.DefineType('vector3D', [('x', my_controller.types.c_double),
                                                  ('y', my_controller.types.c_double),
                                                  ('z', my_controller.types.c_double)])
        """
        slots = []
        for param in parameter_list:
            slots.append(param[0])

        class CustomStructure(ctypes.Structure):
            _fields_ = parameter_list
            __slots__ = slots

        setattr(self.types, type_name, CustomStructure)
        return getattr(self.types, type_name)

    def run(self):
        """
        This method calls the main controller function of the library.
        Under the hood, it calls library_name.main_function_name(*inputs, *outputs),
        where library_name and main_function_name were given in the constructor.

        Parameters -> None

        Returns:
            The output structure as defined by the Define_Outputs() method.

        Raises:
            ValueError: If Define_Inputs() or Define_Outputs() have not been called.
        """
        if self.inputs is None:
            raise ValueError(
                "Must define input type before calling controller.run(). Use Define_Inputs() method."
            )
        if self.outputs is None:
            raise ValueError(
                "Must define output type before calling controller.run(). Use Define_Outputs() method."
            )
        self.main_func(ctypes.byref(self.inputs), ctypes.byref(self.outputs))
        return self.outputs


if __name__ == "__main__":

    pass
