from typing import Any

import ctypes

import numpy.ctypeslib as ctl


class CompiledController:
    """
    Controller class to handle using compiled controllers.
    This class expects that your function has the form: myFunction(*inputs, *outputs)
    where *inputs is a pointer to an inputs structure and
    *outputs is a pointer to an outputs structure.
    You can define these input and output structures however you please.
    See examples folder of repo for examples.

    Parameters:
        library_name (string): The name of the compiled library file, without the *.so
        library_path (string): The path to the directory containing the library. See examples for how to get working directory of parent script.
        main_function_name (string): Name of the main function to call within the library. This is the function that will get called via the run() method
        initialization_function_name (string): Name of an initialization function for your library. This gets called only once when the library is loaded. If you don't have an initialization function, pass None.
        cleanup_function_name (string): Name of a cleanup function for your library. This gets called when the CompiledController class has gone out of scope and is garbage collected. Again, pass None if you don't need this functionality.

    Authors:
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
        initialization_function_name=None,
        cleanup_function_name=None,
    ) -> None:
        self.cleanup_func = None
        self.lib = ctl.load_library(library_name, library_path)
        self.cleanup_func = self._load_function(cleanup_function_name)
        self.main_function = self._load_function(main_function_name)
        self.init_function = self._load_function(initialization_function_name)
        # Note if requested function name is None, returned handle is also none

        if not self.init_function == None:
            self.init_function()

        # This alias makes defining types from top script easier without second import
        self.types = ctypes

        self.DEFAULT_SENSOR_LIST = [
            ("knee_angle", self.types.c_double),
            ("ankle_angle", self.types.c_double),
            ("knee_velocity", self.types.c_double),
            ("ankle_velocity", self.types.c_double),
            ("Fz", self.types.c_double),
        ]

        self._input_type = None
        self.inputs = None
        self._output_type = None
        self.outputs = None

    def __del__(self):
        if not self.cleanup_func == None:
            self.cleanup_func()

    def __repr__(self):
        return f"CompiledController"

    def _load_function(self, function_name):
        if function_name == None:
            return None
        else:
            try:
                function_handle = getattr(self.lib, function_name)
            except AttributeError:
                raise AttributeError(
                    f"Function {function_name} not found in library {self.lib}"
                )
            return function_handle

    def define_inputs(self, input_list: list[Any]) -> None:
        """
        This method defines the input structure to your function.
        See example folder and tutorials for help on using this method.

        Parameters
        -----------
        input_list: Input parameters given as a list of [('field_name', field_type)...]
            field_name is a string you choose as the title of the field.
            field_type is a type either given by a native c_types value or
                a custom type defined via the define_type() method.
                All types can be accessed as CompiledController.types.(type_name)
        """
        self._input_type = self.define_type("inputs", input_list)
        self.inputs = self._input_type()  # type: ignore

    def define_outputs(self, output_list: list[Any]) -> None:
        """
        This method defines the output structure to your function.
        See example folder and tutorials for help on using this method.

        Parameters
        ------------
        output_list: Output parameters given as a list of [('field_name', field_type)...]
            field_name is a string you choose as the title of the field.
            field_type is a type either given by a native c_types value or
                a custom type defined via the define_type() method.
                All types can be accessed as CompiledController.types.(type_name)
        """
        self._output_type = self.define_type("outputs", output_list)
        self.outputs = self._output_type()  # type: ignore

    def define_type(self, type_name: str, parameter_list: list[Any]):
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
                a custom type defined via the define_type() method.
                All types can be accessed as CompiledController.types.(type_name)

        Example Usage
        ------------
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
            The output structure as defined by the define_outputs() method.

        Raises:
            ValueError: If define_inputs() or define_outputs() have not been called.
        """
        if self.inputs is None:
            raise ValueError(
                "Must define input type before calling controller.run(). Use define_inputs() method."
            )
        if self.outputs is None:
            raise ValueError(
                "Must define output type before calling controller.run(). Use define_outputs() method."
            )
        self.main_function(ctypes.byref(self.inputs), ctypes.byref(self.outputs))
        return self.outputs


if __name__ == "__main__":
    pass
