import ctypes
from typing import Any, ClassVar
from unittest.mock import MagicMock

import numpy.ctypeslib as ctl
import pytest

from opensourceleg.control.compiled import CompiledController


def test___init__(monkeypatch):
    # Mock the library and its functions
    mock_lib = MagicMock()

    # Replace load_library with a mock that returns mock_lib
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    # Mock functions in the library
    mock_init_function = MagicMock()
    mock_cleanup_function = MagicMock()
    mock_main_function = MagicMock()
    mock_lib.init_func = mock_init_function
    mock_lib.cleanup_func = mock_cleanup_function
    mock_lib.main_func = mock_main_function

    # Initialize CompiledController with all functions
    controller = CompiledController(
        library_name="test_lib",
        library_path="/path/to/lib",
        main_function_name="main_func",
        initialization_function_name="init_func",
        cleanup_function_name="cleanup_func",
    )

    # Assertions
    assert controller.lib == mock_lib
    assert controller.init_function == mock_init_function
    assert controller.cleanup_func == mock_cleanup_function
    assert controller.main_function == mock_main_function
    mock_init_function.assert_called_once()

    # Initialize CompiledController without init and cleanup functions
    controller_no_init = CompiledController(
        library_name="test_lib",
        library_path="/path/to/lib",
        main_function_name="main_func",
        initialization_function_name=None,
        cleanup_function_name=None,
    )
    assert controller_no_init.init_function is None
    assert controller_no_init.cleanup_func is None


def test___del__(monkeypatch):
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    mock_cleanup_function = MagicMock()
    mock_lib.cleanup_func = mock_cleanup_function

    controller = CompiledController(
        library_name="test_lib",
        library_path="/path/to/lib",
        main_function_name="main_func",
        initialization_function_name=None,
        cleanup_function_name="cleanup_func",
    )

    # Force the __del__ method to be called
    del controller
    import gc

    gc.collect()

    # Check if cleanup_func was called
    mock_cleanup_function.assert_called_once()


def test_define_inputs_outputs_with_ctypes_classes(monkeypatch):
    """Test that define_inputs and define_outputs accept ctypes.Structure classes directly."""
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    mock_main_function = MagicMock()
    mock_lib.main_func = mock_main_function

    controller = CompiledController("test_lib", "/path/to/lib", "main_func")

    # Define simple ctypes.Structure classes
    class Inputs(ctypes.Structure):
        _fields_: ClassVar[list[tuple[str, Any]]] = [("a", ctypes.c_double)]

    class Outputs(ctypes.Structure):
        _fields_: ClassVar[list[tuple[str, Any]]] = [("b", ctypes.c_double)]

    # Pass classes directly
    controller.define_inputs(input_type=Inputs)
    controller.define_outputs(output_type=Outputs)

    # Ensure the classes are registered on controller.types
    assert controller.types.inputs is Inputs
    assert controller.types.outputs is Outputs

    # Ensure instances are created
    assert isinstance(controller.inputs, Inputs)
    assert isinstance(controller.outputs, Outputs)

    # Assign a value and run; ensure main function is called
    controller.inputs.a = 1.23
    controller.main_function = mock_main_function
    controller.run()
    mock_main_function.assert_called_once()


def test___repr__(monkeypatch):
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    controller = CompiledController(
        library_name="test_lib",
        library_path="/path/to/lib",
        main_function_name="main_func",
    )
    assert repr(controller) == "CompiledController"


def test__load_function(monkeypatch):
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    controller = CompiledController(
        library_name="test_lib",
        library_path="/path/to/lib",
        main_function_name="main_func",
    )
    controller.lib = mock_lib
    mock_function = MagicMock()
    controller.lib.existing_function = mock_function

    # Test when function_name is None
    result = controller._load_function(None)
    assert result is None

    # Test when function exists
    result = controller._load_function("existing_function")
    assert result == mock_function

    # # Test when function does not exist
    # with pytest.raises(AttributeError):
    #     controller._load_function('non_existing_function')


def test_define_inputs(monkeypatch):
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    controller = CompiledController(
        library_name="test_lib",
        library_path="/path/to/lib",
        main_function_name="main_func",
    )

    # Valid input_list
    input_list = [("field1", ctypes.c_double), ("field2", ctypes.c_int)]
    controller.define_inputs(input_list=input_list)
    assert controller._input_type is not None
    assert controller.inputs is not None
    assert hasattr(controller.inputs, "field1")
    assert hasattr(controller.inputs, "field2")

    # Empty input_list
    controller.define_inputs(input_list=[])
    assert controller._input_type is not None
    assert controller.inputs is not None

    # Invalid input_list: not a list
    with pytest.raises(TypeError):
        controller.define_inputs(input_list="invalid_input_list")

    # Invalid input_list: elements not tuples
    with pytest.raises(TypeError):
        controller.define_inputs(input_list=[123, "abc"])

    # Invalid input_list: tuple elements are invalid
    with pytest.raises(TypeError):
        controller.define_inputs(input_list=[("field1", "not_a_type")])


def test_define_outputs(monkeypatch):
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    controller = CompiledController(
        library_name="test_lib",
        library_path="/path/to/lib",
        main_function_name="main_func",
    )

    # Valid output_list
    output_list = [("result", ctypes.c_double)]
    controller.define_outputs(output_list=output_list)
    assert controller._output_type is not None
    assert controller.outputs is not None
    assert hasattr(controller.outputs, "result")

    # Empty output_list
    controller.define_outputs(output_list=[])
    assert controller._output_type is not None
    assert controller.outputs is not None

    # Invalid output_list: not a list
    with pytest.raises(TypeError):
        controller.define_outputs(output_list="invalid_output_list")

    # Invalid output_list: elements not tuples
    with pytest.raises(TypeError):
        controller.define_outputs(output_list=[123, "abc"])

    # Invalid output_list: tuple elements are invalid
    with pytest.raises(TypeError):
        controller.define_outputs(output_list=[("result", "not_a_type")])


def test_define_inputs_errors(monkeypatch):
    """Test error cases for define_inputs method."""
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    controller = CompiledController(
        library_name="test_lib",
        library_path="/path/to/lib",
        main_function_name="main_func",
    )

    # Define a ctypes.Structure class for testing
    class TestInput(ctypes.Structure):
        _fields_: ClassVar[list[tuple[str, Any]]] = [("test_field", ctypes.c_double)]

    # Error when both input_list and input_type are provided
    with pytest.raises(ValueError, match="Only one of input_list or input_type should be provided"):
        controller.define_inputs(input_list=[("field1", ctypes.c_double)], input_type=TestInput)

    # Error when neither input_list nor input_type is provided
    with pytest.raises(ValueError, match="Must provide either input_list or input_type"):
        controller.define_inputs()


def test_define_outputs_errors(monkeypatch):
    """Test error cases for define_outputs method."""
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    controller = CompiledController(
        library_name="test_lib",
        library_path="/path/to/lib",
        main_function_name="main_func",
    )

    # Define a ctypes.Structure class for testing
    class TestOutput(ctypes.Structure):
        _fields_: ClassVar[list[tuple[str, Any]]] = [("test_field", ctypes.c_double)]

    # Error when both output_list and output_type are provided
    with pytest.raises(ValueError, match="Only one of output_list or output_type should be provided"):
        controller.define_outputs(output_list=[("field1", ctypes.c_double)], output_type=TestOutput)

    # Error when neither output_list nor output_type is provided
    with pytest.raises(ValueError, match="Must provide either output_list or output_type"):
        controller.define_outputs()


def test_define_type(monkeypatch):
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    controller = CompiledController(
        library_name="test_lib",
        library_path="/path/to/lib",
        main_function_name="main_func",
    )

    # Valid type definition
    parameter_list = [("field1", ctypes.c_double), ("field2", ctypes.c_int)]
    custom_type = controller.define_type("CustomType", parameter_list)
    assert hasattr(controller.types, "CustomType")
    assert custom_type.__name__ == "CustomStructure"
    instance = custom_type()
    assert hasattr(instance, "field1")
    assert hasattr(instance, "field2")

    # Invalid parameter_list: elements not tuples
    with pytest.raises(TypeError):
        controller.define_type("InvalidType", ["field1", "field2"])

    # Invalid parameter_list: param[0] is not a string
    with pytest.raises(TypeError):
        controller.define_type("InvalidType", [(123, ctypes.c_double)])

    # Invalid parameter_list: param[1] is not a ctypes type
    with pytest.raises(TypeError):
        controller.define_type("InvalidType", [("field", "not_a_type")])

    # Invalid type_name: None
    with pytest.raises(TypeError):
        controller.define_type(None, parameter_list)


def test_run(monkeypatch):
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    controller = CompiledController(
        library_name="test_lib",
        library_path="/path/to/lib",
        main_function_name="main_func",
    )
    # Mock main_function
    controller.main_function = MagicMock()

    # Define inputs and outputs
    input_list = [("field1", ctypes.c_double)]
    output_list = [("result", ctypes.c_double)]
    controller.define_inputs(input_list=input_list)
    controller.define_outputs(output_list=output_list)

    # Run the controller
    output = controller.run()

    # Ensure that main_function was called once
    controller.main_function.assert_called_once()

    # Retrieve the arguments that were passed to main_function
    args, kwargs = controller.main_function.call_args

    # Ensure that there are no keyword arguments
    assert kwargs == {}

    # Check that the arguments are pointers to the inputs and outputs
    assert len(args) == 2

    # Since ctypes.byref returns a pointer object, we can check that the _obj attribute matches
    assert args[0]._obj is controller.inputs
    assert args[1]._obj is controller.outputs

    assert output == controller.outputs


def test_library_load_failure(monkeypatch):
    # Simulate library load failure
    def mock_load_library(name, path):
        raise OSError("Library not found")

    monkeypatch.setattr(ctl, "load_library", mock_load_library)

    with pytest.raises(OSError) as exc_info:
        CompiledController(
            library_name="non_existent_lib",
            library_path="/invalid/path",
            main_function_name="main_func",
        )
    assert "Library not found" in str(exc_info.value)


def test_default_sensor_list(monkeypatch):
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    controller = CompiledController(
        library_name="test_lib",
        library_path="/path/to/lib",
        main_function_name="main_func",
    )
    assert isinstance(controller.DEFAULT_SENSOR_LIST, list)
    assert len(controller.DEFAULT_SENSOR_LIST) > 0


def test_define_type_redefinition(monkeypatch):
    """Test redefining a type with the same name."""
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    controller = CompiledController("test_lib", "/path/to/lib", "main_func")

    parameter_list = [("field1", ctypes.c_double)]
    custom_type = controller.define_type("CustomType", parameter_list)
    assert hasattr(controller.types, "CustomType")

    # Redefine the same type
    parameter_list_new = [("field1", ctypes.c_int)]
    custom_type_new = controller.define_type("CustomType", parameter_list_new)
    assert custom_type_new != custom_type
    instance = custom_type_new()
    assert hasattr(instance, "field1")


def test_define_type_inheritance(monkeypatch):
    """Test defining a type that uses another custom type."""
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    controller = CompiledController("test_lib", "/path/to/lib", "main_func")

    # Define a base custom type
    base_parameter_list = [("x", ctypes.c_double), ("y", ctypes.c_double)]
    BaseType = controller.define_type("BaseType", base_parameter_list)
    assert hasattr(controller.types, "BaseType")

    # Define another type using the base type
    parameter_list = [("point", BaseType), ("z", ctypes.c_double)]
    CustomType = controller.define_type("CustomType", parameter_list)
    assert hasattr(controller.types, "CustomType")

    instance = CustomType()
    assert hasattr(instance, "point")
    assert hasattr(instance, "z")
    assert isinstance(instance.point, BaseType)


def test_define_inputs_with_custom_type(monkeypatch):
    """Test define_inputs using a custom type."""
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    controller = CompiledController("test_lib", "/path/to/lib", "main_func")

    # Define a custom type
    parameter_list = [("field1", ctypes.c_double)]
    CustomType = controller.define_type("CustomType", parameter_list)

    # Define inputs using the custom type
    input_list = [("custom_field", CustomType)]
    controller.define_inputs(input_list=input_list)

    assert hasattr(controller.inputs, "custom_field")
    assert isinstance(controller.inputs.custom_field, CustomType)


def test_run_with_exception_in_main_function(monkeypatch):
    """Test handling of exceptions raised by main_function."""
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    controller = CompiledController("test_lib", "/path/to/lib", "main_func")

    # Mock main_function to raise an exception
    def mock_main_function(inputs, outputs):
        raise RuntimeError("Error in main function")

    controller.main_function = mock_main_function

    # Define inputs and outputs
    input_list = [("field1", ctypes.c_double)]
    output_list = [("result", ctypes.c_double)]
    controller.define_inputs(input_list=input_list)
    controller.define_outputs(output_list=output_list)

    # Run and expect exception
    with pytest.raises(RuntimeError) as exc_info:
        controller.run()
    assert "Error in main function" in str(exc_info.value)


def test_cleanup_function_called_on_exception(monkeypatch):
    """Test that cleanup function is called even if an exception occurs."""
    mock_lib = MagicMock()
    monkeypatch.setattr(ctl, "load_library", lambda name, path: mock_lib)

    mock_cleanup_function = MagicMock()
    mock_lib.cleanup_func = mock_cleanup_function

    controller = CompiledController("test_lib", "/path/to/lib", "main_func", cleanup_function_name="cleanup_func")

    # Mock main_function to raise an exception
    def mock_main_function(inputs, outputs):
        raise RuntimeError("Error in main function")

    controller.main_function = mock_main_function

    # Define inputs and outputs
    controller.define_inputs(input_list=[("field1", ctypes.c_double)])
    controller.define_outputs(output_list=[("result", ctypes.c_double)])

    # Run and expect exception
    with pytest.raises(RuntimeError):
        controller.run()

    # Delete controller to trigger cleanup
    del controller
    import gc

    gc.collect()

    # Check if cleanup_func was called
    mock_cleanup_function.assert_called_once()
