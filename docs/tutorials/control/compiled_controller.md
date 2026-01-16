# Compiled Controllers

The `opensourceleg.control` module provides functionality for using controllers written in languages other than Python via the `CompiledController` class. This class wraps a function from a compiled dynamic library and allows it to be called from your main Python code. Using a compiled controller written in another language can be beneficial, especially when speed is a concern.

## Make an Example Library

In this tutorial, we will write an example linear algebra library in `C++` that provides a function to compute the dot product between two 3D vectors. This very simple function will allow us to show how to define input and output structures and call the controller. The full script for this example is available at the end of this file. More complex examples using an actual Finite State Machine controller in both `C++` and MATLAB implementations are provided in the examples folder.

The `CompiledController` class assumes that you have a compiled dynamic library (with extension `*.so` on Linux) with the function prototype `myFunction(Inputs* inputs, Outputs* outputs)`, where `Inputs` and `Outputs` are structures holding all of the input and output data. Thus, we first need to write and compile our library.

```cpp
--8<-- "tutorials/control/compiled_controllers/dot_product_3d.cpp"
```

> **Note**:
> The `extern "C"` linkage-specification is important to prevent the `C++` compiler from name mangling.
> Under the hood, our library uses a `C` style calling convention, which expects to be able to find the library functions with their standard names.

First, navigate to the directory `opensourceleg/tutorials/compiled_control/`. Then run `make` to build the library. If successful, a new library named `lin_alg.so` should be created.

## Load the Example Library

Next, we need to write a Python script to call our newly compiled library. First, we import the library. We also import `os` to get the path of the current directory.

```python
--8<-- "tutorials/control/compiled_controllers/compiled_control.py:1:3"
```

Then we'll make an instance of the ``CompiledController`` wrapper and have it load our linear algebra library.
We need to pass it both the name of the library (without an extension) and the directory in which to find the library.
We also give it the name of our main function as well as any initialization and cleanup functions.

```python
--8<-- "tutorials/control/compiled_controllers/compiled_control.py:5:11"
```

> **Note**:  If your library provides initialization and cleanup functions, they will be called upon loading and cleanup, respectively.
If your library does not need these functions, pass the default argument of `None`.


## Define Custom Datatypes
Our library uses a `Vector3D` structure, which we need to define so that the python code can pass the data to the library
in the right format. Every structure is built using basic types from ``my_linalg.types``,
such as `c_double`, `c_bool`, `c_int16`, etc.
We therefore can add `Vector3D` to the list of known types using the ``define_type()`` method, which
takes two arguments: (1) a name of the new type definition, and (2) a list of tuples where the first entry is the name
of the field and the second entry is the type. For example, the code to define `Vector3D` is

```python
--8<-- "tutorials/control/compiled_controllers/compiled_control.py:13:20"
```

Now the wrapper knows how `Vector3D` is defined, we can use it in other type definitions, the same way as any other basic type.
After all necessary types are defined, we need to define the input and output structures using the ``define_inputs()`` and ``define_outputs()`` methods.
These methods are similar to ``define_type()``, but are special because they tell the wrapper which objects to pass to and from
the compiled library. We define the inputs as two `Vector3D` objects and the output as one double titled result.

### Option 1: Using Lists (Recommended for Simple Structures)
```python
--8<-- "tutorials/control/compiled_controllers/compiled_control.py:21:22"
```

### Option 2: Using ctypes.Structure Classes (Recommended for Complex Structures)
Alternatively, you can define custom `ctypes.Structure` classes and pass them directly to ``define_inputs()`` and ``define_outputs()``.
This approach is useful when you have complex nested structures that you want to define once and reuse:

```python
# Define custom ctypes structures
class Vector3D(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("z", ctypes.c_double),
    ]

class InputsType(ctypes.Structure):
    _fields_ = [
        ("vector1", Vector3D),
        ("vector2", Vector3D),
    ]

class OutputsType(ctypes.Structure):
    _fields_ = [
        ("result", ctypes.c_double),
    ]

# Pass the classes directly to define_inputs and define_outputs
my_linalg.define_inputs(InputsType)
my_linalg.define_outputs(OutputsType)
```

Both approaches are functionally equivalent. Choose the one that best fits your code style and complexity needs.

## Populate Inputs and Test the Function

Now that the input structure has been defined, we can write to the inputs structure at ``my_linalg.inputs``.
First, we declare two vectors and populate their fields with the appropriate values.

```python
--8<-- "tutorials/control/compiled_controllers/compiled_control.py:24:32"
```

Then we can assign those vectors to the input structure.

```python
--8<-- "tutorials/control/compiled_controllers/compiled_control.py:34:35"
```

Finally, we can run the dot product function and print the result from the output structure.
As our input vectors were orthogonal, we get the expected result of zero.

```python
--8<-- "tutorials/control/compiled_controllers/compiled_control.py:37:40"
```

We get the following output, showing that our compiled library successfully calculated the dot product between our two orthogonal vectors:
```bash
Dot product: 3.6549999999971154e-05
```

## Full Code for This Tutorial
```python
--8<-- "tutorials/control/compiled_controllers/compiled_control.py"
```
