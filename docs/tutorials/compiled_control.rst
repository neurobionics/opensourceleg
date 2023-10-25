Compiled Controllers
====================
The ``opensourceleg.control`` module provides functionality for using controllers written in languages other than python via the ``CompiledController`` class.
This class wraps a function from a compiled dynamic library and allows it to be called from your main python code. 
Using a compiled controller written in another language can be beneficial, especially when speed is a concern. 

Make an Example library
-------------------------
In this tutorial, we will write an example linear algebra library in ``c++`` that provides a function to compute the dot product between two 3D vectors. 
This very simple function will allow us to show how to define input and output structures and call the controller. More complex examples using actual controllers in both ``c++`` and MATLAB implementations are provided in the examples folder. 

The ``CompiledController`` class assumes that you have a compiled dynamic library (with extension ``*.so`` on linux) with the function prototype ``myFunction(Inputs* inputs, Outputs* outputs)``, where ``Inputs`` and ``Outputs`` are structures holding all of the input and output data. Thus, we first need to write and compile our library. 

.. literalinclude:: ../../tutorials/compiled_control/dot_product_3d.cpp
    :language: cpp
    :linenos:

First, navigate to the directory ``opensourceleg/tutorials/compiled_control/``. Then run ``make`` to build the library. If succesful, a new library named ``lin_alg.so`` should be created. 

Load the Example Library
-------------------------
Next, we need to write a python script to call our newly compiled library. First, we import the library. We also import ``os`` to get the path of the current directory. 

.. code-block:: python

    import os

    from opensourceleg.control.compiled_controller import CompiledController

Then we'll make an instance of the ``CompiledController`` wrapper and have it load our linear algebra library.
We need to pass it both the name of the library (without an extension) and the directory in which to find the library. 
We also give it the name of our main function as well as any initialization and cleanup functions. 

.. code-block:: python
    
    my_linalg = CompiledController(
        library_name="lin_alg.so",
        library_path=os.path.dirname(__file__),
        main_function_name="dot_product_3d",
        initialization_function_name=None,
        cleanup_function_name=None,
    )   

.. Note::

    If your library provides initialization and cleanup functions, they will be called upon loading and cleanup, respectively. 
    If your library does not need these functions, pass the default argument of `None`. 

Define Custom Datatypes
----------------------
Our library uses a `Vector3D` structure, which we need to define so that the python code can pass the data to the library 
in the right format. Every structure is built using basic types from ``my_linalg.types``,
such as `c_double`, `c_bool`, `c_int16`, etc. 
We therefore can add `Vector3D` to the list of known types using the ``define_type()`` method, which
takes two arguments: (1) a name of the new type definition, and (2) a list of tuples where the first entry is the name 
of the field and the second entry is the type. For example, the code to define `Vector3D` is

.. code-block:: python
    
    my_linalg.define_type(
    "Vector3D",
        [
            ("x", my_linalg.types.c_double),
            ("y", my_linalg.types.c_double),
            ("z", my_linalg.types.c_double),
        ],
    )

Now the wrapper knows how `Vector3D` is defined, we can use it in other type definitions, the same way as any other basic type. 
After all necessary types are defined, we need to define the input and output structures using the ``define_inputs()`` and ``define_outputs()`` methods. 
These methods are similar to ``define_type()``, but are special because they tell the wrapper which objects to pass to and from 
the compiled library. We define the inputs as two `Vector3D` objects and the output as one double titled result. 

.. code-block:: python

    my_linalg.define_inputs([("vector1", my_linalg.types.Vector3D),
                             ("vector2", my_linalg.types.Vector3D)])
    my_linalg.define_outputs([("result", my_linalg.types.c_double)])

Populate Inputs and Test the Function
-------------------------------------
Now that the input structure has been defined, we can write to the inputs structure at ``my_linalg.inputs``.
First, we declare two vectors and populate their fields with the appropriate values. 

.. code-block:: python

    vector1 = my_linalg.types.Vector3D()
    vector2 = my_linalg.types.Vector3D()
    vector1.x = 0.6651
    vector1.y = 0.7395
    vector1.z = 0.1037
    vector2.x = -0.7395
    vector2.y = 0.6716
    vector2.z = -0.0460

Then we can assign those vectors to the input structure. 

.. code-block:: python

    my_linalg.inputs.vector1 = vector1  
    my_linalg.inputs.vector2 = vector2  

Finally, we can run the dot product function and print the result from the output structure. 
As our input vectors were orthogonal, we get the expected result of zero. 

.. code-block:: python

    outputs = my_linalg.run()

    print(f"Dot product: {outputs.result}")

    Dot product: 3.6549999999971154e-05

Code for this tutorial
----------------------

.. literalinclude:: ../../tutorials/compiled_control/compiled_control.py
    :language: python
    :linenos:
