Finite State Machine Controller
================================
The library ships with three example implementations of the same finite state machine walking controller. 
The first implementation is all in Python, and it uses the ``state_machine`` modules from the controls subpackage of this library. 
If you plan to write your controllers exclusively in Python, this example would be a good place to start. 

The library also provides support for using compiled `C` and `C++` library functions through the ``compiled_controller`` modules.
You can see a very basic example usage of this module on the tutorials page, which may be helpful to walk through before starting with this example (:ref:`compiled_controller_tutorial_doc`).
The source code for the compiled controllers (C++ and MATLAB implementations) is available in `this repository
<https://github.com/neurobionics/OSL_CompiledControllers_Source>`_.
Please refer to the documentation in that repository for information on how to compile both the C++ and MATLAB source code. 



.. To include a diagram here of the state mahcine that we're using. 

Python Implementation
-----------------------
Senthur TODO!

C++ and MATLAB Implementation
-----------------------------------
To get started, make sure you have compiled either the `C++` or the `MATLAB` source code and have a `FSM_WalkingController.so` library. 
If not, please see the source repository for compilation instructions.
To run this example as is, copy the library you generated to the `examples` directory. 
Alternatively, you can modify the search path for the library when loading the controller (see below). 

Load Compiled Library
^^^^^^^^^^^^^^^^^^^^^

First, we'll perform standard imports, handle some paths, and setup our OSL object. 
If instantiating the OSL object is unfamiliar, check out the :ref:`adding_joints_tutorial` and :ref:`adding_loadcell_tutorial` tutorial pages.

.. literalinclude:: ../../examples/fsm_walking_compiled_controller.py
    :language: python
    :linenos:
    :lines: 11-44

Next, we'll instantiate a ``CompiledController`` wrapper object. This takes arguments of 
the name of the library (without extension), the path at which it is located (which in this case is the current directory),
and the names of the main, initializaiton, and cleanup functions. 

.. literalinclude:: ../../examples/fsm_walking_compiled_controller.py
    :language: python
    :linenos:
    :lines: 46-52

Define Custom Datatypes
^^^^^^^^^^^^^^^^^^^^^^^^

Next, we define the data structures used in the controller. These must exactly match (size and order) what was used to create the library.
First, we define a type for a single set of impedance parameters called ``impedance_param_type``, where `stiffness`, `damping`, and `eq_angle` are each doubles.
The ``CompiledController`` object provides building block types within its ``types`` attribute. 

.. literalinclude:: ../../examples/fsm_walking_compiled_controller.py
    :language: python
    :linenos:
    :lines: 54-61

We then define the type ``joint_impedance_set`` which contains a set of impedance parameters for each of the four states. 
Because we have already defined the ``impedance_param_type`` in the previous lines, we can now use it to define additional types. 

.. literalinclude:: ../../examples/fsm_walking_compiled_controller.py
    :language: python
    :linenos:
    :lines: 62-70

We then similarly define the ``transition_parameters`` type as well as the overall ``UserParameters`` type.

.. literalinclude:: ../../examples/fsm_walking_compiled_controller.py
    :language: python
    :linenos:
    :lines: 71-92

We also define a sensors type. We're using the default sensors, which the ``CompiledController`` class provides in list form already setup correctly. 

.. literalinclude:: ../../examples/fsm_walking_compiled_controller.py
    :language: python
    :linenos:
    :lines: 93

The final type definitions are for the input and output types. 

.. literalinclude:: ../../examples/fsm_walking_compiled_controller.py
    :language: python
    :linenos:
    :lines: 95-109

Configure Impedance and Transition Parameters 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The next section of code configures the impedance and transition paramters based on a pre-defined tuning.
Feel free to play with these values to get the behavior you want.

.. literalinclude:: ../../examples/fsm_walking_compiled_controller.py
    :language: python
    :linenos:
    :lines: 111-147

Main Loop
^^^^^^^^^^^

Now that the controller is configured, we can home the osl, calibrate the loadcell, set both joints to impedance mode, and begin running. 
Note that within the main loop after calling ``osl.update()``, we assign the relevant sensor values from the ``osl`` object to the controller inputs. 
We also write to any other inputs that change every loop, such as the current time ``t``. 
Once all inputs are assigned, we call the ``run()`` method, which calls our compiled library with the configured inputs.
The run method returns the outputs object, which is populated by the compiled library function. 


.. literalinclude:: ../../examples/fsm_walking_compiled_controller.py
    :language: python
    :linenos:
    :lines: 149-180

.. note:: 
    The OSL library provides sensor values in default units. If your library expects other units,
    you need to convert the values prior to assigning them. You can use the ``units`` module in the ``tools`` subpackage to do this.
    For example you can convert from radians (the default) to degrees using ``ankle_angle_in_deg = units.convert_from_default(osl.ankle.output_position, units.position.deg)``.

Finally, we write from the outputs structure to the hardware.
Be careful with units at this step as well, making sure your values are either in default 
units, or that you call appropriate conversion functions. As our library was in degrees, 
we convert to radians. 

.. literalinclude:: ../../examples/fsm_walking_compiled_controller.py
    :language: python
    :linenos:
    :lines: 182-208

Full Code for Compiled Implementation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../examples/fsm_walking_compiled_controller.py
    :language: python
    :linenos: