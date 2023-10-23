Compiled Controllers
--------------------
The ``opensourceleg.control`` module provides functionality for using controllers written in languages other than python via the ``CompiledController`` class.
This class wraps a function from a compiled dynamic library and allows it to be called from your main python code. 
Using a compiled controller written in another language can be beneficial, especially when speed is a concern. 

In this tutorial, we will write an example linear algebra library in ``c++`` that provides a function to compute the dot product between two 3D vectors. This very simple function will allow us to show how to define input and output structures and call the controller. More complex examples using actual controllers in both ``c++`` and MATLAB implementations are provided in the examples folder. 

The ``CompiledController`` class assumes that you have a compiled dynamic library (with extension ``*.so`` on linux) with the function prototype ``myFunction(Inputs* inputs, Outputs* outputs)``, where ``Inputs`` and ``Outputs`` are structures holding all of the input and output data. Thus, we first need to write and compile our library. 

.. literalinclude:: ../../tutorials/compiled_control/dot_product_3d.cpp
    :language: python
    :linenos:

First, navigate to the directory ``opensourceleg/tutorials/compiled_control/``. Then run ``make`` to build the library. If succesful, a new library named ``lin_alg.so`` should be created. 

Next, we need to write a python script to call our newly compiled library. 
