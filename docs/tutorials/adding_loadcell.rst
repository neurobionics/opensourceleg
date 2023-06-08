Adding a Loadcell
-----------------
In this tutorial, we'll show you how to add a load cell to your open-source leg using the `opensourceleg` library.

.. rubric:: Step 1: Create an OpenSourceLeg Object

First, we need to create an **OpenSourceLeg** object. This object represents your open-source leg.

.. code-block:: python

    osl = OpenSourceLeg(frequency=200, file_name="getting_started.log")

This will create an **OpenSourceLeg** object with a `frequency of 200 Hz` and a log file named ``getting_started.log``.

.. rubric:: Step 2: Add a Load Cell Object

To add a load cell to the ``osl`` object, we can use the `add_loadcell` method.

.. code-block:: python

    osl.add_loadcell(dephy_mode=False, loadcell_matrix=constants.LOADCELL_MATRIX)


This will add a load cell ``osl`` object with the specified `loadcell_matrix`, which is the calibration matrix for the load cell. This calibration matrix is unique to each load cell and can be found in the load cell's datasheet.
The `dephy_mode` argument is set to `False`, which means that the load cell is connected to the Raspberry Pi via the GPIO pins and not to the Dephy actuator using an FFC cable. If you are using a load cell connected to the Dephy actuator, you should set this argument to `True`.

Here is an example of how you would add a load cell to the `osl` object if you were using a load cell connected to the Dephy actuator:  

.. code-block:: python

    osl.add_joint(name="knee", gear_ratio=41.99, has_loadcell=True)
    osl.add_loadcell(dephy_mode=True, joint=osl.knee, loadcell_matrix=constants.LOADCELL_MATRIX)

This method requires a joint to be added to the `osl` object first. This is because the load cell is connected to the Dephy actuator, which reads the load cell data and streams it to the Raspberry Pi. This joint object is passed to the `add_loadcell` method so that the load cell data can be read from the joint object.

.. Note::
    If you are using a different load cell amplifier, your amplifier gain and excitation voltage may be different. You can change these values by passing the ``amp_gain`` and ``exc`` arguments to the `add_loadcell` method. The default values for these arguments are ``amp_gain=128`` and ``exc=5``.

.. rubric:: Here is the code for this tutorial:

.. literalinclude:: ../../tutorials/adding_loadcell.py
    :language: python
    :linenos:


That's it! You've now added a load cell to your open-source leg using the `opensourceleg` library.
