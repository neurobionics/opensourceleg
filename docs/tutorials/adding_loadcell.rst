.. _adding_loadcell_tutorial:

Adding a Loadcell
=================
In this tutorial, we'll show you how to add a load cell to your open-source leg using the ``opensourceleg`` library.

Create an OpenSourceLeg Object
------------------------------

First, we need to create an ``OpenSourceLeg`` object. This object represents your open-source leg.

.. code-block:: python

    osl = OpenSourceLeg(frequency=200, file_name="getting_started.log")

This will create an ``OpenSourceLeg`` object with a `frequency of 200 Hz` and a log file named ``getting_started.log``.

Add a Load Cell Object
----------------------

To add a load cell to the ``osl`` object, we can use the ``add_loadcell`` method.

.. code-block:: python

    LOADCELL_MATRIX = np.array(
        [
            (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
            (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
            (
                -1047.16800,
                8.63900,
                -1047.28200,
                -20.70000,
                -1073.08800,
                -8.92300,
            ),
            (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
            (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
            (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
        ]
    )

    osl.add_loadcell(dephy_mode=False, loadcell_matrix=LOADCELL_MATRIX)


This will add a loadcell to the ``osl`` object with the specified ``loadcell_matrix``, which is the calibration matrix for the loadcell. This calibration matrix is unique to each load cell and can be found in the loadcell's datasheet.
The ``dephy_mode`` argument is set to ``False``, which means that the loadcell is connected to the Raspberry Pi via the GPIO pins and not to the Dephy actuator using an FFC cable. If you are using a loadcell connected to the Dephy actuator, you should set this argument to ``True``.

Here is an example of how you would add a load cell to the `osl` object if you were using a loadcell connected to the Dephy actuator:  

.. code-block:: python

    osl.add_joint(name="knee", gear_ratio=41.99, has_loadcell=True)
    osl.add_loadcell(dephy_mode=True, joint=osl.knee, loadcell_matrix=LOADCELL_MATRIX)

This method requires a joint to be added to the `osl` object first. This is because the loadcell is connected to the Dephy actuator, which reads the loadcell data and streams it to the Raspberry Pi. This joint object is passed to the ``add_loadcell`` method so that the loadcell data can be read from the joint object.

.. Note::
    If you are using a different loadcell amplifier, your amplifier gain and excitation voltage may be different. You can change these values by passing the ``amp_gain`` and ``exc`` arguments to the ``add_loadcell`` method. The default values for these arguments are ``amp_gain=125`` and ``exc=5``.

Code for this tutorial
----------------------

.. literalinclude:: ../../tutorials/adding_loadcell.py
    :language: python
    :linenos:


That's it! You've now added a loadcell to your open-source leg using the ``opensourceleg`` library.
