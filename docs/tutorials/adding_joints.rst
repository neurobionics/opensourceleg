.. _adding_joints_tutorial:

Adding an Actuator
==================
In this tutorial, we'll show you how to add a joint to your open-source leg using the **opensourceleg** library.

Import the OpenSourceLeg Class
------------------------------

To get started, we need to import the ``OpenSourceLeg`` class, which provides an interface for controlling the open-source leg.

.. code-block:: python

    from opensourceleg.osl import OpenSourceLeg

Create an instance of the OpenSourceLeg Class
---------------------------------------------

Next, we need to create an OpenSourceLeg object. This object represents the open-source leg, provides methods for controlling its joint, and provides methods for a variety of other tasks.

.. code-block:: python

    osl = OpenSourceLeg(frequency=200, file_name="getting_started.log")

This will create an OpenSourceLeg object with a `frequency of 200 Hz` and a log file named ``getting_started.log``.

Add a Joint Object
-------------------

To add a joint to the open-source leg, we can use the ``add_joint`` method of the OpenSourceLeg object. 

.. code-block:: python

    osl.add_joint(name="knee", gear_ratio=41.99, has_loadcell=False)

This will add a joint object named ``knee`` with a gear ratio of ``41.99`` to the `osl` object. You can also specify the **port** the joint is connected to using the ``port`` parameter
of the ``add_joint`` method. If you don't specify a port, the joint will be connected to the first available port.


.. Note::

    The ``has_loadcell`` parameter indicates whether or not the actuator has a load cell connected to it via an FFC cable. This feature is only supported by the Dephy actuators. If you 
    are using a TMotor actuator, this parameter should always be set to ``False``. We'll discuss the loadcell in more detail in a later tutorial.

You can also add the ankle joint to the open-source leg by calling the ``add_joint`` method again. If the ``port`` parameter is not specified, the joint will use the next available port.

.. code-block:: python

    osl.add_joint(name="ankle", gear_ratio=41.99, has_loadcell=False)

.. Warning::
    Please ensure that you are powering-on the actuators in the order of initialization, i.e. if you are initializing the knee joint first, then the knee joint should be powered-on first.


Code for this tutorial
----------------------

.. literalinclude:: ../../tutorials/adding_joints.py
    :language: python
    :linenos:
