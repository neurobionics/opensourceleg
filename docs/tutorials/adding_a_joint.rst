Adding a joint
==============
In this tutorial, we'll show you how to add a joint to your open-source leg using the **opensourceleg** library.

.. rubric:: Step 1: Import the OpenSourceLeg Class

To get started, we need to import the ``OpenSourceLeg`` class, which provides an interface for controlling the open-source leg.

.. code-block:: python

    from opensourceleg import OpenSourceLeg

.. rubric:: Step 2: Create an instance of the OpenSourceLeg Class

Next, we need to create an OpenSourceLeg object. This object represents the open-source leg, provides methods for controlling its joint, and provides methods for a variety of other tasks.

.. code-block:: python

    osl = OpenSourceLeg(frequency=200, file_name="getting_started.log")

This will create an OpenSourceLeg object with a `frequency of 200 Hz` and a log file named ``getting_started.log``.

.. rubric:: Step 3: Add a joint

To add a joint to the open-source leg, we can use the ``add_joint`` method of the OpenSourceLeg object. 

.. code-block:: python

    osl.add_joint(name="knee", gear_ratio=41.99)

This will add a joint named ``knee`` with a gear ratio of ``41.99`` to the `osl` object.

.. rubric:: Here is the complete code for this tutorial

.. literalinclude:: ../../tutorials/adding_a_joint.py
    :language: python
    :linenos:
