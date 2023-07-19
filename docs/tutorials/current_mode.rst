Current Mode Controller
--------------------------

In this tutorial, we'll show you how to use the `OpenSourceLeg` library to control the current of a joint.

.. rubric:: Step 1: Import the OpenSourceLeg Class

To use the `OpenSourceLeg` class, we first need to import it from the `opensourceleg.osl` module:

.. code-block:: python

    from opensourceleg.osl import OpenSourceLeg

.. rubric:: Step 2: Create an instance of the OpenSourceLeg Class and add a Joint Object

Once we have imported the `OpenSourceLeg` class, we can create an instance of the class with the desired frequency and joint configuration:

.. code-block:: python

    osl = OpenSourceLeg(frequency=200)
    osl.add_joint(gear_ratio=9.0)

In this code, we create an `OpenSourceLeg` object named `osl` with a frequency of 200 and a joint with a gear ratio of 9.0.

.. rubric:: Step 3: Setting Units for the `position` Attribute

We can set the units for the `position` attribute of the `osl` object using the `units` dictionary:

.. code-block:: python

    osl.units["position"] = "deg"
    osl.log.info(osl.units)

In this code, we set the units for the `position` attribute to "deg" and log the units to the console.

.. rubric:: Step 4: Controlling Joint Current

To control the current of a joint, we can use a `with` block to ensure that the `OpenSourceLeg` object is properly opened and cleaned up after use:

.. code-block:: python

    with osl:
        osl.knee.set_mode("current")
        
        for t in osl.clock:
            osl.knee.set_current(400)
            osl.log.info(osl.knee.motor_position)
            osl.update()

In this code, we enter a `with` block that sets the mode of the `knee` joint to "current". We then loop over the `osl.clock` generator, which generates a sequence of timestamps at the specified frequency, and set the current of the `knee` joint to 400. We then log the motor position of the `knee` joint to the console at each timestamp. We then call the `osl.update()` method to update the state of the `OpenSourceLeg` object.

Note that this code assumes that the `OpenSourceLeg` object is properly configured and calibrated, and that the joint is properly connected and functioning.

.. literalinclude:: ../../tutorials/current_mode.py
    :language: python
    :linenos: