Commanding Position
--------------------------
In this tutorial, we'll show you how to control your open-source leg in position mode usin the `opensourceleg` library.

.. rubric:: Step 1: Importing the `OpenSourceLeg` Class

To use the `OpenSourceLeg` class, we first need to import it from the `opensourceleg.osl` module:

.. code-block:: python

    from opensourceleg.osl import OpenSourceLeg

.. rubric:: Step 2: Creating an `OpenSourceLeg` Object

Once we have imported the `OpenSourceLeg` class, we can create an instance of the class with the desired frequency and joint configuration:

.. code-block:: python

    osl = OpenSourceLeg(frequency=200)  # 200 Hz
    osl.add_joint(gear_ratio=9.0)

In this code, we create an `OpenSourceLeg` object named `osl` with a frequency of 200 Hz and a joint with a gear ratio of 9.0.

.. rubric:: Step 3: Setting Units for the `position` Attribute

We can set the units for the `position` attribute of the `osl` object using the `units` dictionary:

.. code-block:: python

    osl.units["position"] = "deg"
    osl.log.info(osl.units)

In this code, we set the units for the `position` attribute to "deg" and log the units to the console.

.. rubric:: Step 4: Controlling Joint Position

To control the position of a joint, we can use a `with` block to ensure that the `OpenSourceLeg` object is properly opened and cleaned up after use:

.. code-block:: python

    set_point = 50  # motor ticks

    with osl:
        osl.knee.set_mode("position")
        osl.knee.set_motor_position(osl.knee.motor_position + set_point)
        
        for t in osl.clock:
            osl.log.info(osl.knee.motor_position)
            osl.update()

In this code, we enter a `with` block that sets the mode of the `knee` joint to "position", and set the motor position of the `knee` joint to the current position plus the `set_point` value. We then loop over the `osl.clock` generator, which generates a sequence of timestamps at the specified frequency, and log the motor position of the `knee` joint to the console at each timestamp. We then call the `osl.update()` method to update the state of the `OpenSourceLeg` object.

.. warning::
    This code assumes that the `OpenSourceLeg` object is properly configured and calibrated, and that the joint is properly connected and functioning.

.. rubric:: Here is the code for this tutorial:

.. literalinclude:: ../../tutorials/position_mode.py
    :language: python
    :linenos:
