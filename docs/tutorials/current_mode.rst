Commanding Current
==================

In this tutorial, we'll show you how to use the `OpenSourceLeg` library to control the current of a joint.

Import the OpenSourceLeg Class
------------------------------

To use the `OpenSourceLeg` class, we first need to import it from the `opensourceleg.osl` module:

.. code-block:: python

    from opensourceleg.osl import OpenSourceLeg

Add a Joint Object
-------------------

Once we have imported the `OpenSourceLeg` class, we can create an instance of the class with the desired frequency and joint configuration:

.. code-block:: python

    osl = OpenSourceLeg(frequency=200)  # 200 Hz
    osl.add_joint(gear_ratio=9.0)

In this code, we create an `OpenSourceLeg` object named `osl` with a frequency of 200 Hz and a joint with a gear ratio of 9.0.

Controlling Joint Current
--------------------------

To control the current of a joint, we can use a `with` block to ensure that the `OpenSourceLeg` object is properly opened and cleaned up after use:

.. code-block:: python

    with osl:
        osl.knee.set_mode(osl.knee.control_modes.current)
        
        for t in osl.clock:
            osl.knee.set_current(400)  # 400 mA
            osl.log.info(osl.knee.motor_position)
            osl.update()

In this code, we enter a `with` block that sets the mode of the `knee` joint to "current". We then loop over the `osl.clock`, which generates a sequence of timestamps at the specified frequency, and set the current of the `knee` joint to 400 mA. We then log the motor position of the `knee` joint to the console at each timestamp. We then call the `osl.update()` method to update the state of the `OpenSourceLeg` object.

.. warning::
    This code assumes that the `OpenSourceLeg` object is properly configured and calibrated, and that the joint is properly connected and functioning.

Code for this tutorial:
-----------------------

.. literalinclude:: ../../tutorials/current_mode.py
    :language: python
    :linenos:
