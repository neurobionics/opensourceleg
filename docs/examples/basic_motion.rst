Basic Motion Test Script
================================
In this example, we'll write a basic script that commands the OSL to move the joints in simple, periodic motions.
This script can be a helpful first step when comissioning a new OSL and making sure that the actuation systems
are configured appropriately.

We'll assume that this is the first example someone looks at, and therefore we'll go through each set of commands in detail.

Imports 
----------
For this example, we'll import `numpy`, the `OpenSourceLeg` class, and the `units` class from the `tools` module. 

.. literalinclude:: ../../examples/basic_motion.py
    :language: python
    :linenos:
    :lines: 12-15

Making an OSL object
------------------------

We'll then create an instance of the `OpenSourceLeg` class and name it `osl`. 
We pass the desired control loop frequency (200 Hz in our case) as an argument. 
This sets the rate of data streaming for the actuators, as well as the frequency of the loop in ``osl.clock``. 
We'll see more on this below. 

.. literalinclude:: ../../examples/basic_motion.py
    :language: python
    :linenos:
    :lines: 17

Adding Joints
--------------

Next, we add the knee and the ankle joints to the OSL via the ``add_joint`` method.
We supply the name of the joint and the gear ratio. 
Our gear ratio is ``9*83/18=41.5``, which accounts for the internal ActPack ratio as well as the belt transmission.

.. literalinclude:: ../../examples/basic_motion.py
    :language: python
    :linenos:
    :lines: 18-19

.. note:: 
    If you're only using one joint, you can modify this script by simply commenting out method calls for the joint you're not using. 
    For example, if you're using ankle-only, you can comment out ``osl.add_joint("knee"...)`` and the subsequent ``knee`` related methods.


Generating Reference Trajectories 
--------------------------------------

Next, we define a funciton that returns a function handle for a simple harmonic trajectory. 
Given a time input ``t``, instances of these functions return a reference position in degrees. 
We'll make a function for both the knee and ankle joints with a period of 30 seconds.
We'll command the ankle to oscillate between -20 and 20 deg and the knee between 10 and 90 deg. 

.. literalinclude:: ../../examples/basic_motion.py
    :language: python
    :linenos:
    :lines: 22-29

Configuring the OSL 
-----------------------

Now that the trajectories are configured, we're ready to start sending commands to the OSL. 
We start with the syntax ``with osl:``. This syntax allows the library to call certain functions on the entrance and exit of the block.
This way, the library can ensure that the OSL turns off (returns to zero voltage mode) in the event of an error. 

We then home the joints using ``osl.home()``. 
The homing routine drives the joints towards their hardstops in order to initialize the encoders.

.. literalinclude:: ../../examples/basic_motion.py
    :language: python
    :linenos:
    :lines: 31-32

Next, we put both joints into position control mode. 
If you wanted to change the position control gains, this would be an appropriate place to do so using the ``osl.(joint).set_position_gains()`` method.

.. literalinclude:: ../../examples/basic_motion.py
    :language: python
    :linenos:
    :lines: 33-34

Main Loop
-----------

Next, we can enter the main loop. The `OpenSourceLeg` class includes a built-in instance of a `SoftRealTimeLoop`.
This loop, which is called `clock`, will execute the contents of a ``for`` loop at the frequency specified when creating the `OpenSourceLeg` object using the syntax ``for t in osl.clock:``.
The loop will run until either ``osl.clock.stop()`` is called, an exception is raised, or the user presses ``ctrl+c`` on the keyboard. 

.. literalinclude:: ../../examples/basic_motion.py
    :language: python
    :linenos:
    :lines: 36-49

Within the loop, we first call ``osl.update()`` to query the actuators and other sensors for their latest values. 
We then update the knee and ankle position setpoints by calling our trajectory functions. 
Note that we convert the setpoint to default OSL units (radians in this case), e.g., ``units.convert_to_default(knee_traj(t), units.position.deg)``.
Finally, we use the ``set_output_position()`` method to command the OSL joints to move to the new reference value. 
Finally, we print values to the screen. 

Full Code for This Example
--------------------------------------
.. literalinclude:: ../../examples/basic_motion.py
    :language: python
    :linenos: