.. _using_osl_clock_tutorial:

Using the OSL Clock
======================

The ``OpenSourceLeg`` class contains a ``clock`` object that can be used to control the execution of infinite loops. 
The ``clock`` is an iterable object that returns a new value (the time since instantiation in seconds) at a specified rate. 
Using ``osl.clock`` in a ``for`` loop will create an infinite loop that executes at a fixed frequency. That is,

.. code-block:: python

    for t in osl.clock:
        # this code will execute repeatedly at a fixed rate

.. note::
    The frequency that ``osl.clock`` updates at is the same as the frequency selected when instantiating the ``OpenSourceLeg`` object.

The clock will continue ticking until one of three conditions are met:

1. You call ``osl.clock.stop``
2. An exception occurs
3. You press ctrl+c on the keyboard. 

We normally use option 3 to end our scripts, allowing the ctrl+c keyboard combination to basically act as a stop button. 
In general, we suggest using the following overall structure for your code:

.. code-block:: python

    # Imports
    
    # Configuration/Initialization Code
    osl = OpenSourceLeg(frequency=loop_frequency)

    with osl:
        osl.home()

        for t in osl.clock:
            # Main loop code
            osl.update()
        
        # Cleanup code

