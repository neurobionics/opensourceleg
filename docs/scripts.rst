Actuators
---------

.. automodule:: opensourceleg.hardware.actuators
   :members:

   * ``PI`` **float** = 3.14159
   * ``MOTOR_COUNT_PER_REV`` **float** = 16384
   * ``NM_PER_AMP`` **float** = 0.1133
   * ``NM_PER_MILLIAMP`` **float** = ``NM_PER_AMP`` / 1000
   * ``RAD_PER_COUNT`` **float** = 2 * ``PI`` / ``MOTOR_COUNT_PER_REV``
   * ``RAD_PER_DEG`` **float** = ``PI`` / 180
   * ``RAD_PER_SEC_GYROLSB`` **float** = ``PI`` / 180 / 32.8
   * ``M_PER_SEC_SQUARED_ACCLSB`` **float** = 9.80665 / 8192
   * ``IMPEDANCE_A`` **float** = 0.00028444
   * ``IMPEDANCE_C`` **float** = 0.0007812
   * ``NM_PER_RAD_TO_K`` **float** = ``RAD_PER_COUNT`` / ``IMPEDANCE_C`` * 1e3 / ``NM_PER_AMP``
   * ``NM_S_PER_RAD_TO_B`` **float** = ``RAD_PER_DEG`` / ``IMPEDANCE_A`` * 1e3 / ``NM_PER_AMP``
   * ``MAX_CASE_TEMPERATURE`` **float** = 80.0


Joints
-------

.. automodule:: opensourceleg.hardware.joints
   :members:
   :show-inheritance:

Sensors
-------

.. automodule:: opensourceleg.hardware.sensors
   :members:


Open-Source Leg
----------------

.. automodule:: opensourceleg.osl
   :members:
   :show-inheritance:


Control
-------

.. automodule:: opensourceleg.control.compiled_controller
   :members:   

.. automodule:: opensourceleg.control.state_machine
   :members:   

Thermal
--------

.. automodule:: opensourceleg.hardware.thermal
   :members:

Units
------

.. automodule:: opensourceleg.tools.units
   :members:

Utilities
---------

.. automodule:: opensourceleg.tools.utilities
   :members: 

Logger
-------

.. automodule:: opensourceleg.tools.logger
   :members:
   :show-inheritance:
