===========================================================================================================================
An open-source software library for numerical computation, data acquisition, and control of lower-limb robotic prostheses.
===========================================================================================================================

|build| |docs| |python| |license|

.. |build| image:: https://github.com/imsenthur/opensourceleg/workflows/build/badge.svg?branch=master&event=push
    :target: https://github.com/imsenthur/opensourceleg/actions?query=workflow%3Abuild
    :alt: Build Status
.. |docs| image:: https://readthedocs.org/projects/opensourceleg/badge/?version=latest
    :target: https://opensourceleg.readthedocs.io/en/latest/?badge=latest
    :alt: Docs Status
.. |python| image:: https://img.shields.io/pypi/pyversions/opensourceleg.svg
    :target: https://pypi.org/project/opensourceleg/
    :alt: Python Versions
.. |license| image:: https://img.shields.io/github/license/imsenthur/opensourceleg
    :target: https://github.com/imsenthur/opensourceleg/blob/main/LICENSE
    :alt: License

|

.. image:: ../assets/images/banner.gif
    :width: 80%

Prerequisites
=============

Before installing **opensourceleg** library, you should ensure that you have the following prerequisites installed:

* ``Python 3.9`` or later
* ``pip`` package manager

Installation
============

The easiest and quickest way to install the *opensourceleg* library is via `pip <https://pip.pypa.io/en/stable/>`_:

.. code-block:: bash

    pip install opensourceleg

If you plan on installing the *opensourceleg* library on a Raspberry Pi, we recommend using `opensourcelegpi <https://github.com/neurobionics/opensourcelegpi>`_ tool, which is a cloud-based CI tool used to build an up-to-date OS for a `Raspberry Pi <https://www.raspberrypi.com/products/raspberry-pi-4-model-b/>`_ that can be used headless/GUI-less to control autonomous/remote robotic systems. This tool bundles the *opensourceleg* library and its dependencies into a single OS image, which can be flashed onto a microSD card and used to boot a Raspberry Pi. For more information, click `here <https://github.com/neurobionics/opensourcelegpi/blob/main/README.md>`_.

Getting Started
=================

For new users, we recommend visiting the :ref:`getting_started` page for an overview of the library and its documentation.

Developing
==========

To modify, develop, or contribute to the `opensourceleg <https://pypi.org/project/opensourceleg/>`_ library, we encourage you to install `Poetry <https://python-poetry.org>`_, which is a python packaging and dependency management tool. Once you have Poetry installed on your local machine, you can clone the repository and install the *opensourceleg* library by running the following commands:

.. code-block:: bash

    git clone https://github.com/neurobionics/opensourceleg.git
    cd opensourceleg

    poetry install
    poetry shell

License
=======

The *opensourceleg* library is licensed under the terms of the `LGPL-v2.1 license <https://github.com/neurobionics/opensourceleg/raw/main/LICENSE>`_. This license grants users a number of freedoms:

* You are free to use the *opensourceleg* library for any purpose.
* You are free to modify the *opensourceleg* library to suit your needs.
* You can study how the *opensourceleg* library works and change it.
* You can distribute modified versions of the *opensourceleg* library.

The GPL license ensures that all these freedoms are protected, now and in the future, requiring everyone to share their modifications when they also share the library in public.

.. toctree::
    :hidden:
    :caption: Tutorials

    /tutorials/getting_started
    /tutorials/adding_joints
    /tutorials/adding_loadcell
    /tutorials/voltage_mode
    /tutorials/current_mode
    /tutorials/position_mode
    /tutorials/impedance_mode
    /tutorials/compiled_control
    /tutorials/osl_clock
    /tutorials/reading_from_sensors
    
.. toctree::
    :hidden:
    :caption: Examples

    /examples/basic_motion
    /examples/finite_state_machine

.. toctree::
    :hidden:
    :caption: API Reference

    /api/osl
    /api/hardware
    /api/control
    /api/tools

.. toctree::
    :hidden:
    :caption: Contributing

    /contributing/reporting_bugs
    /contributing/contributing_code
    /contributing/code_of_conduct    
