.. documentation master file

|project|
===============================================================================

|description|

This package provides a generalized approach for integration of various electronic input and output devices into ROS.

The two basic concepts in this package are:

- **Pins:** They correspond either to physical input or output pins of a CPU or MCU, or to virtualized pins that e.g.
            internally decode a protocol like 1-Wire and expose the measurements. Multiple pins together form an
            *I/O board*.
- **Devices:** Each device operates on one or more *pins* and interprets their values. Input devices produce ROS topics
               with the readings. Output devices provide a ROS service that can be used to change value of the device (and
               its pins).

These also specify the two ROS nodes that are expected to be running: a board node (exposing the pins), and a devices
node (handling the devices).

See the `examples` folder for inspiration how the nodes and configs can be used and configured.

Python API
----------

.. toctree::
   :maxdepth: 2

   api

Devices
-------

.. toctree::
   :maxdepth: 2

   devices

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`