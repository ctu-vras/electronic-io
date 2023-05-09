.. documentation master file

|project|
===============================================================================

|description|

This package provides a generalized approach for integration of various electronic input and output devices into ROS.

The three basic concepts in this package are:

- **Pins:** They correspond either to physical input or output pins of a CPU or MCU, or to virtualized pins that e.g.
            internally decode a protocol like 1-Wire and expose the measurements. Multiple pins together form an
            *I/O board*.
- **Virtual Pins:** These are pins that have no physical manifestations, but instead use one or more real pins to
                    compute their value. This can be used e.g. for adjusting or combining the values returned by the
                    real pins.
- **Devices:** Each device operates on one or more *pins* or *virtual pins* and interprets their values. Input devices
               produce ROS topics with the readings. Output devices provide a ROS service that can be used to change
               value of the device (and its pins).

The idea is that each of these could be representing a different real-world entity:

- The set of real pins can describe a particular I/O board, like Raspbery Pi.
- The set of virtual pins translates datasheets of the connected components into sensible numbers (e.g. ADC -> Voltage).
- The set of devices interprets the pins and virtual pins in terms of high-level devices.

These also specify the two ROS nodes that are expected to be running: a board node (exposing the pins), and a devices
node (handling the virtual pins and devices).

See the `examples` folder for inspiration how the nodes and configs can be used and configured.

Python API
----------

.. toctree::
   :maxdepth: 2

   api

Virtual Pins
-------

.. toctree::
   :maxdepth: 2

   virtual_pins

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
