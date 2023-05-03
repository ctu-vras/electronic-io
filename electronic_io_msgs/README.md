# electronic_io_msgs

Messages for interaction with analog and digital inputs and outputs.

It is expected that these messages are used with the convenience package `electronic_io`.

This package is partially inspired by the
[dismissed ROS Industrial PR adding generic I/O protocol](https://github.com/ros-industrial/rep/pull/7).

## I/O Pins Metadata

The I/O board driver should publish metadata about all exposed pins on topic `~/io_info` where `~` is the
namespace in which the readings are published (similar to `sensor_msgs/Image` and `sensor_msgs/CameraInfo`).

The metadata should be published latched, and they should contain all known I/O pins of the board at the
moment of publication.

## Analog I/O

This packages implements access to analog I/O pins via two interfaces:

- **Digitized Analog I/O**: Directly using the integer values used by the ADC (analog-digital converter).
- **Raw Analog I/O**: Using the represented analog values (floats, e.g. measured voltage in Volts).

The driver may expose either of these interfaces, or even both. In such case, reading the pin will
yield both the raw and the digitized values, while writing can only be done via one of the two interfaces
at the same time.

Generalized libraries will most probably require the Raw Analog interface. The Digitized Analog interface
is mostly meant as a way of logging debug information that can discover bugs in the A/D conversion.
