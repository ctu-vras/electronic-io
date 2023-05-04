# coding=utf-8

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""
Thermometer
===========

**Device type:** Input device.

**Message type:** :sensor_msgs:`Temperature`.

**Pin requirements:** One raw analog pin. Read required.

YAML config
-----------

::

   topic: 'TOPIC NAME'
   type: electronic_io.Thermometer
   queue_size: 1  # Optional. Topic queue size.
   latch: True  # Optional. Topic latching status.
   frame_id: 'frame'  # Optional. frame_id to be used in the messages; if not set, it is taken from the readings.
   variance: 1.0  # Optional, default 0.0. the reported variance of the measurements; beware this is the variance in
                  # Kelvins, so if you measure in degrees Fahrenheit, you need to scale the variance accordingly.
   input_pin:
     pin: 'Temp1'  # Raw analog pin directly reporting the temperature. If the unit reported by the pin is K, F or °F,
                   # the value is automatically converted to °C.

"""

import rospy

from sensor_msgs.msg import Temperature

from ..device import InputDevice


def _kelvin_to_celsius(kelvin):
    return kelvin + 272.15


def _fahrenheit_to_celsius(fahrenheit):
    return 5 * (fahrenheit - 32) / 9.0


class Thermometer(InputDevice):
    """Thermometer device. Publishes :sensor_msgs:`Temperature` messages."""

    def __init__(self, name, config, io_board):
        super(Thermometer, self).__init__(name, config, io_board, Temperature, 1, True)
        if "input_pin" not in config:
            raise AttributeError("Invalid config of %s, key 'input_pin' is missing." % (self.get_name(),))

        self._input_pin = io_board.get_raw_analog_pin(config["input_pin"])
        self._frame_id = config.get("frame_id", None)
        self._variance = config.get("variance", 0.0)

        if not self._input_pin.pin_info.pin.is_readable:
            raise AttributeError("Input pin %s of %s is not readable!" % (self._input_pin.name, self.get_name()))

        unit = self._input_pin.pin_info.unit
        self.convert_fn = lambda x: x

        if unit == "K":
            self.convert_fn = _kelvin_to_celsius
        elif unit in ("F", "°F"):
            self.convert_fn = _fahrenheit_to_celsius
        elif len(unit) > 0 and unit != "C" and unit != '°C':
            rospy.logwarn("Setting up %s on raw analog pin with unrecognized unit %s." % (
                self.get_name(), self._input_pin.pin_info.unit,))

    def get_name(self):
        return "Thermometer " + self.name

    def add_read_request(self, req):
        self._input_pin.add_read_request(req)

    def get_value(self, readings):
        value = self._input_pin.get_value(readings)
        if value is None:
            return None
        return self.convert_fn(value)

    def create_message(self, value, header):
        msg = Temperature()
        msg.header = header
        if self._frame_id is not None:
            msg.header.frame_id = self._frame_id
        msg.temperature = value
        msg.variance = self._variance

        return msg
