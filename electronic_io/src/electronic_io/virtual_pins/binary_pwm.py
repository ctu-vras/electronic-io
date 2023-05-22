# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""
Binary PWM
==========

**Virtual pin**

**Pin type:** Raw analog pin (input + output as defined by the real pin).

This virtual pin converts a single digital pin to PWM output.

YAML config
-----------

::

   name: "FrontLed"  # Unique name of the pin.
   type: electronic_io.BinaryPWM  # Type of the pin (the class to load).
   digital_pin:  # Real digital pin this pin uses.
     pin: 'C0'
   unit: 'PWM'  # Optional. The unit to use for the virtual pin.
   min_value: 0.0  # Optional. The minimum value this pin can report.
   max_value: 0.0  # Optional. The maximum value this pin can report.
   sampling_period: 0.0  # Optional. Duration of the period of measurement.
   is_readable/is_writable/can_persist: These are inherited from the real pin, but can be overridden here.
   on_threshold: 1e-6
   readback_on_value: 1.0
   readback_off_value: 1.0

"""

from electronic_io_msgs.msg import RawAnalogIOInfo
import rospy

from ..pins import VirtualPin, DigitalPin


class BinaryPWM(VirtualPin):
    """This virtual pin converts a single digital pin to PWM."""

    def __init__(self, name, pin_info, pin, on_threshold, readback_on_value, readback_off_value):
        """Create the digital pin combo virtual pin.

        :param str name: Unique name of the pin.
        :param RawAnalogIOInfo pin_info: Metadata about the virtual pin.
        :param DigitalPin pin: The real pins to operate on.
        :param float on_threshold: When the value to be written is larger than this value, the pin will be set to ON.
        :param float readback_on_value: The PWM value to return when the real pin is ON.
        :param float readback_off_value: The PWM value to return when the real pin is OFF.
        """
        super(BinaryPWM, self).__init__(name, pin_info)
        self._pin = pin
        self._on_threshold = on_threshold
        self._readback_on_value = readback_on_value
        self._readback_off_value = readback_off_value

    def get_value(self, readings):
        value = self._pin.get_value(readings)
        if value is None:
            return None
        return self._readback_on_value if value else self._readback_off_value

    def add_read_request(self, req):
        self._pin.add_read_request(req)

    def add_write_request(self, value, req):
        pin_value = value >= self._on_threshold
        self._pin.add_write_request(pin_value, req)

    @staticmethod
    def from_dict(pin_name, config, io_board):
        if not isinstance(config, dict):
            raise AttributeError("Configuration of virtual pin %s has to be a dictionary." % (pin_name,))

        if "digital_pin" not in config:
            raise AttributeError("Expected 'digital_pin' key in config not found. The config was: " + str(config))

        pin = io_board.get_digital_pin(config["digital_pin"])

        name = config.get("name", pin_name)

        on_threshold = float(config.get('on_threshold', 1e-6))
        readback_on_value = float(config.get('readback_on_value', 1.0))
        readback_off_value = float(config.get('readback_off_value', 0.0))

        pin_info = RawAnalogIOInfo()
        pin_info.unit = config.get('unit', '')
        pin_info.min_value = config.get('min_value', min(readback_on_value, readback_off_value))
        pin_info.max_value = config.get('max_value', max(readback_on_value, readback_off_value))
        pin_info.sampling_period = rospy.Duration(config.get('sampling_period', 0.0))
        pin_info.pin.name = name
        pin_info.pin.is_readable = bool(config.get("is_readable", pin.pin_info.pin.is_readable))
        pin_info.pin.is_writable = bool(config.get("is_writable", pin.pin_info.pin.is_writable))
        pin_info.pin.can_persist = bool(config.get("can_persist", pin.pin_info.pin.can_persist))

        return BinaryPWM(name, pin_info, pin, on_threshold, readback_on_value, readback_off_value)
