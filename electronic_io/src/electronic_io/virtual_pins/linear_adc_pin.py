# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""
Linear ADC Pin
==============

**Virtual pin**

**Pin type:** Raw analog pin (input + output as defined by the real pin).

This virtual pin implements a single linear conversion between analog-digital converter (ADC) values and raw floats.

YAML config
-----------

::

   name: "Voltage1"  # Unique name of the pin.
   type: electronic_io.LinearADCPin  # Type of the pin (the class to load).
   adc_pin:  # The digitized analog pin to use.
     pin: 'I1'
   linear_coeff: 0.005859375  # Linear coefficient of the D->A transform.
   constant_coeff: -12.0  # Constant offset in the D->A transform.
   unit: 'V'  # Optional. The unit to use for the virtual pin.
   min_value: 0.0  # Optional. The minimum value this pin can report.
   max_value: 0.0  # Optional. The maximum value this pin can report.
   sampling_period: 0.0  # Optional. Duration of the period of measurement.
   is_readable/is_writable/can_persist: These are inherited from the real pins, but can be overridden here.

"""

from electronic_io_msgs.msg import RawAnalogIOInfo
import rospy

from ..pins import VirtualPin, DigitizedAnalogPin


class LinearADCPin(VirtualPin):
    """This virtual pin implements a single linear conversion between analog-digital converter (ADC) values and raw
    floats."""

    def __init__(self, name, pin_info, adc_pin, linear_coeff, constant_coeff):
        """Create the linear ADC pin.

        :param str name: Unique name of the pin.
        :param RawAnalogIOInfo pin_info: Metadata about the virtual pin.
        :param DigitizedAnalogPin adc_pin: The digitized analog pin to use.
        :param float linear_coeff: Linear coefficient of the D->A transform.
        :param float constant_coeff: Constant offset in the D->A transform.
        """
        super(LinearADCPin, self).__init__(name, pin_info)
        self._adc_pin = adc_pin
        self._coeffs = [float(constant_coeff), float(linear_coeff)]

    def get_value(self, readings):
        adc_value = self._adc_pin.get_value(readings)
        if adc_value is None:
            return None
        return self._coeffs[0] + adc_value * self._coeffs[1]

    def add_read_request(self, req):
        self._adc_pin.add_read_request(req)

    def add_write_request(self, value, req):
        if self._coeffs[1] == 0.0:
            adc_value = int(self._coeffs[0])
        else:
            adc_value = int((value - self._coeffs[0]) / self._coeffs[1])
        self._adc_pin.add_write_request(adc_value, req)

    @staticmethod
    def from_dict(pin_name, config, io_board):
        if not isinstance(config, dict):
            raise AttributeError("Configuration of virtual pin %s has to be a dictionary." % (pin_name,))

        name = config.get("name", pin_name)

        if "adc_pin" not in config:
            raise AttributeError("Expected 'adc_pin' key in config not found. The config was: " + str(config))
        adc_pin = io_board.get_digitized_analog_pin(config["adc_pin"])

        linear_coeff = config.get("linear_coeff", 1.0)
        constant_coeff = config.get("constant_coeff", 0.0)

        pin_info = RawAnalogIOInfo()
        pin_info.unit = config.get('unit', '')
        pin_info.min_value = config.get('min_value', 0.0)
        pin_info.max_value = config.get('max_value', 0.0)
        pin_info.sampling_period = rospy.Duration(config.get('sampling_period', 0.0))
        pin_info.pin.name = name
        pin = adc_pin.pin_info.pin
        pin_info.pin.is_readable = min(pin.is_readable, bool(config.get("is_readable", pin.is_readable)))
        pin_info.pin.is_writable = min(pin.is_writable, bool(config.get("is_writable", pin.is_writable)))
        pin_info.pin.can_persist = min(pin.can_persist, bool(config.get("can_persist", pin.can_persist)))

        return LinearADCPin(name, pin_info, adc_pin, linear_coeff, constant_coeff)
