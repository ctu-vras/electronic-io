# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""
Digital Pin Combo
=================

**Virtual pin**

**Pin type:** Raw analog pin (input + output as defined by the real pins).

This virtual pin combines the values of several digital pins into a single float value.

YAML config
-----------

::

   name: "FrontLed"  # Unique name of the pin.
   type: electronic_io.DigitalPinCombo  # Type of the pin (the class to load).
   pins:  # Real pins this pin uses. All of them have to be digital.
     - pin: 'C0'
     - pin: 'C1'
   unit: 'PWM'  # Optional. The unit to use for the virtual pin.
   min_value: 0.0  # Optional. The minimum value this pin can report.
   max_value: 0.0  # Optional. The maximum value this pin can report.
   sampling_period: 0.0  # Optional. Duration of the period of measurement.
   is_readable/is_writable/can_persist: These are inherited from the real pins, but can be overridden here.
   combinations:  # Definitions of which combination means what value. Reverse lookup is done in nearest-neighbor
                  # fashion and the first combo specifier (if multiple are defined) is returned.
     - combo: [0b00, 0b01]  # This way, one value can be assigned to multiples combos. The first combo is "canonical"
       value: 0.0           # (returned in reverse lookup).
     - combo: 0b11          # The combo items are binary representation of the pin values in the order they were defined
       value: 0.5           # (the first defined pin is the least significant bit, i.e. the rightmost one).
     - combo: 0b10
       value: 1.0

"""

from electronic_io_msgs.msg import RawAnalogIOInfo
import rospy
from typing import Dict, List, Tuple, Union

from ..pins import VirtualPin, DigitalPin


class DigitalPinCombo(VirtualPin):
    """This virtual pin combines the values of several digital pins into a single float value."""

    def __init__(self, name, pin_info, pins, combinations):
        """Create the digital pin combo virtual pin.

        :param str name: Unique name of the pin.
        :param RawAnalogIOInfo pin_info: Metadata about the virtual pin.
        :param List[DigitalPin] pins: The real pins to operate on.
        :param Dict[Union[int, Tuple[int]], float] combinations: Which combinations of pins mean what value.
        """
        super(DigitalPinCombo, self).__init__(name, pin_info)
        self._pins = pins
        self._combinations = combinations

    def get_value(self, readings):
        combo = 0
        i = 0
        for pin in self._pins:
            value = pin.get_value(readings)
            if value is None:
                return None
            combo += value << i
            i += 1

        for combos in self._combinations:
            if isinstance(combos, int) and combo == combos:
                return self._combinations[combos]
            elif isinstance(combos, tuple):
                for combos1 in combos:
                    if combo == combos1:
                        return self._combinations[combos]

        rospy.logerr("Unknown pin combination " + bin(combo) + " on virtual pin " + self.name + ".")
        return None

    def add_read_request(self, req):
        for pin in self._pins:
            pin.add_read_request(req)

    def add_write_request(self, value, req):
        min_key = min_val = None
        for key, val in self._combinations.items():
            if min_val is None or abs(val - value) < min_val:
                min_key = key
                min_val = abs(val - value)

        combo = min_key
        if isinstance(combo, tuple):
            combo = combo[0]

        i = 0
        for pin in self._pins:
            pin_value = bool(combo & (1 << i))
            pin.add_write_request(pin_value, req)
            i += 1

    @staticmethod
    def from_dict(pin_name, config, io_board):
        if not isinstance(config, dict):
            raise AttributeError("Configuration of virtual pin %s has to be a dictionary." % (pin_name,))

        name = config.get("name", pin_name)

        if "pins" not in config:
            raise AttributeError("Expected 'pins' key in config not found. The config was: " + str(config))

        pins = []
        is_readable = True
        is_writable = True
        can_persist = True
        for pin_config in config["pins"]:
            pin = io_board.get_digital_pin(pin_config)
            is_readable &= pin.pin_info.pin.is_readable
            is_writable &= pin.pin_info.pin.is_writable
            can_persist &= pin.pin_info.pin.can_persist
            pins.append(pin)
        if len(pins) == 0:
            raise AttributeError("At least one pin has to be specified in LinearADCPin " + name + ".")

        if "combinations" not in config:
            raise AttributeError("Expected 'combinations' key in config not found. The config was: " + str(config))
        if not isinstance(config["combinations"], list):
            raise AttributeError("Key 'combinations' has to be a list. The config was: " + str(config))

        combinations = {}
        for combo in config["combinations"]:
            if "combo" not in combo or "value" not in combo:
                raise AttributeError("Each combination in 'combinations' has to be a dict with keys 'combo' and "
                                     "'value'. The config was " + str(config))
            key = combo["combo"]
            if not isinstance(key, int) and not isinstance(key, list) and not isinstance(key, tuple):
                raise AttributeError("Keys 'combo' have to be either integer, list or tuple. "
                                     "The config was: " + str(config))
            if isinstance(key, list):
                key = tuple(key)
            val = combo["value"]
            if not isinstance(val, int) and not isinstance(val, float):
                raise AttributeError("Keys 'value' have to be floats. The config was: " + str(config))
            combinations[key] = val
        if len(config["combinations"]) == 0:
            raise AttributeError("At least one combination has to be specified in LinearADCPin " + name + ".")

        pin_info = RawAnalogIOInfo()
        pin_info.unit = config.get('unit', '')
        pin_info.min_value = config.get('min_value', 0.0)
        pin_info.max_value = config.get('max_value', 0.0)
        pin_info.sampling_period = rospy.Duration(config.get('sampling_period', 0.0))
        pin_info.pin.name = name
        pin_info.pin.is_readable = min(is_readable, bool(config.get("is_readable", is_readable)))
        pin_info.pin.is_writable = min(is_writable, bool(config.get("is_writable", is_writable)))
        pin_info.pin.can_persist = min(can_persist, bool(config.get("can_persist", can_persist)))

        return DigitalPinCombo(name, pin_info, pins, combinations)
