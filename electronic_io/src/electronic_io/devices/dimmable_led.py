# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""
LED light with configurable brightness
======================================

**Device type:** Output device with readback.

**Set output service type:** :cras_msgs:`SetBrightness`.

**Readback messages type:** :cras_msgs:`BrightnessStamped`.

**Pin requirements:** One raw analog pin with PWM + one optional digital pin for turning on/off of the light.

YAML config
-----------

::

   topic: 'TOPIC NAME'  # Base topic for readback and the services. Set to empty string to disable topics and services.
   type: electronic_io.DimmableLED
   queue_size: 1  # Optional. Readback topic queue size.
   latch: True  # Optional. Readback topic latching status.
   frame_id: 'frame'  # Optional. frame_id to be used in the messages; if not set, it is taken from the readings.
   pwm_pin:
     pin: 'PWM3'  # Raw analog PWM pin (with 0.0 - 1.0 values). Write required, readback supported.
   enable_pin:  # Optional. This pin turns the light on/off without the need to specify PWM value.
     pin: 'Output3'  # Digital pin. Write required, readback supported.
     inverted: False  # Optional. Whether to invert values of the pin.

"""

import rospy
from typing import Any, Dict, Optional

from cras_msgs.msg import BrightnessStamped
from cras_msgs.srv import SetBrightness, SetBrightnessRequest, SetBrightnessResponse

from ..pins import DigitalPin, RawAnalogPin
from ..device import InputDevice, OutputDevice
from ..io_board_client import IOBoardClient


class _PWM(object):
    """Backend for controlling LED with a PWM pin (an optionally also a digital enable pin)."""

    def __init__(self, config, name, io_board, require_readable=False, require_writable=False):
        super(_PWM, self).__init__()

        if not isinstance(config, dict) or "pwm_pin" not in config:
            raise AttributeError("Invalid config of %s: pwm_pin not found in pwm." % (name,))

        self._pwm_pin = io_board.get_raw_analog_pin(config["pwm_pin"])  # type: RawAnalogPin
        if require_readable and not self._pwm_pin.pin_info.pin.is_readable:
            raise AttributeError("pwm_pin of %s is not readable." % (name,))
        if require_writable and not self._pwm_pin.pin_info.pin.is_writable:
            raise AttributeError("pwm_pin of %s is not writable." % (name,))

        self._enable_pin = None  # type: Optional[DigitalPin]
        if "enable_pin" in config:
            self._enable_pin = io_board.get_digital_pin(config["enable_pin"])
            if require_readable and not self._enable_pin.pin_info.pin.is_readable:
                raise AttributeError("enable_pin of %s is not readable." % (name,))
            if require_writable and not self._enable_pin.pin_info.pin.is_writable:
                raise AttributeError("enable_pin of %s is not writable." % (name,))

    def is_readable(self):
        return self._pwm_pin.pin_info.pin.is_readable

    def add_write_request(self, value, write_req):
        self._pwm_pin.add_write_request(value, write_req)
        if self._enable_pin is not None:
            self._enable_pin.add_write_request(abs(value) > 1e-3, write_req)

    def add_read_request(self, read_req):
        self._pwm_pin.add_read_request(read_req)
        if self._enable_pin is not None:
            self._enable_pin.add_read_request(read_req)

    def get_value(self, readings):
        value = self._pwm_pin.get_value(readings)
        enable = None if self._enable_pin is None else self._enable_pin.get_value(readings)
        if value is None:
            return None
        if enable is not None and not enable:
            return 0.0
        return value


class DimmableLEDReadback(InputDevice):
    """Readback of a dimmable LED. Can also be used standalone without the write part.
    Publishes :cras_msgs:`BrightnessStamped` readback messages."""

    def __init__(self, name, config, io_board):  # type: (str, Dict[str, Any], IOBoardClient) -> None
        super(DimmableLEDReadback, self).__init__(name, config, io_board, BrightnessStamped, 1, True)

        if not isinstance(config, dict):
            raise AttributeError("Invalid config of " + self.get_name())

        self._frame_id = config.get("frame_id", None)  # type: Optional[str]
        self._backend = _PWM(config, self.get_name(), io_board, require_readable=True)

    def get_name(self):
        return "Dimmable LED " + self.name

    def add_read_request(self, req):
        self._backend.add_read_request(req)

    def get_value(self, readings):
        return self._backend.get_value(readings)

    def create_message(self, value, header):
        msg = BrightnessStamped()
        msg.header = header
        if self._frame_id is not None:
            msg.header.frame_id = self._frame_id
        msg.value.brightness = value

        return msg


class DimmableLED(OutputDevice):
    """Dimmable LED light controlled using :cras_msgs:`SetBrightness` service."""

    def __init__(self, name, config, io_board):
        super(DimmableLED, self).__init__(name, config, io_board, SetBrightness)

        if not isinstance(config, dict):
            raise AttributeError("Invalid config of " + self.get_name())

        self._backend = _PWM(config, self.get_name(), io_board, require_writable=True)

        if self._backend.is_readable():
            dev = DimmableLEDReadback(name, config, io_board)
            self.set_readback_device(dev)

    def get_name(self):
        return "Dimmable LED " + self.name

    def _add_write_request(self, value, write_req):
        if not isinstance(value, (int, float)) or value < 0 or value > 1:
            raise ValueError("Invalid value for %s brightness: %f. It must be in [0.0, 1.0] interval." % (
                self.get_name(), value))
        self._backend.add_write_request(value, write_req)
        rospy.logdebug("%s set to %.2f %%." % (self.get_name(), value * 100.0))

    def set_value_cb(self, request):  # type: (SetBrightnessRequest) -> SetBrightnessResponse
        self.set_value(request.value.brightness)
        return SetBrightnessResponse()
