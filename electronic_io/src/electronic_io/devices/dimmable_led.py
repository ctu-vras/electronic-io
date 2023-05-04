# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""
LED light with configurable brightness
======================================

**Device type:** Output device with readback.

**Set output service type:** :cras_msgs:`SetBrightness`.

**Readback messages type:** :cras_msgs:`BrightnessStamped`.

**Pin requirements:** Multiple options depending on configuration.

- One raw analog pin with PWM + one optional digital pin for on/off of the light.
- Multiple digital pins whose unique combinations define discrete levels of brightness.

YAML config
-----------

::

   topic: 'TOPIC NAME'
   type: electronic_io.DimmableLED
   queue_size: 1  # Optional. Readback topic queue size.
   latch: True  # Optional. Readback topic latching status.
   frame_id: 'frame'  # Optional. frame_id to be used in the messages; if not set, it is taken from the readings.
   # The following keys specify the "backend". Exactly on of them has to be present.
   pwm:  # Optional. Control the LED with a PWM pin (with optional enable pin).
     pwm_pin:
       pin: 'PWM3'  # Raw analog PWM pin (with 0.0 - 1.0 values). Write required, readback supported.
     enable_pin:  # Optional. This pin turns the light on/off without the need to specify PWM value.
       pin: 'Output3'  # Digital pin. Write required, readback supported.
       inverted: False  # Optional. Whether to invert values of the pin.
   discrete_steps:  # Optional. Control the LED with a combination of digital pins.
     # Multiple steps are expected to be defined. The union of their intervals should cover the whole 0.0 - 1.0 range.
     - interval: [0.0, 0.25]  # Interval of setter values that are mapped to this step. Two numbers.
                              # The intervals from different steps should not overlap.
       pins:  # The pin combination uniquely determining this step.
         - pin: 'C1'  # Digital pin.
           value: False  # Value of the digital pin.
       readback_value: 0.0  # Optional. The value returned for this step when it is selected in readback step.
                            # If not specified, the LED will not provide readback.

"""

import rospy
from typing import Any, Dict, List, Optional, Type

from cras_msgs.msg import BrightnessStamped
from cras_msgs.srv import SetBrightness, SetBrightnessRequest, SetBrightnessResponse
from electronic_io_msgs.msg import Readings
from electronic_io_msgs.srv import ReadRequest, WriteRequest

from ..pins import IOPin, DigitalPin, RawAnalogPin
from ..device import InputDevice, OutputDevice
from ..io_board_client import IOBoardClient


class _Backend(object):
    """Backend of the LED driver. Specifies the way how brightness values are translated to electrical ones."""

    def is_readable(self):
        raise NotImplementedError()

    def add_write_request(self, value, write_req):  # type: (float, WriteRequest) -> None
        raise NotImplementedError()

    def add_read_request(self, read_req):  # type: (ReadRequest) -> None
        raise NotImplementedError()

    def get_value(self, readings):  # type: (Readings) -> Optional[float]
        raise NotImplementedError()

    @staticmethod
    def config_key():  # type: () -> str
        """The key in config dictionary that specifies this backend."""
        raise NotImplementedError()


class _Interval:
    """Contiguous interval of real numbers."""

    def __init__(self, bounds):
        self.min = bounds[0]
        self.max = bounds[1]

    def __contains__(self, value):  # type: (float) -> bool
        return self.min <= value <= self.max


class _DiscreteStep:
    """One step of the `DiscreteSteps` backend."""

    def __init__(self, step_config, name, io_board, require_readable=False, require_writable=False):
        if "interval" not in step_config or "pins" not in step_config:
            raise AttributeError("Invalid config of %s: each step must contain keys interval and pins." % (name,))
        interval_config = step_config["interval"]
        if not isinstance(interval_config, list) or len(interval_config) != 2:
            raise AttributeError("Invalid config of %s: interval has to be a 2-element list." % (name,))
        self.interval = _Interval(interval_config)

        pins_config = step_config["pins"]
        if not isinstance(pins_config, list) or len(pins_config) == 0:
            raise AttributeError("Invalid config of %s: pins have to be a nonempty list." % (name,))
        self.pins = []  # type: List[IOPin]
        self.pin_values = {}  # type: Dict[str, bool]
        for pin_config in pins_config:
            pin = io_board.get_digital_pin(pin_config)
            if require_readable and not pin.pin_info.pin.is_readable:
                raise AttributeError("Pin %s of %s is not readable." % (pin.name, name))
            if require_writable and not pin.pin_info.pin.is_writable:
                raise AttributeError("Pin %s of %s is not writable." % (pin.name, name))
            if "value" not in pin_config:
                raise AttributeError("Pin %s of %s does not have 'value' key." % (pin.name, name))
            self.pins.append(pin)
            self.pin_values[pin.name] = bool(pin_config["value"])

        self.readback_value = step_config.get("readback_value", None)

    def add_write_request(self, write_req):  # type: (WriteRequest) -> None
        for pin in self.pins:  # type: IOPin
            pin.add_write_request(self.pin_values[pin.name], write_req)

    def matches_pin_values(self, pin_values):  # type: (Dict[str, IOPin]) -> bool
        """Whether this step matches the given values of pins.

        :param pin_values: The currently read values of pins.
        :return: Whether this step matches.
        """
        if len(pin_values) < len(self.pins):
            return False
        for pin in self.pins:  # type: IOPin
            if pin.name not in pin_values:
                return False
            if pin_values[pin.name] != self.pin_values[pin.name]:
                return False
        return True


class _DiscreteSteps(_Backend):
    """discrete_steps backend for controlling LED light with digital pin combinations."""

    def __init__(self, config, name, io_board, require_readable=False, require_writable=False):
        super(_DiscreteSteps, self).__init__()
        config = config["discrete_steps"]
        if not isinstance(config, list) or len(config) == 0:
            raise AttributeError("Invalid config of %s: discrete_steps must be a list." % (name,))
        steps = []  # type: List[_DiscreteStep]
        for step_config in config:
            step = _DiscreteStep(step_config, name, io_board, require_readable, require_writable)
            steps.append(step)

        self.steps = steps
        self.min = min([s.interval.min for s in steps])
        self.max = max([s.interval.max for s in steps])
        self.pins = dict()  # type: Dict[str, IOPin]
        self._is_readable = True
        for step in steps:  # type: _DiscreteStep
            if step.readback_value is None:
                self._is_readable = False
            for pin in step.pins:
                self.pins[pin.name] = pin
                if not pin.pin_info.pin.is_readable:
                    self._is_readable = False

    def is_readable(self):
        return self._is_readable

    def add_write_request(self, value, write_req):
        for step in self.steps:
            if value in step.interval:
                step.add_write_request(write_req)
                break
        else:
            raise RuntimeError(
                "Value %r is not valid for the discrete steps configuration covering interval [%f,%f]." % (
                    value, self.min, self.max))

    def add_read_request(self, read_req):
        for pin in self.pins.values():  # type: IOPin
            pin.add_read_request(read_req)

    def get_value(self, readings):
        pin_values = dict()
        for pin in self.pins.values():  # type: IOPin
            value = pin.get_value(readings)
            if value is None:
                continue
            pin_values[pin.name] = value

        if len(pin_values) == 0:
            return None

        for step in self.steps:  # type: _DiscreteStep
            if step.matches_pin_values(pin_values):
                return step.readback_value

        return None

    @staticmethod
    def config_key():
        return "discrete_steps"


class _PWM(_Backend):
    """Backend for controlling LED with a PWM pin (an optionally also a digital enable pin)."""

    def __init__(self, config, name, io_board, require_readable=False, require_writable=False):
        super(_PWM, self).__init__()

        config = config["pwm"]
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

    @staticmethod
    def config_key():
        return "pwm"


_backends = (
    _DiscreteSteps,
    _PWM
)


def _get_backend_class(config, name):  # type: (Dict[str, Any], str) -> Type
    """Parse the configured backend class from a config dict.

    :param config: The dimmable LED config.
    :param name: Name of the device (just for nicer error messages).
    :return: The configured backend class.
    :raises AttributeError: When wrong config is provided.
    """
    num_matching = 0
    selected = None
    for backend in _backends:
        if backend.config_key() in config:
            num_matching += 1
            selected = backend
    if num_matching == 1:
        return selected
    elif num_matching == 0:
        raise AttributeError("Invalid config of %s: expected one of keys %r." % (
            name, [b.config_key() for b in _backends]))
    else:
        raise AttributeError("Invalid config of %s: cannot mix together multiple of keys %r." % (
            name, [b.config_key() for b in _backends]))


class DimmableLEDReadback(InputDevice):
    """Readback of a dimmable LED. Can also be used standalone without the write part.
    Publishes :cras_msgs:`BrightnessStamped` readback messages."""

    def __init__(self, name, config, io_board):  # type: (str, Dict[str, Any], IOBoardClient) -> None
        super(DimmableLEDReadback, self).__init__(name, config, io_board, BrightnessStamped, 1, True)

        if not isinstance(config, dict):
            raise AttributeError("Invalid config of " + self.get_name())

        self._frame_id = config.get("frame_id", None)  # type: Optional[str]

        backend_class = _get_backend_class(config, self.get_name())
        self._backend = backend_class(config, self.get_name(), io_board, require_readable=True)

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

        backend_class = _get_backend_class(config, self.get_name())
        self._backend = backend_class(config, self.get_name(), io_board, require_writable=True)

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
