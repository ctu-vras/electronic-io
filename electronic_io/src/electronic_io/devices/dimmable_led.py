# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import rospy

from cras_msgs.msg import BrightnessStamped
from cras_msgs.srv import SetBrightness, SetBrightnessRequest, SetBrightnessResponse
from electronic_io_msgs.srv import ReadRequest, WriteRequest

from ..core import IOPin
from ..device import InputDevice, OutputDevice


class Backend(object):
    def is_readable(self):
        raise NotImplementedError()

    def add_write_request(self, value, write_req):
        """

        :param value:
        :param WriteRequest write_req: 
        :return: 
        """
        raise NotImplementedError()

    def add_read_request(self, read_req):
        """

        :param ReadRequest read_req: 
        :return: 
        """
        raise NotImplementedError()

    def get_value(self, readings):
        raise NotImplementedError()

    @staticmethod
    def config_key():
        raise NotImplementedError()


class Interval:
    def __init__(self, bounds):
        self.min = bounds[0]
        self.max = bounds[1]

    def __contains__(self, value):
        return self.min <= value <= self.max


class DiscreteStep:
    def __init__(self, step_config, name, io_board, require_readable=False, require_writable=False):
        if "interval" not in step_config or "pins" not in step_config:
            raise AttributeError("Invalid config of %s: each step must contain keys interval and pins." % (name,))
        interval_config = step_config["interval"]
        if not isinstance(interval_config, list) or len(interval_config) != 2:
            raise AttributeError("Invalid config of %s: interval has to be a 2-element list." % (name,))
        self.interval = Interval(interval_config)

        pins_config = step_config["pins"]
        if not isinstance(pins_config, list) or len(pins_config) == 0:
            raise AttributeError("Invalid config of %s: pins have to be a nonempty list." % (name,))
        self.pins = []
        self.pin_values = {}
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

    def add_write_request(self, write_req):
        """

        :param WriteRequest write_req: 
        :return: 
        """
        for pin in self.pins:  # type: IOPin
            pin.add_write_request(self.pin_values[pin.name], write_req)

    def matches_pin_values(self, pin_values):
        """
        
        :param dict pin_values: 
        :return: 
        """
        if len(pin_values) < len(self.pins):
            return False
        for pin in self.pins:  # type: IOPin
            if pin.name not in pin_values:
                return False
            if pin_values[pin.name] != self.pin_values[pin.name]:
                return False
        return True


class DiscreteSteps(Backend):
    def __init__(self, config, name, io_board, require_readable=False, require_writable=False):
        super(DiscreteSteps, self).__init__()
        config = config["discrete_steps"]
        if not isinstance(config, list) or len(config) == 0:
            raise AttributeError("Invalid config of %s: discrete_steps must be a list." % (name,))
        steps = []
        for step_config in config:
            step = DiscreteStep(step_config, name, io_board, require_readable, require_writable)
            steps.append(step)

        self.steps = steps
        self.min = min([s.interval.min for s in steps])
        self.max = max([s.interval.max for s in steps])
        self.pins = dict()
        self._is_readable = True
        for step in steps:  # type: DiscreteStep
            if step.readback_value is None:
                self._is_readable = False
            for pin in step.pins:
                self.pins[pin.name] = pin
                if not pin.pin_info.pin.is_readable:
                    self._is_readable = False

    def is_readable(self):
        return self._is_readable

    def add_write_request(self, value, write_req):
        for step in self.steps:  # type: DiscreteStep
            if value in step.interval:
                step.add_write_request(write_req)
                break
        else:
            raise RuntimeError(
                "Value %r is not valid for the discrete steps configuration covering interval [%f,%f]." % (
                    value, self.min, self.max))

    def add_read_request(self, read_req):
        """
        
        :param ReadRequest read_req: 
        :return: 
        """
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

        for step in self.steps:  # type: DiscreteStep
            if step.matches_pin_values(pin_values):
                return step.readback_value

        return None

    @staticmethod
    def config_key():
        return "discrete_steps"


class PWM(Backend):
    def __init__(self, config, name, io_board, require_readable=False, require_writable=False):
        super(PWM, self).__init__()

        config = config["pwm"]
        if not isinstance(config, dict) or "pwm_pin" not in config:
            raise AttributeError("Invalid config of %s: pwm_pin not found in pwm." % (name,))

        self._pwm_pin = io_board.get_raw_analog_pin(config["pwm_pin"])
        if require_readable and not self._pwm_pin.pin_info.pin.is_readable:
            raise AttributeError("pwm_pin of %s is not readable." % (name,))
        if require_writable and not self._pwm_pin.pin_info.pin.is_writable:
            raise AttributeError("pwm_pin of %s is not writable." % (name,))

        self._enable_pin = None
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


backends = (
    DiscreteSteps,
    PWM
)


def get_backend_class(config, name):
    """
    
    :param dict config: 
    :param str name: 
    :return:
    :rtype: class
    """
    num_matching = 0
    selected = None
    for backend in backends:
        if backend.config_key() in config:
            num_matching += 1
            selected = backend
    if num_matching == 1:
        return selected
    elif num_matching == 0:
        raise AttributeError("Invalid config of %s: expected one of keys %r." % (
            name, [b.config_key() for b in backends]))
    else:
        raise AttributeError("Invalid config of %s: cannot mix together multiple of keys %r." % (
            name, [b.config_key() for b in backends]))


class DimmableLEDReadback(InputDevice):

    def __init__(self, name, config, io_board):
        super(DimmableLEDReadback, self).__init__(name, config, io_board, BrightnessStamped, 1, True)

        if not isinstance(config, dict):
            raise AttributeError("Invalid config of " + self.get_name())

        self._frame_id = config.get("frame_id", None)

        backend_class = get_backend_class(config, self.get_name())
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
    """
    Thermometer device.
    
    YAML config is: ::
    
       input_pin:
         pin: 'PIN NAME'  # required; name of the pin (has to be a raw analog pin)
       frame_id: 'frame'  # optional; frame_id to be used in the messages; if not set, it is taken from the readings
       variance: 1.0  # optional, default 0.0; the reported variance of the measurements; beware this is the variance in
                      # Kelvins, so if you measure in degrees Fahrenheit, you need to scale the variance accordingly.

    """

    def __init__(self, name, config, io_board):
        super(DimmableLED, self).__init__(name, config, io_board, SetBrightness)

        if not isinstance(config, dict):
            raise AttributeError("Invalid config of " + self.get_name())

        backend_class = get_backend_class(config, self.get_name())
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

    def set_value_cb(self, request):
        """

        :param SetBrightnessRequest request: 
        :return:
        :rtype: SetBrightnessResponse
        """
        self.set_value(request.value.brightness)
        return SetBrightnessResponse()
