# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import rospy
from cras_msgs.msg import PowerSwitchStateStamped

from ..device import InputDevice, DigitalOutputDevice


class PowerSwitchReadback(InputDevice):

    def __init__(self, name, config, io_board):
        super(PowerSwitchReadback, self).__init__(name, config, io_board, PowerSwitchStateStamped, 1, True)

        self._readback_pin = io_board.get_digital_pin(config)

        if not self._readback_pin.pin_info.pin.is_readable:
            raise AttributeError("Readback pin %s for %s is not readable!" % (self._readback_pin.name, self.get_name()))

        self._frame_id = config.get("frame_id", None)

    def get_name(self):
        return "Power switch " + self.name

    def add_read_request(self, req):
        self._readback_pin.add_read_request(req)

    def get_value(self, readings):
        return self._readback_pin.get_value(readings)

    def create_message(self, value, header):
        msg = PowerSwitchStateStamped()
        msg.header = header
        if self._frame_id is not None:
            msg.header.frame_id = self._frame_id
        msg.state.on = value

        return msg


class PowerSwitch(DigitalOutputDevice):
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
        super(PowerSwitch, self).__init__(name, config, io_board)

        if not isinstance(config, dict):
            raise AttributeError("Invalid config of " + self.get_name())

        if "output_pins" not in config or not isinstance(config["output_pins"], list):
            raise AttributeError("Invalid config of %s: bad definition of output_pins." % (self.get_name(),))

        self._output_pins = []
        for pin_config in config["output_pins"]:
            pin = io_board.get_digital_pin(pin_config)
            if not pin.pin_info.pin.is_writable:
                raise AttributeError("Output pin %s for %s is not writable!" % (pin.name, self.get_name()))
            self._output_pins.append(pin)

        if "readback_pin" in config:
            config["readback_pin"]["topic"] = self.topic
            dev = PowerSwitchReadback(name, config["readback_pin"], io_board)
            self.set_readback_device(dev)
        elif len(self._output_pins) == 1 and self._output_pins[0].pin_info.pin.is_readable:
            config["output_pins"][0]["topic"] = self.topic
            dev = PowerSwitchReadback(name, config["output_pins"][0], io_board)
            self.set_readback_device(dev)

    def get_name(self):
        return "Power switch " + self.name

    def _add_write_request(self, value, write_req):
        for pin in self._output_pins:
            pin.add_write_request(value, write_req)

        rospy.logdebug("%s switched %s." % (self.get_name(), "ON" if value else "OFF"))
