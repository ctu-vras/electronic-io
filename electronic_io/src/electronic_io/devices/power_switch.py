# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""
Power Switch (Relay)
====================

**Device type:** Output device with readback.

**Set output service type:** :std_srvs:`SetBool`.

**Readback messages type:** :cras_msgs:`PowerSwitchStateStamped`.

**Pin requirements:** At least one digital output pin (readback supported).

YAML config
-----------

::

   topic: 'TOPIC NAME'  # Base topic for readback and the services. Set to empty string to disable topics and services.
   type: electronic_io.DimmableLED
   queue_size: 1  # Optional. Readback topic queue size.
   latch: True  # Optional. Readback topic latching status.
   frame_id: 'frame'  # Optional. frame_id to be used in the messages; if not set, it is taken from the readings.
   output_pins:  # One or more output pins.
     - pin: 'C5'  # Digital pin. Write required, readback supported.
       inverted: False  # Whether to invert values of the pin.
   readback_pin:  # Optional readback pin. If not specified, only one output pin is specified and it supports readback,
                  # the readback is automatically provided.
     pin: 'C5'  # Digital pin. Readback required.
     inverted: False  # Whether to invert values of the pin.

"""

import rospy
from cras_msgs.msg import PowerSwitchStateStamped

from ..device import InputDevice, DigitalOutputDevice


class PowerSwitchReadback(InputDevice):
    """Readback of a power switch. Can also be used standalone without the write part.
    Publishes :cras_msgs:`PowerSwitchStateStamped` readback messages."""

    def __init__(self, name, config, io_board):
        super(PowerSwitchReadback, self).__init__(name, config, io_board, PowerSwitchStateStamped, 1, True)

        if "readback_pin" in config:
            self._readback_pin = io_board.get_digital_pin(config["readback_pin"])
        elif len(config.get("output_pins", [])) == 1:
            self._readback_pin = io_board.get_digital_pin(config["output_pins"][0])
        else:
            raise AttributeError("Readback pin for %s is defined neither via readback_pin nor via output_pins." % (
                self.get_name(),))

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
    """Power switch controlled using :std_msgs:`SetBool` service."""

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

        try:
            dev = PowerSwitchReadback(name, config, io_board)
            self.set_readback_device(dev)
        except AttributeError as e:
            # An exception can be thrown if readback setup fails. We only want that to propagate in case readback_pin
            # was explicitly set - otherwise it is not clear whether the user wanted readback or not.
            if "readback_pin" in config:
                raise e

    def get_name(self):
        return "Power switch " + self.name

    def _add_write_request(self, value, write_req):
        for pin in self._output_pins:
            pin.add_write_request(value, write_req)

        rospy.logdebug("%s switched %s." % (self.get_name(), "ON" if value else "OFF"))
