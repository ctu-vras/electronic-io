# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""
Ampere Meter
============

**Device type:** Input device.

**Message type:** :cras_msgs:`ElectricCurrentStamped`.

**Pin requirements:** One raw analog pin. Read required.

YAML config
-----------

::

   topic: 'TOPIC NAME'  # Topic for the measurements. Set to empty string to disable topic publication.
   type: electronic_io.AmpereMeter
   queue_size: 1  # Optional. Topic queue size.
   latch: True  # Optional. Topic latching status.
   frame_id: 'frame'  # Optional. frame_id to be used in the messages; if not set, it is taken from the readings.
   variance: 1.0  # Optional, default 0.0. The reported variance of the measurements.
   input_pin:
     pin: 'Current1'  # Raw analog pin directly reporting the electrical current in A.

"""

import rospy

from cras_msgs.msg import ElectricCurrentStamped

from ..device import InputDevice


class AmpereMeter(InputDevice):
    """Ampere meter device. Publishes messages of type :cras_msgs:`ElectricCurrentStamped`."""

    def __init__(self, name, config, io_board):
        super(AmpereMeter, self).__init__(name, config, io_board, ElectricCurrentStamped, 1, True)
        if "input_pin" not in config:
            raise AttributeError("Invalid config of " + self.get_name())

        self._input_pin = io_board.get_raw_analog_pin(config["input_pin"])
        self._frame_id = config.get("frame_id", None)
        self._variance = config.get("variance", 0.0)

        if not self._input_pin.pin_info.pin.is_readable:
            raise AttributeError("Input pin %s for %s is not readable!" % (self._input_pin.name, self.get_name()))

        if len(self._input_pin.pin_info.unit) > 0 and self._input_pin.pin_info.unit != "A":
            rospy.logwarn("Setting up %s on raw analog pin %s whose units are not Amperes (they are %s)." % (
                self.get_name(), self._input_pin.name, self._input_pin.pin_info.unit,))

    def get_name(self):
        return "Ampere meter " + self.name

    def add_read_request(self, req):
        self._input_pin.add_read_request(req)

    def get_value(self, readings):
        return self._input_pin.get_value(readings)

    def create_message(self, value, header):
        msg = ElectricCurrentStamped()
        msg.header = header
        if self._frame_id is not None:
            msg.header.frame_id = self._frame_id
        msg.measurement.data.current = value
        msg.measurement.variance = self._variance
        msg.measurement.sample_duration = self._input_pin.pin_info.sampling_period

        return msg
