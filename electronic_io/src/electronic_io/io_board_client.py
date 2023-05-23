# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Client for communicating with an I/O board."""

import time
from typing import Dict

from electronic_io_msgs.msg import IOInfo, Readings, DigitalIOInfo, DigitizedAnalogIOInfo, RawAnalogIOInfo
from electronic_io_msgs.srv import Read, ReadRequest, ReadResponse, Write, WriteRequest, WriteResponse
import rospy

from .pins import DigitalPin, DigitizedAnalogPin, RawAnalogPin, VirtualPin


class IOBoardClient(object):
    """Client for communicating with an I/O board."""

    def __init__(self, base_topic):
        """
        :param str base_topic: Base topic (namespace) used by all the other ROS endpoints.
        """
        self.base_topic = rospy.names.resolve_name(base_topic)  # allow easy remapping of just the base topic
        self.info_topic = rospy.names.ns_join(self.base_topic, "io_info")
        self.read_service = rospy.names.ns_join(self.base_topic, "read")
        self.write_service = rospy.names.ns_join(self.base_topic, "write")

        self._virtual_pins = {}  # type: Dict[str, VirtualPin]

        self._io_info = None
        self._read_srv = None
        self._write_srv = None

        self._info_sub = rospy.Subscriber(self.info_topic, IOInfo, self._info_cb, queue_size=1)
        waited = False
        while not rospy.is_shutdown():
            time.sleep(0.1)
            if self._io_info is not None:
                if waited:
                    rospy.loginfo(
                        "IO board info message received on topic " + rospy.names.resolve_name(self.info_topic))
                break
            waited = True
            rospy.logwarn_throttle(1.0, "Waiting for IO board info message on topic %s." % (
                rospy.names.resolve_name(self.info_topic),))

        self._connect_read_srv()

        self._connect_write_srv()

    def _connect_read_srv(self):
        """Wait until the pin reading service is available and then create a :class:`ServiceProxy` for it."""
        waited = False
        if self.read_service is not None:
            while not rospy.is_shutdown():
                try:
                    rospy.wait_for_service(self.read_service, rospy.Duration(1.0))
                    if waited:
                        rospy.loginfo("Service %s ready." % (rospy.names.resolve_name(self.read_service),))
                    break
                except rospy.ROSException:
                    waited = True
                    rospy.logwarn("Waiting for service " + rospy.names.resolve_name(self.read_service))
            self._read_srv = rospy.ServiceProxy(self.read_service, Read, persistent=True)

    def _connect_write_srv(self):
        """Wait until the pin writing service is available and then create a :class:`ServiceProxy` for it."""
        waited = False
        if self.write_service is not None:
            while not rospy.is_shutdown():
                try:
                    rospy.wait_for_service(self.write_service, rospy.Duration(1.0))
                    if waited:
                        rospy.loginfo("Service %s ready." % (rospy.names.resolve_name(self.write_service),))
                    break
                except rospy.ROSException:
                    waited = True
                    rospy.logwarn("Waiting for service " + rospy.names.resolve_name(self.write_service))
            self._write_srv = rospy.ServiceProxy(self.write_service, Write, persistent=True)

    def _info_cb(self, msg):
        """Callback to be called when an `IOInfo` message is received.

        :param IOInfo msg: The pin descriptions.
        """
        self._io_info = msg

    def get_io_info(self):
        """Get the I/O pins description.

        :return: Pin descriptions.
        :rtype: IOInfo
        """
        return self._io_info

    def can_read(self):
        """Whether this board has some readable pins.

        :return: Whether the board can be read.
        :rtype: bool
        """
        return self._read_srv is not None

    def can_write(self):
        """Whether this board has some writable pins.

        :return: Whether the board can be written to.
        :rtype bool:
        """
        return self._write_srv is not None

    def read(self, req):
        """Read values for filling the given request.

        :param ReadRequest req: The request specifying the pins to read.
        :return: The pin values.
        :rtype: Readings
        """
        if not self.can_read():
            raise RuntimeError("Attempted read operation on an IO board that does not support reading")
        try:
            resp = self._read_srv(req)  # type: ReadResponse
            return resp.readings
        except rospy.TransportException:
            self._connect_read_srv()
            return self.read(req)

    def write(self, req):
        """Write values from the given request to the pins.

        :param WriteRequest req: The request specifying the pins to write and their values.
        :return: The pin values.
        :rtype: Readings
        """
        if not self.can_write():
            raise RuntimeError("Attempted write operation on an IO board that does not support writing")
        try:
            self._write_srv(req)
        except rospy.TransportException:
            self._connect_write_srv()
            return self.write(req)

    def add_virtual_pin(self, pin):
        """Add the given virtual pin.

        :param VirtualPin pin: The virtual pin to add.
        """
        self._virtual_pins[pin.name] = pin

    def get_virtual_pin(self, config):
        """Get virtual pin corresponding to the given config.

        :param config: Config with key 'pin' that specifies name of the pin, or just a string with the name.
        :type config: dict or str
        :return: The virtual pin or None if it does not exist.
        :rtype: VirtualPin or None
        """
        if isinstance(config, str):
            config = {"pin": config}
        if "pin" not in config:
            raise AttributeError("Expected 'pin' key in config not found. The config was: " + str(config))
        name = config["pin"]
        if name in self._virtual_pins:
            return self._virtual_pins[name]
        return None

    def get_digital_pin(self, config):
        """Get a digital pin corresponding to the given config.

        :param dict config: The pin's config dictionary.
        :return: The pin.
        :rtype: DigitalPin or VirtualPin
        """
        virtual_pin = self.get_virtual_pin(config)
        if virtual_pin is not None and isinstance(virtual_pin.pin_info, DigitalIOInfo):
            return virtual_pin
        return DigitalPin.from_dict(config, self._io_info)

    def get_digitized_analog_pin(self, config):
        """Get a digitized analog pin corresponding to the given config.

        :param dict config: The pin's config dictionary.
        :return: The pin.
        :rtype: DigitizedAnalogPin or VirtualPin
        """
        virtual_pin = self.get_virtual_pin(config)
        if virtual_pin is not None and isinstance(virtual_pin.pin_info, DigitizedAnalogIOInfo):
            return virtual_pin
        return DigitizedAnalogPin.from_dict(config, self._io_info)

    def get_raw_analog_pin(self, config):
        """Get a raw analog pin corresponding to the given config.

        :param dict config: The pin's config dictionary.
        :return: The pin.
        :rtype: RawAnalogPin or VirtualPin
        """
        virtual_pin = self.get_virtual_pin(config)
        if virtual_pin is not None and isinstance(virtual_pin.pin_info, RawAnalogIOInfo):
            return virtual_pin
        return RawAnalogPin.from_dict(config, self._io_info)
