# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import time

from electronic_io_msgs.msg import IOInfo, Readings
from electronic_io_msgs.srv import Read, ReadRequest, ReadResponse, Write, WriteRequest, WriteResponse
import rospy

from .core import DigitalPin, DigitizedAnalogPin, RawAnalogPin


class IOBoardClient(object):
    def __init__(self, base_topic):
        self.base_topic = rospy.names.resolve_name(base_topic)  # allow easy remapping of just the base topic
        self.info_topic = rospy.names.ns_join(base_topic, "io_info")
        self.read_service = rospy.names.ns_join(base_topic, "read")
        self.write_service = rospy.names.ns_join(base_topic, "write")

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
        """
        
        :param IOInfo msg: 
        :return: 
        """
        self._io_info = msg

    def get_io_info(self):
        """
        
        :return:
        :rtype: IOInfo 
        """
        return self._io_info

    def can_read(self):
        return self._read_srv is not None

    def can_write(self):
        return self._write_srv is not None

    def read(self, req):
        """
        
        :param ReadRequest req: 
        :return:
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
        """
        
        :param WriteRequest req: 
        """
        if not self.can_write():
            raise RuntimeError("Attempted write operation on an IO board that does not support writing")
        try:
            self._write_srv(req)
        except rospy.TransportException:
            self._connect_write_srv()
            return self.write(req)

    def get_digital_pin(self, config):
        return DigitalPin.from_dict(config, self._io_info)

    def get_digitized_analog_pin(self, config):
        return DigitizedAnalogPin.from_dict(config, self._io_info)

    def get_raw_analog_pin(self, config):
        return RawAnalogPin.from_dict(config, self._io_info)
