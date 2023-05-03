# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import rospy

from electronic_io_msgs.msg import Readings, IOInfo, DigitalIOInfo, DigitizedAnalogIOInfo, RawAnalogIOInfo
from electronic_io_msgs.srv import Read, ReadRequest, ReadResponse, Write, WriteRequest, WriteResponse


class IOBoardServer(object):
    def __init__(self, base_topic):
        self.base_topic = rospy.names.resolve_name(base_topic)
        self.info_topic = rospy.names.ns_join(base_topic, "io_info")
        self.read_service = rospy.names.ns_join(base_topic, "read")
        self.write_service = rospy.names.ns_join(base_topic, "write")

        self.io_info = self._create_io_info()  # type: IOInfo

        self._full_poll_req = ReadRequest()
        for info in self.io_info.digital:  # type: DigitalIOInfo
            if info.pin.is_readable:
                self._full_poll_req.digital_pins.append(info.pin.name)
        for info in self.io_info.digitized_analog:  # type: DigitizedAnalogIOInfo
            if info.pin.is_readable:
                self._full_poll_req.digitized_analog_pins.append(info.pin.name)
        for info in self.io_info.raw_analog:  # type: RawAnalogIOInfo
            if info.pin.is_readable:
                self._full_poll_req.raw_analog_pins.append(info.pin.name)

        self._report_pins()

        self._setup_publishers()

        self._info_pub.publish(self.io_info)

    def _report_pins(self):
        def pin_rw(pin):
            rw = []
            if pin.is_readable:
                rw.append("R")
            if pin.is_writable:
                rw.append("W")
            return "/".join(rw)

        def pin_list(pins):
            if len(pins) == 0:
                return "None"
            return ", ".join(["%s [%s]" % (p.pin.name, pin_rw(p.pin)) for p in pins])

        rospy.loginfo("The following electronic I/O pins are available on topic %s:" % (self.base_topic,))
        rospy.loginfo("- Digital: " + pin_list(self.io_info.digital))
        rospy.loginfo("- Digitized analog: " + pin_list(self.io_info.digitized_analog))
        rospy.loginfo("- Raw analog: " + pin_list(self.io_info.raw_analog))

    def _setup_publishers(self):
        self._readings_pub = rospy.Publisher(self.base_topic, Readings, queue_size=1, latch=True)
        self._info_pub = rospy.Publisher(self.info_topic, IOInfo, queue_size=1, latch=True)
        self._read_srv = rospy.Service(self.read_service, Read, self._handle_read)
        self._write_srv = rospy.Service(self.write_service, Write, self._handle_write)

    def _create_io_info(self):
        """
        
        :return:
        :rtype: IOInfo 
        """
        raise NotImplementedError()

    def _handle_read(self, read_req):
        """
        
        :param ReadRequest read_req: 
        :return: 
        :rtype: ReadResponse
        """
        raise NotImplementedError()

    def _handle_write(self, write_req):
        """
        
        :param WriteRequest write_req: 
        :return: 
        :rtype: WriteResponse
        """
        raise NotImplementedError()

    def poll_once(self):
        resp = self._handle_read(self._full_poll_req)
        self._readings_pub.publish(resp.readings)

    def poll_forever(self, rate):
        while not rospy.is_shutdown():
            try:
                self.poll_once()
            except Exception as e:
                rospy.logerr(str(e))
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
