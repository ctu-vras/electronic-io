# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import rospy
from electronic_io_msgs.msg import IOInfo, Readings, DigitalReading, DigitizedAnalogReading, RawAnalogReading
from electronic_io_msgs.srv import ReadRequest, ReadResponse, WriteRequest
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse

from .io_board_client import IOBoardClient


class Device(object):
    def __init__(self, name, config, io_board):
        """
        
        :param str topic: 
        :param dict config: 
        :param IOBoardClient io_board: 
        """
        self.name = name
        if "topic" not in config:
            raise AttributeError("Key 'topic' is missing for device " + name + ".")
        self.topic = config["topic"]
        self.config = config
        self.io_board = io_board

    def get_name(self):
        """
        
        :return:
        :rtype: str 
        """
        raise NotImplementedError()


class InputDevice(Device):
    def __init__(self, name, config, io_board, topic_type, queue_size, latch, initial_value=None):
        super(InputDevice, self).__init__(name, config, io_board)

        if not io_board.can_read():
            raise RuntimeError("Tried to create input device on a non-readable IO board.")

        self._last_value = initial_value
        self._pub = rospy.Publisher(self.topic, topic_type, queue_size=queue_size, latch=latch)

    def add_read_request(self, req):
        """
        
        :param ReadRequest req: 
        """
        raise NotImplementedError()

    def read(self, use_last_value=False):
        if use_last_value and self._last_value is not None:
            return self._last_value

        req = ReadRequest()
        self.add_read_request(req)
        readings = self.io_board.read(req)  # type: Readings

        value = self.get_value(readings)
        if value is not None:
            self._last_value = value
        return value

    def get_value(self, readings):
        """
        
        :param Readings readings: 
        :return: 
        """
        raise NotImplementedError()

    def create_message(self, value, header):
        """
        
        :param value: 
        :param std_msgs.msg.Header header:
        :return: 
        """
        raise NotImplementedError()

    def update(self, readings):
        """
        
        :param Readings readings: 
        :return: 
        """
        value = self.get_value(readings)
        if value is not None:
            self._last_value = value
            msg = self.create_message(value, readings.header)
            if msg is not None:
                self._pub.publish(msg)


class OutputDevice(Device):
    def __init__(self, name, config, io_board, service_type):
        super(OutputDevice, self).__init__(name, config, io_board)

        if not io_board.can_write():
            raise RuntimeError("Tried to create output device on a non-writable IO board.")

        self._readback_device = None
        self._set_srv = rospy.Service(self.topic + "/set", service_type, self.set_value_cb)

    def _add_write_request(self, value, request):
        """
        
        :param value: 
        :param WriteRequest request: 
        """
        raise NotImplementedError()

    def set_value_cb(self, request):
        raise NotImplementedError()

    def set_value(self, value):
        """
        
        :param Any value: 
        :return:
        :rtype: bool 
        """
        write_req = WriteRequest()
        self._add_write_request(value, write_req)

        # might throw ServiceException
        self.io_board.write(write_req)

        return True

    def has_readback(self):
        return self._readback_device is not None
    
    def get_readback_device(self):
        """
        
        :return:
        :rtype: InputDevice 
        """
        return self._readback_device

    def set_readback_device(self, device):
        """
        
        :param InputDevice device: 
        """
        self._readback_device = device


class DigitalOutputDevice(OutputDevice):
    def __init__(self, name, config, io_board):
        super(DigitalOutputDevice, self).__init__(name, config, io_board, SetBool)

    def set_readback_device(self, device):
        super(DigitalOutputDevice, self).set_readback_device(device)
        self._toggle_srv = rospy.Service(self.topic + "/toggle", Trigger, self.toggle)

    def toggle(self, _request):
        if self._readback_device is None:
            raise RuntimeError("Cannot toggle the value of digital output without readback.")

        value = self._readback_device.read()
        self.set_value(not value)

        resp = TriggerResponse()
        resp.success = True
        return resp

    def set_value_cb(self, request):
        resp = SetBoolResponse()
        resp.success = self.set_value(request.data)
        return resp
