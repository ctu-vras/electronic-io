# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import importlib
import rospy
from electronic_io_msgs.msg import Readings
from electronic_io_msgs.srv import ReadRequest, WriteRequest
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
        self.io_board = io_board

        if not isinstance(config, dict):
            raise AttributeError("Configuration of %s has to be a dictionary." % (self.get_name(),))
        self.config = config

        if "topic" not in config:
            raise AttributeError("Key 'topic' is missing for " + self.get_name() + ".")
        self.topic = config["topic"]

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

        self.service_type = service_type
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


class MetaDevice(object):
    """Tagging class denoting devices that work on other devices, i.e. need to be loaded last."""

    def __init__(self, name, config, io_board, devices):
        if "devices" not in config or not isinstance(config["devices"], list):
            raise AttributeError("Group %s has to contain key 'devices' that is a list." % (name,))

        if len(config["devices"]) == 0:
            raise AttributeError("List 'devices' of group %s cannot be empty." % (name,))

        self._devices = {}
        for device_name in config["devices"]:
            if device_name not in devices:
                raise AttributeError("Sub-device %s referenced from group %s does not exist." % (
                    device_name, name))
            self._devices[device_name] = devices[device_name]


def load_devices(devices_conf, io_board):
    meta_device_args = []
    devices = {}
    for device_name in devices_conf:
        device_conf = devices_conf[device_name]
        if "type" not in device_conf:
            rospy.logerr("Invalid configuration of device " + device_name + ". It has to contain 'type' key.")
            continue
        device_type = device_conf["type"]
        if "." not in device_type:
            rospy.logerr("Invalid type of device " + device_name + ". It has to be of form 'package.Class'.")

        device_module, device_class = device_type.rsplit(".", 1)

        try:
            module = importlib.import_module(device_module)
        except Exception as e:
            rospy.logerr("Could not import module " + device_module + ": " + str(e))
            continue

        if not getattr(module, device_class):
            rospy.logerr("Could not find class " + device_class + " in module " + device_module + ".")
            continue

        clazz = getattr(module, device_class)

        if issubclass(clazz, MetaDevice):
            meta_device_args.append([clazz, device_name, device_conf])
            continue

        try:
            device = clazz(device_name, device_conf, io_board)
        except Exception as e:
            rospy.logerr("Could not setup device " + device_name + ": " + str(e))
            continue

        if not isinstance(device, Device):
            rospy.logerr("Device " + device_name + " does not inherit from electronic_io.Device class.")
            continue

        devices[device_name] = device
        rospy.loginfo("Successfully added " + device.get_name())

    # Delayed loading of meta devices after all direct devices have finished loading
    for clazz, device_name, device_conf in meta_device_args:
        try:
            device = clazz(device_name, device_conf, io_board, devices)
        except Exception as e:
            import traceback
            traceback.print_exc()
            rospy.logerr("Could not setup device " + device_name + ": " + str(e))
            continue

        if not isinstance(device, Device):
            rospy.logerr("Device " + device_name + " does not inherit from electronic_io.Device class.")
            continue

        devices[device_name] = device
        rospy.loginfo("Successfully added " + device.get_name())

    return devices
