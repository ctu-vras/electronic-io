# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""
This module provides basic APIs for implementing I/O devices.

An I/O device as understood by this package is a collection of I/O pins whose values together form a logical unit.

There are several basic types of devices:

- :class:`InputDevice`: Device that reads some pins and publishes values on a topic.
- :class:`OutputDevice`: Device that receives service requests to change to values of some pins. These devices can have
                         readback, which means the values of the pins can also be read.
- :class:`MetaDevice`: A composite device consisting from several other (non-meta) devices.
"""

import importlib
import rospy
from electronic_io_msgs.msg import Readings
from electronic_io_msgs.srv import ReadRequest, WriteRequest
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse

from .io_board_client import IOBoardClient
from .pins import VirtualPin


class Device(object):
    def __init__(self, name, config, io_board):
        """Base class for all devices. They must extend it.

        :param str name: Name of the device. The name has to be unique.
        :param dict config: Configuration of the device as a dictionary. Key 'topic' is required, but can be empty.
        :param IOBoardClient io_board: The I/O board on which the device will be controlling pins.
        :raises AttributeError: If invalid config is provided.
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
        """Get a human-friendly name of the device.

        :return: The name.
        :rtype: str
        """
        raise NotImplementedError()


class InputDevice(Device):
    def __init__(self, name, config, io_board, topic_type, queue_size=1, latch=True, initial_value=None):
        """Base class for input devices.

        :param str name: Name of the device. The name has to be unique.
        :param dict config: Configuration of the device as a dictionary.
        :param IOBoardClient io_board: The I/O board on which the device will be controlling pins.
                                       The board has to be readable.
        :param type topic_type: ROS message type of the published topic.
        :param int queue_size: Default queue size used for the publisher if not explicitly overridden in `config`.
        :param bool latch: Default latching value used for the publisher if not explicitly overridden in `config`.
        :param initial_value: Optional initial value that should be used before the first physical read succeeds.
        """
        super(InputDevice, self).__init__(name, config, io_board)

        if not io_board.can_read():
            raise RuntimeError("Tried to create input device on a non-readable IO board.")

        self._last_value = initial_value

        self._pub = None
        if self.topic:
            self._pub = rospy.Publisher(self.topic, topic_type,
                                        queue_size=config.get("queue_size", queue_size),
                                        latch=config.get("latch", latch))

    def add_read_request(self, req):
        """Modify the passed read request to collect all information needed by this device.

        :param ReadRequest req: The request to modify.
        """
        raise NotImplementedError()

    def read(self, use_last_value=False):
        """Read values of the pins and convert them to a value.

        :param bool use_last_value: When true, this call returns the previously returned value instead of actually
                                    reading the pins.
        :return: The value.
        """
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
        """Extract value of this device from the given pin readings.

        :param Readings readings: Values of pins.
        :return: The value.
        """
        raise NotImplementedError()

    def create_message(self, value, header):
        """Convert the given value to a ROS message that can be published by this device.

        :param value: The value to convert.
        :param std_msgs.msg.Header header: Header to be used in the message (if needed).
        :return: The ROS message.
        :rtype: genpy.Message
        """
        raise NotImplementedError()

    def update(self, readings):
        """Parse the given pin readings, convert them to a ROS message and publish it.

        :param Readings readings: Values of pins.
        """
        if self._pub is None:
            return
        value = self.get_value(readings)
        if value is not None:
            self._last_value = value
            msg = self.create_message(value, readings.header)
            if msg is not None:
                self._pub.publish(msg)


class OutputDevice(Device):
    """Base class for output devices.

    :param str name: Name of the device. The name has to be unique.
    :param dict config: Configuration of the device as a dictionary.
    :param IOBoardClient io_board: The I/O board on which the device will be controlling pins.
                                   The board has to be writable.
    :param type service_type: ROS service type of the setter service.
    """
    def __init__(self, name, config, io_board, service_type):
        super(OutputDevice, self).__init__(name, config, io_board)

        if not io_board.can_write():
            raise RuntimeError("Tried to create output device on a non-writable IO board.")

        self.service_type = service_type
        self._readback_device = None

        if self.topic:
            self._set_srv = rospy.Service(self.topic + "/set", service_type, self.set_value_cb)

    def _add_write_request(self, value, request):
        """Modify the passed write request to set pin values corresponding to the given value.

        :param value: The value to set.
        :param WriteRequest request: The request to modify.
        """
        raise NotImplementedError()

    def set_value_cb(self, request):
        """Callback called by the `~/set` service.

        :param request: The setter service request.
        """
        raise NotImplementedError()

    def set_value(self, value):
        """Set the given value and propagate it to pins.

        :param Any value: The value to set.
        :return: Whether setting the value succeeded.
        :rtype: bool
        """
        write_req = WriteRequest()
        self._add_write_request(value, write_req)

        # might throw ServiceException
        self.io_board.write(write_req)

        return True

    def has_readback(self):
        """Whether this device has readback pin.

        :rtype: bool
        """
        return self._readback_device is not None

    def get_readback_device(self):
        """Get the device used for readback.

        :return: The device or None if it is not set.
        :rtype: InputDevice
        """
        return self._readback_device

    def set_readback_device(self, device):
        """Set the readback device.

        :param InputDevice device: The device to set.
        """
        self._readback_device = device


class DigitalOutputDevice(OutputDevice):
    """Specialized output device which also provides service `~/toggle` to toggle the digital value if readback is
    provided."""

    def __init__(self, name, config, io_board):
        """Create the specialized output device.

        :param str name: Name of the device. The name has to be unique.
        :param dict config: Configuration of the device as a dictionary.
        :param IOBoardClient io_board: The I/O board on which the device will be controlling pins.
                                       The board has to be writable.
        """
        super(DigitalOutputDevice, self).__init__(name, config, io_board, SetBool)

    def set_readback_device(self, device):
        super(DigitalOutputDevice, self).set_readback_device(device)
        if self.topic:
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
        """Base class for composite devices.

        :param str name: Name of the device. The name has to be unique.
        :param dict config: Configuration of the device as a dictionary.
        :param IOBoardClient io_board: The I/O board on which the device will be controlling pins.
                                       The board has to be writable.
        :param dict devices: Dictionary of already loaded devices.
        """
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


def load_class(conf, name):
    """Dynamically load a class specified by the given config's 'type' key.

    :param dict conf: Configuration dictionary. Has to contain key 'type'.
    :param str name: The name to be used in error messages.
    :return: The corresponding type, or None.
    :rtype: type or None
    """
    if "type" not in conf:
        rospy.logerr("Invalid configuration of " + name + ". It has to contain 'type' key.")
        return None

    class_type = conf["type"]
    if "." not in class_type:
        rospy.logerr("Invalid type of " + name + ". It has to be of form 'package.Class'.")

    class_module, class_class = class_type.rsplit(".", 1)

    try:
        module = importlib.import_module(class_module)
    except Exception as e:
        rospy.logerr("Could not import module " + class_module + ": " + str(e))
        return None

    if not getattr(module, class_class):
        rospy.logerr("Could not find class " + class_class + " in module " + class_module + ".")
        return None

    return getattr(module, class_class)


def load_virtual_pins(virtual_pins_conf, io_board):
    """Load virtual pins from the given config dictionary into the `io_board`.

    :param dict virtual_pins_conf: The dictionary with configuration.
    :param IOBoardClient io_board: The I/O board for which the virtual pins are defined.
    :raises AttributeError: If invalid configuration has been provided.
    """
    for pin_name in virtual_pins_conf:
        pin_conf = virtual_pins_conf[pin_name]

        name = "virtual pin " + pin_name
        clazz = load_class(pin_conf, name)
        if clazz is None:
            continue

        if not hasattr(clazz, "from_dict"):
            rospy.logerr("Virtual pin class " + str(clazz) + " does not have method from_dict().")
            continue

        from_dict = getattr(clazz, "from_dict")
        try:
            pin = from_dict(pin_name, pin_conf, io_board)
        except Exception as e:
            rospy.logerr("Could not setup " + name + ": " + str(e))
            continue

        if not isinstance(pin, VirtualPin):
            rospy.logerr(name.capitalize() + " does not inherit from electronic_io.VirtualPin class.")
            continue

        io_board.add_virtual_pin(pin)
        rospy.loginfo("Successfully added " + name)


def load_devices(devices_conf, io_board):
    """Load devices from the given config dictionary.

    :param dict devices_conf: The dictionary with configuration.
    :param IOBoardClient io_board: The I/O board on which the devices will operate.
    :return: The loaded devices.
    :rtype: dict
    :raises AttributeError: If invalid configuration has been provided.
    """
    meta_device_args = []
    devices = {}
    for device_name in devices_conf:
        device_conf = devices_conf[device_name]
        name = "device " + device_name

        clazz = load_class(device_conf, name)
        if clazz is None:
            continue

        if issubclass(clazz, MetaDevice):
            meta_device_args.append([clazz, device_name, device_conf])
            continue

        try:
            device = clazz(device_name, device_conf, io_board)
        except Exception as e:
            rospy.logerr("Could not setup " + name + ": " + str(e))
            continue

        if not isinstance(device, Device):
            rospy.logerr(name.capitalize() + " does not inherit from electronic_io.Device class.")
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
