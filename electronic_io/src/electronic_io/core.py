# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

from electronic_io_msgs.msg import *
from electronic_io_msgs.srv import ReadRequest, WriteRequest


class IOPin(object):
    def __init__(self, name, pin_info):
        """

        :param str name: 
        :param pin_info:
        :type pin_info: DigitalIOInfo or DigitizedAnalogIOInfo or RawAnalogIOInfo
        """
        self.name = name
        self.pin_info = pin_info

    def get_value(self, readings):
        """

        :param Readings readings: 
        :return: 
        :rtype: Any
        """
        raise NotImplementedError()

    def add_read_request(self, req):
        """

        :param ReadRequest req: 
        """
        raise NotImplementedError()

    def add_write_request(self, value, req):
        """

        :param Any value: 
        :param WriteRequest req: 
        """
        raise NotImplementedError()


class DigitalPin(IOPin):
    def __init__(self, name, pin_info, inverted):
        """

        :param str name: 
        :param DigitalIOInfo pin_info: 
        :param bool inverted: 
        """
        super(DigitalPin, self).__init__(name, pin_info)
        self.inverted = inverted

    @staticmethod
    def from_dict(config, io_info):
        """

        :param dict config: 
        :param IOInfo io_info: 
        :return: 
        :rtype: DigitalPin
        """
        if "pin" not in config:
            raise AttributeError("Expected 'pin' key in config not found. The config was: " + str(config))
        name = config["pin"]
        for info in io_info.digital:  # type: DigitalIOInfo
            if info.pin.name == name:
                pin_info = info
                break
        else:
            raise AttributeError("Could not find digital pin " + name)
        return DigitalPin(name, pin_info, config.get("inverted", False))

    def get_value(self, readings):
        for reading in readings.digital:
            if reading.name == self.name:
                return reading.value if not self.inverted else not reading.value
        return None

    def add_write_request(self, value, req):
        write = DigitalWrite()
        write.name = self.name
        write.value = bool(value) if not self.inverted else not value
        req.digital.append(write)

    def add_read_request(self, req):
        req.digital_pins.append(self.name)


class DigitizedAnalogPin(IOPin):
    def __init__(self, name, pin_info):
        """

        :param str name: 
        :param DigitizedAnalogIOInfo pin_info: 
        """
        super(DigitizedAnalogPin, self).__init__(name, pin_info)

    @staticmethod
    def from_dict(config, io_info):
        """

        :param dict config: 
        :param IOInfo io_info: 
        :return: 
        :rtype: DigitizedAnalogPin
        """
        if "pin" not in config:
            raise AttributeError("Expected 'pin' key in config not found. The config was: " + str(config))
        name = config["pin"]
        for info in io_info.digitized_analog:  # type: DigitizedAnalogIOInfo
            if info.pin.name == name:
                pin_info = info
                break
        else:
            raise AttributeError("Could not find digitized analog pin " + name)
        return DigitizedAnalogPin(name, pin_info)

    def get_value(self, readings):
        for reading in readings.digitized_analog:
            if reading.name == self.name:
                return reading.value
        return None

    def add_write_request(self, value, req):
        write = DigitizedAnalogWrite()
        write.name = self.name
        write.value = int(value)
        req.digitized_analog.append(write)

    def add_read_request(self, req):
        req.digitized_analog_pins.append(self.name)


class RawAnalogPin(IOPin):
    def __init__(self, name, pin_info):
        """

        :param str name: 
        :param RawAnalogIOInfo pin_info: 
        """
        super(RawAnalogPin, self).__init__(name, pin_info)

    @staticmethod
    def from_dict(config, io_info):
        """

        :param dict config: 
        :param IOInfo io_info: 
        :return: 
        :rtype: RawAnalogPin
        """
        if "pin" not in config:
            raise AttributeError("Expected 'pin' key in config not found. The config was: " + str(config))
        name = config["pin"]
        for info in io_info.raw_analog:  # type: RawAnalogIOInfo
            if info.pin.name == name:
                pin_info = info
                break
        else:
            raise AttributeError("Could not find raw analog pin " + name)
        return RawAnalogPin(name, pin_info)

    def get_value(self, readings):
        for reading in readings.raw_analog:
            if reading.name == self.name:
                return reading.value
        return None

    def add_write_request(self, value, req):
        write = RawAnalogWrite()
        write.name = self.name
        write.value = float(value)
        req.raw_analog.append(write)

    def add_read_request(self, req):
        req.raw_analog_pins.append(self.name)
