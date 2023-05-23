# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""API for working with electrical I/O pins."""

from electronic_io_msgs.msg import *
from electronic_io_msgs.srv import ReadRequest, WriteRequest


class IOPin(object):
    """Generic input-output pin representation."""

    def __init__(self, name, pin_info):
        """Initialize the pin with the given parameters.

        :param str name: Name of the pin. This name is used to refer to the pin in all parts of this library.
        :param pin_info: Metadata about the pin.
        :type pin_info: DigitalIOInfo or DigitizedAnalogIOInfo or RawAnalogIOInfo
        """
        self.name = name
        self.pin_info = pin_info

    def get_value(self, readings):
        """Read value of the pin from the given set of readings.

        :param Readings readings: The readings to parse.
        :return: The parsed value. If there is no reading for this pin in the given data, None is returned.
        :rtype: bool or int or float or None
        """
        raise NotImplementedError()

    def add_read_request(self, req):
        """Modify the passed read request to also query information relevant for this pin.

        :param ReadRequest req: The read request to modify.
        """
        raise NotImplementedError()

    def add_write_request(self, value, req):
        """Modify the passed write request to write the given value to this pin.

        :param value: The value to write.
        :type value: bool or int or float
        :param WriteRequest req: The write request to modify.
        """
        raise NotImplementedError()


class RealIOPin(IOPin):
    """A real input-output pin on an I/O board."""

    @staticmethod
    def from_dict(config, io_info):
        """Create the pin from the given config dictionary and board information.

        :param config: The configuration dictionary (or a string containing just the pin name).
        :type config: dict or str
        :param IOInfo io_info: Info about the I/O pins of a board.
        :return: The configured pin.
        :rtype: IOPin
        :raises AttributeError: When wrong configuration is passed.
        """
        raise NotImplementedError()

    @staticmethod
    def find_pin_info(config, io_info):
        """Create the pin from the given config dictionary and board information.

        :param config: The configuration dictionary (or a string containing just the pin name).
        :type config: dict or str
        :param io_info: Info about the I/O pins of a board.
        :type io_info: list of DigitalIOInfo or list of DigitizedAnalogIOInfo or list of RawAnalogIOInfo
        :return: The corresponding pin info if it was found, and the config dictionary. If the function does not raise,
                 the config is guaranteed to contain key 'pin' with the name of the requested pin.
        :rtype: (DigitalIOInfo or DigitizedAnalogIOInfo or RawAnalogIOInfo or None, dict)
        :raises AttributeError: When wrong configuration is passed.
        """
        if isinstance(config, str):
            config = {"pin": config}
        if not isinstance(config, dict):
            raise AttributeError("Config has to be a dictionary or string. The config was: " + repr(config))
        if "pin" not in config:
            raise AttributeError("Expected 'pin' key not found in config. The config was: " + repr(config))
        name = config["pin"]
        for info in io_info:
            if info.pin.name == name:
                return info, config
        return None, config


class DigitalPin(RealIOPin):
    """Digital I/O pin (represents a binary value)."""

    def __init__(self, name, pin_info, inverted):
        """Initialize the pin with the given parameters.

        :param str name: Name of the pin. This name is used to refer to the pin in all parts of this library.
        :param DigitalIOInfo pin_info: Metadata about the pin.
        :param bool inverted: Whether readings of this pin should be seamlessly inverted.
        """
        super(DigitalPin, self).__init__(name, pin_info)
        self.inverted = inverted

    @staticmethod
    def from_dict(config, io_info):  # type: (dict, IOInfo) -> DigitalPin
        pin_info, config = RealIOPin.find_pin_info(config, io_info.digital)
        name = config["pin"]
        if pin_info is None:
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


class DigitizedAnalogPin(RealIOPin):
    """Digitized analog I/O pin (works with integral readings of an A/D converter)."""

    def __init__(self, name, pin_info):
        """Initialize the pin with the given parameters.

        :param str name: Name of the pin. This name is used to refer to the pin in all parts of this library.
        :param DigitizedAnalogIOInfo pin_info: Metadata about the pin.
        """
        super(DigitizedAnalogPin, self).__init__(name, pin_info)

    @staticmethod
    def from_dict(config, io_info):  # type: (dict, IOInfo) -> DigitizedAnalogPin
        pin_info, config = RealIOPin.find_pin_info(config, io_info.digitized_analog)
        name = config["pin"]
        if pin_info is None:
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


class RawAnalogPin(RealIOPin):
    """Raw analog I/O pin (works directly with float values)."""

    def __init__(self, name, pin_info):
        """Initialize the pin with the given parameters.

        :param str name: Name of the pin. This name is used to refer to the pin in all parts of this library.
        :param RawAnalogIOInfo pin_info: Metadata about the pin.
        """
        super(RawAnalogPin, self).__init__(name, pin_info)

    @staticmethod
    def from_dict(config, io_info):  # type: (dict, IOInfo) -> RawAnalogPin
        pin_info, config = RealIOPin.find_pin_info(config, io_info.raw_analog)
        name = config["pin"]
        if pin_info is None:
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


class VirtualPin(IOPin):
    """Virtual I/O board pin which performs some computation on real I/O pins."""

    @staticmethod
    def from_dict(pin_name, config, io_board):
        """Create the pin from the given config dictionary and board information.

        :param str pin_name: Name of the pin (its config dictionary key).
        :param dict config: Configuration of the pin.
        :param IOBoardClient io_board: The I/O board this pin should use (but it is not added to it automatically).
        :return: The virtual pin.
        :rtype: VirtualPin
        :raises AttributeError: When wrong configuration is passed.
        """
        raise NotImplementedError()
