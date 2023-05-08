# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""
Group of Output Devices
=======================

**Device type:** Output device without readback.

**Set output service type:** Same as the service of the grouped devices.

**Pin requirements:** Does not use any pins itself.

This is a meta-device that offers easy synchronized control of multiple output devices of the same type.

YAML config
-----------

::

   topic: 'TOPIC NAME'  # Base topic for readback and the services. Set to empty string to disable topics and services.
   type: electronic_io.OutputGroup
   devices:  # List of grouped devices. All devices must be defined in the same config and must have the same type.
     - device1  # Name of the device.

"""

from ..device import OutputDevice, MetaDevice


class OutputGroup(OutputDevice, MetaDevice):
    """A group of output devices"""

    def __init__(self, name, config, io_board, devices):
        MetaDevice.__init__(self, name, config, io_board, devices)

        device_types = set()
        service_type = None

        for device in self._devices.values():
            if not isinstance(device, OutputDevice):
                raise AttributeError("Sub-device %s of %s is not an output device." % (
                    device.get_name(), self.get_name()))
            device_types.add(type(device))
            service_type = device.service_type

        if len(device_types) != 1:
            raise AttributeError("%s has to contain only devices of the exact same type, but the following were "
                                 "found: %s" % (self.get_name(), ", ".join(map(str, device_types))))

        OutputDevice.__init__(self, name, config, io_board, service_type)

    def get_name(self):
        return "Group " + self.name

    def _add_write_request(self, value, write_req):
        for device in self._devices.values():
            # noinspection PyProtectedMember
            device._add_write_request(value, write_req)

    def set_value_cb(self, request):
        resp = None
        for device in self._devices.values():
            resp = device.set_value_cb(request)
        return resp
