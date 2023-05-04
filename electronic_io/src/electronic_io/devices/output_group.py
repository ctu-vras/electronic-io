# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import rospy

from cras_msgs.msg import VoltageStamped

from ..device import OutputDevice, MetaDevice


class OutputGroup(OutputDevice, MetaDevice):
    """
    A group of output devices of the same type that can all be controlled simultaneously using a single command.
    
    YAML config is: ::
    
       input_pin:
         pin: 'PIN NAME'  # required; name of the pin (has to be a raw analog pin)
       frame_id: 'frame'  # optional; frame_id to be used in the messages; if not set, it is taken from the readings
       variance: 1.0  # optional, default 0.0; the reported variance of the measurements (in Volt^2).

    """

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
            device._add_write_request(value, write_req)

    def set_value_cb(self, request):
        resp = None
        for device in self._devices.values():
            resp = device.set_value_cb(request)
        return resp
