# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""
Battery
=======

**Device type:** Input device.

**Message type:** :sensor_msgs:`BatteryState`.

**Pin requirements:** This is a composite device working on other devices and not pins directly.
                      One or more Voltmeter devices. Optional Ampere meter device.

YAML config
-----------

::

   topic: 'TOPIC NAME'
   type: electronic_io.Battery
   queue_size: 1  # Optional. Topic queue size.
   latch: True  # Optional. Topic latching status.
   frame_id: 'frame'  # Optional. frame_id to be used in the messages; if not set, it is taken from the readings.
   # overall_voltmeter and cell_voltmeters are optional, but at least one of them has to be specified.
   overall_voltmeter: voltmeter_overall  # Optional. Voltmeter device. If missing, the overall voltage will be summed
                                         # from cells.
   cell_voltmeters:  # Optional. Measurements of individual cells.
     - voltmeter_cell1  # Voltmeter device.
   ampere_meter: ampere_meter_1  # Optional. AmpereMeter device.
   percentage_pin:  # Optional. Raw analog pin with percentage in 0.0 - 1.0.
     pin: percentage_1
   present_pin:  # Optional. Digital pin. Tells whether the battery is present.
     pin: input_1

"""

from sensor_msgs.msg import BatteryState

from .ampere_meter import AmpereMeter
from ..device import InputDevice, MetaDevice
from .voltmeter import Voltmeter


class Battery(InputDevice, MetaDevice):
    """Battery device. Publishes messages of type :sensor_msgs:`BatteryState`."""

    def __init__(self, name, config, io_board, devices):
        if not isinstance(config, dict):
            raise AttributeError("Invalid config, dict expected, but got " + str(type(config)))

        config["devices"] = []
        if "overall_voltmeter" in config:
            config["devices"].append(config["overall_voltmeter"])
        if "ampere_meter" in config:
            config["devices"].append(config["ampere_meter"])
        if "cell_voltmeters" in config:
            config["devices"] += config["cell_voltmeters"]

        MetaDevice.__init__(self, name, config, io_board, devices)
        InputDevice.__init__(self, name, config, io_board, BatteryState, 1, True)

        self._frame_id = config.get("frame_id", None)
        self._capacity = float(config.get("capacity", float('nan')))
        self._design_capacity = float(config.get("design_capacity", float('nan')))
        self._power_supply_status = config.get("power_supply_status", BatteryState.POWER_SUPPLY_STATUS_UNKNOWN)
        self._power_supply_health = config.get("power_supply_health", BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN)
        self._power_supply_technology = config.get("power_supply_technology",
                                                   BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
        self._location = str(config.get("location", ""))
        self._serial_number = str(config.get("serial_number", ""))

        if isinstance(self._power_supply_status, str):
            self._power_supply_status = getattr(BatteryState, self._power_supply_status)
        if isinstance(self._power_supply_health, str):
            self._power_supply_health = getattr(BatteryState, self._power_supply_health)
        if isinstance(self._power_supply_technology, str):
            self._power_supply_technology = getattr(BatteryState, self._power_supply_technology)

        self._overall_voltmeter = None
        if "overall_voltmeter" in config:
            device = self._devices[config["overall_voltmeter"]]
            if not isinstance(device, Voltmeter):
                raise AttributeError("overall_voltmeter %s in battery %s is not a Voltmeter." % (
                    config["overall_voltmeter"], self.get_name()))
            self._overall_voltmeter = device

        self._ampere_meter = None
        if "ampere_meter" in config:
            device = self._devices[config["ampere_meter"]]
            if not isinstance(device, AmpereMeter):
                raise AttributeError("ampere_meter %s in battery %s is not an Ampere meter." % (
                    config["ampere_meter"], self.get_name()))
            self._ampere_meter = device

        self._cell_voltmeters = []
        if "cell_voltmeters" in config:
            if not isinstance(config["cell_voltmeters"], list):
                raise AttributeError("Key 'cell_voltmeters' in %s has to contain a list." % (self.get_name(),))
            for cell_voltmeter in config["cell_voltmeters"]:
                device = self._devices[cell_voltmeter]
                if not isinstance(device, Voltmeter):
                    raise AttributeError("cell_voltmeter %s in battery %s is not a Voltmeter." % (
                        cell_voltmeter, self.get_name()))
                self._cell_voltmeters.append(device)

        if self._overall_voltmeter is None and len(self._cell_voltmeters) == 0:
            raise AttributeError("No voltmeter was specified in %s." % (self.get_name(),))

        self._percentage_pin = None
        if "percentage_pin" in config:
            self._percentage_pin = io_board.get_raw_analog_pin(config["percentage_pin"])

        self._present_pin = None
        if "present_pin" in config:
            self._present_pin = io_board.get_digital_pin(config["present_pin"])

    def get_name(self):
        return "Battery " + self.name

    def add_read_request(self, req):
        for device in self._devices.values():  # type: InputDevice
            device.add_read_request(req)
        if self._percentage_pin is not None:
            self._percentage_pin.add_read_request(req)
        if self._present_pin is not None:
            self._present_pin.add_read_request(req)

    def get_value(self, readings):
        overall_voltage = 0.0
        current = float('nan')
        cell_voltages = []
        percentage = float('nan')
        present = True

        if len(self._cell_voltmeters) > 0:
            for cell in self._cell_voltmeters:
                cell_voltages.append(cell.get_value(readings))

        if self._overall_voltmeter is not None:
            overall_voltage = self._overall_voltmeter.get_value(readings)
        else:
            overall_voltage = sum(cell_voltages)

        if self._ampere_meter is not None:
            current = self._ampere_meter.get_value(readings)

        if self._percentage_pin is not None:
            percentage = self._percentage_pin.get_value(readings)

        if self._present_pin is not None:
            present = self._present_pin.get_value(readings)

        return overall_voltage, current, cell_voltages, percentage, present

    def create_message(self, value, header):
        overall_voltage, current, cell_voltages, percentage, present = value

        msg = BatteryState()
        msg.header = header
        if self._frame_id is not None:
            msg.header.frame_id = self._frame_id
        msg.voltage = overall_voltage
        msg.current = current
        msg.cell_voltage = cell_voltages
        msg.charge = float('nan')
        msg.percentage = percentage
        msg.capacity = self._capacity
        msg.design_capacity = self._design_capacity
        msg.power_supply_status = self._power_supply_status
        msg.power_supply_health = self._power_supply_health
        msg.power_supply_technology = self._power_supply_technology
        msg.present = present
        msg.location = self._location
        msg.serial_number = self._serial_number

        return msg
