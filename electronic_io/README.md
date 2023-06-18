# electronic_io

Convenience library for interaction with analog and digital inputs and outputs.

This package provides a generalized approach for integration of various electronic input and output devices into ROS.

The three basic concepts in this package are:

- **Pins:** They correspond either to physical input or output pins of a CPU or MCU, or to virtualized pins that e.g.
  internally decode a protocol like 1-Wire and expose the measurements. Multiple pins together form an
  *I/O board*.
- **Virtual Pins:** These are pins that have no physical manifestations, but instead use one or more real pins to
  compute their value. This can be used e.g. for adjusting or combining the values returned by the
  real pins.
- **Devices:** Each device operates on one or more *pins* or *virtual pins* and interprets their values. Input devices
  produce ROS topics with the readings. Output devices provide a ROS service that can be used to change
  value of the device (and its pins).

The idea is that each of these could be representing a different real-world entity:

- The set of real pins can describe a particular I/O board, like Raspberry Pi.
- The set of virtual pins translates datasheets of the connected components into sensible numbers (e.g. ADC -> Voltage).
- The set of devices interprets the pins and virtual pins in terms of high-level devices.

These also specify the two ROS nodes that are expected to be running: a board node (exposing the pins), and a devices
node (handling the virtual pins and devices).

See the [`examples` folder](examples) for inspiration how the nodes and configs can be used and configured.

Configuration of the devices is pretty easy. To add a relay that can power on/off the motors, configure it as simply as:

```yaml
power_switch_motors:
  topic: "power_switch/motors"
  type: electronic_io.PowerSwitch
  output_pins:
    - 'Output7'
```

This configuration provides topic `power_switch/motors` with the state of the switch, and services
`power_switch/motors/set` and `power_switch/motors/toggle` that can change the state of the relay.

The only thing that needs to be coded is the I/O board driver that can actually read and write to pin `Output7`.

Digital pins can also easily be inverted for usage in devices. That can be achieved via advanced pin syntax. Instead of:

```yaml
output_pins:
  - 'Output7'
```

write

```yaml
output_pins:
  - pin: 'Output7'
    inverted: True
```

## Devices available in this package

- Ampere Meter (type `electronic_io.AmpereMeter`)
- Battery (type `electronic_io.Battery`)
- Dimmable LED (type `electronic_io.DimmableLED`)
- Output Group (type `electronic_io.OutputGroup`)
- Power Switch (type `electronic_io.PowerSwitch`)
- Thermometer (type `electronic_io.Thermometer`)
- Voltmeter (type `electronic_io.Voltmeter`)

## Virtual pins available in this package

- Binary PWM (type `electronic_io.BinaryPWM`)
- Digital Pin Combo (type `electronic_io.DigitalPinCombo`)
- Linear ADC Pin (type `electronic_io.LinearADCPin`)

## Adding a new device type

These steps are generally required when you want to specify a new device type (inside or outside this package):

- Add the class that implements the device to an installed Python module, i.e. you need to have `catkin_python_setup()`
  in CMakeLists.txt, a setup.py file and the device's package has to be installed.
- Add the following to package.xml `<export>` section: `<electronic_io device="my_package.MyDevice" />`.

Specifically when adding a device to this package, do not forget to:

- Add it to `src/electronic_io/__init__.py`.
- Add it to this readme.
- Add it to `doc/devices.rst` so that it gets documented.
- Add it to the list of catkin-linted files in `CMakeLists.txt`.

## Adding a new virtual pin

These steps are generally required when you want to specify a new virtual pin (inside or outside this package):

- Add the class that implements the pin to an installed Python module, i.e. you need to have `catkin_python_setup()`
  in CMakeLists.txt, a setup.py file and the device's package has to be installed.
- Add the following to package.xml `<export>` section: `<electronic_io virtual_pin="my_package.MyPin" />`.

Specifically when adding a virtual pin to this package, do not forget to:

- Add it to `src/electronic_io/__init__.py`.
- Add it to this readme.
- Add it to `doc/virtual_pins.rst` so that it gets documented.
- Add it to the list of catkin-linted files in `CMakeLists.txt`.