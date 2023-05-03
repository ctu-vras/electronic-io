# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Metadata of a group of digital or analog I/O pins.
# This message should be published latched on topic "~/io_info" where "~" is the private namespace of the I/O group.
# This message contains the complete description of the I/O group, i.e. all pins at once.
# Raw Analog and Digitized Analog pins can be duplicated, i.e. it is possible that one pin can be reported both with the
# "raw" and "digitized" interface.

Header header  # Stamp should be time of the driver startup. frame_id should identify the I/O board. It does not need
               # to be a TF frame.
DigitalIOInfo[] digital  # Digital I/O pins.
RawAnalogIOInfo[] raw_analog  # Raw analog I/O pins (directly working with the represented analog value).
DigitizedAnalogIOInfo[] digitized_analog  # Digitized analog I/O pins (working with the integral output of an ADC).
