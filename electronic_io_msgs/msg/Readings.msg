# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Readings of a (sub)set of I/O pins.

Header header  # Timestamp of all readings (or of the first reading if synchronous reads are not supported).
               # frame_id should be the same as in the corresponding IOInfo message.
DigitalReading[] digital
RawAnalogReading[] raw_analog
DigitizedAnalogReading[] digitized_analog
