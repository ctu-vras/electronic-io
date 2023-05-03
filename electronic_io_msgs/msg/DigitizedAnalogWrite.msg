# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# One write request of a digitized analog I/O pin (working with raw ADC values).

string name  # Name of the pin (from SDK, datasheet, package marking etc.)
int64 value  # The requested ADC value.
bool persist  # If true and the pin has can_persist set to True, try to make the value persistent.