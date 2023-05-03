# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# One write request of a raw analog I/O pin (working with the represented analog values).

string name  # Name of the pin (from SDK, datasheet, package marking etc.)
float64 value  # The represented analog value. If possible, it should be in SI units.
bool persist  # If true and the pin has can_persist set to True, try to make the value persistent.