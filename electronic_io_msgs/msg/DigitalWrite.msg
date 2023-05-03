# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# One write request of a digital I/O pin.

string name  # Name of the pin (from SDK, datasheet, package marking etc.)
bool value  # Requested value of the pin (true = ON, false = OFF).
bool persist  # If true and the pin has can_persist set to True, try to make the value persistent.