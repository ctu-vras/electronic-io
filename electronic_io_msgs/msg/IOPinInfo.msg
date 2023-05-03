# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Metadata of a single I/O pin.

string name  # Name of the pin (from SDK, datasheet, package marking etc.)
bool is_readable  # True if the pin value can be read (can also be true for output pins if readback is supported).
bool is_writable  # True if the pin value can be written to (output pins).
bool can_persist  # Whether a writable pin's value can be persisted (i.e. stored in some kind of non-volatile memory).