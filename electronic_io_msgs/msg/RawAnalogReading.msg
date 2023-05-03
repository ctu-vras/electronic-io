# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# One reading of a raw analog I/O pin (working directly with the represented analog value).

string name  # Name of the pin (from SDK, datasheet, package marking etc.)
float64 value  # The represented analog value. If possible, it should be in SI units.
