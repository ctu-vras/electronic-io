# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Metadata of a digitized analog I/O pin (working with ADC values).

IOPinInfo pin  # The I/O pin metadata.
duration sampling_period  # Duration of sampling (inverse of sampling frequency).
                          # If not known, output-only or variable, set to 0.
int64 min_value  # Minimum value that can be read from / written to the pin (usually zero).
int64 max_value  # Maximum value that can be read from / written to the pin. If equal to min_value, there is no limit.
