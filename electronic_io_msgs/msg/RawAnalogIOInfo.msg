# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Metadata of a raw analog I/O pin (working directly with the represented analog values).

IOPinInfo pin  # The I/O pin metadata.
string unit  # The unit in which the analog value is expressed (if possible, it should be an SI unit).
duration sampling_period  # Duration of sampling (inverse of sampling frequency).
                          # If not known, output-only or variable, set to 0.
float64 min_value  # Minimum value that can be read from / written to the pin.
                   # If possible, it should be in SI units. Use -inf if unbounded (or zeros for both min/max).
                   # Do not use NaN. 
float64 max_value  # Maximum value that can be read from / written to the pin.
                   # If possible, it should be in SI units. Use +inf if range is not known (or zeros for both min/max).
                   # Do not use NaN.
