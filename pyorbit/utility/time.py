# **********************************************************************************************************************
#   FileName:
#       time.py
#
#   Description:
#       Timing utilities for interacting with the OrbitESC device
#
#   11/25/2023 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************


def period_to_hz(a: float, b: float, unit: float = 1e-6) -> float:
    """
    Converts a timestamp delta to a frequency in Hz. This is meant as a helper method for the
    various data packets coming out of OrbitESC.

    Args:
        a: Timestamp A
        b: Timestamp B
        unit: Unit of the timestamps to convert to seconds

    Returns:
        Frequency in Hz
    """
    return 1.0 / (abs(a - b) * unit)
