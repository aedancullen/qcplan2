import numpy as np
from numba import njit

@njit(cache=True)
def discretize(low, high, length, value):
    if value < low or value > high:
        return None
    return round((value - low) / (high - low) * (length - 1))
