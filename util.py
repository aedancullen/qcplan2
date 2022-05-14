import numpy as np
from numba import njit

@njit(cache=True)
def discretize(low, high, length, value):
    return round((value - low) / (high - low) * (length - 1))
