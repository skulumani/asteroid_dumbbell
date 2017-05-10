"""Test numba code compiling for speed

"""

import numpy as np
import numba

def test_numba_installed():
    ver = '0.32.0'
    assert isinstance(numba.__version__,str)
