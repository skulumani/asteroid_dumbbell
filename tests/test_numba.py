"""Test numba code compiling for speed

"""

import numpy as np
import numba

def test_numba_installed():
    ver = '0.32.0'
    np.testing.assert_string_equal(numba.__version__, ver)
