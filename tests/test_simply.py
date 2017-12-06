"""Simple test for travis
"""

import numpy as np

def test_true():
    np.testing.assert_allclose(1, 1)
