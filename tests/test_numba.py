"""Test numba code compiling for speed

"""

from __future__ import absolute_import, division, print_function, unicode_literals
import numpy as np
import pytest
pytestmark = pytest.importorskip('numba')
    
def test_numba_installed():
    ver = '0.32.0'
    assert isinstance(numba.__version__,str)
