import python_example
import numpy as np

def test_addition():
    np.testing.assert_allclose(python_example.add(1, 2), 3)

def test_subtraction():
    np.testing.assert_allclose(python_example.subtract(3,1), 2)
