"""Testing out multithreading
"""
from multiprocessing import Pool
import numpy as np

def return_norm(vec):
    """Return norm of an input vector
    """
    norm = np.linalg.norm(vec)
    return norm

array = np.random.rand(10, 3)

with Pool(5) as p:
    print(p.map(return_norm, [array[ii, :] for ii in range(10)]))
