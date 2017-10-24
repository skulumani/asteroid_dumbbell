import numpy as np
from mayavi import mlab

def test_triangular_mesh():
    """An example of a cone

    http://docs.enthought.com/mayavi/mayavi/auto/mlab_helper_functions.html#triangular-mesh
    """
    n = 8
    t = np.linspace(-np.pi, np.pi, n)
    z = np.exp(1j * t)
    x = z.real.copy()
    y = z.imag.copy()
    z = np.zeros_like(x)

    triangles = [(0, i, i + 1) for i in range(1, n)]
    x = np.r_[0, x]
    y = np.r_[0, y]
    z = np.r_[1, z]
    t = np.r_[0, t]
    
    mlab.triangular_mesh(x, y, z, triangles)
    mlab.show()
    return 0

if __name__ == '__main__':
    test_triangular_mesh()
