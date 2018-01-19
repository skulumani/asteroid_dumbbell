"""Examples to test out CGAL operations

https://doc.cgal.org/latest/Manual/tutorial_hello_world.html
"""

import numpy as np
import CGAL.CGAL_Kernel as kernel
import CGAL

def points_and_segments():
    """Example from CGAL Hello world
    """
    p = kernel.Point_2(1, 1)
    q = kernel.Point_2(10, 10)
    
    print("p = {}".format(p))
    print("q = {}".format(q))

    dist = kernel.squared_distance(p, q)

    print("sqdist(p, q) = {}".format(dist))

    s = kernel.Segment_2(p, q)
    m = kernel.Point_2(5, 9)
    
    print("m = {}".format(m))
    print("sqdist(seg(p,q, m)) = {}".format(kernel.squared_distance(s, m)))

