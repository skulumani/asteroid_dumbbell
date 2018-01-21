"""Examples to test out CGAL operations

https://doc.cgal.org/latest/Manual/tutorial_hello_world.html
"""

import numpy as np
import CGAL.CGAL_Kernel as kernel
from CGAL import CGAL_Convex_hull_2

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
    print("s = {}".format(s))

def convex_hull_2d():
    """Convex hull for two dimensions
    """
    points = []
    points.append(kernel.Point_2(0, 0))
    points.append(kernel.Point_2(1, 0))
    points.append(kernel.Point_2(0, 1))
    points.append(kernel.Point_2(1, 1,))
    points.append(kernel.Point_2(0.5, 0.5))
    points.append(kernel.Point_2(0.25, 0.25))

    result = []
    CGAL_Convex_hull_2.convex_hull_2(points, result)

    for p in result:
        print(p)
    
    points.append(kernel.Point_2(2, 2))
    n = kernel.Point_2()

    CGAL_Convex_hull_2.ch_n_point(points, n)
    print(n)
