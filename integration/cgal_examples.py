"""Examples to test out CGAL operations

https://doc.cgal.org/latest/Manual/tutorial_hello_world.html
"""
import pdb

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

def convex_hull_3d():
    """Convex hull for three dimensions

    This is not implemented in CGAL swig
    """
    from CGAL.CGAL_Kernel import Point_3
    from CGAL.CGAL_Kernel import Plane_3
    from CGAL import CGAL_Convex_hull_3
    from CGAL.CGAL_Polyhedron_3 import Polyhedron_3

    pts = []
    pts.append(Point_3(0, 0, 0))
    pts.append(Point_3(0, 1, 0))
    pts.append(Point_3(1, 1, 0))
    pts.append(Point_3(1, 0, 0))
    pts.append(Point_3(0, 0, 1))
    pts.append(Point_3(0, 1, 1))
    pts.append(Point_3(1, 1, 1))
    pts.append(Point_3(1, 0, 1))

    res = Polyhedron_3()

    CGAL_Convex_hull_3.convex_hull_3(pts, res)

    print('Convex hull has {} vertices'.format(res.size_of_vertices()))
    print('is strongly convex: {}'.format(CGAL_Convex_hull_3.is_strongly_convex_3(res)))

def AABB_polyhedron_facet_intersection():
    """Facet intersection mode
    AABB (axis aligned bounding boxes)

    The AABB is a data structure for finding intersections

    We do the following:
        * Create a polyhedron
        * Create a segment and plane
        * Find the intersection of segment and plane with polyhedron
    """

    from CGAL.CGAL_Kernel import Point_3
    from CGAL.CGAL_Kernel import Vector_3
    from CGAL.CGAL_Kernel import Plane_3
    from CGAL.CGAL_Kernel import Segment_3
    from CGAL.CGAL_Polyhedron_3 import Polyhedron_3
    from CGAL.CGAL_AABB_tree import AABB_tree_Polyhedron_3_Facet_handle

    p = Point_3(1.0, 0.0, 0.0)
    q = Point_3(0.0, 1.0, 0.0)
    r = Point_3(0.0, 0.0, 1.0)
    s = Point_3(0.0, 0.0, 0.0)
    polyhedron = Polyhedron_3()
    polyhedron.make_tetrahedron(p, q, r, s)

    # construct the AABB tree
    tree = AABB_tree_Polyhedron_3_Facet_handle(polyhedron.facets())

    # construct segment query
    a = Point_3(-0.2, 0.2, -0.2)
    b = Point_3(1.3, 0.2, 1.3)
    segment_query = Segment_3(a,b)

    # do the intersection of segment with polyhedron
    if tree.do_intersect(segment_query):
        print("Intersection")
    else:
        print("No intersection")

    # compute the number of intersections with the segment
    print(tree.number_of_intersected_primitives(segment_query), " intersection")

    # compute first intersection with segment 
    intersection = tree.any_intersection(segment_query)
    if not intersection.empty():
        # get the intersection object
        op = intersection.value()
        object = op[0]
        if object.is_Point_3():
            print("intersection object is a point")

    # compute all intersections with the segment query
    primitives = []
    tree.all_intersected_primitives(segment_query,primitives)

    # construct plane query
    vec = Vector_3(0, 0, 1.0)
    plane_query = Plane_3(Point_3(0, 0, 0.5), vec)
    # compute first intersection of tetrahedron and plane
    intersection = tree.any_intersection(plane_query)
    if not intersection.empty():
        op = intersection.value()
        object = op[0]
        if object.is_Segment_3():
            print("intersection object is a segment")
