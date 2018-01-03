import pdb

import numpy as np

from dynamics import asteroid
from point_cloud import wavefront, raycaster

class TestRayCaster():
    
    # load a polyhedron
    v, f = wavefront.read_obj('./data/shape_model/ITOKAWA/itokawa_low.obj')

    # define the raycaster object
    caster = raycaster.RayCaster.loadmesh(v, f, flag='obb', scale=1.0)

    # test methods of caster
    V, F = wavefront.polydatatomesh(caster.polydata)
    bounds = caster.polydata.GetBounds()

    def test_vertices_equal(self):
        np.testing.assert_allclose(self.V, self.v)

    def test_faces_equal(self):
        np.testing.assert_allclose(self.F, self.f)
    
    def test_bounds_x(self):
        xmin, xmax = self.bounds[0:2]
        np.testing.assert_allclose((xmin, xmax), (min(self.v[:,0]), max(self.v[:, 0])))

    def test_bounds_y(self):
        ymin, ymax = self.bounds[2:4]
        np.testing.assert_allclose((ymin, ymax), (min(self.v[:,1]), max(self.v[:, 1])))

    def test_bounds_z(self):
        zmin, zmax = self.bounds[4:6]
        np.testing.assert_allclose((zmin, zmax), (min(self.v[:,2]), max(self.v[:, 2])))
    
    def test_scale_factor(self):
        scale = 1e3
        caster = raycaster.RayCaster.loadmesh(self.v, self.f, scale=scale)
        xmin, xmax = caster.polydata.GetBounds()[0:2]
        np.testing.assert_allclose((xmin, xmax), scale * np.array([min(self.v[:, 0]), max(self.v[:, 0])]))
    
    def test_castray_number_intersections(self):
        intersections = self.caster.castray([5, 0, 0], [-5, 0, 0], all_out=True)
        np.testing.assert_allclose(intersections.shape, (2,3))
    
    def test_castray_no_intersections(self):
        intersections = self.caster.castray([5, 0, 0], [6, 0, 0])
        np.testing.assert_allclose(intersections.shape, (0,))

    def test_castray_coordinates(self):
        intersections = self.caster.castray([5, 0, 0], [-5, 0, 0], all_out=True)
        np.testing.assert_allclose(intersections[0, :], np.array([0.29648888, 0, 0]))
        np.testing.assert_allclose(intersections[1, :], np.array([-0.2460786, 0, 0]))
    
    def test_ispointinside_yes(self):
        point = np.array([0,0,0])
        np.testing.assert_allclose(self.caster.ispointinside(point), True)

    def test_ispointinside_no(self):
        point = np.array([5,5,5])
        np.testing.assert_allclose(self.caster.ispointinside(point),False)

    def test_distance(self):
        pa = np.array([5, 0, 0])
        pb = np.array([0, 0, 0])
        np.testing.assert_allclose(self.caster.distance(pa, pb), 5)
    
    def test_source_inside_body(self):
        psource = np.array([0, 0, 0])
        ptarget = np.array([5, 0, 0])
        intersections = self.caster.castray(psource, ptarget)
        np.testing.assert_allclose(intersections.shape, (3))

        
def test_update_raycasting_mesh(): 
    # load a polyhedron
    ast = asteroid.Asteroid('castalia', 0, 'obj')
    v, f = ast.V, ast.F
    nv = ast.rotate_vertices(0)
    nf = f

    # define the raycaster object
    caster = raycaster.RayCaster.loadmesh(v, f, flag='obb', scale=1.0)
    ncaster = raycaster.RayCaster.updatemesh(nv, nf)

    # test methods of caster
    V, F = wavefront.polydatatomesh(caster.polydata)
    nV, nF = wavefront.polydatatomesh(ncaster.polydata)

    np.testing.assert_allclose(V, v)
