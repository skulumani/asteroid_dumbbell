import numpy as np
from point_cloud import wavefront, raycaster

class TestRayCaster():
    
    # load a polyhedron
    v, f = wavefront.read_obj('./data/shape_model/ITOKAWA/itokawa_low.obj')

    # define the raycaster object
    caster = raycaster.RayCaster.loadmesh(v, f, scale=1.0)

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

    # test scale method
