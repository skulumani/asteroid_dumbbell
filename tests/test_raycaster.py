import numpy as np
from point_cloud import wavefront

class TestRayCaster():
    
    # load a polyhedron
    v, f = wavefront.read_obj('./data/shape_model/ITOKAWA/itokawa_low.obj')

    # define the raycaster object
    caster = wavefront.RayCaster.loadmesh(v, f, scale=1.0)
    V, F = wavefront.polydatatomesh(caster.polydata)

    def test_vertices_equal(self):
        np.testing.assert_allclose(self.V, self.v)

    def test_faces_equal(self):
        np.testing.assert_allclose(self.F, self.f)

    # test that raycasting works

    # test scale method
