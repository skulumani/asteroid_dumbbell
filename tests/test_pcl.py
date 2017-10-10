"""Testing for Point Cloud Library
"""
import numpy as np
import pickle
try:
    import pcl
except ImportError:
    print("PCL not installed. Run utilities/build_pcl.sh")

_data = [(i, 2 * i, 3 * i + 0.2) for i in range(5)]
_DATA = """0.0, 0.0, 0.2;
           1.0, 2.0, 3.2;
           2.0, 4.0, 6.2;
           3.0, 6.0, 9.2;
           4.0, 8.0, 12.2"""


class TestListIO():
    p = pcl.PointCloud(_data)

    def test_from_list(self):
        for i, d in enumerate(_data):
            pt = self.p[i]
            np.testing.assert_allclose(pt, _data[i])

    def test_to_list(self):
        l = self.p.to_list()
        np.testing.assert_allclose(l, _data)

class TestNumpyIO():
    a = np.array(np.mat(_DATA, dtype=np.float32))
    p = pcl.PointCloud(a)

    def testFromNumpy(self):
        for i, d in enumerate(_data):
            pt = self.p[i]
            np.testing.assert_allclose(pt, _data[i])

    def testToNumpy(self):
        a = self.p.to_array()
        np.testing.assert_allclose(a, self.a)

    def test_asarray(self):
        p = pcl.PointCloud(self.p)      # copy
        # old0 = p[0]
        a = np.asarray(p)               # view
        a[:] += 6
        np.testing.assert_array_almost_equal(p[0], a[0])

        # Regression test: deleting a second view would previously
        # reset the view count to zero.
        b = np.asarray(p)
        del b

        np.testing.assert_raises(ValueError, p.resize, 2 * p.size)

    def test_pickle(self):
        """Test pickle support."""
        # In this testcase because picking reduces to pickling NumPy arrays.
        s = pickle.dumps(self.p)
        p = pickle.loads(s)
        np.testing.assert_allclose(self.a , p.to_array())
