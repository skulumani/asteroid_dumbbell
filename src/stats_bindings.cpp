/**
    Polyhedron stats bindings

    @author Shankar Kulumani
    @version 31 May 2018
*/

#include "stats.hpp"
#include "mesh.hpp"
#include "potential.hpp"
#include "reconstruct.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

PYBIND11_MODULE(stats, m) {
    m.doc() = "Statistics for Polyhedrons in C++";

    m.def("volume", (double (*)(const Eigen::Ref<const Eigen::MatrixXd>&,
                                const Eigen::Ref<const Eigen::MatrixXi>&)) &PolyVolume::volume, "Volume of polyhedron from matrices",
            pybind11::arg("vertices"), pybind11::arg("faces"));
    m.def("volume", (double (*)(std::shared_ptr<const MeshData>)) &PolyVolume::volume, "Volume of a polyhedron from meshdata");
    m.def("volume", (double (*)(std::shared_ptr<const Asteroid>)) &PolyVolume::volume, "Volume of a polyhedron from Asteroid");
    m.def("volume", (double (*)(std::shared_ptr<const ReconstructMesh>)) &PolyVolume::volume, "Volume of a polyhedron from ReconstructMesh");

}
