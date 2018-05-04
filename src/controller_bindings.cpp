/**
    Bindings for controller module

    @author Shankar Kulumani
    @version 4 May 2018
*/
#include "controller.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

PYBIND11_MODULE(controller, m) {
    m.doc() = "Controller functions in C++";

    pybind11::class_<AttitudeController, std::shared_ptr<AttitudeController>>(m, "AttitudeController")
        .def(pybind11::init<>(), "Attitude Controller constructor")
        .def("body_fixed_pointing_attitude", (void (AttitudeController::*)(const double&, const Eigen::Ref<const Eigen::Matrix<double, 1, 18> >&)) &AttitudeController::body_fixed_pointing_attitude,
                "Body fixed pointing direction",
                pybind11::arg("time"), pybind11::arg("state"));

}
