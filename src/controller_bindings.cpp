/**
    Bindings for controller module

    @author Shankar Kulumani
    @version 4 May 2018
*/
#include "controller.hpp"
#include "reconstruct.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

PYBIND11_MODULE(controller, m) {
    m.doc() = "Controller functions in C++";

    pybind11::class_<AttitudeController, std::shared_ptr<AttitudeController>>(m, "AttitudeController")
        .def(pybind11::init<>(), "Attitude Controller constructor")
        .def("body_fixed_pointing_attitude", (void (AttitudeController::*)(const double&, const Eigen::Ref<const Eigen::Matrix<double, 1, 18> >&)) &AttitudeController::body_fixed_pointing_attitude,
                "Body fixed pointing direction",
                pybind11::arg("time"), pybind11::arg("state"))
        .def("get_Rd", &AttitudeController::get_Rd, "Get the rotation matrix")
        .def("get_Rd_dot", &AttitudeController::get_Rd_dot, "Get teh rotation matrix derivative")
        .def("get_ang_vel_d", &AttitudeController::get_ang_vel_d, "Get the angular velocity")
        .def("get_ang_vel_d_dot", &AttitudeController::get_ang_vel_d, "Get the angular velocity derivative");

    pybind11::class_<TranslationController, std::shared_ptr<TranslationController>>(m, "TranslationController")
        .def(pybind11::init<>(), "Translation Controller constructor")
        .def("inertial_fixed_state", (void (TranslationController::*)(const double&,
                        const Eigen::Ref<const Eigen::Matrix<double, 1, 18> >&,
                        const Eigen::Ref<const Eigen::Matrix<double, 1, 3> >&)) &TranslationController::inertial_fixed_state,
                "Inertially fixed state")
        .def("get_posd", &TranslationController::get_posd, "Get the desired position")
        .def("get_veld", &TranslationController::get_veld, "Get the desired velocity")
        .def("get_acceld", &TranslationController::get_acceld, "Get the desired acceleration")
        .def("minimize_uncertainty", (void (TranslationController::*)(const Eigen::Ref<const Eigen::Matrix<double, 1, 18> >&,
                        std::shared_ptr<const ReconstructMesh>)) &TranslationController::minimize_uncertainty,
                "Find position to minimize uncertainty",
                pybind11::arg("state"), pybind11::arg("rmesh shared_ptr"));

    pybind11::class_<Controller, AttitudeController, TranslationController, std::shared_ptr<Controller>>(m, "Controller")
        .def(pybind11::init<>(), "Combinded controller constructor")
        .def("explore_asteroid", (void (Controller::*)(const Eigen::Ref<const Eigen::Matrix<double, 1, 18> >&,
                        std::shared_ptr<const ReconstructMesh>)) &Controller::explore_asteroid,
                "Explore an asteroid by minimizing uncertainty",
                pybind11::arg("state"), pybind11::arg("rmesh shared_ptr"));
    
    // TODO Add overload for explore asteroid function then add here as well
}
