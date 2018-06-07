#include "potential.hpp"
#include "loader.hpp"
#include "mesh.hpp"

#include "gtest/gtest.h"

#include <iostream>

TEST(TestAsteroid, CubeGravity) {
    std::shared_ptr<MeshData> mesh_data = Loader::load("./integration/cube.obj");
    Eigen::Matrix<double, 1, 3> state;
    state << 1, 2, 3;

    Asteroid ast("cube", mesh_data);
    ast.polyhedron_potential(state);

    const double U_true = 1.7834673284883827e-08;
    Eigen::Matrix<double, 3, 1> U_grad_true;
    U_grad_true <<-1.27357660e-09, -2.54759656e-09, -3.82240787e-09;
    Eigen::Matrix<double, 3, 3> U_grad_mat_true;
    U_grad_mat_true << -1.00116623e-09,  5.45412364e-10,  8.18746722e-10,
 5.45412364e-10, -1.82972253e-10,  1.63835424e-09,
 8.18746722e-10,  1.63835424e-09,  1.18413848e-09;
    const double Ulaplace_true = 9.260647804154587e-25;

    ASSERT_NEAR(ast.get_potential(), U_true, 1e-7);
    ASSERT_TRUE(ast.get_acceleration().isApprox(U_grad_true, 1e-7));
    ASSERT_TRUE(ast.get_gradient_mat().isApprox(U_grad_mat_true, 1e-7));
    ASSERT_NEAR(ast.get_laplace(), Ulaplace_true, 1e-7);
}

TEST(TestAsteroid, CastaliaGravity) {
    std::shared_ptr<MeshData> mesh_data = Loader::load("./data/shape_model/CASTALIA/castalia.obj");
    Eigen::Matrix<double, 1, 3> state;
    state << 1, 2, 3;

    Asteroid ast("cube", mesh_data);
    ast.polyhedron_potential(state);

    const double U_true = 1.7834673284883827e-08;
    Eigen::Matrix<double, 3, 1> U_grad_true;
    U_grad_true <<-1.27357660e-09, -2.54759656e-09, -3.82240787e-09;
    Eigen::Matrix<double, 3, 3> U_grad_mat_true;
    U_grad_mat_true << -1.00116623e-09,  5.45412364e-10,  8.18746722e-10,
 5.45412364e-10, -1.82972253e-10,  1.63835424e-09,
 8.18746722e-10,  1.63835424e-09,  1.18413848e-09;
    const double Ulaplace_true = 9.260647804154587e-25;

    ASSERT_NEAR(ast.get_potential(), U_true, 1e-7);
    ASSERT_TRUE(ast.get_acceleration().isApprox(U_grad_true, 1e-7));
    ASSERT_TRUE(ast.get_gradient_mat().isApprox(U_grad_mat_true, 1e-7));
    ASSERT_NEAR(ast.get_laplace(), Ulaplace_true, 1e-7);

}
