#include "stats.hpp"
#include "surface_mesher.hpp"
#include "mesh.hpp"
#include "potential.hpp"
#include "reconstruct.hpp"
#include "loader.hpp"

#include <Eigen/Dense>

#include "gtest/gtest.h"

#include <iostream>

class TestStats: public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  TestStats() {
    // You can do set-up work for each test here.
    // initialize the mesh
    Ve_true << -0.5,   -0.5, -0.5,
            -0.5, -0.5, 0.5,
            -0.5, 0.5,  -0.5,
            -0.5, 0.5,  0.5,
            0.5,  -0.5, -0.5,
            0.5,  -0.5, 0.5,
            0.5,  0.5,  -0.5,
            0.5,  0.5,  0.5;

    Fe_true << 1, 7, 5,
            1, 3, 7,
            1, 4, 3,
            1, 2, 4,
            3, 8, 7,
            3, 4, 8,
            5, 7, 8,
            5, 8, 6,
            1, 5, 6,
            1, 6, 2,
            2, 6, 8,
            2, 8, 4;
    Fe_true = Fe_true.array() - 1;
  }

  virtual ~TestStats() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for Foo.
    const std::string input_file = "./integration/cube.obj";
    Eigen::Matrix<double, 8, 3> Ve_true;
    Eigen::Matrix<int, 12, 3> Fe_true;
};

TEST_F(TestStats, CubeVolume) {
    double volume;
    volume = PolyVolume::volume(Ve_true, Fe_true);
    ASSERT_EQ(volume, 1);
}

TEST_F(TestStats, SphereVolume) {
    // create a sphere (ish) shape
    SurfMesh smesh(1, 1, 1, 10, 0.1, 0.1);
    double volume = PolyVolume::volume(smesh.get_verts(), smesh.get_faces());
    ASSERT_NEAR(volume, 4.19, 1e-1);
}

TEST_F(TestStats, MeshDataVolume) {
    SurfMesh smesh(1, 1, 1, 10, 0.1, 0.1);
    double volume_verts = PolyVolume::volume(smesh.get_verts(), smesh.get_faces());
    
    std::shared_ptr<MeshData> mesh_ptr = std::make_shared<MeshData>(smesh.get_verts(), smesh.get_faces());
    double volume_mesh = PolyVolume::volume(mesh_ptr);
    ASSERT_NEAR(volume_verts, volume_mesh, 1e-6);
}

TEST_F(TestStats, AsteroidVolume) {
    SurfMesh smesh(1, 1, 1, 10, 0.1, 0.1);
    double volume_verts = PolyVolume::volume(smesh.get_verts(), smesh.get_faces());
    std::shared_ptr<Asteroid> ast = std::make_shared<Asteroid>("cube", smesh.get_verts(), 
                                                               smesh.get_faces());
    double volume_ast = PolyVolume::volume(ast);
    ASSERT_NEAR(volume_verts, volume_ast, 1e-6);
}

TEST_F(TestStats, ReconstructMeshVolume) {
    SurfMesh smesh(1, 1, 1, 10, 0.1, 0.1);
    double volume_verts = PolyVolume::volume(smesh.get_verts(), smesh.get_faces());
    std::shared_ptr<ReconstructMesh> rmesh_ptr = std::make_shared<ReconstructMesh>(smesh.get_verts(), smesh.get_faces()); 
    double volume_rmesh = PolyVolume::volume(rmesh_ptr);
    ASSERT_NEAR(volume_verts, volume_rmesh, 1e-6);
}

TEST_F(TestStats, MeshDataEqualtoVertices) {
    std::shared_ptr<MeshData> mesh = Loader::load("./data/shape_model/CASTALIA/castalia.obj");
    double volume_verts = PolyVolume::volume(mesh->get_verts(), mesh->get_faces());
    double volume_mesh = PolyVolume::volume(mesh);
    ASSERT_NEAR(volume_verts, volume_mesh, 1e-6);
}
