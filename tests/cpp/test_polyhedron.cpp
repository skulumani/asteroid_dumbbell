#include "polyhedron.hpp"

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

typedef CGAL::Simple_cartesian<double>     Kernel;
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3>         Polyhedron;
typedef Polyhedron::HalfedgeDS             HalfedgeDS;

// The fixture for testing class Foo.
class TestPolyhedron: public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  TestPolyhedron() {
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

  virtual ~TestPolyhedron() {
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

// Tests that the Foo::Bar() method does Abc.
TEST_F(TestPolyhedron, PolyhedronIsValid) {
    Mesh mesh(input_file);
    ASSERT_EQ(mesh.Poly.is_valid(), 1);
    ASSERT_EQ(1, 1);
}

// Test number of vertices and faces
TEST_F(TestPolyhedron, PolyhedronVertices) {
    Mesh mesh(input_file);
    EXPECT_EQ(mesh.Poly.size_of_vertices(), 8);
}

TEST_F(TestPolyhedron, PolyhedronFaces) {
    Mesh mesh(input_file);
    EXPECT_EQ(mesh.Poly.size_of_facets(), 12);
}

TEST_F(TestPolyhedron, PolyhedronToEigen) {
    Mesh mesh(input_file);
    EXPECT_EQ(mesh.vertices.size(), Ve_true.size());
    EXPECT_EQ(mesh.faces.size(), Fe_true.size());
}
