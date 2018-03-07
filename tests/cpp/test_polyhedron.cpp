#include "wavefront.hpp"
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
    read_flag = obj::read_to_eigen(input_file, V,  F);
    Polyhedron_builder<HalfedgeDS> builder(V, F);
    P.delegate(builder);
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
    int read_flag;
    Polyhedron P;
    const std::string input_file = "./integration/cube.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
};

// Tests that the Foo::Bar() method does Abc.
TEST_F(TestPolyhedron, PolyhedronIsValid) {
    ASSERT_EQ(P.is_valid(), 1);
}

// Test number of vertices and faces
TEST_F(TestPolyhedron, PolyhedronVertices) {
    EXPECT_EQ(P.size_of_vertices(), 8);
}

TEST_F(TestPolyhedron, PolyhedronFaces) {
    EXPECT_EQ(P.size_of_facets(), 12);
}

TEST(Polyhedron, PolyhedronToEigen) {
// TODO test that we can go from a P to V, F
}
