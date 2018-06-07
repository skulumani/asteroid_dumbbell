#include "mesh.hpp"
#include "loader.hpp"

#include "gtest/gtest.h"

#include <memory>

// The fixture for testing class Foo.
class TestMeshData: public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  TestMeshData() {
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

  virtual ~TestMeshData() {
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

TEST_F(TestMeshData, EigenConstructor) {
    MeshData mesh(Ve_true, Fe_true);
    ASSERT_TRUE(mesh.vertices.isApprox(Ve_true));
    ASSERT_TRUE(mesh.faces.isApprox(Fe_true));
}

TEST_F(TestMeshData, UpdateMesh) {
    MeshData mesh;
    mesh.update_mesh(Ve_true, Fe_true);
    ASSERT_TRUE(mesh.vertices.isApprox(Ve_true));
    ASSERT_TRUE(mesh.faces.isApprox(Fe_true));
    ASSERT_EQ(mesh.polyhedron.is_valid(), 1);
    
}

TEST_F(TestMeshData, PolyhedronVertexIndexMatch) {
    // check to make sure eigen_v.row(n) == Polyhedron vertex(n)
    // initialize the mesh data object
    MeshData mesh(Ve_true, Fe_true);
    // go to a specific vertex and get teh vector
    Polyhedron::Vertex_iterator pv_iter = mesh.polyhedron.vertices_begin();
    
    for (int ii = 0; ii < Ve_true.rows(); ++ii) {
        EXPECT_EQ(pv_iter->point().x(), Ve_true(ii, 0));
        EXPECT_EQ(pv_iter->point().y(), Ve_true(ii, 1));
        EXPECT_EQ(pv_iter->point().z(), Ve_true(ii, 2));
        std::advance(pv_iter, 1);
    }
}

TEST_F(TestMeshData, PolyhedronFaceIndexMatch) {
    MeshData mesh(Ve_true, Fe_true);
    std::size_t f_index = 0;    
    for (Polyhedron::Facet_iterator f_iter = mesh.polyhedron.facets_begin(); f_iter != mesh.polyhedron.facets_end(); ++f_iter) {
        /* std::advance(f_iter, 1); */
        Polyhedron::Halfedge_around_facet_const_circulator he = f_iter->facet_begin();
        // loop over all vertices of the face
        std::size_t c = 0;
        do {
            EXPECT_EQ(he->vertex()->id(), Fe_true(f_index, c));
            c++;
        } while (++he != f_iter->facet_begin());
        f_index++;
    }
}

TEST_F(TestMeshData, SurfaceMeshVertexIndexMatch) {
    MeshData mesh(Ve_true, Fe_true);
    
    for (int ii = 0; ii < mesh.vertex_descriptor.size(); ++ii) {
        EXPECT_EQ(mesh.surface_mesh.point(mesh.vertex_descriptor[ii]).x(), Ve_true(ii, 0));
        EXPECT_EQ(mesh.surface_mesh.point(mesh.vertex_descriptor[ii]).y(), Ve_true(ii, 1));
        EXPECT_EQ(mesh.surface_mesh.point(mesh.vertex_descriptor[ii]).z(), Ve_true(ii, 2));
    }

    /* for(Mesh::Vertex_index vd : mesh.surface_mesh.vertices()){ */
    /*     std::cout << mesh.surface_mesh.point(vd).x() << std::endl; */
    /* } */
}

TEST_F(TestMeshData, SurfaceMeshFaceIndexMatch) {
    MeshData mesh(Ve_true, Fe_true);

    for (int ii = 0; ii < mesh.vertex_in_face_descriptor.size(); ++ii) {
        // get the vertices of this face
        for (int jj = 0; jj < mesh.vertex_in_face_descriptor[ii].size(); ++jj) {
            EXPECT_EQ(mesh.surface_mesh.point(mesh.vertex_in_face_descriptor[ii][jj]).x(), Ve_true(Fe_true(ii, jj), 0));
            EXPECT_EQ(mesh.surface_mesh.point(mesh.vertex_in_face_descriptor[ii][jj]).y(), Ve_true(Fe_true(ii, jj), 1));
            EXPECT_EQ(mesh.surface_mesh.point(mesh.vertex_in_face_descriptor[ii][jj]).z(), Ve_true(Fe_true(ii, jj), 2));
        }
    }
}

TEST_F(TestMeshData, GetSurfaceMeshVerticesCube) {
    MeshData mesh(Ve_true, Fe_true);
    Eigen::Matrix<double, Eigen::Dynamic, 3> out_verts;
    out_verts = mesh.get_surface_mesh_vertices();
    ASSERT_TRUE(out_verts.isApprox(Ve_true));
}

TEST_F(TestMeshData, GetSurfaceMeshFacesCube) {
    MeshData mesh(Ve_true, Fe_true);
    Eigen::Matrix<int, Eigen::Dynamic, 3> out_faces;
    out_faces = mesh.get_surface_mesh_faces();
    ASSERT_TRUE(out_faces.isApprox(Fe_true)); 
}

TEST_F(TestMeshData, GetSurfaceMeshVertexCube) {
    MeshData mesh(Ve_true, Fe_true);
    std::size_t index_1(0);
    int index_2(0);
    EXPECT_TRUE(mesh.get_vertex(index_1).isApprox(Ve_true.row(index_1))); 
    ASSERT_TRUE(mesh.get_vertex(index_1).isApprox(mesh.get_vertex(index_2)));
}

TEST_F(TestMeshData, GetSurfaceMeshFaceVerticesCube) {
    MeshData mesh(Ve_true, Fe_true);
    std::size_t index1(0);
    int index2(0);
    
    EXPECT_TRUE(mesh.get_face_vertices(index1).isApprox(Fe_true.row(index1)));
    ASSERT_TRUE(mesh.get_face_vertices(index1).isApprox(mesh.get_face_vertices(index2)));
}

TEST_F(TestMeshData, BuildSurfaceMeshFaceNormalsCube) {
    MeshData mesh(Ve_true, Fe_true);
    Face_index fd(0);
    ASSERT_EQ(mesh.get_face_normal(fd).size(), 3);
}

TEST_F(TestMeshData, BuildSurfaceMeshCenterFaceCube) {
    MeshData mesh(Ve_true, Fe_true);
    Face_index fd(0);
    ASSERT_EQ(mesh.get_face_center(fd).size(), 3);
}

TEST_F(TestMeshData, BuildSurfaceMeshHalfedgeNormalsCube) {
    MeshData mesh(Ve_true, Fe_true);
    Halfedge_index hd(0);
    ASSERT_EQ(mesh.get_halfedge_normal(hd).size(), 3);
}

TEST_F(TestMeshData, EdgeFactorCube) {
    MeshData mesh(Ve_true, Fe_true);
    Eigen::VectorXd edge_factor_true(mesh.surface_mesh.number_of_edges());
    edge_factor_true << 0.690726, 0.533437, 0.424906, 0.424906, 0.533437,
                     0.690726, 0.533437, 0.424906, 0.533437, 1.00573, 0.838132,
                     0.838132, 1.00573, 0.838132, 0.533437, 0.690726, 0.533437,
                     1.00573;

    // build edge factor
    Eigen::Vector3d pos;
    pos << 1, 1, 1;
    mesh.build_edge_factor(pos);
    for(Edge_index ed : mesh.surface_mesh.edges()) {
        EXPECT_NEAR(mesh.get_edge_factor(ed), edge_factor_true((int)ed), 1e-3);
    }
}

TEST(TestMeshDataCastalia, OutwardFaceNormals) {
    std::shared_ptr<MeshData> mesh = Loader::load("./data/shape_model/CASTALIA/castalia.obj");
    for (Face_index fd: mesh->surface_mesh.faces() ) {
        Eigen::Vector3d face_normal, center_face;
        face_normal = mesh->get_face_normal(fd);
        center_face = mesh->get_face_center(fd);
        EXPECT_GT(face_normal.dot(center_face), 0);
    }
}

TEST(TestMeshDataCastalia, SymmetricFaceDyad) {
    std::shared_ptr<MeshData> mesh = Loader::load("./data/shape_model/CASTALIA/castalia.obj");
    for (Face_index fd: mesh->surface_mesh.faces()) {
        EXPECT_TRUE(mesh->get_face_dyad(fd).isApprox(
                    mesh->get_face_dyad(fd).transpose(), 1e-3));
    }
}


TEST(TestMeshDataCastalia, SymmetricEdgeDyad) {
    std::shared_ptr<MeshData> mesh = Loader::load("./data/shape_model/CASTALIA/castalia.obj");
    for (Edge_index ed: mesh->surface_mesh.edges()) {

        EXPECT_TRUE(mesh->get_edge_dyad(ed).isApprox(
                    mesh->get_edge_dyad(ed).transpose(), 1e-3));
    }
}

TEST(TestMeshDataItokawa, SymmetricFaceDyad) {
    std::shared_ptr<MeshData> mesh = Loader::load("./data/shape_model/ITOKAWA/itokawa_low.obj");
    for (Face_index fd: mesh->surface_mesh.faces()) {
        EXPECT_TRUE(mesh->get_face_dyad(fd).isApprox(
                    mesh->get_face_dyad(fd).transpose(), 1e-3));
    }
}


TEST(TestMeshDataItokawa, SymmetricEdgeDyad) {
    std::shared_ptr<MeshData> mesh = Loader::load("./data/shape_model/ITOKAWA/itokawa_low.obj");
    for (Edge_index ed: mesh->surface_mesh.edges()) {
        EXPECT_TRUE((mesh->get_edge_dyad(ed) - mesh->get_edge_dyad(ed))
                .isApprox(Eigen::Matrix3d::Zero(), 1e-3));
    }
}
