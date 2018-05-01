#include "potential.hpp"

#include "gtest/gtest.h"

#include <iostream>

class TestMeshParam: public ::testing::Test {
    protected:
        TestMeshParam() {
            /* state << 2, 0, 0; */

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

            Fe_true << 1, 7, 5, 1, 3, 7, 1, 4, 3, 1, 2, 4, 3, 8, 7, 3, 4, 8, 5,
                    7, 8, 5, 8, 6, 1, 5, 6, 1, 6, 2, 2, 6, 8, 2, 8, 4;
            Fe_true = Fe_true - 1;

            mesh_param =  MeshParam(Ve_true, Fe_true);
            /* Fa = Fe_true.col(0); */
            /* Fb = Fe_true.col(1); */
            /* Fc = Fe_true.col(2); */


            /* e1 << 1., 1., 0., */
            /*    0.,    1., 0., */
            /*    0.,    1., 1., */
            /*    0.,    0., 1., */
            /*    1.,    0., 1., */
            /*    0.,    0., 1., */
            /*    0.,    1., 0., */
            /*    0.,    1., 1., */
            /*    1.,    0., 0., */
            /*    1.,    0., 1., */
            /*    1.,    0., 0., */
            /*    1.,    1., 0.; */

            /* e2 << 0.,  -1., 0., */
            /*    1.,     0.,  0., */
            /*    0.,     0.,  -1., */
            /*    0.,     1.,  0., */
            /*    0.,     0.,  -1., */
            /*    1.,     0.,  0., */
            /*    0.,     0.,  1., */
            /*    0.,     -1., 0., */
            /*    0.,     0.,  1., */
            /*    -1.,    0.,  0., */
            /*    0.,     1.,  0., */
            /*    -1.,    0.,  0.; */

            /* e3 << -1., 0.,  0., */
            /*    -1.,    -1., 0., */
            /*    0.,     -1., 0., */
            /*    0.,     -1., -1., */
            /*    -1.,    0.,  0., */
            /*    -1.,    0.,  -1., */
            /*    0.,     -1., -1., */
            /*    0.,     0.,  -1., */
            /*    -1.,    0.,  -1., */
            /*    0.,     0.,  -1., */
            /*    -1.,    -1., 0., */
            /*    0.,     -1., 0.; */


            /* L1_edge_true << 0.6907257, */                    
            /*              0.38976051, */                    
            /*              0.55840222, */                    
            /*              0.38976051, */                    
            /*              0.6907257, */                    
            /*              0.38976051, */                    
            /*              0.6223625, */                    
            /*              0.91098028, */                    
            /*              0.47882542, */                    
            /*              0.6907257, */                    
            /*              0.47882542, */                    
            /*              0.6907257; */
            /* L2_edge_true << 0.6223625, */                    
            /*              0.47882542, */                    
            /*              0.38976051, */                    
            /*              0.38976051, */                    
            /*              0.6223625, */                    
            /*              0.47882542, */                    
            /*              0.6223625, */                    
            /*              0.6223625, */                    
            /*              0.6223625, */                    
            /*              0.47882542, */                    
            /*              0.6223625, */                    
            /*              0.47882542; */
            /* L3_edge_true << 0.47882542, */
            /*              0.6907257, */
            /*              0.38976051, */
            /*              0.55840222, */
            /*              0.47882542, */
            /*              0.6907257, */
            /*              0.91098028, */
            /*              0.6223625, */
            /*              0.6907257, */
            /*              0.38976051, */
            /*              0.6907257, */
            /*              0.38976051; */
        }

        virtual ~TestMeshParam() {
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
        Eigen::Array<double, 8, 3> Ve_true;
        Eigen::Array<int, 12, 3> Fe_true;
        MeshParam mesh_param;
        /* Eigen::Array<int, 12, 1> Fa, Fb, Fc; */
        /* Eigen::Array<double, 12, 3> e1, e2, e3; */
        /* Eigen::Array<double, 12, 1> L1_edge_true, L2_edge_true, L3_edge_true; */
    
        /* Eigen::Array<double, 1, 3> state; */

};

TEST_F(TestMeshParam, NumberVertices) {
    ASSERT_EQ(mesh_param.num_v, 8);
}
TEST_F(TestMeshParam, NumberFaces) {
    ASSERT_EQ(mesh_param.num_f, 12);
}
TEST_F(TestMeshParam, NumberEdges) {
    ASSERT_EQ(mesh_param.num_e, 18);
}

TEST_F(TestMeshParam, Faces) {
    ASSERT_TRUE(mesh_param.Fa.isApprox(Fe_true.col(0)));
    ASSERT_TRUE(mesh_param.Fb.isApprox(Fe_true.col(1)));
    ASSERT_TRUE(mesh_param.Fc.isApprox(Fe_true.col(2)));
}

TEST_F(TestMeshParam, Vertices) {
    Eigen::Matrix<double, 12, 3> V1, V2, V3;
    V1 << -0.5, -0.5, -0.5, 
       -0.5, -0.5, -0.5, 
       -0.5, -0.5, -0.5, 
       -0.5, -0.5, -0.5, 
       -0.5,  0.5, -0.5, 
       -0.5,  0.5, -0.5, 
       0.5, -0.5, -0.5, 
       0.5, -0.5, -0.5, 
       -0.5, -0.5, -0.5, 
       -0.5, -0.5, -0.5, 
       -0.5, -0.5,  0.5, 
       -0.5, -0.5,  0.5;
    V2 << 0.5,  0.5, -0.5,
       -0.5,  0.5, -0.5, 
       -0.5,  0.5,  0.5, 
       -0.5, -0.5,  0.5, 
       0.5,  0.5,  0.5, 
       -0.5,  0.5,  0.5, 
       0.5,  0.5, -0.5, 
       0.5,  0.5,  0.5, 
       0.5, -0.5, -0.5, 
       0.5, -0.5,  0.5, 
       0.5, -0.5,  0.5, 
       0.5,  0.5,  0.5;
    V3 << 0.5, -0.5, -0.5,
       0.5,  0.5, -0.5,  
       -0.5,  0.5, -0.5,  
       -0.5,  0.5,  0.5,  
       0.5,  0.5, -0.5,  
       0.5,  0.5,  0.5,  
       0.5,  0.5,  0.5,  
       0.5, -0.5,  0.5,  
       0.5, -0.5,  0.5,  
       -0.5, -0.5,  0.5,  
       0.5,  0.5,  0.5,  
       -0.5,  0.5,  0.5;
    
    ASSERT_TRUE(mesh_param.V1.isApprox(V1));
    ASSERT_TRUE(mesh_param.V2.isApprox(V2));
    ASSERT_TRUE(mesh_param.V3.isApprox(V3));
}

TEST_F(TestMeshParam, Edges) {
    Eigen::Matrix<double, 12, 3> e1, e2, e3; 
    e1 << 1., 1., 0.,
       0., 1., 0.,  
       0., 1., 1.,  
       0., 0., 1.,  
       1., 0., 1.,  
       0., 0., 1.,  
       0., 1., 0.,  
       0., 1., 1.,  
       1., 0., 0.,  
       1., 0., 1.,  
       1., 0., 0.,  
       1., 1., 0.;
    e2 << 0., -1.,  0.,
       1.,  0.,  0.,  
       0.,  0., -1.,  
       0.,  1.,  0.,  
       0.,  0., -1.,  
       1.,  0.,  0.,  
       0.,  0.,  1.,  
       0., -1.,  0.,  
       0.,  0.,  1.,  
       -1.,  0.,  0.,  
       0.,  1.,  0.,  
       -1.,  0.,  0 ;
    e3 << -1.,  0.,  0.,
       -1., -1.,  0.,  
       0., -1.,  0.,  
       0., -1., -1.,  
       -1.,  0.,  0.,  
       -1.,  0., -1.,  
       0., -1., -1.,  
       0.,  0., -1.,  
       -1.,  0., -1.,  
       0.,  0., -1.,  
       -1., -1.,  0.,  
       0., -1.,  0.;

    ASSERT_TRUE(mesh_param.e1.isApprox(e1));
    ASSERT_TRUE(mesh_param.e2.isApprox(e2));
    ASSERT_TRUE(mesh_param.e3.isApprox(e3));
}

TEST_F(TestMeshParam, EdgeVertexMap) {
    Eigen::Array<int, 12, 2> e1_vertex_map, e2_vertex_map, e3_vertex_map;
    e1_vertex_map << 6, 0, 2, 0, 3, 0, 1, 0, 7, 2, 3, 2, 6, 4, 7, 4, 4,
                  0, 5, 0, 5, 1, 7, 1;
    e2_vertex_map << 4, 6, 6, 2, 2, 3, 3, 1, 6, 7, 7, 3, 7, 6, 5, 7, 5,
                  4, 1, 5, 7, 5, 3, 7;
    e3_vertex_map << 0, 4, 0, 6, 0, 2, 0, 3, 2, 6, 2, 7, 4, 7, 4, 5, 0,
                  5, 0, 1, 1, 7, 1, 3;
    ASSERT_TRUE(mesh_param.e1_vertex_map.isApprox(e1_vertex_map));
    ASSERT_TRUE(mesh_param.e2_vertex_map.isApprox(e2_vertex_map));
    ASSERT_TRUE(mesh_param.e3_vertex_map.isApprox(e3_vertex_map));
}

TEST_F(TestMeshParam, UniqueEdgeVertexMap) {
    Eigen::Matrix<int, 18, 2> e_vertex_map;
    Eigen::Matrix<int, 18, 1> unique_index;
    e_vertex_map << 0, 1,
       0, 2, 
       0, 3, 
       0, 4, 
       0, 5, 
       0, 6, 
       1, 3, 
       1, 5, 
       1, 7, 
       2, 3, 
       2, 6, 
       2, 7, 
       3, 7, 
       4, 5, 
       4, 6, 
       4, 7, 
       5, 7, 
       6, 7;
    ASSERT_TRUE(mesh_param.e_vertex_map.isApprox(e_vertex_map));
    // unique index is correct but gets the second one sometimes
}

TEST_F(TestMeshParam, NormalFace) {
    Eigen::Matrix<double, 12, 3> normal_face;
    normal_face << 0.,  0., -1.,
       0.,  0., -1.,  
       -1.,  0.,  0.,  
       -1.,  0.,  0.,  
       -0.,  1.,  0.,  
       0.,  1.,  0.,  
       1.,  0.,  0.,  
       1.,  0., -0.,  
       0., -1.,  0.,  
       0., -1.,  0.,  
       0.,  0.,  1.,  
       0., -0.,  1.;
    ASSERT_TRUE(mesh_param.normal_face.isApprox(normal_face));
}

TEST_F(TestMeshParam, EdgeNormals) {
    Eigen::Matrix<double, 12, 3> e1_normal, e2_normal, e3_normal;
    e1_normal << -0.70710678,  0.70710678,  0.        ,
       -1.        ,  0.        ,  0.,  
       0.        , -0.70710678,  0.70710678,  
       0.        , -1.        ,  0.,  
       -0.70710678, -0.        ,  0.70710678,  
       -1.        ,  0.        ,  0.,  
       0.        ,  0.        , -1.,  
       -0.        ,  0.70710678, -0.70710678,  
       0.        ,  0.        , -1.,  
       0.70710678,  0.        , -0.70710678,  
       0.        , -1.        ,  0.,  
       0.70710678, -0.70710678, -0.;
    e2_normal << 1.,  0.,  0.,
       -0.,  1.,  0.,  
       0.,  1.,  0.,  
       0., -0.,  1.,  
       1.,  0.,  0.,  
       0.,  0.,  1.,  
       0.,  1.,  0.,  
       0.,  0.,  1.,  
       1.,  0., -0.,  
       0.,  0.,  1.,  
       1.,  0.,  0.,  
       0.,  1.,  0.;
    e3_normal << -0.        , -1.        , -0.        ,
       0.70710678, -0.70710678,  0., 
       -0.        , -0.        , -1., 
       0.        ,  0.70710678, -0.70710678, 
       0.        ,  0.        , -1., 
       0.70710678,  0.        , -0.70710678, 
       0.        , -0.70710678,  0.70710678, 
       0.        , -1.        ,  0., 
       -0.70710678,  0.        ,  0.70710678, 
       -1.        , -0.        , -0., 
       -0.70710678,  0.70710678,  0., 
       -1.        ,  0.        ,  0.;


    ASSERT_TRUE(mesh_param.e1_normal.isApprox(e1_normal, 1e-3));
    ASSERT_TRUE(mesh_param.e2_normal.isApprox(e2_normal, 1e-3));
    ASSERT_TRUE(mesh_param.e3_normal.isApprox(e3_normal, 1e-3));

}

TEST_F(TestMeshParam, CenterFace) {
    Eigen::Matrix<double, 12, 3> center_face;
    center_face << 0.16666667, -0.16666667, -0.5       ,
       -0.16666667,  0.16666667, -0.5, 
       -0.5       ,  0.16666667, -0.16666667, 
       -0.5       , -0.16666667,  0.16666667, 
       0.16666667,  0.5       , -0.16666667, 
       -0.16666667,  0.5       ,  0.16666667, 
       0.5       ,  0.16666667, -0.16666667, 
       0.5       , -0.16666667,  0.16666667, 
       0.16666667, -0.5       , -0.16666667, 
       -0.16666667, -0.5       ,  0.16666667, 
       0.16666667, -0.16666667,  0.5, 
       -0.16666667,  0.16666667,  0.5;
    ASSERT_TRUE(mesh_param.center_face.isApprox(center_face, 1e-3));
 
}

TEST_F(TestMeshParam, VertexFaceMap) {
    
    /* [[0, 1, 2, 3, 8, 9], */   
 /* [3, 9, 10, 11], */       
 /* [1, 2, 4, 5], */         
 /* [2, 3, 5, 11], */        
 /* [0, 6, 7, 8], */         
 /* [7, 8, 9, 10], */        
 /* [0, 1, 4, 6], */         
 /* [4, 5, 6, 7, 10, 11]] */ 

    // iterate and print vf_map
    //assuming you have a "2D" vector vvi (vector of vector of int's)
    /* std::vector< std::vector<int> >::iterator row; */
    /* std::vector<int>::iterator col; */
    /* for (row = vf_map.begin(); row != vf_map.end(); ++row) { */
    /*     for (col = row->begin(); col != row->end(); ++col) { */
    /*         std::cout << *col; */
    /*     } */
    /*     std::cout << std::endl; */
    /* } */

    ASSERT_EQ(mesh_param.vf_map[0][0], 0);
    ASSERT_EQ(mesh_param.vf_map[0][5], 9);
}

TEST_F(TestMeshParam, EdgeIndexMap) {
    Eigen::Matrix<int, 12, 1> e1_ind1b, e1_ind2b, e1_ind3b,
                              e2_ind1b, e2_ind2b, e2_ind3b,
                              e3_ind1b, e3_ind2b, e3_ind3b;
    e1_ind1b << -1, -1, -1, -1, -1, -1, -1, -1, -1 , -1, -1, -1;
    e1_ind2b << -1, -1, -1, -1, -1,  2,  0, -1, -1 , -1,  9, -1;
    e1_ind3b <<1,  2,  3,  9,  5, -1, -1,  6,  0 ,  8, -1, 10;
    
    e2_ind1b << 6, -1,  5, -1, -1, -1, -1, -1, -1, 10, -1, -1; 
    e2_ind2b << -1, -1, -1, -1,  6, 11,  4, 10, -1 , -1,  7,  5 ;
    e2_ind3b << -1,  4, -1, 11, -1, -1, -1, -1,  7 , -1, -1, -1;
    
    e3_ind1b <<  8,  0,  1,  2, -1,  4,  7, -1,  9,  3, 11, -1;          
    e3_ind2b << -1, -1, -1, -1,  1, -1, -1,  8, -1, -1, -1,  3;         
    e3_ind3b << -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1; 

    ASSERT_TRUE(mesh_param.e1_ind1b.isApprox(e1_ind1b));
    ASSERT_TRUE(mesh_param.e1_ind2b.isApprox(e1_ind2b));
    ASSERT_TRUE(mesh_param.e1_ind3b.isApprox(e1_ind3b));

    ASSERT_TRUE(mesh_param.e2_ind1b.isApprox(e2_ind1b));
    ASSERT_TRUE(mesh_param.e2_ind2b.isApprox(e2_ind2b));
    ASSERT_TRUE(mesh_param.e2_ind3b.isApprox(e2_ind3b));

    ASSERT_TRUE(mesh_param.e3_ind1b.isApprox(e3_ind1b));
    ASSERT_TRUE(mesh_param.e3_ind2b.isApprox(e3_ind2b));
    ASSERT_TRUE(mesh_param.e3_ind3b.isApprox(e3_ind3b));
}

TEST_F(TestMeshParam, EdgeFaceMap) {
    Eigen::Matrix<int, 12, 4> e1_face_map, e2_face_map, e3_face_map;
    e1_face_map << 0, -1, -1,  1,
       1, -1, -1,  2, 
       2, -1, -1,  3, 
       3, -1, -1,  9, 
       4, -1, -1,  5, 
       5, -1,  2, -1, 
       6, -1,  0, -1, 
       7, -1, -1,  6, 
       8, -1, -1,  0, 
       9, -1, -1,  8, 
       10, -1,  9, -1, 
       11, -1, -1, 10; 
    e2_face_map <<0,  6, -1, -1, 
       1, -1, -1,  4, 
       2,  5, -1, -1, 
       3, -1, -1, 11, 
       4, -1,  6, -1, 
       5, -1, 11, -1, 
       6, -1,  4, -1, 
       7, -1, 10, -1, 
       8, -1, -1,  7, 
       9, 10, -1, -1, 
       10, -1,  7, -1, 
       11, -1,  5, -1; 
    e3_face_map << 0,  8, -1, -1, 
       1,  0, -1, -1, 
       2,  1, -1, -1, 
       3,  2, -1, -1, 
       4, -1,  1, -1, 
       5,  4, -1, -1, 
       6,  7, -1, -1, 
       7, -1,  8, -1, 
       8,  9, -1, -1, 
       9,  3, -1, -1, 
       10, 11, -1, -1, 
       11, -1,  3, -1; 

    ASSERT_TRUE(mesh_param.e1_face_map.isApprox(e1_face_map));
    ASSERT_TRUE(mesh_param.e2_face_map.isApprox(e2_face_map));
    ASSERT_TRUE(mesh_param.e3_face_map.isApprox(e3_face_map));
}

TEST_F(TestMeshParam, FaceDyad) {
    Eigen::Matrix<double, 3, 3> F_face_zero, F_face_eleven;
    F_face_zero << 0, 0, 0, 0, 0, 0, 0, 0, 1;
    F_face_eleven << 0, 0, 0, 0, 0, 0, 0, 0, 1;
    ASSERT_TRUE(mesh_param.F_face[0].isApprox(F_face_zero));
    ASSERT_TRUE(mesh_param.F_face[11].isApprox(F_face_eleven));
}

TEST_F(TestMeshParam, LaplacianFactor) {
    Eigen::Matrix<double, 1, 3> state;
    state << 1, 2, 3;
    Eigen::Matrix<double, Eigen::Dynamic, 3> r_v;
    r_v = mesh_param.V.rowwise() - state;
    Eigen::Matrix<double, 12, 1> w_face;
    w_face = laplacian_factor(r_v, mesh_param.Fa, mesh_param.Fb, mesh_param.Fc);

    Eigen::Matrix<double, 12, 1> w_face_true;
    w_face_true << 0.02362863, 0.02502464, 0.01241267, 0.01325258, -0.01641329,
           -0.01932066, -0.00512972, -0.00553228, 0.01806088, 0.02041505,
           -0.03174127, -0.03465722;
    
    std::cout << w_face.transpose() << std::endl;
    EXPECT_TRUE(w_face.isApprox(w_face_true, 1e-4));
}

/* TEST_F(TestPotential, EdgeFactor) { */
/*     Eigen::Array<double, Eigen::Dynamic, 3> r_v; */
/*     r_v = Ve_true.rowwise() - state; */
/*     Eigen::Array<double, 12, 1> L1_edge, L2_edge, L3_edge; */
/*     int flag = edge_factor(r_v, e1, e2, e3, e1_vertex_map, e2_vertex_map, e3_vertex_map, */
/*             L1_edge, L2_edge, L3_edge); */
    
/*     EXPECT_TRUE(L1_edge.isApprox(L1_edge_true, 1e-7)); */
/*     EXPECT_TRUE(L2_edge.isApprox(L2_edge_true, 1e-7)); */
/*     EXPECT_TRUE(L3_edge.isApprox(L3_edge_true, 1e-7)); */
/* } */

