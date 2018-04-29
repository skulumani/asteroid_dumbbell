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

            /* w_face_true << 0.03808065, 0.02361574, 0.07694205, 0.07694205, */
            /*             0.03808065, 0.02361574, -0.20033484, -0.20033484, */
            /*             0.03808065, 0.02361574, 0.03808065,0.02361574; */

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
        /* Eigen::Array<double, 12, 1> w_face_true; */
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
}
/* TEST_F(TestPotential, LaplacianFactor) { */
/*     /1* Eigen::Array<double, 1, 3> state; *1/ */
/*     /1* state << 2, 0, 0; *1/ */
/*     Eigen::Array<double, Eigen::Dynamic, 3> r_v; */
/*     r_v = Ve_true.rowwise() - state; */
/*     Eigen::Array<double, 12, 1> w_face; */
/*     int flag = laplacian_factor(r_v, Fa, Fb, Fc, w_face); */
/*     EXPECT_TRUE(w_face.isApprox(w_face_true, 1e-7)); */
/* } */

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

    /* ASSERT_EQ(mesh_param.vf_map[0][0], 0); */
    /* ASSERT_EQ(mesh_param.vf_map[0][5], 9); */
}

TEST_F(TestMeshParam, EdgeVertexMap) {
    Eigen::Array<int, 12, 2> e1_vertex_map, e2_vertex_map, e3_vertex_map;
    e1_vertex_map << 6, 0, 2, 0, 3, 0, 1, 0, 7, 2, 3, 2, 6, 4, 7, 4, 4,
                  0, 5, 0, 5, 1, 7, 1;
    e2_vertex_map << 4, 6, 6, 2, 2, 3, 3, 1, 6, 7, 7, 3, 7, 6, 5, 7, 5,
                  4, 1, 5, 7, 5, 3, 7;
    e3_vertex_map << 0, 4, 0, 6, 0, 2, 0, 3, 2, 6, 2, 7, 4, 7, 4, 5, 0,
                  5, 0, 1, 1, 7, 1, 3;
    MeshParam mesh_param(Ve_true, Fe_true);
    /* std::cout << mesh_param.e1_vertex_map << std::endl; */
    /* ASSERT_TRUE(mesh_param.e1_vertex_map.isApprox(e1_vertex_map)); */
}
