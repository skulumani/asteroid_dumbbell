#include "surface_mesher.hpp"

#include "gtest/gtest.h"

TEST(SurfMeshLoading, PolyhedronVertices) {
    SurfMesh smesh(0.5, 0.5, 0.5, 10, 0.1, 0.5);
    ASSERT_GT(smesh.poly.size_of_vertices(), 100);
}

TEST(SurfMeshLoading, PolyhedronFaces) {
    SurfMesh smesh(0.5, 0.5, 0.5, 10, 0.1, 0.5);
    ASSERT_GT(smesh.poly.size_of_facets(), 100);
}

TEST(SurfMeshLoading, VertexGetter) {
    SurfMesh smesh(0.5, 0.5, 0.5, 10, 0.1, 0.5);
    ASSERT_TRUE(smesh.get_verts().isApprox(smesh.v));
}

TEST(SurfMeshLoading, FacesGetter) {
    SurfMesh smesh(0.5, 0.5, 0.5, 10, 0.1, 0.5);
    ASSERT_TRUE(smesh.get_faces().isApprox(smesh.f));
}

TEST(SurfMeshLoading, EigenAndPolyhedronEqual){
    SurfMesh smesh(0.5, 0.5, 0.5, 10, 0.1, 0.5);
    ASSERT_EQ(smesh.poly.size_of_vertices(),  smesh.get_verts().rows());
    ASSERT_EQ(smesh.poly.size_of_facets(),  smesh.get_faces().rows());
}
