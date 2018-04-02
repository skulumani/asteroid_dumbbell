#include "surface_mesher.hpp"

#include "gtest/gtest.h"

TEST(SurfMeshLoading, PolyhedronVertices) {
    SurfMesh smesh(0.5, 0.5, 0.5, 10, 0.1, 0.5);
    ASSERT_GT(smesh.poly.size_of_vertices(), 100);
}
