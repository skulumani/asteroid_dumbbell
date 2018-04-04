#include "loader.hpp"
#include "mesh.hpp"

#include "gtest/gtest.h"

TEST(LoaderTest, OBJString) {
    std::string input_filename("./integration/cube.obj");
    std::shared_ptr<MeshData> mesh;
    mesh = Loader::load(input_filename);
    ASSERT_EQ(mesh->polyhedron.is_valid(), 1);
    ASSERT_EQ(mesh->vertices.rows(), 8);
}


