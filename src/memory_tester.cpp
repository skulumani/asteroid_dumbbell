/**
    Test out each class and find memory issues

    @author Shankar Kulumani
    @version 10 May 2018
*/
#include "loader.hpp"
#include "mesh.hpp"

int main() {
    std::string input_filename("./data/shape_model/ITOKAWA/itokawa_low.obj");
    std::shared_ptr<MeshData> mesh;
    mesh = Loader::load(input_filename);
    return 0;
}
