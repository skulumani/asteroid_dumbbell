#include "loader.hpp"
#include "mesh.hpp"
#include "cgal.hpp"
#include "polyhedron.hpp"
#include "stats.hpp"

#include "input_parser.hpp"

#include <CGAL/Polygon_mesh_processing/refine.h>

#include <memory>
#include <iostream>
#include <fstream>

int main(int argc, char* argv[]) {
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Usage: refine -i input_file.obj" << std::endl;
    }

    const std::string input_file = input.get_command_option("-i");
    if (input_file.empty()) {
        std::cout << "No input file!" << std::endl;
        return 1;
    } 

    std::shared_ptr<MeshData> mesh = Loader::load(input_file);
    
    // define a set of faces to refine
    std::vector<Face_index> faces_to_refine;
    faces_to_refine[0] = mesh->face_descriptor[0];

}
