#include "loader.hpp"
#include "mesh.hpp"
#include "cgal.hpp"
#include "polyhedron.hpp"
#include "stats.hpp"

#include "input_parser.hpp"

#include <CGAL/Polygon_mesh_processing/refine.h>

#include <igl/writeOBJ.h>

#include <memory>
#include <iostream>
#include <fstream>
#include <string>

template<class T>
T base_name(T const & path, T const & delims = "/\\")
{
  return path.substr(path.find_last_of(delims) + 1);
}
template<class T>
T remove_extension(T const & filename)
{
  typename T::size_type const p(filename.find_last_of('.'));
  return p > 0 && p != T::npos ? filename.substr(0, p) : filename;
}

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
    faces_to_refine.push_back(mesh->face_descriptor[0]);
    
    // vectors to store the new faces/vertices
    std::vector<Face_index> new_faces;
    std::vector<Vertex_index> new_vertices;
    CGAL::Polygon_mesh_processing::refine(
            mesh->surface_mesh,
            faces_to_refine, // faces(mesh->surface_mesh)
            std::back_inserter(new_faces),
            std::back_inserter(new_vertices),
            CGAL::Polygon_mesh_processing::parameters::density_control_factor(4.0));
    
    std::cout << "Refinement added " << new_vertices.size() << " vertices." << std::endl;
    // write to OBJ
    std::string output_file = "/tmp/" + remove_extension(base_name(input_file)) + "_refine.obj";
    std::cout << "Saving to: " + output_file << std::endl;
    
    // take surface mesh and convert to vertices
    igl::writeOBJ(output_file, mesh->get_surface_mesh_vertices(), mesh->get_surface_mesh_faces());

    return 0;
}
