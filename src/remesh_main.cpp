#include "loader.hpp"
#include "mesh.hpp"
#include "cgal.hpp"
#include "stats.hpp"

#include "input_parser.hpp"

#include <CGAL/Polygon_mesh_processing/remesh.h>

#include <igl/writeOBJ.h>

#include <memory>
#include <iostream>
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
        std::cout << "Usage: remesh -i input_file.obj" << std::endl;
    }

    const std::string input_file = input.get_command_option("-i");
    if (input_file.empty()) {
        std::cout << "No input file!" << std::endl;
        return 1;
    } 

    std::shared_ptr<MeshData> mesh = Loader::load(input_file);

    // define a set of faces to refine
    std::vector<Face_index> faces_to_refine;
    Face_index fd(0);
    faces_to_refine.push_back(fd);
    faces_to_refine.push_back(fd);

    std::cout << "Old number of vertices: " << mesh->surface_mesh.number_of_vertices() << std::endl;

    double target_edge_length(0.1);

    CGAL::Polygon_mesh_processing::isotropic_remeshing(
            faces(mesh->surface_mesh),
            target_edge_length,
            mesh->surface_mesh,
            CGAL::Polygon_mesh_processing::parameters::number_of_iterations(3));
    
    mesh->surface_mesh.collect_garbage();

    std::cout << "New Number of vertices: " << mesh->surface_mesh.number_of_vertices() << std::endl;
    
    std::string output_file = "/tmp/" + remove_extension(base_name(input_file)) + "_remesh.obj";
    std::cout << "Saving to: " + output_file << std::endl;

    igl::writeOBJ(output_file, mesh->get_verts(), mesh->get_faces());
    return 0;
}
