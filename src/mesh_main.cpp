#include "loader.hpp"
#include "mesh.hpp"

#include "input_parser.hpp"

#include "stats.hpp"

#include <memory>


int main(int argc, char* argv[]) {
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Usage mesh -i input_file.obj" << std::endl;
    }

    const std::string input_file = input.get_command_option("-i");
    std::shared_ptr<MeshData> mesh;

    // create a mesh object
    if (!input_file.empty()) {
        mesh = Loader::load(input_file);

    }

    /* std::cout << "Vertices: \n" << mesh->vertices << std::endl; */
    /* std::cout << "Faces: \n" << mesh->faces << std::endl; */

    print_polyhedron_vertices(mesh);
    
    // lets try and build a surface mesh now
    
    std::cout << "#Vertices : " << mesh->surface_mesh.number_of_vertices() << std::endl;
    std::cout << "#Faces: " << mesh->surface_mesh.number_of_faces() << std::endl;
    
    // add vertex to to surface mesh
    /* typedef CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3> Mesh; */
    /* typedef Mesh::Vertex_index vertex_descriptor; */
    /* typedef Mesh::Face_index face_descriptor; */

    /* const unsigned int num_v = mesh->surface_mesh.number_of_vertices(); */
    /* const unsigned int num_f = mesh->surface_mesh.number_of_faces(); */

    /* // vertex iterators over the mesh */
    /* Mesh::Vertex_range::iterator vb, ve; */
    /* Mesh::Vertex_range r = mesh->surface_mesh.vertices(); */

    /* vb = r.begin(); */
    /* ve = r.end(); */
    
    /* // iterate over vertices and print to stdout */
    /* for (vertex_descriptor vd : mesh->surface_mesh.vertices()) { */
    /*     std::cout << vd << std::endl; */ 
    /* } */
    return 0;
}
