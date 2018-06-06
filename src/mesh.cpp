#include "mesh.hpp"
#include "polyhedron.hpp"

#include <igl/copyleft/cgal/mesh_to_polyhedron.h>
#include <igl/copyleft/cgal/polyhedron_to_mesh.h>

// TODO Convert polyhedron/surface_mesh back to Eigen
// Member methods
MeshData::MeshData(const Eigen::Ref<const Eigen::MatrixXd> &V, const Eigen::Ref<const Eigen::MatrixXi> &F) {
    this->vertices = V;
    this->faces = F;

    this->build_polyhedron();
    this->build_surface_mesh();
}

void MeshData::build_polyhedron() {
    // only called after initialization
    eigen_to_polyhedron(this->vertices, this->faces, this->polyhedron);
    /* igl::copyleft::cgal::mesh_to_polyhedron(vertices, faces, polyhedron); */
}

void MeshData::build_surface_mesh() {
    typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
    typedef Mesh::Vertex_index vertex_descriptor;

    // create some vertices
    Kernel::Point_3 p, p1, p2, p3;
    Mesh::Vertex_index v, v1, v2, v3;

    // save the data to the class
    Eigen::MatrixXd& V = this->vertices;
    Eigen::MatrixXi& F = this->faces;

    // build the mesh
    // build vector of vertex descriptors

    for (int ii = 0; ii < V.rows(); ++ii) {
        p = Kernel::Point_3(V(ii, 0), V(ii, 1), V(ii, 2));
        v = this->surface_mesh.add_vertex(p);

        this->vertex_descriptor.push_back(v);
    }
    

    std::vector<vertex_descriptor> face_indices;

    for (int ii = 0; ii < F.rows(); ++ii) {
        p1 = Kernel::Point_3(V(F(ii, 0), 0), V(F(ii, 0), 1), V(F(ii, 0), 2));
        p2 = Kernel::Point_3(V(F(ii, 1), 0), V(F(ii, 1), 1), V(F(ii, 2), 2));
        p3 = Kernel::Point_3(V(F(ii, 2), 0), V(F(ii, 2), 1), V(F(ii, 2), 2));

        v1 = this->vertex_descriptor[F(ii, 0)];
        v2 = this->vertex_descriptor[F(ii, 1)];
        v3 = this->vertex_descriptor[F(ii, 2)];

        this->surface_mesh.add_face(v1, v2, v3);
        face_indices = {v1, v2, v3};
        this->vertex_in_face_descriptor.push_back(face_indices);
    }
    
    // store face descriptors to an array
    for(Face_index fd: surface_mesh.faces()) {
        face_descriptor.push_back(fd);
    }

    /* // can also loop over edges */
    /* for(Edge_index ed: surface_mesh.edges()) { */
    /*     // returns a vertex_index for the start/finish of the edge */
    /*     vds = source(ed, surface_mesh); */
    /*     vde = end(ed, surface_mesh); */
    /* } */
}

void MeshData::update_mesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F) {
    // update the polyhedron and surface mesh
    this->vertices = V;
    this->faces = F;
    
    // clear the mesh
    this->surface_mesh.clear();
    this->polyhedron.clear();
    
    // clear the descriptors
    vertex_descriptor.clear();
    vertex_in_face_descriptor.clear();
    face_descriptor.clear();

    this->build_polyhedron();
    this->build_surface_mesh();
}


Eigen::Matrix<double, Eigen::Dynamic, 3> MeshData::get_surface_mesh_vertices( void ) {
    // extract out vertices from surface_mesh
    std::size_t num_v = surface_mesh.number_of_vertices();
    std::size_t row_index = 0;

    Eigen::Matrix<double, Eigen::Dynamic, 3> vertices(num_v, 3);

    for (Vertex_index vd: surface_mesh.vertices() ) {
        Eigen::Matrix<double, 1, 3> row; 
        row << surface_mesh.point(vd).x(), surface_mesh.point(vd).y(), surface_mesh.point(vd).z();
        
        vertices.row(row_index) = row;
        ++row_index;
    }

    return vertices;
}

Eigen::Matrix<int, Eigen::Dynamic, 3> MeshData::get_surface_mesh_faces( void ) {
    std::size_t num_f = surface_mesh.number_of_faces();
    std::size_t row_index = 0;

    Eigen::Matrix<int, Eigen::Dynamic, 3> faces(num_f, 3);

    for ( Face_index fd: surface_mesh.faces() ) {
        std::cout << (std::size_t)fd;
        std::cout <<  " ";
        std::size_t col_index = 0;
        for (Vertex_index vd: vertices_around_face(surface_mesh.halfedge(fd), surface_mesh)) {
            faces(row_index, col_index) = (int)vd;
            std::cout << (int)vd; 
            ++col_index;
        }
        std::cout << std::endl;
        ++row_index;
    }

    return faces;
}
// Template Specialization
