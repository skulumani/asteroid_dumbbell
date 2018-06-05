#include "mesh.hpp"
#include "polyhedron.hpp"

#include <igl/copyleft/cgal/mesh_to_polyhedron.h>
#include <igl/copyleft/cgal/polyhedron_to_mesh.h>

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
        this->face_descriptor.push_back(face_indices);
    }

}

void MeshData::update_mesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F) {
    // update the polyhedron and surface mesh
    this->vertices = V;
    this->faces = F;
    
    // clear the mesh
    this->surface_mesh.clear();
    this->polyhedron.clear();

    this->build_polyhedron();
    this->build_surface_mesh();
}

template<typename VectorType, typename IndexType>
void surface_mesh_to_eigen(MeshData mesh) {
    // TODO add inputs for eigen v and f

    const unsigned int num_v = mesh.surface_mesh.number_of_vertices();
    const unsigned int num_f = mesh.surface_mesh.number_of_faces();

    /* V.resize(num_v, 3); */
    /* F.resize(num_f, 3); */
    
    // vertex iterators over the mesh
    Mesh::Vertex_range::iterator vb, ve;
    Mesh::Vertex_range r = mesh.vertices;

    vb = r.begin();
    ve = r.end();
    
    // iterate over vertices and print to stdout
    for (Vertex_index vd : mesh.vertices()) {
        std::cout << vd << std::endl; 
    }
}

