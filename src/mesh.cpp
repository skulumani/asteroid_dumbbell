#include "mesh.hpp"
#include "polyhedron.hpp"

#include <igl/copyleft/cgal/mesh_to_polyhedron.h>
#include <igl/copyleft/cgal/polyhedron_to_mesh.h>

#include <tuple>
#include <assert.h>
#include <cmath>

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

        assert(surface_mesh.is_valid(v));
    }
    

    std::vector<Vertex_index> face_indices;

    for (int ii = 0; ii < F.rows(); ++ii) {
        p1 = Kernel::Point_3(V(F(ii, 0), 0), V(F(ii, 0), 1), V(F(ii, 0), 2));
        p2 = Kernel::Point_3(V(F(ii, 1), 0), V(F(ii, 1), 1), V(F(ii, 2), 2));
        p3 = Kernel::Point_3(V(F(ii, 2), 0), V(F(ii, 2), 1), V(F(ii, 2), 2));

        v1 = this->vertex_descriptor[F(ii, 0)];
        v2 = this->vertex_descriptor[F(ii, 1)];
        v3 = this->vertex_descriptor[F(ii, 2)];

        Face_index f = this->surface_mesh.add_face(v1, v2, v3);
        assert(surface_mesh.is_valid(f));

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

    assert(surface_mesh.is_valid());

    // face_normal, edge normal, halfedge normal, face dyad, edge dyad
    build_face_properties();
    build_halfedge_properties();
    build_edge_properties();
}

bool MeshData::build_face_properties( void ) {
    // Build property maps for the surface mesh
    Mesh::Property_map<Face_index, Eigen::Vector3d> face_unit_normal;
    bool created;
    std::tie( face_unit_normal, created ) 
        = surface_mesh.add_property_map<Face_index, Eigen::Vector3d>(
                "f:face_unit_normal", (Eigen::Vector3d() << 0, 0, 0).finished());
    assert(created);
    
    // Center face property map
    Mesh::Property_map<Face_index, Eigen::Vector3d> face_center;
    /* bool created; */
    std::tie(face_center, created) 
        = surface_mesh.add_property_map<Face_index, Eigen::Vector3d>(
                "f:face_center", (Eigen::Vector3d() << 0 ,0 ,0).finished());
    assert(created);
    
    // Face dyad
    Mesh::Property_map<Face_index, Eigen::Matrix3d> face_dyad;
    std::tie(face_dyad, created)
        = surface_mesh.add_property_map<Face_index, Eigen::Matrix3d>(
                "f:face_dyad", Eigen::Matrix3d::Zero());
    assert(created);

    // loop over all faces need to dereference the iterators but not the index
    for (Face_index fd: surface_mesh.faces() ){
        // need to consecutive vertices in face to get teh normal
        Halfedge_index h1, h2, h3;
        h1 = surface_mesh.halfedge(fd);
        h2 = surface_mesh.next(h1);
        h3 = surface_mesh.next(h2);
        assert(surface_mesh.next(h3) == h1);

        Vertex_index v1, v2, v3;
        v1 = surface_mesh.source(h1);
        v2 = surface_mesh.source(h2);
        v3 = surface_mesh.source(h3);
        assert(surface_mesh.target(h1) == v2);
        assert(surface_mesh.target(h2) == v3);
        assert(surface_mesh.target(h3) == v1);

        // now extract the point into Eigen arrays
        Eigen::Vector3d vec1, vec2, vec3;
        vec1 = get_vertex(v1);
        vec2 = get_vertex(v2);
        vec3 = get_vertex(v3);

        Eigen::Vector3d edge1, edge2;
        edge1 = vec2 - vec1;
        edge2 = vec3 - vec1;

        face_unit_normal[fd] = edge1.cross(edge2).normalized();
        face_center[fd] = 1.0 / 3.0 * (vec1 + vec2 + vec3);
        face_dyad[fd] = face_unit_normal[fd] * face_unit_normal[fd].transpose();

        assert(face_dyad[fd].isApprox(face_dyad[fd].transpose(), 1e-3));
    }
    return true;
}

bool MeshData::build_halfedge_properties( void ) {
    Mesh::Property_map<Halfedge_index, Eigen::Vector3d> halfedge_unit_normal;
    bool created;
    std::tie(halfedge_unit_normal, created) 
        = surface_mesh.add_property_map<Halfedge_index, Eigen::Vector3d>(
                "h:halfedge_unit_normal", (Eigen::Vector3d() << 0, 0, 0).finished());
    
    assert(created);
    
    Mesh::Property_map<Face_index, Eigen::Vector3d> face_unit_normal;
    bool found;
    std::tie(face_unit_normal, found) 
        = surface_mesh.property_map<Face_index, Eigen::Vector3d>(
                "f:face_unit_normal");

    for (Halfedge_index hd: surface_mesh.halfedges()) {
        Vertex_index vs, ve;
        vs = surface_mesh.source(hd);
        ve = surface_mesh.target(hd);

        Eigen::Vector3d vec_start, vec_end, vec_edge, face_normal;
        vec_start = get_vertex(vs);
        vec_end = get_vertex(ve);
        vec_edge = vec_end - vec_start;

        Face_index fd;
        fd = surface_mesh.face(hd);
        face_normal = face_unit_normal[fd];

        halfedge_unit_normal[hd] = vec_edge.cross(face_normal).normalized();
    }
    return true;
}

bool MeshData::build_edge_properties( void ){
    // edge dyad
    Mesh::Property_map<Edge_index, Eigen::Matrix3d> edge_dyad;
    bool created;
    std::tie(edge_dyad, created)
        = surface_mesh.add_property_map<Edge_index, Eigen::Matrix3d>(
                "e:edge_dyad", Eigen::Matrix3d::Zero());
    assert(created);
    
    // normal face property map
    Mesh::Property_map<Face_index, Eigen::Vector3d> face_unit_normal;
    bool found;
    std::tie(face_unit_normal, found) 
        = surface_mesh.property_map<Face_index, Eigen::Vector3d>(
                "f:face_unit_normal");
    assert(found);

    // halfedge normal property map
    Mesh::Property_map<Halfedge_index, Eigen::Vector3d> halfedge_unit_normal;
    std::tie(halfedge_unit_normal, found)
        = surface_mesh.property_map<Halfedge_index, Eigen::Vector3d>(
                "h:halfedge_unit_normal");
    
    for (Edge_index ed: surface_mesh.edges()) {

        Halfedge_index h1, h2;
        h1 = surface_mesh.halfedge(ed, 0);
        h2 = surface_mesh.halfedge(ed, 1);
        
        Face_index f1, f2;
        f1 = surface_mesh.face(h1);
        f2 = surface_mesh.face(h2);

        edge_dyad[ed] = face_unit_normal[f1] * halfedge_unit_normal[h2].transpose() 
            + face_unit_normal[f2] * halfedge_unit_normal[h1].transpose();
        // doesn't work for matrices close to zero
        /* assert((edge_dyad[ed] - edge_dyad[ed].transpose()).isApprox(Eigen::Matrix3d::Zero(), 1e-3)); */
    }
    return true;
}

bool MeshData::build_edge_factor( const Eigen::Ref<const Eigen::Vector3d>& pos ) {
    // create a property map
    Mesh::Property_map<Edge_index, double> edge_factor;
    bool created;
    std::tie(edge_factor, created)
        = surface_mesh.add_property_map<Edge_index, double>(
                "e:edge_factor", 0);
    assert(created);

    // loop over edges
    for (Edge_index ed : surface_mesh.edges()) {

        // get vertex of each edge endpoitn
        Vertex_index v1, v2;
        v1 = surface_mesh.vertex(ed, 0);
        v2 = surface_mesh.vertex(ed, 1);

        Eigen::Vector3d vec1, vec2;
        vec1 = get_vertex(v1);
        vec2 = get_vertex(v2);
        // subtract from state adn find norm
        double r1, r2;
        r1 = (vec1 - pos).norm();
        r2 = (vec2 - pos).norm();
        // find length of edge
        double e = (vec1 - vec2).norm();
        // take natural log
        edge_factor[ed] = std::log((r1 + r2 + e)/(r1 + r2 - e));
    }
    return true;
}

bool MeshData::build_face_factor(const Eigen::Ref<const Eigen::Vector3d>& pos) {
    // face factor w property map
    Mesh::Property_map<Face_index, double> face_factor;
    bool created;
    std::tie(face_factor, created) = surface_mesh.add_property_map<Face_index, double>(
            "f:face_factor", 0);
    assert(created);
    
    for (Face_index fd: surface_mesh.faces()) {
        Halfedge_index h1, h2, h3;
        h1 = surface_mesh.halfedge(fd);
        h2 = surface_mesh.next(h1);
        h3 = surface_mesh.next(h2);
        assert(surface_mesh.next(h3) == h1);

        Vertex_index v1, v2, v3;
        v1 = surface_mesh.source(h1);
        v2 = surface_mesh.source(h2);
        v3 = surface_mesh.source(h3);
        assert(surface_mesh.target(h1) == v2);
        assert(surface_mesh.target(h2) == v3);
        assert(surface_mesh.target(h3) == v1);

        // now extract the point into Eigen arrays
        Eigen::Vector3d vec1, vec2, vec3, r1, r2, r3;
        vec1 = get_vertex(v1);
        vec2 = get_vertex(v2);
        vec3 = get_vertex(v3);
        r1 = vec1 - pos;
        r2 = vec2 - pos;
        r3 = vec3 - pos;

        double num, den;
        num = r1.dot(r2.cross(r3));
        den = r1.norm() * r2.norm() * r3.norm() 
            + r1.norm() * r2.dot(r3) 
            + r2.norm() * r3.dot(r1)
            + r3.norm() * r1.dot(r2);

        face_factor[fd] = 2.0 * atan2(num, den);
    }

    return true;
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

// MeshData Getters
Eigen::Matrix<double, Eigen::Dynamic, 3> MeshData::get_verts( void ) const {
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

Eigen::Matrix<int, Eigen::Dynamic, 3> MeshData::get_faces( void ) const {
    std::size_t num_f = surface_mesh.number_of_faces();
    std::size_t row_index = 0;

    Eigen::Matrix<int, Eigen::Dynamic, 3> faces(num_f, 3);

    for ( Face_index fd: surface_mesh.faces() ) {
        std::size_t col_index = 0;
        for (Vertex_index vd: vertices_around_face(surface_mesh.halfedge(fd), surface_mesh)) {
            faces(row_index, col_index) = (int)vd;
            ++col_index;
        }
        ++row_index;
    }

    return faces;
}

template<typename Index>
Eigen::Vector3d MeshData::get_face_normal(const Index& fd_in) {
    Mesh::Property_map<Face_index, Eigen::Vector3d> face_unit_normal;
    bool found;
    std::tie(face_unit_normal, found) = surface_mesh.property_map<
        Face_index,  Eigen::Vector3d>("f:face_unit_normal");
    assert(found);
    Face_index fd(fd_in);
    return face_unit_normal[fd];
}

template<typename Index>
Eigen::Vector3d MeshData::get_face_center(const Index& fd_in) {
    Mesh::Property_map<Face_index, Eigen::Vector3d> face_center;
    bool found;
    std::tie(face_center, found) = surface_mesh.property_map<
        Face_index, Eigen::Vector3d>("f:face_center");
    assert(found);
    Face_index fd(fd_in);
    return face_center[fd];
}

template<typename Index>
Eigen::Matrix3d MeshData::get_face_dyad(const Index& fd_in) {
    Mesh::Property_map<Face_index, Eigen::Matrix3d> face_dyad;
    bool found;
    std::tie(face_dyad, found) = surface_mesh.property_map<
        Face_index, Eigen::Matrix3d>("f:face_dyad");
    assert(found);
    Face_index fd(fd_in);
    return face_dyad[fd];
}

template<typename Index>
Eigen::Vector3d MeshData::get_halfedge_normal(const Index& hd_in) {
    Mesh::Property_map<Halfedge_index, Eigen::Vector3d> halfedge_unit_normal;
    bool found;
    std::tie(halfedge_unit_normal, found) = surface_mesh.property_map<
        Halfedge_index, Eigen::Vector3d>("h:halfedge_unit_normal");
    Halfedge_index hd(hd_in);
    return halfedge_unit_normal[hd];
}

template<typename Index>
Eigen::Matrix3d MeshData::get_edge_dyad(const Index& ed_in) {
    Mesh::Property_map<Edge_index, Eigen::Matrix3d> edge_dyad;
    bool found;
    std::tie(edge_dyad, found) = surface_mesh.property_map<
        Edge_index, Eigen::Matrix3d>("e:edge_dyad");
    Edge_index ed(ed_in);
    return edge_dyad[ed];
}

template<typename Index>
double MeshData::get_edge_factor(const Index& ed_in) {
    Mesh::Property_map<Edge_index, double> edge_factor;
    bool found;
    std::tie(edge_factor, found) = surface_mesh.property_map<
        Edge_index, double>("e:edge_factor");
    Edge_index ed(ed_in);
    return edge_factor[ed];
}

template<typename Index>
double MeshData::get_face_factor(const Index& fd_in) {
    Mesh::Property_map<Face_index, double> face_factor;
    bool found;
    std::tie(face_factor, found) = surface_mesh.property_map<
        Face_index, double>("f:face_factor");
    Face_index fd(fd_in);
    return face_factor[fd];
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
        std::size_t col_index = 0;
        for (Vertex_index vd: vertices_around_face(surface_mesh.halfedge(fd), surface_mesh)) {
            faces(row_index, col_index) = (int)vd;
            ++col_index;
        }
        ++row_index;
    }

    return faces;
}

template<typename Index>
Eigen::RowVector3d MeshData::get_vertex(const Index& index) {
    // form Vertex_index
    Vertex_index vd(index);
    
    // create an array and return
    Eigen::RowVector3d vertex;
    vertex(0) = surface_mesh.point(vd).x();
    vertex(1) = surface_mesh.point(vd).y();
    vertex(2) = surface_mesh.point(vd).z();

    return vertex;
}

template<typename Index>
Eigen::RowVector3i MeshData::get_face_vertices(const Index& index) {
    Face_index fd(index);

    Eigen::RowVector3i face_vertices;
    std::size_t col_index = 0;
    for (Vertex_index vd: vertices_around_face(surface_mesh.halfedge(fd), surface_mesh)) {
        face_vertices(col_index) = (int)vd;
        ++col_index;
    }
    return face_vertices;
}

// Template Specialization
template Eigen::RowVector3d MeshData::get_vertex<std::size_t>(const std::size_t&);
template Eigen::RowVector3d MeshData::get_vertex<int>(const int&);
template Eigen::RowVector3d MeshData::get_vertex<Vertex_index>(const Vertex_index&);

template Eigen::RowVector3i MeshData::get_face_vertices<std::size_t>(const std::size_t&);
template Eigen::RowVector3i MeshData::get_face_vertices<int>(const int&);
template Eigen::RowVector3i MeshData::get_face_vertices<Face_index>(const Face_index&);

template Eigen::Vector3d MeshData::get_face_normal<Face_index>(const Face_index&);
template Eigen::Vector3d MeshData::get_face_center<Face_index>(const Face_index&);
template Eigen::Matrix3d MeshData::get_face_dyad<Face_index>(const Face_index&);

template Eigen::Vector3d MeshData::get_halfedge_normal<Halfedge_index>(const Halfedge_index&);

template Eigen::Matrix3d MeshData::get_edge_dyad<Edge_index>(const Edge_index&);

template double MeshData::get_edge_factor<Edge_index>(const Edge_index&);
template double MeshData::get_face_factor<Face_index>(const Face_index&);
