#include "reconstruct.hpp"
#include "mesh.hpp"
#include "geodesic.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

// Forward declaration
Eigen::Matrix<double, Eigen::Dynamic, 1> initial_weight(const Eigen::Ref<const Eigen::MatrixXd> &v_in);

// Constructor
ReconstructMesh::ReconstructMesh( const Eigen::Ref<const Eigen::MatrixXd> &v_in,
                                  const Eigen::Ref<const Eigen::MatrixXi> &f_in,
                                  const Eigen::Ref<const Eigen::VectorXd> &w_in) {
    // need to initialize the meshdata shared_ptr
    this->mesh = std::make_shared<MeshData>(v_in, f_in);
    
    set_all_weights(w_in);
}

// build object with only v and f
ReconstructMesh::ReconstructMesh( const Eigen::Ref<const Eigen::MatrixXd> &v_in,
                                 const Eigen::Ref<const Eigen::MatrixXi> &f_in) {
    // need to initialize the meshdata shared_ptr
    this->mesh = std::make_shared<MeshData>(v_in, f_in);
    initialize_weight();
}

ReconstructMesh::ReconstructMesh(std::shared_ptr<MeshData> mesh_in) {
    // save another ptr to object
    this->mesh = mesh_in;
    initialize_weight();
}

ReconstructMesh::ReconstructMesh(std::shared_ptr<MeshData> mesh_in,
        const Eigen::Ref<const Eigen::VectorXd> &w_in) {
    this->mesh = mesh_in;
    set_all_weights(w_in);
}

// Member functions
Eigen::MatrixXd ReconstructMesh::get_verts( void ) const {
    return this->mesh->get_verts();
}

Eigen::Vector3d ReconstructMesh::get_vertex( const Vertex_index& vd) const {
    return mesh->get_vertex(vd);
}

Eigen::MatrixXi ReconstructMesh::get_faces( void ) const {
    return this->mesh->get_faces();
}

Eigen::VectorXd ReconstructMesh::get_weights( void ) const {
    Eigen::VectorXd weights(mesh->number_of_vertices());
    Mesh::Property_map<Vertex_index, double> weight_property;
    bool found;
    std::tie(weight_property, found) 
        = mesh->surface_mesh.property_map<Vertex_index, double>(
                "v:weight");
    assert(found);
    std::size_t row = 0;
    // read the weight property and build a matrix
    for (Vertex_index vd : mesh->vertices()) {
        weights(row) = weight_property[vd]; 
        ++row;
    }
    return weights;
}

Eigen::VectorXd ReconstructMesh::get_vertex_weights( const std::vector<Vertex_index>& vertices) const {
    Eigen::VectorXd weights(vertices.size());
    std::size_t row = 0;
    for (Vertex_index vd : vertices) {
        weights(row) = get_weight(vd);
        ++row;
    }
    return weights;
}
void ReconstructMesh::single_update(const Eigen::Ref<const Eigen::RowVector3d> &pt,
                                    const double &max_angle) {
     
    Eigen::Vector3d pt_uvec = pt.normalized();
    double pt_radius = pt.norm();
    // loop over all vertices
    for (Vertex_index vd : mesh->vertices()){
        // get a radius vector for this vertex
        Eigen::Vector3d vert_uvec = mesh->get_vertex(vd).normalized();
        // compute the central angle between vertex and teh measurement
        double delta_sigma = single_central_angle(pt_uvec, vert_uvec);
        // check if central angle is less than the max angle
        if ( delta_sigma < max_angle) {
            // if true then compute new radius and new weight
            double meas_weight = pow(delta_sigma * pt_radius, 2);
            // modify teh surface mesh with the new data
            double radius_new = (mesh->get_vertex(vd).norm() * meas_weight 
                    + pt_radius * get_weight(vd)) / ( get_weight(vd) + meas_weight ) ;
            double weight_new = (get_weight(vd) * meas_weight) / ( get_weight(vd) + meas_weight);

            // now update the mesh with new data
            mesh->set_vertex(vd, radius_new * vert_uvec);
            set_weight(vd, weight_new);

        }
    }
}

void ReconstructMesh::update(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& pts,
        const double& max_angle) {
    std::size_t num_pts(pts.rows());
    
    for (std::size_t ii = 0; ii < num_pts; ++ii) {
        single_update(pts.row(ii), max_angle);
    }
}

void ReconstructMesh::update_meshdata( void ) {
    // update the data inside the mesh_ptr
    /* this->mesh->update_mesh(this->vertices, this->faces); */
}
double ReconstructMesh::maximum_weight(const Eigen::Ref<const Eigen::Vector3d>& v_in) {
    return pow(kPI * v_in.norm(), 2);
    /* return 6.0; */
}

bool ReconstructMesh::initialize_weight( void ) {
    // created is true if new and false if exisiting

    for (Vertex_index vd : mesh->surface_mesh.vertices() ) {
        set_weight(vd, maximum_weight(mesh->get_vertex(vd)));
    }

    return true;
}

bool ReconstructMesh::set_all_weights(const Eigen::Ref<const Eigen::VectorXd>& w_in) {
    assert(w_in.rows() == mesh->number_of_vertices());
    for (Vertex_index vd : mesh->surface_mesh.vertices() ) {
        set_weight(vd, w_in((int)vd));
    }

    return true;
}

bool ReconstructMesh::set_weight(const Vertex_index& vd, const double& w) {
    Mesh::Property_map<Vertex_index, double> weight_property;
    bool found;
    std::tie(weight_property, found) 
        = mesh->surface_mesh.add_property_map<Vertex_index, double>(
                "v:weight", 10);
    /* assert(found); */
    // created is true if new and false if exisiting
    weight_property[vd] = w;
    
    return true;
}

double ReconstructMesh::get_weight(const Vertex_index& vd) const {
    Mesh::Property_map<Vertex_index, double> weight_property;
    bool found;
    std::tie(weight_property, found) 
        = mesh->surface_mesh.property_map<Vertex_index, double>(
                "v:weight");
    assert(found);
    
    return weight_property[vd];
}

// compute the initial weighting matrix given the vertices
Eigen::Matrix<double, Eigen::Dynamic, 1> initial_weight(const Eigen::Ref<const Eigen::MatrixXd> &v_in) {
    Eigen::Matrix<double, Eigen::Dynamic, 1> vert_radius(v_in.rows(), 1);
    vert_radius = v_in.rowwise().norm();
    // define the weights
    Eigen::Matrix<double, Eigen::Dynamic, 1> weights(v_in.rows(), 1);

    double max_radius = vert_radius.maxCoeff();
    weights.fill(pow(kPI * max_radius, 2));
    
    return weights;
}

/* template<typename T> */
/* Eigen::VectorXi vector_find(const Eigen::Ref<const T> &logical_vec) { */
/*     Eigen::VectorXi index = Eigen::VectorXi::LinSpaced(logical_vec.size(), 0, logical_vec.size() - 1); */
/*     index.conservativeResize(std::stable_partition( */
/*                 index.data(), index.data() + index.size(), [&logical_vec](int i){return logical_vec(i);}) - index.data()); */
/*     return index; */
/* } */

// TEMPLATE SPECIALIZATION
