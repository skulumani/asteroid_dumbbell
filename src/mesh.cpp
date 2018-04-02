#include "mesh.hpp"
#include "polyhedron.hpp"

// forward declare functions
void eigen_to_polyhedron(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, Polyhedron &P);

template<typename VectorType, typename IndexType>
void polyhedron_to_eigen(Polyhedron &P, Eigen::PlainObjectBase<VectorType> &V, Eigen::PlainObjectBase<IndexType> &F);

// Member methods
MeshData::MeshData(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F) {
    this->vertices = V;
    this->faces = F;

    this->build_polyhedron();
    this->build_surface_mesh();
}

void MeshData::build_polyhedron() {
    // only called after initialization
    eigen_to_polyhedron(this->vertices, this->faces, this->polyhedron);
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

// helper functions
void eigen_to_polyhedron(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, Polyhedron &P) {
    Polyhedron_builder<HalfedgeDS> builder(V, F);
    P.delegate(builder);
    CGAL_assertion(P.is_triangle(P.halfedges_begin()));
}

// TODO Add documentation - overload the () operator
template<typename HDS>
void Polyhedron_builder<HDS>::operator() (HDS &hds) {

    typedef typename HDS::Vertex Vertex;
    typedef typename Vertex::Point Point;

    // create the cgal incremental builder
    CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
    // initialize with #v, #f, #half-edges (optional)
    B.begin_surface(V.rows(), F.rows());

    // add all of the vertices
    for (int ii = 0; ii < V.rows(); ++ii) {
        B.add_vertex(Point(V(ii, 0), V(ii, 1), V(ii, 2)));
    }
    // add all of the faces
    for (int ii = 0; ii < F.rows(); ++ii) {
        B.begin_facet();
        B.add_vertex_to_facet(F(ii, 0));
        B.add_vertex_to_facet(F(ii, 1));
        B.add_vertex_to_facet(F(ii, 2));
        B.end_facet();
    }
    B.end_surface();
}


// TODO Add documentation
template<typename VectorType, typename IndexType>
void polyhedron_to_eigen(Polyhedron &P, Eigen::PlainObjectBase<VectorType> &V, Eigen::PlainObjectBase<IndexType> &F) {
    // loop over all the faces first
    
    // create some eigen arrays to store all the vertices
    const unsigned int num_v = P.size_of_vertices();
    const unsigned int num_f = P.size_of_facets();
    
    V.resize(num_v, 3);
    F.resize(num_f, 3);

    // loop and fill the eigen array
    build_polyhedron_index(P);

    std::size_t row, col;
    row = 0;
    col = 0;
    /* loop_over_facets(P); */

    // Build V
    for (Vertex_iterator vert = P.vertices_begin(); vert != P.vertices_end(); ++vert) {
        V(row, 0)  = vert->point().x();
        V(row, 1)  = vert->point().y();
        V(row, 2)  = vert->point().z();
        row += 1;
    }

    // Build F
    row = 0;
    for ( Facet_iterator face = P.facets_begin(); face != P.facets_end(); ++face) {
        Halfedge_facet_circulator vert = face->facet_begin();
        col = 0;
        do {
            F(row, col) = vert->vertex()->id();
            col += 1;
        } while( ++vert != face->facet_begin());
        row += 1;
    }
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

// Explicit initialization of the template
template void Polyhedron_builder<CGAL::HalfedgeDS_default<CGAL::Simple_cartesian<double>, CGAL::I_Polyhedron_derived_items_3<CGAL::Polyhedron_items_with_id_3>, std::allocator<int> > >::operator()(CGAL::HalfedgeDS_default<CGAL::Simple_cartesian<double>, CGAL::I_Polyhedron_derived_items_3<CGAL::Polyhedron_items_with_id_3>, std::allocator<int> >&);

template void polyhedron_to_eigen<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >
(CGAL::Polyhedron_3<CGAL::Simple_cartesian<double>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> >&, 
 Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&, 
 Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&);

/* template void eigen_to_polyhedron(Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, Eigen::Matrix<int, -1, -1, 0, -1, -1> const&, CGAL::Polyhedron_3<CGAL::Simple_cartesian<double>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> >&) */
