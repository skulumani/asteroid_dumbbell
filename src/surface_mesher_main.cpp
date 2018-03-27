// Surface mesh from implicit function
#include "input_parser.hpp"

#include "surface_mesher.hpp"

template<typename PolyType>
void build_polyhedron_index(PolyType &P) {
    std::size_t ii = 0;
    for (Vertex_iterator vert = P.vertices_begin(); vert != P.vertices_end(); ++vert) {
        vert->id() = ii++; 
    }
    ii = 0; // reset the counter
    for (Facet_iterator facet = P.facets_begin(); facet != P.facets_end(); ++facet) {
        facet->id() = ii++;
    }

}

template<typename PolyType, typename VectorType, typename IndexType>
void polyhedron_to_eigen(PolyType &P, Eigen::PlainObjectBase<VectorType> &V, Eigen::PlainObjectBase<IndexType> &F) {
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
// these need to be used in the ellispoid function
double a, b, c;

FT ellipsoid_function (Point_3 p) {
    const FT x2 = (p.x()*p.x()) / (a * a);
    const FT y2 = (p.y()*p.y()) / (b * b);
    const FT z2 = (p.z()*p.z()) / (c * c);
    return x2 + y2 + z2 - 1;
}

int main(int argc, char* argv[]) {
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Usage surface_mesher a b c min_angle max_radius max_distance\n  Where a, b, c are the semi-major axes of the ellipsoid" << std::endl;
        return 0;
    }
    
    if (argc != 7) {
        std::cout << "Insufficient number of inputs: surface_mesher a b c" << std::endl;
        return 1;
    }
    // initialize axes
    a = atof(argv[1]);
    b = atof(argv[2]);
    c = atof(argv[3]);
    
    double min_angle, max_radius, max_distance;
    min_angle = atof(argv[4]);
    max_radius = atof(argv[5]);
    max_distance = atof(argv[6]);

    Tr tr; // 3D delaunay triangulation
    C2t3 c2t3 (tr); // 2D-complex in 3D-delaunay triangulation

    // define a surface
    Function surf_func;
    surf_func = &ellipsoid_function;
    Surface_3 surface(surf_func, // pointer to function
                      Sphere_3(CGAL::ORIGIN, pow(std::max({a, b, c}) + 0.01, 2.0) )); // bounding sphere
    // Make sure you input a squared radius of the bound sphere
    // define the meshing criteria
    CGAL::Surface_mesh_default_criteria_3<Tr> criteria(min_angle, // angular bound
                                                       max_radius, // raidus bound
                                                       max_distance); // distance bound
    
    std::cout << std::max({a, b, c}) << std::endl;
    // meshing surface
    CGAL::make_surface_mesh(c2t3, surface, criteria, CGAL::Manifold_tag());

    Polyhedron poly;    
    CGAL::output_surface_facets_to_polyhedron(c2t3, poly); 

    std::cout << "Final number of points: " << tr.number_of_vertices() << std::endl;
    std::cout << "Polyhedron vertices: " << poly.size_of_vertices() << std::endl;
    
    // convert to Eigen matrices
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    polyhedron_to_eigen(poly, V, F);
    
    std::cout << V << std::endl;
    return 0;
}
