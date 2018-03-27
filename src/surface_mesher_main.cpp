// Surface mesh from implicit function
#include "input_parser.hpp"

#include "surface_mesher.hpp"


// these need to be used in the ellispoid function
double a, b, c;

FT ellipsoid_function (Point_3 p) {
    const FT x2 = (p.x()*p.x()) / (a * a);
    const FT y2 = (p.y()*p.y()) / (b * b);
    const FT z2 = (p.z()*p.z()) / (c * c);
    return x2 + y2 + z2 - 1;
}

template<typename PolyType>
int ellipsoid_surface_mesher(const double& a_in, const double& b_in, const double& c_in,
        const double& min_angle, const double& max_radius, const double& max_distance,
        PolyType& poly) {
    a = a_in;
    b = b_in;
    c = c_in;

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
    
    // meshing surface
    CGAL::make_surface_mesh(c2t3, surface, criteria, CGAL::Manifold_tag());

    CGAL::output_surface_facets_to_polyhedron(c2t3, poly); 

    std::cout << "Final number of points: " << tr.number_of_vertices() << std::endl;

    return 0;
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
    
    double min_angle, max_radius, max_distance;
    min_angle = atof(argv[4]);
    max_radius = atof(argv[5]);
    max_distance = atof(argv[6]);
    Polyhedron poly;    
    
    ellipsoid_surface_mesher(atof(argv[1]), atof(argv[2]), atof(argv[3]),
            min_angle, max_radius, max_distance, poly);

    std::cout << "Polyhedron vertices: " << poly.size_of_vertices() << std::endl;
     
    // convert to Eigen matrices
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    polyhedron_to_eigen(poly, V, F);
    
    std::cout << V << std::endl;
    return 0;
}
