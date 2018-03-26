// Surface mesh from implicit function
#include "input_parser.hpp"

#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/IO/output_surface_facets_to_polyhedron.h>
#include <CGAL/Complex_2_in_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>

#include <Eigen/Dense>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>

#include <stdlib.h>
#include <cmath>

typedef CGAL::Simple_cartesian<double> Kernel;
// default triangulation for surface_mesher
typedef CGAL::Surface_mesh_default_triangulation_3 Tr;

// c2t3
typedef CGAL::Complex_2_in_triangulation_3<Tr> C2t3;

typedef Tr::Geom_traits GT;
typedef GT::Sphere_3 Sphere_3;
typedef GT::Point_3 Point_3;
typedef GT::FT FT;
typedef CGAL::Polyhedron_3<GT> Polyhedron;

typedef FT (*Function)(Point_3); // Function is a pointer with takes Point_3 as input and returns type FT

typedef CGAL::Implicit_surface_3<GT, Function> Surface_3;

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
    Surface_3 surface(ellipsoid_function, // pointer to function
                      Sphere_3(CGAL::ORIGIN, pow(std::max({a, b, c}), 2.0) )); // bounding sphere
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

    return 0;
}
