// Surface mesh from implicit function
#include "input_parser.hpp"
#include "surface_mesher.hpp"

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
