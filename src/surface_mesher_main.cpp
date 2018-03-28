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

	// Create surface mesh object
	SurfMesh smesh(atof(argv[1]), atof(argv[2]), atof(argv[3]),
            min_angle, max_radius, max_distance);

	std::cout << "Smesh poly vertices: " << smesh.poly.size_of_vertices() << std::endl;
	std::cout << "Smesh v vertices: " << smesh.v.rows() << std::endl;

	std::cout << smesh.verts() << std::endl;
    return 0;
}
