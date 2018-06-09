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
        std::cout << "Insufficient number of inputs:\n surface_mesher a b c min_angle max_radius max_distance" << std::endl;
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

	std::cout << "Surface vertices: " << smesh.get_verts().rows() << std::endl;
	std::cout << "Surface faces: " << smesh.get_faces().rows() << std::endl;
    return 0;
}
