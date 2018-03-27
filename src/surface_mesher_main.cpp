// Surface mesh from implicit function
#include "input_parser.hpp"
#include "surface_mesher.hpp"

void ellipsoid_mesh(const double& a_in, const double& b_in, const double& c_in,
        const double& max_radius, Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 3> > V, 
        Eigen::Ref<Eigen::Matrix<int, Eigen::Dynamic, 3> > F) {
    
    /* ellipsoid_surface_mesher(a_in, b_in, c_in, 10.0, max_radius, 3 * max_radius, V, F); */
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
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    
    ellipsoid_surface_mesher(atof(argv[1]), atof(argv[2]), atof(argv[3]),
            min_angle, max_radius, max_distance, poly);
    
    ellipsoid_surface_mesher(atof(argv[1]), atof(argv[2]), atof(argv[3]),
            min_angle, max_radius, max_distance, V, F);
    
    Eigen::Matrix<double, Eigen::Dynamic, 3> Va;
    Eigen::Matrix<int, Eigen::Dynamic, 3> Fa;

    /* ellipsoid_mesh(atof(argv[1]), atof(argv[2]), atof(argv[3]), */
    /*         max_radius, Va, Fa); */

    std::cout << "Polyhedron vertices: " << poly.size_of_vertices() << std::endl;
    return 0;
}
