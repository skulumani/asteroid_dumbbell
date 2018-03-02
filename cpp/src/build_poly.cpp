#include "input_parser.hpp"
#include "read_obj.hpp"

#include <Eigen/Dense>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>

#include <iostream>
#include <fstream>
#include <vector>

// definition for the polyhedron builder
template<typename HDS, typename Derived> 
class Polyhedron_builder : public CGAL::Modifier_base<HDS> {
    public:
        Polyhedron_builder(Eigen::PlainObjectBase<Derived> &V, Eigen::PlainObjectBase<Derived> &F);
        void operator() (HDS &hds);
};

template<typename HDS> 
class Build_triangle : public CGAL::Modifier_base<HDS> {
    public:
        Build_triangle () {}

        void operator() (HDS& hds) {
            // postcondition hds is a valid polyhedral surface
            CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
            B.begin_surface(3, 1, 6);
            typedef typename HDS::Vertex Vertex;
            typedef typename Vertex::Point Point;
            B.add_vertex(Point(0, 0, 0));
            B.add_vertex(Point(1, 0, 0));
            B.add_vertex(Point(0, 1, 0));
            B.begin_facet();
            B.add_vertex_to_facet(0);
            B.add_vertex_to_facet(1);
            B.add_vertex_to_facet(2);
            B.end_facet();
            B.end_surface();
        }
};

int main(int argc, char* argv[]) {
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Usage read_obj -i input_file.obj" << std::endl;
    }
    
    // vectors of vectors to store the data
    std::vector<std::vector<double>> vector_V;
    std::vector<std::vector<int>> vector_F;
    int read_flag = 1;

    const std::string input_file = input.get_command_option("-i");
    if (!input_file.empty()) {
        std::cout << "Reading " << input_file << std::endl;
        /* std::ifstream input_stream(input_file); */
        read_flag = obj::read(input_file, vector_V, vector_F);
        if (read_flag == 0) {
            std::cout << "Converting to Eigen arrays" << std::endl;
            Eigen::MatrixXd V;
            Eigen::MatrixXi F;
            vector_array_to_eigen(vector_V, V);
            vector_array_to_eigen(vector_F, F);
        }
         
    }  // input file is closed when leaving the scope

    return 0;
}
