#include "input_parser.hpp"
#include "read_obj.hpp"

#include <Eigen/Dense>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>

#include <iostream>
#include <fstream>
#include <vector>

typedef CGAL::Simple_cartesian<double>     Kernel;
typedef CGAL::Polyhedron_3<Kernel>         Polyhedron;
typedef Polyhedron::HalfedgeDS             HalfedgeDS;<Paste>

// declaration for the polyhedron builder
template<typename HDS, typename VectorType, typename IndexType> 
class Polyhedron_builder : public CGAL::Modifier_base<HDS> {
    public:
        Eigen::PlainObjectBase<VectorType> &V;
        Eigen::PlainObjectBase<IndexType> &F;

        Polyhedron_builder(Eigen::PlainObjectBase<VectorType> &V_input, Eigen::PlainObjectBase<IndexType> &F_input) : V(V_input), F(F_input) {};

        void operator() (HDS &hds) {

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
                B.end_facet()
            }
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
