#include "potential.hpp"
#include "loader.hpp"
#include "mesh.hpp"
#include <omp.h>

#include <iostream>

int main() {
    
    std::shared_ptr<MeshData> mesh_data = Loader::load("./data/shape_model/ITOKAWA/itokawa_low.obj");
    
    // check if mesh is valid
    mesh_data->surface_mesh.is_valid();
    CGAL::is_triangle_mesh(mesh_data->surface_mesh);
    /* double start = omp_get_wtime(); */
    /* std::shared_ptr<MeshParam> mesh_param = std::make_shared<MeshParam>(mesh_data); */
    /* double end = omp_get_wtime() - start; */
    /* std::cout << "MeshParam: " << end << " sec" << std::endl; */

    /* Asteroid ast("castalia", mesh_param); */
    /* Eigen::Vector3d state; */ 
    /* state << 1, 2, 3; */
    
    /* /1* start = omp_get_wtime(); *1/ */
    /* ast.polyhedron_potential(state); */
    /* /1* end = omp_get_wtime() - start; *1/ */
    /* /1* std::cout << "PolyPotential: " << end << " sec " << std::endl; *1/ */

    /* std::cout << "U : " << ast.get_potential() << std::endl; */ 
    /* std::cout << "U_grad : " << ast.get_acceleration().transpose() << std::endl; */
    /* std::cout << "U_grad_mat : \n" << ast.get_gradient_mat() << std::endl; */
    /* std::cout << "U_laplace : " << ast.get_laplace() << std::endl; */
    return 0;
}
