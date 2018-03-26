// Surface mesh from implicit function
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/IO/output_surface_facets_to_polyhedron.h>
#include <CGAL/Complex_2_in_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>

#include <Eigen/Dense>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>

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

typedef FT (*Function)(Point_3);

typedef CGAL::Implicit_surface_3<GT, Function> Surface_3;

FT sphere_function (Point_3 p) {
    const FT x2 = p.x()*p.x(), y2=p.y()*p.y(), z2=p.z()*p.z();
    return x2+y2+z2-1;
}

int main() {
    Tr tr; // 3D delaunay triangulation
    C2t3 c2t3 (tr); // 2D-complex in 3D-delaunay triangulation

    // define a surface
    Surface_3 surface(sphere_function, // pointer to function
                      Sphere_3(CGAL::ORIGIN, 2)); // bounding sphere
    // Make sure you input a squared radius of the bound sphere
    // define the meshing criteria
    CGAL::Surface_mesh_default_criteria_3<Tr> criteria(30, // angular bound
                                                       0.1, // raidus bound
                                                       0.1); // distance bound

    // meshing surface
    CGAL::make_surface_mesh(c2t3, surface, criteria, CGAL::Manifold_tag());

    std::cout << "Final number of points: " << tr.number_of_vertices() << std::endl;

    for (Tr::Finite_vertices_iterator vit = tr.finite_vertices_begin(); vit != tr.finite_vertices_end(); ++vit) {
        std::cout << vit->point() << std::endl;
    }
    Polyhedron poly;    
    CGAL::output_surface_facets_to_polyhedron(c2t3, poly); 
    return 0;
}
