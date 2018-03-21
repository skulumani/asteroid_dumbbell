#ifndef CGAL_H
#define CGAL_H

#include "cgal_types.hpp"
#include "mesh.hpp"

#include <memory>

void distance_to_polyhedron(Eigen::Vector3d& pt, std::shared_ptr<MeshData> mesh);

// Use the dD spatial searching package for finding nearest vertices/primitives
class MeshDistance {
    public:
        MeshDistance(std::shared_ptr<MeshData> mesh_in);
        
        // funciton to compute distance from pt to mesh and return minimum distance, and primitive
    private:
        std::shared_ptr<MeshData> mesh;
};

class RayCaster {
    public:
        RayCaster(std::shared_ptr<MeshData> mesh_in);

        // cast ray function
        int castray(const Eigen::Ref<const Eigen::Vector3d>& psource, const Eigen::Ref<const Eigen::Vector3d>& ptarget, Eigen::Ref<Eigen::Vector3d> intersection);

        // cast many rays function
        void update_mesh(std::shared_ptr<MeshData> mesh_in);
    private:
        // needs the mesh to operate on
        std::shared_ptr<MeshData> mesh;
        Tree tree; // holds the AABB tree for CGAL distance computations
};

#endif
