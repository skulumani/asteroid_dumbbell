## Castalia simulations

### Point Cloud
All of these use the point cloud data contained inside

* 20180110_raycasting_castalia.npz - one full period of castlia that holds all
of the point cloud data

### Reconstruction

* 20180110_raycasting_castalia_reconstruct.hdf5
    * uses face, edge, vertex contributions by finding the closest primitive 
    and then adding it
    * The process crashed because it took forever
    * The mesh was kind of crazy
* 20180110_raycasting_castalia_reconstruct_vertexonly_smaller.hdf5
    * Only modifies the closest vertex to a given point measurement
    * The initial ellipse is smaller than the asteroid
* 20180215_castalia_highres_ellipse_reconstruct_vertexonly_smaller.hdf5
    * Uses the reconstruction by only modifying the closest vertex
    * The ellipse is a higher resolution
    * Some of the points are skipped to save on time
* 20180110_raycasting_castalia_reconstruct_vertexonly.hdf5
    * Uses only the vertices and modifies the closest vertex to a point
    * The size of the initial asteroid mesh is the semimajor axes of castalia

### Castalia testing using the radius with edge/face modifications

These simulations are using the point cloud data from

20180110_raycasting_castalia.npz

There are four related simulations with slightly different settings:

1. `20180226_castalia_reconstruct_highres_45deg_cone.hdf5`
2. `20180226_castalia_reconstruct_highres_5deg_cone.hdf5`
3. `20180226_castalia_reconstruct_lowres_45deg_cone.hdf5`
4. `20180226_castalia_reconstruct_lowres_5deg_cone.hdf5`

## Itokawa simulations

20180131_itokawa_raycasting.npz - around itokawa

## Instructions to reproduce

1. Run raycasting_sim.simulate to generate a numpy npz data file with the point cloud data
2. Run raycasting_sim.incremental_reconstruction to transform point cloud into a mesh incrmentally. Data is output to a HDF5 file
3. Run raycasting_sim.read_mesh_reconstruct to output a series of images from the reconstruction
4. Create a video using ffmpeg
~~~
ffmpeg -framerate 60 -i test%06d.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" output.mp4 
~~~
