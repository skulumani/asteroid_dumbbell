## Castalia simulations

20180110_raycasting_castalia.npz - one full period of castlia
20180110_raycasting_castalia_reconstruct.hdf5 - uses face, edge, vertex contributions

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
