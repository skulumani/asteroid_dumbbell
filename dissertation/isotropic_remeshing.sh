

./bin/remesh -i data/shape_model/CUBE/cube.obj 

# draw and save the original cube
mkdir -p /tmp/isotropic

PYTHONPATH=./ python dissertation/draw_cube_wireframe.py data/shape_model/CUBE/cube.obj
mv /tmp/isotropic.jpg /tmp/isotropic/original_cube.jpg
PYTHONPATH=./ python dissertation/draw_cube_wireframe.py /tmp/cube_remesh.obj
mv /tmp/isotropic.jpg /tmp/isotropic/remesh_cube.jpg

