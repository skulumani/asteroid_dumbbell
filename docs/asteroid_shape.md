## Polyhedron Shape model

There are a variety of resources which hold shape models for various asteroids.

* [Small Body Mapping Tool](http://sbmt.jhuapl.edu/index.html) - interactive tool to view asteroid imagery.
This one is also very useful as it will export the asteroids to wavefront `obj` files

All of the shape data is exported from the SBMT and written to `OBJ` formatted files.
The `point_cloud.wavefront` module allows for reading and writing this data to numpy arrays.

All of the shape data is stored under `data/shape_model`
