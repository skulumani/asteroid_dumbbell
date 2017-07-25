"""Testing out Blender Python API

First you'll need to build Blender using the script in utilities/build_blender.sh

Then you can try and API instructions at https://docs.blender.org/api/2.78b/info_quickstart.html

Hopefully this works
"""
import os
import bpy
import mathutils
import numpy as np

import pdb

for object in bpy.data.objects:
    print(object.name + " is at location " + str(object.location))

def look_at(camera, point):
    r"""Ensure camera is pointing at object

    This function will ensure that within a blender scene that the camera is always pointed at the object

    Parameters
    ----------
    camera : blender object
        camera object
    point : blender point object
        the object to look at

    Returns
    -------
    None

    Other Parameters
    ----------------
    None

    Raises
    ------
    None

    See Also
    --------

    Notes
    -----

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

    References
    ----------

    """ 
    
    loc_camera = camera.matrix_world.to_translation()

    direction = point - loc_camera

    rot_quat = direction.to_track_quat('-Z', 'Y')
    camera.rotation_euler = rot_quat.to_euler()

def load_asteroid(asteroid):
    """Load the desired asteroid

    doc
    """

    # import OBJ model
    bpy.ops.import_scene.obj(filepath='data/itokawa_low.obj')

    camera = bpy.data.objects['Camera']
    lamp = bpy.data.objects['Lamp']
    itokawa = bpy.data.objects['itokawa_low']
    layers = [False] * 32
    layers[0] = True

    bpy.types.UnitSettings.system = 'METRIC'
    bpy.types.UnitSettings.scale_length = 1e3

    # delete the cube
    radians = np.arange(0, 2*np.pi, 0.1*np.pi/180)

    itokawa.location.z = 2

    lamp.location.x = 0
    lamp.location.y = 0 
    lamp.location.z = 5
    for ii,rad in enumerate(radians):
        cube.location.x = np.sin(rad)
        itokawa.rotation_euler.z = rad * 180 / np.pi
        camera.location.x = np.sin(rad)
        look_at(camera, itokawa.matrix_world.to_translation())
        scene.render.filepath = os.path.join(output_path, 'cube_' + str(ii)  + '.png')
        bpy.ops.render.render(write_still=True)

    for object in bpy.data.objects:
        print(object.name + " is at location " + str(object.location))

def reset_scene():
    """Reset blender
    """
    bpy.ops.wm.read_factory_settings(use_empty=True)

    for scene in bpy.data.scenes:
        for obj in scene.objects:
            scene.objects.unlink(obj)

    # only worry about data in the startup scene
    for bpy_data_iter in (
            bpy.data.objects,
            bpy.data.meshes,
            bpy.data.lamps,
            bpy.data.cameras,
            ):
        for id_data in bpy_data_iter:
            bpy_data_iter.remove(id_data)

def blender_init(render_engine='BLENDER'):
    """Initialize blender scene and render settings

    """
    # setup the scene
    bpy.ops.wm.read_homefile()
    # start new empty scene
    scene = bpy.context.scene
    
    # delete the cube
    bpy.data.objects['Cube'].select = True
    bpy.ops.object.delete()
    
    # delete default lamp
    bpy.data.objects['Lamp'].select = True
    bpy.ops.object.delete()

    # set scene render options
    scene.render.resolution_percentage = 100
    # scene.render.filepath = 'visualization/blender/'
    scene.render.engine = 'BLENDER_RENDER'
    
    bpy.types.UnitSettings.system = 'METRIC'
    bpy.types.UnitSettings.scale_length = 1e3

    scene.render.resolution_x = 537
    scene.render.resolution_y = 244
    scene.render.resolution_percentage = 100
    if render_engine == 'BLENDER':
        scene.render.engine = 'BLENDER_RENDER'
    else:
        scene.render.engine = 'CYCLES'

    # setup the camera
    camera_obj = bpy.data.objects['Camera']
    camera = bpy.data.cameras['Camera']
    camera.angle_x = np.deg2rad(2.93)
    camera.angle_y = np.deg2rad(2.25)
    camera.lens = 167.35 # focal length in mm

    # setup the lamp 
    lamp = bpy.data.lamps.new('Lamp', 'SUN')
    lamp_obj = bpy.data.objects.new('Lamp', lamp)
    scene.objects.link(lamp_obj)
    
    return (camera_obj, camera, lamp_obj, lamp, scene)

def blender_example():
    """

    Each Blender Unit is 1 km

    """

    bpy.ops.wm.read_homefile() 
    scene = bpy.context.scene

    # delete the cube
    bpy.data.objects['Cube'].select = True
    bpy.ops.object.delete()
    
    # delete default lamp
    bpy.data.objects['Lamp'].select = True
    bpy.ops.object.delete()

    output_path = 'visualization/blender'


    # import OBJ model
    bpy.ops.import_scene.obj(filepath='data/itokawa_low.obj')

    # add a ground plane
    # add_plane = bpy.ops.mesh.primitive_plane_add
    # add_cube = bpy.ops.mesh.primitive_cube_add
    # cube = bpy.data.objects['Cube']
    # bpy.ops.object.camera_add()
    # bpy.ops.object.lamp_add()

    # set scene render options
    scene.render.resolution_x = 537
    scene.render.resolution_y = 244
    scene.render.resolution_percentage = 100
    # scene.render.filepath = 'visualization/blender/'
    scene.render.engine = 'BLENDER_RENDER'
    # scene.render.engine = 'CYCLES'

    bpy.data.worlds['World'].horizon_color = [0, 0, 0]

    camera_obj = bpy.data.objects['Camera']
    camera = bpy.data.cameras['Camera']

    # new sun lamp
    lamp = bpy.data.lamps.new('Lamp', 'SUN')
    lamp_obj = bpy.data.objects.new('Lamp', lamp)
    scene.objects.link(lamp_obj)

    itokawa_obj = bpy.data.objects['itokawa_low']
    itokawa_obj.scale = [1,1,1]
    
    bpy.types.UnitSettings.system = 'METRIC'
    bpy.types.UnitSettings.scale_length = 1e3

    # delete the cube
    time = np.arange(0, 12.132*3600, 1)

    itokawa_obj.location.x = 0
    itokawa_obj.location.y = 0
    itokawa_obj.location.z = 0
    
    # set camera properties to match NEAR MSI
    camera.angle_x = np.deg2rad(2.93)
    camera.angle_y = np.deg2rad(2.25)
    # camera.sensor_height = 
    # camera.sensor_width = 
    camera.lens = 167.35 # focal length in mm

    camera_obj.location.x = 0
    camera_obj.location.y = 4
    camera_obj.location.z = 0
    # camera.rotation_euler.x = -90
    # camera.rotation_euler.y = 0
    # camera.rotation_euler.z = 0

    lamp_obj.location.x = 0
    lamp_obj.location.y = 5
    lamp_obj.location.z = 0

    lamp_obj.rotation_mode = 'QUATERNION'
    lamp_obj.rotation_quaternion = itokawa_obj.location.to_track_quat('Z', 'Y')

    for ii, t in enumerate(time):
        itokawa_obj.rotation_euler.z = 360 / (12.132 * 3600) * t
        camera_obj.location.x = 3 * np.sin(360 / (12.132 * 3600) * 2 * t)
        camera_obj.location.y = 3 * np.cos(360 / (12.132 * 3600) * 2 * t)

        look_at(camera_obj, itokawa_obj.matrix_world.to_translation())
        scene.render.filepath = os.path.join(output_path, 'cube_' + str(ii)  + '.png')
        bpy.ops.render.render(write_still=True)

    for object in bpy.data.objects:
        print(object.name + " is at location " + str(object.location))

    
    print("Itokawa Size: ", end='')
    print(itokawa_obj.dimensions)
    # bpy.ops.import_scene.obj('../data/itokawa_low.obj')
    # bpy.ops.render.render(write_still=True)
