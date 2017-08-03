"""Testing out Blender Python API

First you'll need to build Blender using the script in utilities/build_blender.sh

Then you can try and API instructions at https://docs.blender.org/api/2.78b/info_quickstart.html

Hopefully this works
"""
import os
import bpy
import numpy as np
import mathutils
import pdb
output_path = 'visualization/blender'


def print_object_locations():
    r"""Print out object properties for all the objects in a Blender scene

    This function will output all the positions of the objects in the scene.

    Parameters
    ----------
    None

    Returns
    -------
    None

    Other Parameters
    ----------------
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

    .. [1] Blender Python API - https://docs.blender.org/api/2.78b/

    """
    for object in bpy.data.objects:
        print("{0} is at: {1}".format(object.name, str(object.location)))


def cylinder_between(x1, y1, z1, x2, y2, z2, r):

    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1    
    dist = np.sqrt(dx**2 + dy**2 + dz**2)

    bpy.ops.mesh.primitive_cylinder_add(
        radius = r, 
        depth = dist,
        location = (dx/2 + x1, dy/2 + y1, dz/2 + z1)   
    ) 

    phi = np.arctan2(dy, dx) 
    theta = np.arccos(dz/dist) 

    bpy.context.object.rotation_euler[1] = theta 
    bpy.context.object.rotation_euler[2] = phi 

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


def load_asteroid(asteroid='itokawa_low'):
    """Load the desired asteroid

    doc
    """

    # import OBJ model
    bpy.ops.import_scene.obj(filepath=os.path.join('data',asteroid + '.obj'))

    itokawa_obj = bpy.data.objects[asteroid]
    itokawa_obj.scale = [1, 1, 1] 

    itokawa_obj.location.x = 0
    itokawa_obj.location.y = 0
    itokawa_obj.location.z = 0
    # set the material properties for the asteroid to match something realistic
    
    return itokawa_obj

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

def blender_init(render_engine='BLENDER', resolution=[537,244],
                 fov=[2.93,2.235],focal_length=167.35):
    r"""Initialize the Blender scene for a camera around an asteroid

    This function will initialize the objects for a scene involving a camera, asteroid, and light source

    Parameters
    ----------
    render_engine : string
        Option to choose the type of rendering engine
            'BLENDER' - Default Blender render - fast
            'CYCLES' - Better rendering at the expense of speed/computation
    resolution : list or array_like
        The x,y pixel resolution of the resulting output image.
        The default matches the MSI camera on NEAR
    fov : list or array_like
        Field of View of the camera sensor in degrees - default is chosen to math NEAR 
    focal_length : float
        Focal length of camera in millimeters

    Returns
    -------
    camera : Blender camera
        Camera which lets you set camera properties
    camera_obj : Blender object
        Camera object - move it/rotation
    lamp : Blender Lamp
        Use to set lamp specific properties
    lamp_obj : Blender object
        Use to move and position
    itokawa_obj : Blender object
        Asteroid object
    scene : Blender scene
        Use to set general parameters for rendering

    See Also
    --------
    blender_example : Example for rendering an image 

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

    """
    # setup the scene
    bpy.ops.wm.read_homefile()
    # start new empty scene
    scene = bpy.context.scene
    
    # delete the cube
    bpy.data.objects['Cube'].select = True
    bpy.ops.object.delete()
    
    # empty object for camera tracking purposes
    empty = bpy.data.objects.new('Empty', None)
    bpy.context.scene.objects.link(empty)
    bpy.context.scene.update()
    
    itokawa_obj = load_asteroid(asteroid='itokawa_low')

    # render options
    bpy.data.worlds['World'].horizon_color = [0, 0, 0]
    bpy.types.UnitSettings.system = 'METRIC'
    bpy.types.UnitSettings.scale_length = 1e3

    scene.render.resolution_x = resolution[0]
    scene.render.resolution_y = resolution[1]
    scene.render.resolution_percentage = 100
    if render_engine == 'BLENDER':
        scene.render.engine = 'BLENDER_RENDER'
    else:
        scene.render.engine = 'CYCLES'

    # setup the camera
    camera_obj = bpy.data.objects['Camera']
    camera = bpy.data.cameras['Camera']
    camera.angle_x = np.deg2rad(fov[0])
    camera.angle_y = np.deg2rad(fov[1])
    camera.lens = focal_length# focal length in mm
    
    cam_con = camera_obj.constraints.new('TRACK_TO')
    cam_con.target = itokawa_obj
    cam_con.track_axis = 'TRACK_NEGATIVE_Z'
    cam_con.up_axis = 'UP_Y'

    # setup the lamp 
    lamp = bpy.data.lamps['Lamp']
    lamp_obj = bpy.data.objects['Lamp']
    lamp.type = 'SUN'
    lamp.energy = 0.8
    lamp.use_specular = False
    lamp.use_diffuse = True
    
    # use spiceypy here to determine location of the light source
    lamp_obj.location.x = 5
    lamp_obj.location.y = 0
    lamp_obj.location.z = 1
    
    lamp_con = lamp_obj.constraints.new('TRACK_TO')
    lamp_con.target = itokawa_obj
    lamp_con.track_axis = 'TRACK_NEGATIVE_Z'
    lamp_con.up_axis = 'UP_Y'

    return (camera_obj, camera, lamp_obj, lamp, itokawa_obj, scene)

def blender_example():
    """
    Example to render an image

    """

    bpy.ops.wm.read_homefile() 
    scene = bpy.context.scene

    # delete the cube
    bpy.data.objects['Cube'].select = True
    bpy.ops.object.delete()
    
    # # delete default lamp
    # bpy.data.objects['Lamp'].select = True
    # bpy.ops.object.delete()

    # import OBJ model
    bpy.ops.import_scene.obj(filepath='data/itokawa_high.obj')

    # add a ground plane
    # add_plane = bpy.ops.mesh.primitive_plane_add
    # add_cube = bpy.ops.mesh.primitive_cube_add
    # cube = bpy.data.objects['Cube']
    # bpy.ops.object.camera_add()
    # bpy.ops.object.lamp_add()

    # set scene render options
    scene.render.resolution_x = 800
    scene.render.resolution_y = 600
    scene.render.resolution_percentage = 100
    # scene.render.filepath = 'visualization/blender/'
    scene.render.engine = 'BLENDER_RENDER'
    # scene.render.engine = 'CYCLES'


    camera_obj = bpy.data.objects['Camera']
    camera = bpy.data.cameras['Camera']
    
    empty = bpy.data.objects.new('Empty', None)
    bpy.context.scene.objects.link(empty)
    bpy.context.scene.update()

    # new sun lamp
    # lamp = bpy.data.lamps.new('Lamp', 'SUN')
    # lamp_obj = bpy.data.objects.new('Lamp', lamp)
    # scene.objects.link(lamp_obj)
    # lamp_obj.select = True
    # scene.objects.active = lamp_obj
    lamp = bpy.data.lamps['Lamp'] 
    lamp_obj = bpy.data.objects['Lamp']
    
    lamp.type = 'SUN'
    lamp.energy = 0.8
    lamp.use_specular = False
    lamp.use_diffuse = True

    itokawa_obj = bpy.data.objects['itokawa_high']
    itokawa_obj.scale = [1,1,1]
    
    bpy.types.UnitSettings.system = 'METRIC'
    bpy.types.UnitSettings.scale_length = 1e3
    bpy.data.worlds['World'].horizon_color = [0, 0, 0]

    # draw cylinders for axes
    # cylinder_between(0, 0, 0, 5, 0, 0, 0.01)
    # cylinder_between(0, 0, 0, 0, 5, 0, 0.01)
    # cylinder_between(0, 0, 0, 0, 0, 5, 0.01)

    # delete the cube
    num_steps = 10
    time = np.arange(0, num_steps, 1)

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
    camera_obj.location.y = -2
    camera_obj.location.z = 0


    con = camera_obj.constraints.new('TRACK_TO')
    con.target = itokawa_obj
    con.track_axis = 'TRACK_NEGATIVE_Z'
    con.up_axis = 'UP_Y'

    # camera.rotation_euler.x = -90
    # camera.rotation_euler.y = 0
    # camera.rotation_euler.z = 0

    lamp_obj.location.x = 5
    lamp_obj.location.y = 0
    lamp_obj.location.z = 1
    
    lamp_obj.rotation_mode = 'XYZ'
    lamp_obj.rotation_euler = np.deg2rad([82, -10, 89])
    for ii, t in enumerate(time):
        # itokawa_obj.rotation_euler.z = 360 / (12.132 * 3600) * t
        camera_obj.location.x = 3* np.cos(t * 2*np.pi/num_steps)
        camera_obj.location.y = 3* np.sin(t * 2*np.pi/num_steps)

        camera_obj.location.z = 0

        # look_at(camera_obj, itokawa_obj.matrix_world.to_translation())
        # center_camera(camera_obj, mathutils.Vector([0, 0, 0]))

        scene.render.filepath = os.path.join(output_path, 'cube_' + str(ii)  + '.png')
        bpy.ops.render.render(write_still=True, use_viewport=True)

    for object in bpy.data.objects:
        print(object.name + " is at location " + str(object.location))

    
    print("Itokawa Size: ", end='')
    print(itokawa_obj.dimensions)
    # bpy.ops.import_scene.obj('../data/itokawa_low.obj')
    # bpy.ops.render.render(write_still=True)

def blender_render(name, scene, save_file=False):
    """Call this function to render the current scene and output the pixels

    If you want you can save to a file as well
    """
    
    if save_file:
        scene.render.filepath = os.path.join(output_path + '/' + name + '.png')
        bpy.ops.render.render(write_still=True)

    else:
        # need to modify the source for this to work
        # https://blender.stackexchange.com/questions/69230/python-render-script-different-outcome-when-run-in-background/81240#81240
        bpy.context.scene.use_nodes = True
        tree = bpy.context.scene.node_tree
        links = tree.links

        # clear default nodes
        for n in tree.nodes:
            tree.nodes.remove(n)

        # create input render layer node
        rl = tree.nodes.new('CompositorNodeRLayers')
        
        # output node
        v = tree.nodes.new('CompositorNodeViewer')
        v.use_alpha = True

        # create a link
        links.new(rl.outputs[0], v.inputs[0])

        bpy.ops.render.render()

        img_data = bpy.data.images['Viewer Node'].pixels

def driver(sc_pos=[2,0,0], R_sc2ast=np.eye(3), filename='test'):
    """Driver for blender animation

    """
   
    # TODO - Implement attitude pointing instead of a camera constraint

    # initialize the scene
    camera_obj, camera, lamp_obj, lamp, itokawa_obj, scene = blender_init('CYCLES')

    # move camera and asteroid 
    camera_obj.location.x = sc_pos[0]
    camera_obj.location.y = sc_pos[1]
    camera_obj.location.z = sc_pos[2]

    # rot_quat = mathutils.Matrix(R_sc2ast).to_quaternion()
    # # camera_obj.rotation_quaternion = rot_quat
    # look_at(camera_obj, itokawa_obj.matrix_world.to_translation())

    # render an image from the spacecraft at this state
    # blender_render(filename,scene, True)
    scene.render.filepath = os.path.join(output_path + '/' + filename + '.png')
    bpy.ops.render.render(write_still=True)

def vertical_landing():
    """Test with a vertical descent to the surface

    Use to track motion between imagery

    """

    # define the trajectory
    radius = np.arange(10,1, -0.1)
    theta = np.deg2rad(315)
    for ii,r in enumerate(radius):
        sc_pos = np.array([r*np.cos(theta), r*np.sin(theta), 0])
        
        driver(sc_pos=sc_pos, R_sc2ast=np.eye(3), filename='test'+str.zfill(str(ii),2))

