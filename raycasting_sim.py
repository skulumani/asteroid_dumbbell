"""Simulation of a spacecraft with a LIDAR taking measurements 
around an asteroid
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import pdb
from collections import defaultdict

import numpy as np
from scipy import integrate

from dynamics import asteroid, dumbbell, eoms, controller
from kinematics import attitude
from visualization import plotting, graphics, animation
from point_cloud import wavefront, raycaster

# simulate dumbbell moving aroudn asteroid
ast = asteroid.Asteroid('castalia', 4092, 'mat')
dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)
des_att_func = controller.body_fixed_pointing_attitude
des_tran_func = controller.inertial_fixed_state
AbsTol = 1e-9
RelTol = 1e-9

num_steps = int(1e4)
time = np.linspace(0, num_steps, num_steps)
t0, tf = time[0], time[-1]
dt = time[1] - time[0]

initial_pos = np.array([1.5, 0, 0])
initial_vel = np.array([0, 0, 0])
initial_R = attitude.rot3(np.pi/2).reshape(-1)
initial_w = np.array([0, 0, 0])
initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

# TODO Initialize a coarse asteroid mesh model and combine with piont cloud data

# initialize the raycaster and lidar
polydata = wavefront.meshtopolydata(ast.V, ast.F)
caster = raycaster.RayCaster(polydata)
sensor = raycaster.Lidar(dist=5)
# try both a controlled and uncontrolled simulation
# t, istate, astate, bstate = eoms.inertial_eoms_driver(initial_state, time, ast, dum)

# TODO Dynamics should be based on the course model 
# TODO Asteroid class will need a method to update mesh
system = integrate.ode(eoms.eoms_controlled_inertial)
system.set_integrator('lsoda', atol=AbsTol, rtol=RelTol, nsteps=num_steps)
system.set_initial_value(initial_state, t0)
system.set_f_params(ast, dum, des_att_func, des_tran_func)

point_cloud = defaultdict(list)

state = np.zeros((num_steps+1, 18))
t = np.zeros(num_steps+1)
int_array = []
state[0, :] = initial_state

ii = 1
while system.successful() and system.t < tf:
    # integrate the system and save state to an array
    t[ii] = (system.t + dt)
    state[ii, :] = system.integrate(system.t + dt)

    # now do the raycasting
    if not (np.floor(t[ii]) % 1):
        targets = sensor.define_targets(state[ii, 0:3],
                                        state[ii, 6:15].reshape((3,3)), 
                                        np.linalg.norm(state[ii, 0:3]))
        
        # new asteroid rotation with vertices
        nv = ast.rotate_vertices(t[ii])
        Ra = ast.rot_ast2int(t[ii])
        # update the mesh inside the caster
        caster = raycaster.RayCaster.updatemesh(nv, ast.F)
        
        # these intersections are points in the inertial frame
        intersections = caster.castarray(state[ii, 0:3], targets)

        point_cloud['time'].append(t[ii])
        point_cloud['ast_state'].append(Ra.reshape(-1))
        point_cloud['sc_state'].append(state[ii,:])
        point_cloud['targets'].append(targets)
        point_cloud['inertial_ints'].append(intersections)
        ast_ints = []
        for pt in intersections:
            if pt.size > 0:
                pt_ast = Ra.T.dot(pt)
            else:
                pt_ast = []
            ast_ints.append(pt_ast)

        point_cloud['ast_ints'].append(np.asarray(ast_ints))

    # TODO Eventually call the surface reconstruction function and update asteroid model

    # create an asteroid and dumbbell
    ii += 1

# plot the simulation
# plotting.animate_inertial_trajectory(t, istate, ast, dum)
# plotting.plot_controlled_inertial(t, istate, ast, dum, fwidth=1)
graphics.point_cloud_asteroid_frame(point_cloud)

mfig = graphics.mayavi_figure(size=(800,600)) 
mesh, ast_axes = graphics.draw_polyhedron_mayavi(ast.V, ast.F, mfig)

com, dum_axes = graphics.draw_dumbbell_mayavi(state[0, :], dum, mfig)

pc_lines = [graphics.mayavi_addLine(mfig, state[0, 0:3], p) for p in point_cloud['inertial_ints'][0]]

animation.inertial_asteroid_trajectory(time, state, ast, dum, point_cloud,
                                       (mesh, ast_axes, com, dum_axes,
                                        pc_lines))


# TODO second animation to show the points measured of the asteroid


