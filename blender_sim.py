"""Simulation of controlled dumbbell around Itokawa with 
simulated imagery using Blender

4 August 2017 - Shankar Kulumani
"""
from __future__ import absolute_import, division, print_function, unicode_literals
from scipy import integrate
import numpy as np
import pdb

import visualization.plotting as plotting
import dynamics.asteroid as asteroid
import dynamics.dumbbell as dumbbell
from kinematics import attitude

import inertial_driver as idriver
import relative_driver as rdriver

def eoms_controlled_blender(t, state, dum, ast):
    """Inertial dumbbell equations of motion about an asteroid 
    
    This method must be used with the scipy.integrate.ode class instead of the
    more convienent scipy.integrate.odeint. In addition, we can control the
    dumbbell given full state feedback. Blender is used to generate imagery
    during the simulation.

    Inputs:
        t - Current simulation time step
        state - (18,) array which defines the state of the vehicle 
            pos - (3,) position of the dumbbell with respect to the
            asteroid center of mass and expressed in the inertial frame
            vel - (3,) velocity of the dumbbell with respect to the
            asteroid center of mass and expressed in the inertial frame
            R - (9,) attitude of the dumbbell with defines the
            transformation of a vector in the dumbbell frame to the
            inertial frame ang_vel - (3,) angular velocity of the dumbbell
            with respect to the inertial frame and represented in the
            dumbbell frame
        ast - Asteroid class object holding the asteroid gravitational
        model and other useful parameters
    """
    # unpack the state
    pos = state[0:3] # location of the center of mass in the inertial frame
    vel = state[3:6] # vel of com in inertial frame
    R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
    ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame

    Ra = attitude.rot3(ast.omega*t, 'c') # asteroid body frame to inertial frame

    # unpack parameters for the dumbbell
    J = dum.J

    rho1 = dum.zeta1
    rho2 = dum.zeta2

    # position of each mass in the asteroid frame
    z1 = Ra.T.dot(pos + R.dot(rho1))
    z2 = Ra.T.dot(pos + R.dot(rho2))

    z = Ra.T.dot(pos) # position of COM in asteroid frame

    # compute the potential at this state
    (U1, U1_grad, U1_grad_mat, U1laplace) = ast.polyhedron_potential(z1)
    (U2, U2_grad, U2_grad_mat, U2laplace) = ast.polyhedron_potential(z2)

    F1 = self.m1*Ra.dot(U1_grad)
    F2 = self.m2*Ra.dot(U2_grad)

    M1 = self.m1 * attitude.hat_map(rho1).dot(R.T.dot(Ra).dot(U1_grad))
    M2 = self.m2 * attitude.hat_map(rho2).dot(R.T.dot(Ra).dot(U2_grad))
    
    # generate image at this current state

    # use the imagery to figure out motion and pass to the controller instead
    # of the true state


    # compute the control input
    u_m = self.attitude_controller(t, state, M1+M2)
    u_f = self.translation_controller(t, state, F1+F2)

    pos_dot = vel
    vel_dot = 1/(self.m1+self.m2) *(F1 + F2 + u_f)
    R_dot = R.dot(attitude.hat_map(ang_vel)).reshape(9)
    ang_vel_dot = np.linalg.inv(J).dot(-np.cross(ang_vel,J.dot(ang_vel)) + M1 + M2 + u_m)

    statedot = np.hstack((pos_dot, vel_dot, R_dot, ang_vel_dot))

    return statedot

RelTol = 1e-6
AbsTol = 1e-6
ast_name = 'itokawa'
num_faces = 64
tf = 1e3
num_steps = 1e3
time = np.linspace(0,int(tf),int(num_steps))

periodic_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607])
periodic_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327])

ast = asteroid.Asteroid(ast_name,num_faces)
dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

t0 = 0
tf = 1e2
dt = 1
num_steps = 100

# set initial state for inertial EOMs
initial_pos = periodic_pos # km for center of mass in body frame
initial_vel = periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
initial_R = attitude.rot2(np.pi/2).reshape(9) # transforms from dumbbell body frame to the inertial frame
initial_w = np.array([0.01, 0.01, 0.01])
initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

# instantiate ode object
system = integrate.ode(dum.eoms_inertial_control_ode)
system.set_integrator('lsoda', atol=AbsTol, rtol=RelTol, nsteps=1000)
system.set_initial_value(initial_state, t0)
system.set_f_params(ast)

i_state = np.zeros((num_steps+1, 18))
time = np.zeros(num_steps+1)

ii = 1
while system.successful() and system.t < tf:
    # integrate the system and save state to an array
    
    time[ii] = (system.t + dt)
    i_state[ii, :] = (system.integrate(system.t + dt))
    # generate the view of the asteroid at this state

    # do some image processing and visual odometry
    ii += 1
