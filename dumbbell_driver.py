# Run the simulation

import dynamics.asteroid
import dynamics.dumbbell

import numpy as np

fontsize = 18;
fontname = 'Times'

periodic_tspan = np.array([0,30000])
RelTol = 1e-9
AbsTol = 1e-9
tol = 1e-6
diffcorr_plot = 0
jacobi_step = 1e-9

ast = dynamics.asteroid.Asteroid('castalia',32)
dum = dynamics.dumbbell.Dumbbell()

# set initial state
initial_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607]) # km for center of mass in body frame
initial_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327]) # km/sec for COM in asteroid fixed frame
initial_vel = initial_vel + hat_map(ast.omega*np.array([0;0;1]))*initial_pos

initial_R = np.eye(3,3).reshape(9,1) # transforms from dumbbell body frame to the inertial frame
initial_w = np.array([0,0,0]) # angular velocity of dumbbell wrt to asteroid represented in sc body frame

initial_state = 

r = ode(f, jac).set_integrator('zvode', method='bdf')
r.set_initial_value(y0, t0).set_f_params(2.0).set_jac_params(2.0)
t1 = 10
dt = 1

while r.successful() and r.t < t1:
    print(r.t+dt, r.integrate(r.t+dt))