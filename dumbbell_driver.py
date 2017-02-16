# Run the simulation

import dynamics.asteroid as asteroid
import dynamics.dumbbell as dumbbell
import attitude_ref.attitude as attitude

import numpy as np
from scipy import integrate
import pdb

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# ode options
RelTol = 1e-9
AbsTol = 1e-9

ast = asteroid.Asteroid('castalia',32)
dum = dumbbell.Dumbbell()

# set initial state
initial_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607]) # km for center of mass in body frame
# km/sec for COM in asteroid fixed frame
initial_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327]) + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
initial_R = np.eye(3,3).reshape(9) # transforms from dumbbell body frame to the inertial frame
initial_w = np.array([0,0,0]) # angular velocity of dumbbell wrt to inertial frame represented in sc body frame

initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

# time span
t0 = 0
tf = 1e5 # sec
num_steps = 1e5

time = np.linspace(t0,tf,num_steps)

state = integrate.odeint(dum.eoms_inertial, initial_state, time, args=(ast,), atol=AbsTol, rtol=RelTol)

pos = state[:,0:3]
vel = state[:,3:6]
R = state[:,6:15]
ang_vel = state[:,15:18]

KE, PE = dum.inertial_energy(time,state,ast)

mpl.rcParams['legend.fontsize'] = 10

# trajectory plot
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(pos[:,0],pos[:,1],pos[:,2])

# kinetic energy
plt.figure()

plt.plot(time,KE, label='Kinetic Energy')
plt.plot(time,PE, label='Potential Energy')
plt.plot(time,PE+KE, label='Total Energy')
plt.xlabel('Time')
plt.ylabel('Energy')
plt.legend()
plt.grid(True)

plt.show()