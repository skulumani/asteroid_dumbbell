import dynamics.dumbbell as dumbbell
import dynamics.asteroid as asteroid
import attitude_ref.attitude as attitude

import numpy as np
import scipy as sp

import matplotlib as mpl
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import matplotlib.colors as colors

with np.load('data/20170218_long_inertial.npz', allow_pickle=True) as data:

    ast_name = data['ast_name']
    num_faces = data['num_faces']
    time = data['time']
    state = data['state']

    dum = dumbbell.Dumbbell()
    ast = asteroid.Asteroid(data['ast_name'],data['num_faces'])

    KE, PE = dum.inertial_energy(time,state,ast)

    # generate some plots
    traj_fig = plt.figure()
    plotting.plot_trajectory(state[:,0:3],traj_fig)

    # kinetic energy
    energy_fig = plt.figure()
    plotting.plot_energy(time,KE,PE,energy_fig)

    plt.show()

np.savez('data/energy_added.npz',ast_name=ast_name,num_faces=num_faces,time=time,state=state,KE=KE,PE=PE)


