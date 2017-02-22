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

    dum = dumbbell.Dumbbell()
    ast = asteroid.Asteroid('castalia',1024)
    time = np.linspace(0,1e6,1e6)
    # dum = data['dum']
    
    state = data['state']

    KE, PE = dum.inertial_energy(time,state,ast)

    # generate some plots
    traj_fig = plt.figure()
    plotting.plot_trajectory(state[:,0:3],traj_fig)

    # kinetic energy
    energy_fig = plt.figure()
    plotting.plot_energy(time,KE,PE,energy_fig)

    plt.show()