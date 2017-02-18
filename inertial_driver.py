import dynamics.asteroid as asteroid
import dynamics.dumbbell as dumbbell
import attitude_ref.attitude as attitude

import numpy as np
from scipy import integrate

def inertial_eoms_driver(ast_name,num_faces,tf,num_steps):
    # ode options
    RelTol = 1e-9
    AbsTol = 1e-9

    ast = asteroid.Asteroid(ast_name,num_faces)
    dum = dumbbell.Dumbbell()

    # set initial state
    initial_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607]) # km for center of mass in body frame
    # km/sec for COM in asteroid fixed frame
    initial_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327]) + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
    initial_R = np.eye(3,3).reshape(9) # transforms from dumbbell body frame to the inertial frame
    initial_w = np.array([0,0,0]) # angular velocity of dumbbell wrt to inertial frame represented in sc body frame

    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

    time = np.linspace(0,tf,num_steps)

    state = integrate.odeint(dum.eoms_inertial, initial_state, time, args=(ast,), atol=AbsTol, rtol=RelTol)

    return (state, ast, dum)

if __name__ == '__main__':
    print("This will run a long simulation!")
    print("Starting the simulation....")

    (state, ast, dum) = inertial_eoms_driver('castalia',4092,1e7,1e7)

    print("Finished the simulation...")
    print("Saving to npz file")
    np.savez('long_inertial_sim',state=state, ast=ast, dum=dum)

    print("All finished!")
