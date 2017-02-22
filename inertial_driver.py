import dynamics.asteroid as asteroid
import dynamics.dumbbell as dumbbell
import attitude_ref.attitude as attitude

import numpy as np
from scipy import integrate
import argparse

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

    return (time,state, ast, dum)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Inertial EOMs simulator for a dumbbell around an asteroid')
    parser.add_argument('ast_name', help="String - asteroid name: castalia or itokawa", type=str)
    parser.add_argument('num_faces', help='Integer - Number of faces in polyhedron model', type=int)
    parser.add_argument('tf', help='Float - Terminal time for simulation', type=float)
    parser.add_argument('num_steps', help='Float - Number of steps in integration', type=float)
    parser.add_argument('file_name', help='String - Filename for npz archive', type=str)
    args = parser.parse_args()

    print("This will run a long simulation with the following parameters!")
    print("     ast_name  - %s" % args.ast_name)
    print("     num_faces  - %s" % args.num_faces)
    print("     tf        - %s" % args.tf)
    print("     num_steps - %s" % args.num_steps)
    print("     file_name - %s" % args.file_name)
    print("")
    print("Starting the simulation...")

    (time,state, ast, dum) = inertial_eoms_driver(args.ast_name,args.num_faces,args.tf,args.num_steps)

    print("Finished the simulation...")
    print("Saving to npz file")

    np.savez(args.file_name,state=state, ast=ast, dum=dum, time)

    print("All finished!")
