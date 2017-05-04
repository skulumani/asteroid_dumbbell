import dynamics.asteroid as asteroid
import dynamics.dumbbell as dumbbell
import kinematics.attitude as attitude
from visualization import plotting
from eom_comparison import transform
import pdb

import numpy as np
from scipy import integrate
import argparse

# valid initial condtions for castalia
# these are in the asteroid fixed frame
periodic_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607])
periodic_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327])
AbsTol = 1e-9
RelTol = 1e-9
def hamilton_eoms_driver(initial_state, time, ast, dum):
    """ODEINT Hamilton Relative EOMs driver
    
    This function will simulate the relative hamiltonian equations of motion  using
    scipy.integrate.odeint and then convert them into the inertial and
    asteroid frames.

    Parameters
    ----------
    initial_state - (18,) numpy ndarray representing the initial state of the dumbbell
        pos - initial_state[0:3] in km position of the dumbbell with respect to the
        asteroid and defined in the asteroid frame
        lin_mom - initial_state[3:6] in Jkm/sec is the linear momentum of dumbbell wrt the asteroid
        and defined in the asteroid frame
        R - initial_state[6:15] rotation matrix which converts vectors from the dumbbell frame to
        the asteroid frame
        ang_mom - state[15:18] rad/sec angular momentum of the dumbbell wrt inertial frame and
        defined in the asteroid frame
    time - (n,) numpy ndarray representing the simulation time span
    ast - asteroid object instance
    dum - dumbbell object instance

    Returns
    -------
    time - (n,) numpy ndarray time vector. Same as the input
    inertial_state - (n,18) ndarray with the simulation results represented in the inertial frame
        pos - inertial_state[0:3] in km position of the dumbbell with respect to the
        asteroid and defined in the inertial frame
        vel - inertial_state[3:6] in km/sec is the velocity of dumbbell wrt the asteroid
        and defined in the inertial frame
        R - inertial_state[6:15] rotation matrix which converts vectors from the dumbbell frame to
        the inertial frame
        w - inertial_state[15:18] rad/sec angular velocity of the dumbbell wrt inertial frame and
        defined in the inertial frame
    asteroid_state - (n,18) ndarray with the simulation results represented in the asteroid frame
        pos - asteroid_state[0:3] in km position of the dumbbell with respect to the
        asteroid and defined in the asteroid frame
        vel - asteroid_state[3:6] in km/sec is the velocity of dumbbell wrt the asteroid
        and defined in the asteroid frame
        R - asteroid_state[6:15] rotation matrix which converts vectors from the dumbbell frame to
        the asteroid frame
        w - asteroid_state[15:18] rad/sec angular velocity of the dumbbell wrt inertial frame and
        defined in the asteroid frame
    ast_state - (n,18) ndarray with the simulation results represented in the asteroid frame
        pos - asteroid_state[0:3] in km position of the dumbbell with respect to the
        asteroid and defined in the inertial frame
        vel - asteroid_state[3:6] in km/sec is the velocity of dumbbell wrt the asteroid
        and defined in the inertial frame
        R - asteroid_state[6:15] rotation matrix which converts vectors from the dumbbell frame to
        the inertial frame
        w - asteroid_state[15:18] rad/sec angular momentum of the dumbbell wrt inertial frame and
        defined in the asteroid frame

        This output is exactly what dum.eoms_hamilton_relative  will output without any alduteration
    """
    ast_state = integrate.odeint(dum.eoms_hamilton_relative, initial_state, time, args=(ast,), atol=AbsTol, rtol=RelTol)

    # convert to inertial and asteroid frames
    inertial_state = transform.eoms_hamilton_relative_to_inertial(time, ast_state, ast, dum)
    asteroid_state = transform.eoms_hamilton_relative_to_asteroid(time, ast_state, ast, dum)

    return (time, inertial_state, asteroid_state, ast_state)

def compute_energy(file_name):
    with np.load(file_name, allow_pickle=True) as data:

        ast_name = data['ast_name']
        num_faces = data['num_faces']
        time = data['time']
        asteroid_state = data['asteroid_state']

        dum = data['dum'][()]
        ast = data['ast'][()]

        KE, PE = dum.relative_energy(time,asteroid_state,ast)

    return KE, PE

def hamilton_eoms_energy_behavior(initial_state, time, ast, dum):
    """See \Delta E for varying tolerances of the ODE function

    """
    tol_array = np.logspace(-4, -12, 9)
    time_dict = {}
    ast_dict = {}
    inertial_state_dict = {}
    asteroid_state_dict = {}
    KE_dict = {}
    PE_dict = {}

    for tol in tol_array:
        print('Tolerance - %4.2e' % tol)
        (_, inertial_state, asteroid_state, ast_state) = hamilton_eoms_driver(initial_state, time, ast, dum)
        KE, PE = dum.relative_energy(time, asteroid_state, ast)

        time_dict[str(tol)] = time
        KE_dict[str(tol)] = KE
        PE_dict[str(tol)] = PE
        asteroid_state_dict[str(tol)] = ast_state
        inertial_state_dict[str(tol)] = inertial_state
        asteroid_state_dict[str(tol)] = asteroid_state

    return (time_dict, KE_dict, PE_dict, asteroid_state_dict, inertial_state_dict, asteroid_state_dict)

def relative_sim_plotter(file_name, mode):
    """Plot the simulation results

    """

    # load the data
    with np.load(file_name, allow_pickle=True) as data:
        if mode == 0:
            time = data['time']
            KE = data['KE']
            PE = data['PE']

            # compute the
            e_fig = plotting.plt.figure()
            plotting.plot_energy(time,KE, PE, e_fig)

            # plotting.animate_relative_trajectory(time, state, ast, dum, file_name[:-4])
        elif mode == 1:
            time_dict = data['time_dict'][()]
            KE_dict = data['KE_dict'][()]
            PE_dict = data['PE_dict'][()]

            e_fig = plotting.plt.figure()
            for tol in time_dict:
                E = KE_dict[tol] + PE_dict[tol]
                Ediff = np.absolute(E - E[0])

                plotting.plt.plot(time_dict[tol], Ediff, label=tol)
                print("Tol - %s, DeltaE - %4e" % (tol, E[-1]))
                
            plotting.plt.legend()
        else:

            print("Incorrect mode.")
            return 1


    plotting.plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Relative EOMs simulator for a dumbbell around an asteroid')
    parser.add_argument('ast_name', help="String - asteroid name: castalia or itokawa", type=str)
    parser.add_argument('num_faces', help='Integer - Number of faces in polyhedron model', type=int)
    parser.add_argument('tf', help='Float - Terminal time for simulation', type=float)
    parser.add_argument('num_steps', help='Float - Number of steps in integration', type=float)
    parser.add_argument('file_name', help='String - Filename for npz archive', type=str)
    parser.add_argument("-m", "--mode", type=int, choices=[0, 1],
                    help="Choose which relative energy mode to run. 0 - relative energy, 1 - \Delta E behavior")
    args = parser.parse_args()

    print("Starting the simulation...")

    # instantiate the asteroid and dumbbell objects
    ast = asteroid.Asteroid(args.ast_name, args.num_faces)
    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

    # initialize simulation parameters
    time = np.linspace(0, args.tf, args.num_steps)
    initial_pos = periodic_pos
    initial_R = attitude.rot2(0).reshape(9)
    initial_w = np.array([0.01, 0.01, 0.01])

    initial_lin_mom = (dum.m1 + dum.m2) * (periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos))
    initial_ang_mom = initial_R.reshape((3,3)).dot(dum.J).dot(initial_w)
    initial_ham_state = np.hstack((initial_pos, initial_lin_mom, initial_R, initial_ang_mom))

    print("This will run a long simulation with the following parameters!")
    print("     ast_name  - %s" % args.ast_name)
    print("     num_faces  - %s" % args.num_faces)
    print("     tf        - %s" % args.tf)
    print("     num_steps - %s" % args.num_steps)
    print("     file_name - %s" % args.file_name)
    print("")

    if args.mode == 0:
        (_, inertial_state, asteroid_state, ast_state) = hamilton_eoms_driver(initial_ham_state, time, ast, dum)

        print("Finished the simulation...")
        print("Saving to npz file")

        np.savez(args.file_name,inertial_state=inertial_state, time=time, asteroid_state=asteroid_state,
                ast_state=ast_state, ast_name=args.ast_name, num_steps=args.num_steps,tf=args.tf, num_faces=args.num_faces,
                ast=ast, dum=dum)

        print("Now working to calculate the KE/PE of the simulation!")

        KE, PE = compute_energy(args.file_name)

        print("Finished with energy computations!")
        np.savez('hamilton_relative_energy_' + args.file_name, args.file_name,inertial_state=inertial_state, time=time, asteroid_state=asteroid_state,
                ast_state=ast_state, ast_name=args.ast_name, num_steps=args.num_steps,tf=args.tf, num_faces=args.num_faces,
                ast=ast, dum=dum, KE=KE, PE=PE)
        print("All finished!")
    elif args.mode == 1:

        print("We're running many simulations wiht varying tolerances.")

        (time_dict, KE_dict, PE_dict, ast_state_dict, inertial_state_dict, asteroid_state_dict) \
        = hamilton_eoms_energy_behavior(initial_ham_state, time, ast, dum)

        print("Finished with simulations. Now saving to data file")

        np.savez('hamilton_relative_energy_behavior_' + args.file_name, KE_dict=KE_dict,
                PE_dict=PE_dict, time_dict=time_dict, ast_name=args.ast_name, num_steps=args.num_steps,
                tf=args.tf, num_faces=args.num_faces, ast_state_dict=ast_state_dict, 
                inertial_state_dict=inertial_state_dict, asteroid_state_dict=asteroid_state_dict, 
                ast=ast, dum=dum)
        
        print("All finished!")
    else:
        print("Missing mode argument.")
