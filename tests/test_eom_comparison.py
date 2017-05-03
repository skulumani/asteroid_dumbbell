import numpy as np
from scipy import integrate

from eom_comparison import transform
from kinematics import attitude as att
from dynamics import dumbbell, asteroid
import inertial_driver as idriver
import relative_driver as rdriver

class TestInertialTransform():

    inertial_pos = np.array([1, 1, 1])
    inertial_vel = np.random.rand(3)
    R_sc2int = att.rot2(np.pi/2)
    body_ang_vel = np.random.rand(3)

    time = np.array([0])
    ast = asteroid.Asteroid(name='castalia', num_faces=64)
    dum = dumbbell.Dumbbell()

    input_state = np.hstack((inertial_pos, inertial_vel, R_sc2int.reshape(9), body_ang_vel))
    inertial_state = transform.eoms_inertial_to_inertial(time, input_state, ast, dum) 
    def test_eoms_inertial_to_inertial_scalar_pos(self):
        np.testing.assert_almost_equal(self.inertial_pos, self.inertial_state[0:3])

    def test_eoms_inertial_to_inertial_scalar_vel(self):
        np.testing.assert_almost_equal(self.inertial_vel, self.inertial_state[3:6])

    def test_eoms_inertial_to_inertial_scalar_att(self):
        np.testing.assert_almost_equal(self.R_sc2int.reshape(9), self.inertial_state[6:15])

    def test_eoms_inertial_to_inertial_scalar_ang_vel(self):
        np.testing.assert_almost_equal(self.R_sc2int.dot(self.body_ang_vel), self.inertial_state[15:18])
    
class TestHamiltonRelativeTransform():

    time = np.array([0])
    ast = asteroid.Asteroid(name='castalia', num_faces=64)
    dum = dumbbell.Dumbbell()

    inertial_pos = np.array([1, 1, 1])
    inertial_vel = np.random.rand(3)+ att.hat_map(ast.omega*np.array([0,0,1])).dot(inertial_pos)
    R_sc2int = att.rot2(np.pi/2)
    R_ast2int = att.rot3(time * ast.omega)
    body_ang_vel = np.random.rand(3)

    initial_lin_mom = (dum.m1 + dum.m2) * (inertial_vel )
    initial_ang_mom = R_sc2int.dot(dum.J).dot(body_ang_vel)
    initial_ham_state = np.hstack((inertial_pos, initial_lin_mom,R_ast2int.dot(R_sc2int).reshape(9), initial_ang_mom))


    inertial_state = transform.eoms_hamilton_relative_to_inertial(time,initial_ham_state, ast, dum) 

    def test_eoms_hamilton_relative_to_inertial_scalar_pos(self):
        np.testing.assert_almost_equal(self.inertial_pos, self.inertial_state[0:3])

    def test_eoms_hamilton_relative_to_inertial_scalar_vel(self):
        np.testing.assert_almost_equal(self.inertial_vel, self.inertial_state[3:6])

    def test_eoms_hamilton_relative_to_inertial_scalar_att(self):
        np.testing.assert_almost_equal(self.R_sc2int.reshape(9), self.inertial_state[6:15])

    def test_eoms_hamilton_relative_to_inertial_scalar_ang_vel(self):
        np.testing.assert_almost_equal(self.R_sc2int.dot(self.body_ang_vel), self.inertial_state[15:18])

class TestInertialandRelativeEOMS():
    """Compare the inertial and relative eoms against one another

    """
    RelTol = 1e-9
    AbsTol = 1e-9
    ast_name = 'castalia'
    num_faces = 64
    tf = 1e2
    num_steps = 1e2
    time = np.linspace(0,tf,num_steps)

    periodic_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607])
    periodic_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327])

    ast = asteroid.Asteroid(ast_name,num_faces)
    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

    # set initial state for inertial EOMs
    initial_pos = periodic_pos # km for center of mass in body frame
    initial_vel = periodic_vel +att.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
    initial_R = att.rot2(np.pi/2).reshape(9) # transforms from dumbbell body frame to the inertial frame
    initial_w = np.array([0.01, 0.01, 0.01])
    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

    i_state = integrate.odeint(dum.eoms_inertial, initial_state, time, args=(ast,), atol=AbsTol, rtol=RelTol)
    # (i_time, i_state) = idriver.eom_inertial_driver(initial_state, time, ast, dum, AbsTol=1e-9, RelTol=1e-9)

    initial_lin_mom = (dum.m1 + dum.m2) * (periodic_vel + att.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos))
    initial_ang_mom = initial_R.reshape((3,3)).dot(dum.J).dot(initial_w)
    initial_ham_state = np.hstack((initial_pos, initial_lin_mom, initial_R, initial_ang_mom))

    rh_state = integrate.odeint(dum.eoms_hamilton_relative, initial_ham_state, time, args=(ast,), atol=AbsTol, rtol=RelTol)
    
    # now convert both into the inertial frame
    istate_ham = transform.eoms_hamilton_relative_to_inertial(time,rh_state,ast, dum) 
    istate_int = transform.eoms_inertial_to_inertial(time,i_state, ast, dum) 

    # also convert both into the asteroid frame and compare
    astate_ham = transform.eoms_hamilton_relative_to_asteroid(time, rh_state, ast, dum)
    astate_int = transform.eoms_inertial_to_asteroid(time, i_state, ast, dum)

    def test_inertial_frame_comparison_pos(self):
        np.testing.assert_array_almost_equal(self.istate_ham[:, 0:3], self.istate_int[:, 0:3])

    def test_inertial_frame_comparison_vel(self):
        np.testing.assert_array_almost_equal(self.istate_ham[:, 3:6], self.istate_int[:, 3:6])

    def test_inertial_frame_comparison_att(self):
        np.testing.assert_array_almost_equal(self.istate_ham[:, 6:15], self.istate_int[:, 6:15])

    def test_inertial_frame_comparison_ang_vel(self):
        np.testing.assert_array_almost_equal(self.istate_ham[:, 15:18], self.istate_int[:, 15:18])

    def test_asteroid_frame_comparison_pos(self):
        np.testing.assert_array_almost_equal(self.astate_ham[:, 0:3], self.astate_int[:, 0:3])

    def test_asteroid_frame_comparison_vel(self):
        np.testing.assert_array_almost_equal(self.astate_ham[:, 3:6], self.astate_int[:, 3:6])

    def test_asteroid_frame_comparison_att(self):
        np.testing.assert_array_almost_equal(self.astate_ham[:, 6:15], self.astate_int[:, 6:15])

    def test_asteroid_frame_comparison_ang_vel(self):
        np.testing.assert_array_almost_equal(self.astate_ham[:, 15:18], self.astate_int[:, 15:18])
