import numpy as np
from eom_comparison import transform
from kinematics import attitude as att
from dynamics import dumbbell, asteroid

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
