import numpy as np
import pdb

class Dumbbell():
    """Dumbbell spacecraft model

    """
    def __init__(self):
        """Initialize the dumbbell instance with all of it's properties
            
        """

        self.m1 = 100 # kg first mass
        self.m2 = 100 # kg second mass
        self.l = 0.003 # km rigid link
        self.r1 = 0.001 # km radius of each spherical mass 
        self.r2 = 0.001

        self.lcg1 = self.m2/(self.m1+self.m2)*self.l; # distance from m1 to the CG along the b1hat direction
        self.lcg2 = self.l - self.lcg1

        self.zeta1 = np.array([-self.lcg1,0,0])
        self.zeta2 = np.array([self.lcg2,0,0])

        self.Jm1 = 2/5*self.m1*self.r1**2 * np.diag([1,1,1])
        self.Jm2 = 2/5*self.m2*self.r2**2 * np.diag([1,1,1])

        self.J = self.Jm1 + self.Jm2 + self.m1 *(np.inner(self.zeta1,self.zeta1)*np.eye(3,3) - np.outer(self.zeta1,self.zeta1)) + self.m2 * (np.inner(self.zeta2,self.zeta2)*np.eye(3,3) - np.outer(self.zeta2,self.zeta2))

    def eoms_intertial(t, state, ast):
        """Inertial dumbbell equations of motion about an asteroid
        
        Inputs:
            t - 
            state -
            ast - Asteroid class object holding the asteroid gravitational model and
            other useful parameters
        """

        # unpack the state
        pos = state[0:2] # location of the center of mass in the inertial frame
        vel = state[3:5] # vel of com in inertial frame
        R = np.reshape(state[6:14],(3,3)) # sc body frame to inertial frame
        w = state(15:17) # angular velocity of sc wrt inertial frame defined in body frame

        Ra = ROT3(-constants.omega*t); % asteroid body frame to inertial frame

        % unpack constants
        m1 = constants.m1;
        m2 = constants.m2;
        lcg = constants.lcg;
        l = constants.l;
        J = constants.J;

        % the position of each mass in the asteroid body frame
        rho_1 = lcg*[1;0;0];
        rho_2= (l-lcg)*[-1;0;0];

        % position of each mass in the inertial frame
        z1 = Ra'*(pos + R * rho_1);
        z2 = Ra'*(pos + R * rho_2);

        z = Ra' * pos; % position of COM in asteroid frame

        % these gradients are defined in the asteroid fixed body frame
        switch constants.pot_model
        case 'polyhedron'
        switch constants.asteroid_grav.num_f
            case 1024
                [~,U_grad_m1,~, ~] = polyhedron_potential_mex_1024(z1, constants.asteroid_grav);
                [~,U_grad_m2,~, ~] = polyhedron_potential_mex_1024(z2, constants.asteroid_grav);
                [~,U_grad_com,~, ~] = polyhedron_potential_mex_1024(z, constants.asteroid_grav);
            case 4092
                [~,U_grad_m1,~, ~] = polyhedron_potential_mex_4092(z1, constants.asteroid_grav);
                [~,U_grad_m2,~, ~] = polyhedron_potential_mex_4092(z2, constants.asteroid_grav);
                [~,U_grad_com,~, ~] = polyhedron_potential_mex_4092(z, constants.asteroid_grav);
            case 32
                [~,U_grad_m1,~, ~] = polyhedron_potential_mex_32(z1, constants.asteroid_grav);
                [~,U_grad_m2,~, ~] = polyhedron_potential_mex_32(z2, constants.asteroid_grav);
                [~,U_grad_com,~, ~] = polyhedron_potential_mex_32(z, constants.asteroid_grav);
        end
        case 'mascon'
        [~,U_grad_m1] = mascon_potential(z1,constants.asteroid_grav,constants);
        [~,U_grad_m2] = mascon_potential(z2,constants.asteroid_grav,constants);
        [~,U_grad_com] = mascon_potential(z,constants.asteroid_grav,constants);
        case 'matlab'
        [~,U_grad_m1, ~, ~] = polyhedron_potential(z1, constants.asteroid_grav);
        [~,U_grad_m2, ~, ~] = polyhedron_potential(z2, constants.asteroid_grav);
        [~,U_grad_com, ~, ~] = polyhedron_potential(z, constants.asteroid_grav);
        end

        % force due to each mass expressed in the inertial body frame
        F1 = m1*Ra*U_grad_m1;
        F2 = m2*Ra*U_grad_m2;
        F_com = (m1+m2)*Ra*U_grad_com;

        % compute the moments due to each mass
        M1 = m1*(R'*U_grad_m1*(Ra'*rho_1)' - (Ra'*rho_1)*U_grad_m1'*R);
        M2 = m2*(R'*U_grad_m2*(Ra'*rho_2)' - (Ra'*rho_2)*U_grad_m2'*R);

        % apply the vee map to get a vector
        M1 = vee_map(M1);
        M2 = vee_map(M2);

        % state derivatives
        pos_dot = vel;
        vel_dot = 1/(m1+m2) * (F1+F2);
        % vel_dot = 1/(m1+m2) * (F_com) ;
        R_dot = R*hat_map(w);
        R_dot = reshape(R_dot,9,1);
        w_dot = J\(M1 + M2 - hat_map(w)*(J*w));

        state_dot = [pos_dot;vel_dot;R_dot;w_dot];

    def eoms_relative(t,state,ast):

        pass