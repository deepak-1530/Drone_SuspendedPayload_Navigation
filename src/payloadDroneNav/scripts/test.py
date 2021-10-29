import time, math
import numpy as np
import scipy as sp
from quadprog import solve_qp
from scipy.spatial.transform import Rotation as rotation
from scipy.linalg import block_diag as blkdiag

from drl.utils.flat2state import Flat2State
from drl.utils.trajectories.basics import setpoint, circleXY

from control import lqr
import casadi

class Quadrotor(object):
    """
    Quadrotor dynamics on SE3
    """

    def __init__(self, CTRL='pd'):        
        
        self._e1 = np.array([1.0, 0.0, 0.0])
        self._e2 = np.array([0.0, 1.0, 0.0])
        self._e3 = np.array([0.0, 0.0, 1.0])
        self._g = 9.81
        self._mass = 1
        self._inertia = np.array([0.0023, 0.0023, 0.004])
        self._inertia_matrix = np.array([[0.0023, 0., 0.], [0., 0.0023, 0.], [0., 0., 0.004]])
        eig, _ = np.linalg.eig(self._inertia_matrix)
        self.J = self._inertia_matrix
        self.J_scale = (1/np.min(eig))*self._inertia_matrix
        self.J_inv = np.linalg.inv(self._inertia_matrix)

        self._time_step = 1/240

        self._max_force     = 100
        self._max_torque    = 10

        self._state = {'position': np.zeros((3)),
                        'velocity': np.zeros((3)),
                        'orientation': np.eye(3),
                        'ang_vel': np.zeros((3))}

        self._des_state = {'position': np.zeros((3)),
                            'velocity': np.zeros((3)),
                            'orientation': np.eye(3),
                            'ang_vel': np.zeros((3))}

        self._gains = {'position':     np.array([8., 8., 10.]),
                        'velocity':    np.array([8., 8, 8.]),
                        'orientation':  5*12.*np.array([1.0, 1.0, 1.0]),
                        'ang_vel':      5*2*np.array([1.0, 1.0, 1.0])}

        # self._gains = {'position': 1*np.array([2., 2.0, 2.0]),
        #                         'velocity': 2*np.array([1.0, 1.0, 1.0]),
        #                         'orientation': 2*np.array([.60, .60, .60]),
        #                         'ang_vel': 2*np.array([.06, .06, .06])}

        if CTRL == 'pd':
            self.compute_control = lambda t: self.geometric_pd(t)
        elif CTRL == 'clf':
            self.compute_control = lambda t: self.seq_clf_qp(t)
        elif CTRL=='vbl-lqr':
            self.compute_control = lambda t: self.vbl_lqr(t)
        elif CTRL=='mpc':
            self.compute_control = lambda t: self.discrete_mpc(t)
            self._mpc_horizon= int(20)
            self.solver_initialized= False
            self.mpc_solver = None
        else:
            self.compute_control = lambda t: self.geometric_pd()

        self.eta1, self.epsilon1, self.c1 = 2.5, 2, 6
        self.eta2, self.epsilon2, self.c2 = 150, 4, 10

        self.get_flat_traj = lambda t: setpoint(t, sp=np.array([1., -1., 1.0]))
        # self.get_flat_traj = lambda t: circleXY(t, r=2, w=0.5*math.pi, c=np.array([0,0,1.]))
        self.des_state = None

    @property
    def mass(self):
        return self._mass 
    
    @property
    def inertia(self):
        return self._inertia

    @property
    def inertia_matrix(self):
        return self._inertia_matrix

    @property
    def state(self):
        return self._state

    # @state.setter
    # def state(self, position=np.zeros((3,1)), velocity=np.zeros((3,1)), orientation=np.eye(3), ang_vel=np.zeros((3,1))):
    #     self._state['position']     = position
    #     self._state['velocity']     = velocity
    #     self._state['orientation']  = orientation
    #     self._state['ang_vel']      = ang_vel

    @property
    def position(self):
        return self._state['position']
    
    @position.setter
    def position(self, value):
        self._state['position'] = np.array(value)

    @property
    def velocity(self):
        return self._state['velocity']
    
    @velocity.setter
    def velocity(self, value):
        self._state['velocity'] = np.array(value)

    @property
    def orientation(self):
        return self._state['orientation']
    
    @orientation.setter
    def orientation(self, value):
        self._state['orientation'] = value
        # r = rotation.from_quat(value)
        # self._state['orientation'] = r.as_dcm()

    @property
    def ang_vel(self):
        return self._state['ang_vel']
    
    @ang_vel.setter
    def ang_vel(self, value):
        self._state['ang_vel'] = np.array(value)

    @property
    def des_position(self):
        return self._des_state['position']

    @des_position.setter
    def des_position(self, position):
        self._des_state['position']     = position

    def hat(self, vector):
        return np.array([[0., -vector[2], vector[1]], 
                            [vector[2], 0., -vector[0]],
                            [-vector[1], vector[0], 0.]])

    def vee(self, matrix):
        return np.array([matrix[2,1], matrix[0,2], matrix[1,0]])

    def geometric_pd(self, t=0):
        """
        geometric control for quadrotor on SE3
        """
        des_state = self.get_desired_traj(t)

        # position control
        ex = self._state['position']-des_state['xQ']
        ev = self._state['velocity']-des_state['vQ']
        Fpd = -self._gains['position']*ex  - self._gains['velocity']*ev
        Fff = self._mass*(self._g*self._e3+des_state['aQ'])
        thrust_force = Fpd + Fff
        norm_thrust = np.linalg.norm(thrust_force)

        # computing command attitude
        b1d = self._e1
        b3 = thrust_force/norm_thrust
        b3_b1d = np.cross(b3, b1d)
        norm_b3_b1d = np.linalg.norm(b3_b1d)
        b1 = (-1/norm_b3_b1d)*np.cross(b3, b3_b1d)
        b2 = np.cross(b3, b1)
        Rd = np.hstack([np.expand_dims(b1,axis=1), np.expand_dims(b2,axis=1), np.expand_dims(b3,axis=1)])

        R = self._state['orientation']
        Omega = self._state['ang_vel']
        Omegad = des_state['Omega']
        dOmegad = des_state['dOmega']

        # attitude control
        tmp = 0.5*(np.matmul(Rd.T,R)-np.matmul(R.T,Rd))
        eR  = np.array([tmp[2,1], tmp[0,2], tmp[1,0]]) # vee-map
        eOmega   = Omega - np.matmul(np.matmul(R.T, Rd), Omegad)

        M = -self._gains['orientation']*eR - self._gains['ang_vel']*eOmega + np.cross(Omega, self._inertia*Omega)
        M += -1*self._inertia_matrix.dot(np.matmul(self.hat(Omega), np.matmul(R.T, np.matmul(Rd, Omegad)))-np.matmul(R.T, np.matmul(Rd, dOmegad))) # ignoring this for since Omegad is zero
        f = thrust_force.dot(R[:,2])

        # f, M = des_state['f'], des_state['M']

        # compute lyapunov function
        eta1, epsilon1, c1 = 2.5, 2, 6
        V1 = 0.5*ev.dot(ev) + epsilon1*ex.dot(ev) + 0.5*c1*ex.dot(ex)
        
        eta2, epsilon2, c2 = 150, 4, 20
        V2 = 0.5*eOmega.dot(np.matmul(self.J_scale, eOmega)) + epsilon2*eR.dot(eOmega) + 0.5*c2*eR.dot(eR)

        return (f, M), (V1, V2)