from trajectory_generation.utils import r3
import cvxpy
import numpy as np
class Trajectory:
    def __init__(self, t_opt, t_segments, patterns, config, start_waypoint, end_waypoint):
        self._t_opt = t_opt
        self._t_segments = t_segments
        self._acceleration_segments = []
        self._start_waypoint = start_waypoint
        self._end_waypoint = end_waypoint
        self._config = config
        if patterns != None:
            for index, elem_for_axis in enumerate(patterns):
                if elem_for_axis == '(+a, 0, -a)':
                    self._acceleration_segments.append([config[3+index], 0, -config[3+index]])
                elif elem_for_axis == '(-a, 0, +a)':
                    self._acceleration_segments.append([-config[3 + index], 0, config[3 + index]])
                elif elem_for_axis == '(+a, 0, +a)':
                    self._acceleration_segments.append([config[3 + index], 0, config[3 + index]])
                elif elem_for_axis == '(-a, 0, -a)':
                    self._acceleration_segments.append([-config[3 + index], 0, -config[3 + index]])

    def get_optimal_trajectory_duration(self):
        return self._t_opt

    def get_traj(self, dt, step_size_output):
        if self._t_segments == None:
            v_max = self._config[0]
            a_max = self._config[3]
            nx = 2  # number of state
            nu = 1  # number of input
            mpc = MPC(a_max, -a_max, v_max, -v_max, nx, nu, dt)
            p_x, v_x, a_x = mpc.mpc_control(np.array([[self._start_waypoint.px], [self._start_waypoint.vx]]),
                                            np.array([[self._end_waypoint.px], [self._end_waypoint.vx]]), self._t_opt)
            p_y, v_y, a_y = mpc.mpc_control(np.array([[self._start_waypoint.py], [self._start_waypoint.vy]]),
                                            np.array([[self._end_waypoint.py], [self._end_waypoint.vy]]), self._t_opt)
            p_z, v_z, a_z = mpc.mpc_control(np.array([[self._start_waypoint.pz], [self._start_waypoint.vz]]),
                                            np.array([[self._end_waypoint.pz], [self._end_waypoint.vz]]), self._t_opt)
            tVec = np.arange(0, self._t_opt, dt)
            min_len = min(len(a_x), len(a_y), len(a_z), len(tVec))
            return tVec[:min_len:step_size_output], \
                   p_x[:min_len:step_size_output], v_x[:min_len:step_size_output], a_x[:min_len:step_size_output], \
                   p_y[:min_len:step_size_output], v_y[:min_len:step_size_output], a_y[:min_len:step_size_output], \
                   p_z[:min_len:step_size_output], v_z[:min_len:step_size_output], a_z[:min_len:step_size_output]

        # x-axis
        t_x = 0
        t_seg_x = [r3(elem) for elem in self._t_segments[0]]
        tVec_x = []
        p_x = []
        v_x = []
        a_x = []

        p_0_x = self._start_waypoint.px
        v_0_x = self._start_waypoint.vx
        t_ref_x = 0
        while t_x <= t_seg_x[0]:
            tVec_x.append(t_x)
            p_x.append(p_0_x + (t_x - t_ref_x) * v_0_x + 0.5 * (t_x - t_ref_x) ** 2 * self._acceleration_segments[0][0])
            v_x.append(v_0_x + (t_x - t_ref_x) * self._acceleration_segments[0][0])
            a_x.append(self._acceleration_segments[0][0])
            t_x = t_x + dt

        v_1_x = v_x[-1]
        p_1_x = p_x[-1]
        t_ref_x = tVec_x[-1]
        while t_x - t_ref_x > 0 and t_x - t_ref_x <= t_seg_x[1]:
            tVec_x.append(t_x)
            p_x.append(p_1_x + (t_x - t_ref_x) * v_1_x)
            v_x.append(v_1_x)
            a_x.append(self._acceleration_segments[0][1])

            t_x = t_x + dt

        v_2_x = v_x[-1]
        p_2_x = p_x[-1]
        t_ref_x = tVec_x[-1]
        while t_x - t_ref_x > 0 and t_x - t_ref_x <= t_seg_x[2]:
            tVec_x.append(t_x)
            p_x.append(p_2_x + (t_x - t_ref_x) * v_2_x + 0.5 * (t_x - t_ref_x) ** 2 * self._acceleration_segments[0][2])
            v_x.append(v_2_x + (t_x - t_ref_x) * self._acceleration_segments[0][2])
            a_x.append(self._acceleration_segments[0][2])

            t_x = t_x + dt

        # y-axis
        t_y = 0
        t_seg_y = [r3(elem) for elem in self._t_segments[1]]
        tVec_y = []
        p_y = []
        v_y = []
        a_y = []

        p_0_y = self._start_waypoint.py
        v_0_y = self._start_waypoint.vy
        t_ref_y = 0
        while t_y <= t_seg_y[0]:
            tVec_y.append(t_y)
            p_y.append(p_0_y + (t_y - t_ref_y) * v_0_y + 0.5 * (t_y - t_ref_y) ** 2 * self._acceleration_segments[1][0])
            v_y.append(v_0_y + (t_y - t_ref_y) * self._acceleration_segments[1][0])
            a_y.append(self._acceleration_segments[1][0])
            t_y = t_y + dt

        v_1_y = v_y[-1]
        p_1_y = p_y[-1]
        t_ref_y = tVec_y[-1]
        while t_y - t_ref_y > 0 and t_y - t_ref_y <= t_seg_y[1]:
            tVec_y.append(t_y)
            p_y.append(p_1_y + (t_y - t_ref_y) * v_1_y)
            v_y.append(v_1_y)
            a_y.append(self._acceleration_segments[1][1])

            t_y = t_y + dt

        v_2_y = v_y[-1]
        p_2_y = p_y[-1]
        t_ref_y = tVec_y[-1]
        while t_y - t_ref_y > 0 and t_y - t_ref_y <= t_seg_y[2]:
            tVec_y.append(t_y)
            p_y.append(p_2_y + (t_y - t_ref_y) * v_2_y + 0.5 * (t_y - t_ref_y) ** 2 * self._acceleration_segments[1][2])
            v_y.append(v_2_y + (t_y - t_ref_y) * self._acceleration_segments[1][2])
            a_y.append(self._acceleration_segments[1][2])

            t_y = t_y + dt

        # z-axis
        t_z = 0
        t_seg_z = [r3(elem) for elem in self._t_segments[2]]
        tVec_z = []
        p_z = []
        v_z = []
        a_z = []

        p_0_z = self._start_waypoint.pz
        v_0_z = self._start_waypoint.vz
        t_ref_z = 0
        while t_z <= t_seg_z[0]:
            tVec_z.append(t_z)
            p_z.append(p_0_z + (t_z - t_ref_z) * v_0_z + 0.5 * (t_z - t_ref_z) ** 2 * self._acceleration_segments[2][0])
            v_z.append(v_0_z + (t_z - t_ref_z) * self._acceleration_segments[2][0])
            a_z.append(self._acceleration_segments[2][0])
            t_z = t_z + dt

        v_1_z = v_z[-1]
        p_1_z = p_z[-1]
        t_ref_z = tVec_z[-1]
        while t_z - t_ref_z > 0 and t_z - t_ref_z <= t_seg_z[1]:
            tVec_z.append(t_z)
            p_z.append(p_1_z + (t_z - t_ref_z) * v_1_z)
            v_z.append(v_1_z)
            a_z.append(self._acceleration_segments[2][1])

            t_z = t_z + dt

        v_2_z = v_z[-1]
        p_2_z = p_z[-1]
        t_ref_z = tVec_z[-1]
        while t_z - t_ref_z > 0 and t_z - t_ref_z <= t_seg_z[2]:
            tVec_z.append(t_z)
            p_z.append(p_2_z + (t_z - t_ref_z) * v_2_z + 0.5 * (t_z - t_ref_z) ** 2 * self._acceleration_segments[2][2])
            v_z.append(v_2_z + (t_z - t_ref_z) * self._acceleration_segments[2][2])
            a_z.append(self._acceleration_segments[2][2])

            t_z = t_z + dt

        min_len = min(len(tVec_x), len(tVec_y), len(tVec_z))
        return tVec_x[:min_len:step_size_output], \
               p_x[:min_len:step_size_output], v_x[:min_len:step_size_output], a_x[:min_len:step_size_output], \
               p_y[:min_len:step_size_output], v_y[:min_len:step_size_output], a_y[:min_len:step_size_output], \
               p_z[:min_len:step_size_output], v_z[:min_len:step_size_output], a_z[:min_len:step_size_output]



class MPC:
    def __init__(self, amax, amin, vmax, vmin, nx, nu, delta_t):
        self.amax = amax
        self.amin = amin
        self.vmax = vmax
        self.vmin = vmin
        self.nx = nx
        self.nu = nu
        self.delta_t = delta_t

        self.Q = np.diag([1000.0, 1000.0])
        self.R = np.diag([0.001])

    def mpc_control(self, x0, xr, T):
        x = cvxpy.Variable((self.nx, int(T / self.delta_t) + 1))
        u = cvxpy.Variable((self.nu, int(T / self.delta_t)))

        A, B = self.get_model_matrix()

        cost = 0.0
        cost += cvxpy.quad_form(x[:,-1] - xr[:, 0], self.Q)
        constr = []
        for t in range(int(T / self.delta_t)):
            # cost += cvxpy.quad_form(x[:, t + 1] - xr[:, 0], self.Q)
            cost += cvxpy.quad_form(u[:, t], self.R)
            constr += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t]]

            constr += [x[1, t] <= self.vmax]
            constr += [x[1, t] >= self.vmin]

            constr += [u[:, t] <= self.amax]
            constr += [u[:, t] >= self.amin]

        # print(x0)
        constr += [x[:, 0] == x0[:, 0]]
        # constr += [x[:, int(T/delta_t)] == xr[:, 0]]
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constr)

        # start = time.time()
        prob.solve(verbose=False)
        # elapsed_time = time.time() - start
        # print("calc time:{0} [sec]".format(elapsed_time))

        if prob.status == cvxpy.OPTIMAL:
            x1 = self.get_nparray_from_matrix(x.value[0, :])
            x2 = self.get_nparray_from_matrix(x.value[1, :])

            ou = self.get_nparray_from_matrix(u.value[0, :])

        return x1, x2, ou

    def get_model_matrix(self):
        # Model Parameter
        A = np.array([
            [1.0, self.delta_t],
            [0, 1.0]
        ])

        B = np.array([[0.5 * self.delta_t ** 2],
                      [self.delta_t]
                      ])

        return A, B

    def flatten(self,a):
        return np.array(a).flatten()

    def get_nparray_from_matrix(self, x):
        """
        get build-in list from matrix
        """
        return np.array(x).flatten()