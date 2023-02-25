from math import sqrt

from trajectory_generation.Waypoint import Waypoint
from trajectory_generation.utils import r, r3
from trajectory_generation.Trajectory import Trajectory


class TrajectoryPlanner:
    def __init__(self, v_max: float, a_max: float, version='basic'):
        self.version = version
        if version == 'basic' or version == 'sota':
            self.configs = [
                (v_max / sqrt(3), v_max / sqrt(3), v_max / sqrt(3), a_max / sqrt(3), a_max / sqrt(3), a_max / sqrt(3))]
        elif version == 'improved':
            self.configs = [
                (v_max / sqrt(3), v_max / sqrt(3), v_max / sqrt(3), a_max / sqrt(3), a_max / sqrt(3), a_max / sqrt(3)),
                # undistorted
                (v_max * sqrt(3) / 2, v_max / sqrt(8), v_max / sqrt(8), a_max * sqrt(3) / 2, a_max / sqrt(8),
                 a_max / sqrt(8)),  # towards x
                (v_max / sqrt(8), v_max * sqrt(3) / 2, v_max / sqrt(8), a_max / sqrt(8), a_max * sqrt(3) / 2,
                 a_max / sqrt(8)),  # towards y
                (v_max / sqrt(8), v_max / sqrt(8), v_max * sqrt(3) / 2, a_max / sqrt(8), a_max / sqrt(8),
                 a_max * sqrt(3) / 2)  # towards z
            ]
        else:
            raise ValueError('Entered version not supported! Choose basic or improved..')

    def plan(self, x_s, y_s, z_s, x_e, y_e, z_e, vx_s, vy_s, vz_s, vx_e, vy_e, vz_e):
        wp_start = Waypoint(x_s, y_s, z_s, vx_s, vy_s, vz_s)
        wp_end = Waypoint(x_e, y_e, z_e, vx_e, vy_e, vz_e)

        return self.plan_wp(wp_start, wp_end)

    def plan_wp(self, waypoint1, waypoint2):
        t_opt_best = float('inf')
        t_x_best = None
        pattern_x_best = None
        t_y_best = None
        pattern_y_best = None
        t_z_best = None
        pattern_z_best = None
        config_best = None
        for config in self.configs:
            if self.check_inputs(waypoint1, waypoint2, config):
                t_tot_x = self.calc_opt_traj_time_axis(waypoint1, waypoint2, "X", config)
                t_tot_y = self.calc_opt_traj_time_axis(waypoint1, waypoint2, "Y", config)
                t_tot_z = self.calc_opt_traj_time_axis(waypoint1, waypoint2, "Z", config)
                t_LB = max(min(t_tot_x), min(t_tot_y), min(t_tot_z))

                if self.version == "sota":
                    return Trajectory(t_LB, None, None, config, waypoint1, waypoint2)
                sync_possible_x, t_x, pattern_x = self.check_synchronization_possible_axis(t_LB, waypoint1, waypoint2,
                                                                                           "X", config)
                sync_possible_y, t_y, pattern_y = self.check_synchronization_possible_axis(t_LB, waypoint1, waypoint2,
                                                                                           "Y", config)
                sync_possible_z, t_z, pattern_z = self.check_synchronization_possible_axis(t_LB, waypoint1, waypoint2,
                                                                                           "Z", config)
                if sync_possible_x and sync_possible_y and sync_possible_z and t_LB < t_opt_best:
                    t_opt_best = t_LB
                    t_x_best = t_x
                    pattern_x_best = pattern_x
                    t_y_best = t_y
                    pattern_y_best = pattern_y
                    t_z_best = t_z
                    pattern_z_best = pattern_z
                    config_best = config
                else:
                    t_sync_cand_list = self.get_t_sync_candidates(config, t_LB, waypoint1, waypoint2)

                    for t_sync_cand in t_sync_cand_list:
                        sync_possible_x, t_x, pattern_x = self.check_synchronization_possible_axis(t_sync_cand,
                                                                                                   waypoint1, waypoint2,
                                                                                                   "X", config)
                        sync_possible_y, t_y, pattern_y = self.check_synchronization_possible_axis(t_sync_cand,
                                                                                                   waypoint1, waypoint2,
                                                                                                   "Y", config)
                        sync_possible_z, t_z, pattern_z = self.check_synchronization_possible_axis(t_sync_cand,
                                                                                                   waypoint1, waypoint2,
                                                                                                   "Z", config)
                        if sync_possible_x and sync_possible_y and sync_possible_z and t_sync_cand < t_opt_best:
                            t_opt_best = t_sync_cand
                            t_x_best = t_x
                            pattern_x_best = pattern_x
                            t_y_best = t_y
                            pattern_y_best = pattern_y
                            t_z_best = t_z
                            pattern_z_best = pattern_z
                            config_best = config
                            break

        if t_opt_best < float('inf'):
            return Trajectory(t_opt_best,
                                [t_x_best, t_y_best, t_z_best],
                                [pattern_x_best, pattern_y_best, pattern_z_best],
                                config_best,
                                waypoint1,
                                waypoint2)
        else:
            print('sync time not found by optimality method')
            self.check_inputs(waypoint1, waypoint2, config, raise_error=True)

    def get_t_sync_candidates(self, config, t_LB, waypoint1, waypoint2):
        t_sync_cand_list = []
        ##### X ######
        t_sync_cand_list = t_sync_cand_list + self.sync_pattern_1(waypoint1.px, waypoint1.vx, waypoint2.px,
                                                                  waypoint2.vx, "X",
                                                                  config)
        t_sync_cand_list = t_sync_cand_list + self.sync_pattern_2(waypoint1.px, waypoint1.vx, waypoint2.px,
                                                                  waypoint2.vx, "X",
                                                                  config)
        t_sync_cand_list = t_sync_cand_list + self.sync_pattern_3(waypoint1.px, waypoint1.vx, waypoint2.px,
                                                                  waypoint2.vx, "X",
                                                                  config)
        t_sync_cand_list = t_sync_cand_list + self.sync_pattern_4(waypoint1.px, waypoint1.vx, waypoint2.px,
                                                                  waypoint2.vx, "X",
                                                                  config)
        ##### Y ######
        t_sync_cand_list = t_sync_cand_list + self.sync_pattern_1(waypoint1.py, waypoint1.vy, waypoint2.py,
                                                                  waypoint2.vy, "Y",
                                                                  config)
        t_sync_cand_list = t_sync_cand_list + self.sync_pattern_2(waypoint1.py, waypoint1.vy, waypoint2.py,
                                                                  waypoint2.vy, "Y",
                                                                  config)
        t_sync_cand_list = t_sync_cand_list + self.sync_pattern_3(waypoint1.py, waypoint1.vy, waypoint2.py,
                                                                  waypoint2.vy, "Y",
                                                                  config)
        t_sync_cand_list = t_sync_cand_list + self.sync_pattern_4(waypoint1.py, waypoint1.vy, waypoint2.py,
                                                                  waypoint2.vy, "Y",
                                                                  config)
        ##### Z ######
        t_sync_cand_list = t_sync_cand_list + self.sync_pattern_1(waypoint1.pz, waypoint1.vz, waypoint2.pz,
                                                                  waypoint2.vz, "Z", config)
        t_sync_cand_list = t_sync_cand_list + self.sync_pattern_2(waypoint1.pz, waypoint1.vz, waypoint2.pz,
                                                                  waypoint2.vz, "Z", config)
        t_sync_cand_list = t_sync_cand_list + self.sync_pattern_3(waypoint1.pz, waypoint1.vz, waypoint2.pz,
                                                                  waypoint2.vz, "Z", config)
        t_sync_cand_list = t_sync_cand_list + self.sync_pattern_4(waypoint1.pz, waypoint1.vz, waypoint2.pz,
                                                                  waypoint2.vz, "Z", config)
        t_sync_cand_list = sorted([elem for elem in t_sync_cand_list if elem >= t_LB])
        return t_sync_cand_list

    def check_inputs(self, waypoint1, waypoint2, config, raise_error=False):
        v_max_x = config[0]
        v_max_y = config[1]
        v_max_z = config[2]

        inputs_feasible = True

        ############ X ###############
        if waypoint1.vx > v_max_x:
            inputs_feasible = False
            if raise_error:
                raise ValueError('waypoint1.vx higher than maximum allowed velocity')
        if waypoint1.vx < -v_max_x:
            inputs_feasible = False
            if raise_error:
                raise ValueError('waypoint1.vx lower than minimum allowed velocity')
        if waypoint2.vx > v_max_x:
            inputs_feasible = False
            if raise_error:
                raise ValueError('waypoint2.vx higher than maximum allowed velocity')
        if waypoint2.vx < -v_max_x:
            inputs_feasible = False
            if raise_error:
                raise ValueError('waypoint2.vx lower than minimum allowed velocity')

        ############ Y ###############
        if waypoint1.vy > v_max_y:
            inputs_feasible = False
            if raise_error:
                raise ValueError('waypoint1.vy higher than maximum allowed velocity')
        if waypoint1.vy < -v_max_y:
            inputs_feasible = False
            if raise_error:
                raise ValueError('waypoint1.vy lower than minimum allowed velocity')
        if waypoint2.vy > v_max_y:
            inputs_feasible = False
            if raise_error:
                raise ValueError('waypoint2.vy higher than maximum allowed velocity')
        if waypoint2.vy < -v_max_y:
            inputs_feasible = False
            if raise_error:
                raise ValueError('waypoint2.vy lower than minimum allowed velocity')

        ############ Z ###############
        if waypoint1.vz > v_max_z:
            inputs_feasible = False
            if raise_error:
                raise ValueError('waypoint1.vz higher than maximum allowed velocity')
        if waypoint1.vz < -v_max_z:
            inputs_feasible = False
            if raise_error:
                raise ValueError('waypoint1.vz lower than minimum allowed velocity')
        if waypoint2.vz > v_max_z:
            inputs_feasible = False
            if raise_error:
                raise ValueError('waypoint2.vz higher than maximum allowed velocity')
        if waypoint2.vz < -v_max_z:
            inputs_feasible = False
            if raise_error:
                raise ValueError('waypoint2.vz lower than minimum allowed velocity')
        return inputs_feasible

    def calc_opt_traj_time_axis(self, waypoint1: Waypoint, waypoint2: Waypoint, type, config):
        if type == "X":
            p_s = waypoint1.px
            v_s = waypoint1.vx
            p_e = waypoint2.px
            v_e = waypoint2.vx
            v_max = config[0]  # v_max_x
            a_max = config[3]  # a_max_x

        if type == "Y":
            p_s = waypoint1.py
            v_s = waypoint1.vy
            p_e = waypoint2.py
            v_e = waypoint2.vy
            v_max = config[1]  # v_max_y
            a_max = config[4]  # a_max_y

        if type == "Z":
            p_s = waypoint1.pz
            v_s = waypoint1.vz
            p_e = waypoint2.pz
            v_e = waypoint2.vz
            v_max = config[2]  # v_max_z
            a_max = config[5]  # a_max_z

        t_tot = []

        valid, t_seg = self._case_one(p_s, v_s, p_e, v_e, v_max, a_max)
        if valid:
            t_tot.append(sum(t_seg))

        valid, t_seg = self._case_two(p_s, v_s, p_e, v_e, v_max, a_max)
        if valid:
            t_tot.append(sum(t_seg))

        valid, t_seg = self._case_one_mirrored(p_s, v_s, p_e, v_e, v_max, a_max)
        if valid:
            t_tot.append(sum(t_seg))

        valid, t_seg = self._case_two_mirrored(p_s, v_s, p_e, v_e, v_max, a_max)
        if valid:
            t_tot.append(sum(t_seg))

        if len(t_tot) == 0:
            print("No trajectory found", 'red')
            t_tot = [999999]
        return t_tot

    def _case_one(self, p_s, v_s, p_e, v_e, v_max, a_max):
        #############################
        # control input (+a_max, 0, -a_max) #
        #############################
        v_max = v_max
        v_min = -v_max
        a_max = a_max
        a_min = -a_max

        valid = True
        t = []
        try:
            t.append((v_max - 0.1e1 * v_s) / a_max)
            t.append(0.5000000000e0 * (
                    0.2e1 * p_e * a_max * a_min - 0.2e1 * p_s * a_max * a_min - 0.1e1 * a_max * v_e ** 2 + a_max * v_max ** 2 - 0.1e1 * v_max ** 2 * a_min + v_s ** 2 * a_min) / a_max / a_min / v_max)
            t.append((v_e - 0.1e1 * v_max) / a_min)
            for elem in t:
                if elem < -0.000001:
                    valid = False

            v = []
            v.append(v_s)
            v.append(v_max)
            v.append(v_max)
            v.append(v_e)
            for elem in v:
                if elem < v_min or elem > v_max:
                    valid = False
        except:
            valid = False

        return valid, t

    def _case_one_mirrored(self, p_s, v_s, p_e, v_e, v_max, a_max):
        #############################
        # control input (-a_max, 0, +a_max) #
        #############################
        v_max = v_max
        v_min = -v_max
        a_max = a_max
        a_min = -a_max

        valid = True
        t = []
        try:
            t.append((v_min - 0.1e1 * v_s) / a_min)
            t.append(0.5000000000e0 * (
                    0.2e1 * p_e * a_max * a_min - 0.2e1 * p_s * a_max * a_min - 0.1e1 * v_min ** 2 * a_max + v_s ** 2 * a_max - 0.1e1 * a_min * v_e ** 2 + a_min * v_min ** 2) / a_max / a_min / v_min)
            t.append((v_e - 0.1e1 * v_min) / a_max)
            for elem in t:
                if elem < -0.000001:
                    valid = False

            v = []
            v.append(v_s)
            v.append(v_min)
            v.append(v_min)
            v.append(v_e)

            for elem in v:
                if elem < v_min or elem > v_max:
                    valid = False
        except:
            valid = False

        return valid, t

    def _case_two(self, p_s, v_s, p_e, v_e, v_max, a_max):
        ##################################
        # control input (+a_max, -a_max) #
        ##################################

        v_max = v_max
        v_min = -v_max
        a_max = a_max
        a_min = -a_max

        valid = True
        t = []
        try:
            t.append(-0.1e1 * ((a_max * v_e - 0.1e1 * a_min * v_e - sqrt(
                -0.2e1 * a_max ** 2 * a_min * p_e + 0.2e1 * a_max ** 2 * a_min * p_s + a_max ** 2 * v_e ** 2 + 0.2e1 * a_max * a_min ** 2 * p_e - 0.2e1 * a_max * a_min ** 2 * p_s - 0.1e1 * a_max * a_min * v_e ** 2 - 0.1e1 * a_max * a_min * v_s ** 2 + a_min ** 2 * v_s ** 2)) / (
                                       a_max - 0.1e1 * a_min) - 0.1e1 * v_e + v_s) / a_max)

            t.append(0.0e0)
            t.append((a_max * v_e - 0.1e1 * a_min * v_e - sqrt(
                -0.2e1 * a_max ** 2 * a_min * p_e + 0.2e1 * a_max ** 2 * a_min * p_s + a_max ** 2 * v_e ** 2 + 0.2e1 * a_max * a_min ** 2 * p_e - 0.2e1 * a_max * a_min ** 2 * p_s - 0.1e1 * a_max * a_min * v_e ** 2 - 0.1e1 * a_max * a_min * v_s ** 2 + a_min ** 2 * v_s ** 2)) / a_min / (
                             a_max - 0.1e1 * a_min))

            for elem in t:
                if elem < -0.000001:
                    valid = False

            v = []
            v.append(v_s)
            v.append(-0.1e1 * (a_max * v_e - 0.1e1 * a_min * v_e + sqrt(
                -0.2e1 * a_max ** 2 * a_min * p_e + 0.2e1 * a_max ** 2 * a_min * p_s + a_max ** 2 * v_e ** 2 + 0.2e1 * a_max * a_min ** 2 * p_e - 0.2e1 * a_max * a_min ** 2 * p_s - 0.1e1 * a_max * a_min * v_e ** 2 - 0.1e1 * a_max * a_min * v_s ** 2 + a_min ** 2 * v_s ** 2)) / (
                             a_max - 0.1e1 * a_min) + v_e)
            v.append(-0.1e1 * (a_max * v_e - 0.1e1 * a_min * v_e + sqrt(
                -0.2e1 * a_max ** 2 * a_min * p_e + 0.2e1 * a_max ** 2 * a_min * p_s + a_max ** 2 * v_e ** 2 + 0.2e1 * a_max * a_min ** 2 * p_e - 0.2e1 * a_max * a_min ** 2 * p_s - 0.1e1 * a_max * a_min * v_e ** 2 - 0.1e1 * a_max * a_min * v_s ** 2 + a_min ** 2 * v_s ** 2)) / (
                             a_max - 0.1e1 * a_min) + v_e)
            v.append(v_e)
            for elem in v:
                if elem < v_min or elem > v_max:
                    valid = False
        except:
            valid = False

        return valid, t

    def _case_two_mirrored(self, p_s, v_s, p_e, v_e, v_max, a_max):
        ##################################
        # control input (-a_max, +a_max) #
        ##################################
        v_max = v_max
        v_min = -v_max
        a_max = a_max
        a_min = -a_max

        valid = True
        t = []
        try:
            t.append(-0.1e1 * ((a_max * v_e - 0.1e1 * a_min * v_e + sqrt(
                0.2e1 * a_max ** 2 * a_min * p_e - 0.2e1 * a_max ** 2 * a_min * p_s + a_max ** 2 * v_s ** 2 - 0.2e1 * a_max * a_min ** 2 * p_e + 0.2e1 * a_max * a_min ** 2 * p_s - 0.1e1 * a_max * a_min * v_e ** 2 - 0.1e1 * a_max * a_min * v_s ** 2 + a_min ** 2 * v_e ** 2)) / (
                                       a_max - 0.1e1 * a_min) - 0.1e1 * v_e + v_s) / a_min)
            t.append(0.0e0)
            t.append((a_max * v_e - 0.1e1 * a_min * v_e + sqrt(
                0.2e1 * a_max ** 2 * a_min * p_e - 0.2e1 * a_max ** 2 * a_min * p_s + a_max ** 2 * v_s ** 2 - 0.2e1 * a_max * a_min ** 2 * p_e + 0.2e1 * a_max * a_min ** 2 * p_s - 0.1e1 * a_max * a_min * v_e ** 2 - 0.1e1 * a_max * a_min * v_s ** 2 + a_min ** 2 * v_e ** 2)) / a_max / (
                             a_max - 0.1e1 * a_min))
            for elem in t:
                if elem < -0.000001:
                    valid = False

            v = []
            v.append(v_s)
            v.append(-0.1e1 * (a_max * v_e - 0.1e1 * a_min * v_e + sqrt(
                0.2e1 * a_max ** 2 * a_min * p_e - 0.2e1 * a_max ** 2 * a_min * p_s + a_max ** 2 * v_s ** 2 - 0.2e1 * a_max * a_min ** 2 * p_e + 0.2e1 * a_max * a_min ** 2 * p_s - 0.1e1 * a_max * a_min * v_e ** 2 - 0.1e1 * a_max * a_min * v_s ** 2 + a_min ** 2 * v_e ** 2)) / (
                             a_max - 0.1e1 * a_min) + v_e)
            v.append(-0.1e1 * (a_max * v_e - 0.1e1 * a_min * v_e + sqrt(
                0.2e1 * a_max ** 2 * a_min * p_e - 0.2e1 * a_max ** 2 * a_min * p_s + a_max ** 2 * v_s ** 2 - 0.2e1 * a_max * a_min ** 2 * p_e + 0.2e1 * a_max * a_min ** 2 * p_s - 0.1e1 * a_max * a_min * v_e ** 2 - 0.1e1 * a_max * a_min * v_s ** 2 + a_min ** 2 * v_e ** 2)) / (
                             a_max - 0.1e1 * a_min) + v_e)
            v.append(v_e)
            for elem in v:
                if elem < v_min or elem > v_max:
                    valid = False
        except:
            valid = False

        return valid, t

    def check_synchronization_possible_axis(self, t_end: float, waypoint1: Waypoint, waypoint2: Waypoint, type, config):
        if type == "X":
            p_s = waypoint1.px
            v_s = waypoint1.vx
            p_e = waypoint2.px
            v_e = waypoint2.vx
            v_max = config[0]  # v_max_x
            v_min = -v_max
            a_max = config[3]  # a_max_x
            a_min = -a_max

        if type == "Y":
            p_s = waypoint1.py
            v_s = waypoint1.vy
            p_e = waypoint2.py
            v_e = waypoint2.vy
            v_max = config[1]  # v_max_y
            v_min = -v_max
            a_max = config[4]  # a_max_y
            a_min = -a_max

        if type == "Z":
            p_s = waypoint1.pz
            v_s = waypoint1.vz
            p_e = waypoint2.pz
            v_e = waypoint2.vz
            v_max = config[2]  # v_max_z
            v_min = -v_max
            a_max = config[5]  # a_max_z
            a_min = -a_max

        sync_valid = False

        ########################
        # PATTERN 1: (-a,0,+a) #
        ########################
        a = a_min
        A = r(a ** 2 * t_end ** 2 + 2 * (v_e + v_s) * a * t_end - 4 * p_e * a + 4 * p_s * a - (v_e - v_s) ** 2)
        if A >= 0:
            t_1 = (a * t_end + v_e - v_s + sqrt(A)) / (2 * a)
            t_2 = (-sqrt(A)) / a
            t_3 = (a * t_end - v_e + v_s + sqrt(A)) / (2 * a)
            v_k = (a * t_end + v_e + v_s + sqrt(A)) / 2

            if t_1 > -0.0001 and t_2 > -0.0001 and t_3 > -0.0001 and v_k - v_min > -0.0001 and v_k - v_max < 0.0001:
                sync_valid = True
                return sync_valid, [t_1, t_2, t_3], "(-a, 0, +a)"

        ########################
        # PATTERN 2: (+a,0,-a) #
        ########################
        a = a_max
        A = r(a ** 2 * t_end ** 2 + 2 * (v_e + v_s) * a * t_end - 4 * p_e * a + 4 * p_s * a - (v_e - v_s) ** 2)
        if A >= 0:
            t_1 = (a * t_end + v_e - v_s - sqrt(A)) / (2 * a)
            t_2 = +sqrt(A) / a
            t_3 = (a * t_end - v_e + v_s - sqrt(A)) / (2 * a)
            v_k = (a * t_end + v_e + v_s - sqrt(A)) / 2

            if t_1 > -0.0001 and t_2 > -0.0001 and t_3 > -0.0001 and v_k - v_min > -0.0001 and v_k - v_max < 0.0001:
                sync_valid = True
                return sync_valid, [t_1, t_2, t_3], "(+a, 0, -a)"

        ########################
        # PATTERN 3: (+a,0,+a) #
        ########################
        a = a_max
        if t_end * a - v_e + v_s != 0:
            t_1 = ((-2 * v_s * t_end + 2 * p_e - 2 * p_s) * a - (v_e - v_s) ** 2) / (2 * a * (t_end * a - v_e + v_s))
            t_2 = (t_end * a - v_e + v_s) / a
            t_3 = ((2 * v_e * t_end - 2 * p_e + 2 * p_s) * a - (v_e - v_s) ** 2) / (2 * a * (t_end * a - v_e + v_s))
            v_k = v_s + a * t_1
            if t_1 > -0.0001 and t_2 > -0.0001 and t_3 > -0.0001 and v_k - v_min > -0.0001 and v_k - v_max < 0.0001:
                sync_valid = True
                return sync_valid, [t_1, t_2, t_3], "(+a, 0, +a)"

        ########################
        # PATTERN 4: (-a,0,-a) #
        ########################
        a = a_min
        if t_end * a - v_e + v_s != 0:
            t_1 = ((-2 * v_s * t_end + 2 * p_e - 2 * p_s) * a - (v_e - v_s) ** 2) / (2 * a * (t_end * a - v_e + v_s))
            t_2 = (t_end * a - v_e + v_s) / a
            t_3 = ((2 * v_e * t_end - 2 * p_e + 2 * p_s) * a - (v_e - v_s) ** 2) / (2 * a * (t_end * a - v_e + v_s))
            v_k = v_s + a * t_1
            if t_1 > 0.0001 and t_2 > 0.0001 and t_3 > -0.0001 and v_k - v_min > -0.0001 and v_k - v_max < 0.0001:
                sync_valid = True
                return sync_valid, [t_1, t_2, t_3], "(-a, 0, -a)"

        return sync_valid, [-1, -1, -1], ""

    def sync_pattern_1(self, p_s, v_s, p_e, v_e, type, config):
        if type == "X":
            v_max = config[0]
            a_max = config[3]

        if type == "Y":
            v_max = config[2]
            a_max = config[4]

        if type == "Z":
            v_max = config[3]
            a_max = config[5]

        #########################
        # PATTERN 1 (+a, 0, -a) #
        #########################

        a = a_max

        # t1(te) == 0
        if v_s == 0:
            te_1 = 0
        else:
            te_1 = (v_s ** 2 - 2 * v_e * v_s + (2 * p_e - 2 * p_s) * a + v_e ** 2) / (2 * a * v_s)

        # t2(te) == 0
        if 4 * a * p_e - 4 * a * p_s + 2 * v_e ** 2 + 2 * v_s ** 2 > 0:
            te_2_a = (-v_e - v_s + sqrt(4 * a * p_e - 4 * a * p_s + 2 * v_e ** 2 + 2 * v_s ** 2)) / a
            te_2_b = (-v_e - v_s - sqrt(4 * a * p_e - 4 * a * p_s + 2 * v_e ** 2 + 2 * v_s ** 2)) / a
        else:
            # t2(te) > 0, forall te
            te_2_a = float('-inf')
            te_2_b = float('-inf')

        # t3(te) == 0
        if v_e == 0:
            te_3 = 0
        else:
            te_3 = (2 * a * p_e - 2 * a * p_s + v_e ** 2 - 2 * v_e * v_s + v_s ** 2) / (2 * a * v_e)

        # v_k(te) == v_max
        te_v = (
                       2 * a * p_e - 2 * a * p_s + v_e ** 2 - 2 * v_e * v_max + 2 * v_max ** 2 - 2 * v_max * v_s + v_s ** 2) / (
                       2 * a * v_max)
        out = [te_1, te_2_a, te_2_b, te_3, te_v]
        return out

    def sync_pattern_2(self, p_s, v_s, p_e, v_e, type, config):
        if type == "X":
            v_max = config[0]
            v_min = v_max
            a_max = config[3]
            a_min = -a_max

        if type == "Y":
            v_max = config[2]
            v_min = v_max
            a_max = config[4]
            a_min = -a_max

        if type == "Z":
            v_max = config[3]
            v_min = v_max
            a_max = config[5]
            a_min = -a_max

        #########################
        # PATTERN 2 (-a, 0, +a) #
        #########################

        a = a_min

        # t1(te) == 0
        if v_s == 0:
            te_1 = 0
        else:
            te_1 = (v_s ** 2 - 2 * v_e * v_s + (2 * p_e - 2 * p_s) * a + v_e ** 2) / (2 * a * v_s)

        # t2(te) == 0
        if 4 * a * p_e - 4 * a * p_s + 2 * v_e ** 2 + 2 * v_s ** 2 > 0:
            te_2_a = (-v_e - v_s + sqrt(4 * a * p_e - 4 * a * p_s + 2 * v_e ** 2 + 2 * v_s ** 2)) / a
            te_2_b = (-v_e - v_s - sqrt(4 * a * p_e - 4 * a * p_s + 2 * v_e ** 2 + 2 * v_s ** 2)) / a
        else:
            # t2(te) > 0, forall te
            te_2_a = float('-inf')
            te_2_b = float('-inf')

        # t3(te) == 0
        if v_e == 0:
            te_3 = 0
        else:
            te_3 = (2 * a * p_e - 2 * a * p_s + v_e ** 2 - 2 * v_e * v_s + v_s ** 2) / (2 * a * v_e)

        # v_k(te) == v_min
        te_v = (
                           2 * a * p_e - 2 * a * p_s + v_e ** 2 - 2 * v_e * v_min + 2 * v_min ** 2 - 2 * v_min * v_s + v_s ** 2) / (
                           2 * a * v_min)

        out = [te_1, te_2_a, te_2_b, te_3, te_v]
        return out

    def sync_pattern_3(self, p_s, v_s, p_e, v_e, type, config):
        if type == "X":
            a_max = config[3]

        if type == "Y":
            a_max = config[4]

        if type == "Z":
            a_max = config[5]

        a = a_max
        if v_s == 0:
            te_1 = 0
        else:
            te_1 = (2 * a * p_e - 2 * a * p_s - v_e ** 2 + 2 * v_e * v_s - v_s ** 2) / (2 * a * v_s)
        te_2 = (v_e - v_s) / a

        if v_e == 0:
            te_3 = 0
        else:
            te_3 = (2 * a * p_e - 2 * a * p_s + v_e ** 2 - 2 * v_e * v_s + v_s ** 2) / (2 * a * v_e)

        out = [te_1, te_2, te_3]
        return out

    def sync_pattern_4(self, p_s, v_s, p_e, v_e, type, config):
        if type == "X":
            a_max = config[3]
            a_min = -a_max

        if type == "Y":
            a_max = config[4]
            a_min = -a_max

        if type == "Z":
            a_max = config[5]
            a_min = -a_max

        a = a_min
        if v_s == 0:
            te_1 = 0
        else:
            te_1 = (2 * a * p_e - 2 * a * p_s - v_e ** 2 + 2 * v_e * v_s - v_s ** 2) / (2 * a * v_s)
        te_2 = (v_e - v_s) / a

        if v_e == 0:
            te_3 = 0
        else:
            te_3 = (2 * a * p_e - 2 * a * p_s + v_e ** 2 - 2 * v_e * v_s + v_s ** 2) / (2 * a * v_e)

        out = [te_1, te_2, te_3]
        return out
