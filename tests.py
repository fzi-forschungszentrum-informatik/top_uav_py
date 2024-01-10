import time
import matplotlib.pyplot as plt
from trajectory_generation.TrajectoryPlanner import TrajectoryPlanner

v_max = 4  # in m/s
a_max = 1  # in m/s^2

px_s = 0.1  # start position x
py_s = 2.0  # start position y
pz_s = 4.3  # start position z
vx_s = 0.1  # start velocity x
vy_s = -1.9  # start velocity y
vz_s = -0.4  # start velocity z

px_e = 3.6  # end position x
py_e = 0.4  # end position y
pz_e = 2.6  # end position z
vx_e = 0.1  # end velocity x
vy_e = -1.8  # end velocity y
vz_e = 0.6  # end velocity z

def test_sota():
    traj_planner_sota = TrajectoryPlanner(v_max, a_max, 'sota')
    assert round(traj_planner_sota.plan(px_s, py_s, pz_s, px_e, py_e, pz_e, vx_s, vy_s, vz_s, vx_e, vy_e,
                                        vz_e).get_optimal_trajectory_duration() * 10000) / 10000 == 4.5901

def test_top_uav():
    traj_planner_basic = TrajectoryPlanner(v_max, a_max, 'basic')
    assert round(traj_planner_basic.plan(px_s, py_s, pz_s, px_e, py_e, pz_e, vx_s, vy_s, vz_s, vx_e, vy_e,
                                        vz_e).get_optimal_trajectory_duration() * 10000) / 10000 == 11.8872

def test_top_uav_pp():
    traj_planner_improved = TrajectoryPlanner(v_max, a_max, 'improved')
    assert round(traj_planner_improved.plan(px_s, py_s, pz_s, px_e, py_e, pz_e, vx_s, vy_s, vz_s, vx_e, vy_e,
                                         vz_e).get_optimal_trajectory_duration() * 10000) / 10000 == 7.5704



