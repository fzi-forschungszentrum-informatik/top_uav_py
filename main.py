import time
import matplotlib.pyplot as plt
from trajectory_generation.TrajectoryPlanner import TrajectoryPlanner

v_max = 4  # in m/s
a_max = 1  # in m/s^2

traj_planner_sota = TrajectoryPlanner(v_max, a_max, 'sota')
traj_planner_basic = TrajectoryPlanner(v_max, a_max, 'basic')
traj_planner_improved = TrajectoryPlanner(v_max, a_max, 'improved')

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

#################################
# CALCULATE OPTIMAL TRAJECTORYS #
#################################
start_sota = time.time()
traj_sota = traj_planner_sota.plan(px_s, py_s, pz_s, px_e, py_e, pz_e, vx_s, vy_s, vz_s, vx_e, vy_e, vz_e)
end_sota = time.time()
print('Optimal trajectory duration (sota): ', traj_sota.get_optimal_trajectory_duration(), 'Calculation time (s): ', end_sota - start_sota)

start_basic = time.time()
traj_basic = traj_planner_basic.plan(px_s, py_s, pz_s, px_e, py_e, pz_e, vx_s, vy_s, vz_s, vx_e, vy_e, vz_e)
end_basic = time.time()
print('Optimal trajectory duration (basic): ', traj_basic.get_optimal_trajectory_duration(), 'Calculation time (s): ', end_basic - start_basic)

start_improved = time.time()
traj_improved = traj_planner_improved.plan(px_s, py_s, pz_s, px_e, py_e, pz_e, vx_s, vy_s, vz_s, vx_e, vy_e, vz_e)
end_improved = time.time()
print('Optimal trajectory duration (improved): ', traj_improved.get_optimal_trajectory_duration(), 'Calculation time (s): ', end_improved - start_improved)

t_vec_sota, p_x_sota, v_x_sota,  a_x_sota, p_y_sota,  v_y_sota, a_z_sota, p_z_sota, v_z_sota, a_y_sota = traj_sota.get_traj(0.05, 1)
t_vec_basic, p_x_basic, v_x_basic,  a_x_basic, p_y_basic,  v_y_basic, a_z_basic, p_z_basic, v_z_basic, a_y_basic = traj_basic.get_traj(0.0001, 1000)
t_vec_improved, p_x_improved, v_x_improved,  a_x_improved, p_y_improved,  v_y_improved, a_z_improved, p_z_improved, v_z_improved, a_y_improved = traj_improved.get_traj(0.0001, 1000)

ax = plt.axes(projection='3d')

ax.scatter3D(p_x_sota, p_y_sota, p_z_sota, c='g', label='sota')
ax.scatter3D(p_x_basic, p_y_basic, p_z_basic, c='b', label='basic')
ax.scatter3D(p_x_improved, p_y_improved, p_z_improved, c='r', label='improved')

ax.set_xlabel('x in m')
ax.set_ylabel('y in m')
ax.set_zlabel('z in m')
ax.set_xlim(-1, 5)
ax.set_ylim(-2, 5)
ax.set_zlim(-1, 5)
ax.legend()
plt.show()
