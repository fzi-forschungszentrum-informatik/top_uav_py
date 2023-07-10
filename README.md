# TOP-UAV: Open-Source Time-optimal Trajectory Planner for Point-Masses under Acceleration and Velocity Constraints (Python)
top_uav_py is a Python library to generate 3-dimensional time-optimal trajectories from an initial position and velocity vector to a final position and velocity vector with constraints on the maximum velocity and acceleration for the entire motion.

## 💈 Installation
```shell
pip install requirements.txt
```

To access the C++ implementation see [top_uav_cpp](https://github.com/fzi-forschungszentrum-informatik/top_uav_cpp).

## 🍫 Quickstart
The example in main.py generates time-optimal trajectories according to the state-of-the-art method as well as of our basic generally valid version as well as our version with improved exploitation of kinematic properties.


## 🏫 Affiliations
<p align="center">
    <img src="https://upload.wikimedia.org/wikipedia/de/thumb/4/44/Fzi_logo.svg/1200px-Fzi_logo.svg.png?raw=true" alt="FZI Logo" height="200"/>
</p>

## Citation

If you use top_uav_py in your research, please consider citing our original paper. 

```
@INPROCEEDINGS{Meyer.2023,
	author={Meyer, Fabian and Glock, Katharina and Sayah, David}, 
  booktitle={2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
	year={2023},
	title={TOP-UAV: Open-Source Time-optimal Trajectory Planner for Point-Masses under Acceleration and Velocity Constraints},
	volume={},
	issue={}, 
	pages={tba.}
}
```