# TOP-UAV: Open-Source Time-optimal Trajectory Planner for Point-Masses under Acceleration and Velocity Constraints (Python)
top_uav_py is a Python library to generate 3-dimensional time-optimal trajectories from an initial position and velocity vector to a final position and velocity vector with constraints on the maximum velocity and acceleration for the entire motion.

To access the C++ implementation see [top_uav_cpp](https://github.com/fzi-forschungszentrum-informatik/top_uav_cpp).


## 💈 Setup
```shell
#####################
# setup environment #
#####################
python -m venv ./top_uav_py_interpreter

##############################
# define used python version #
##############################
# open ./top_uav_py_interpreter/pyvenv.cfg and configure base interpreter

########################
# activate interpreter #
########################
cd ./top_uav_py_interpreter/Scripts
./activate

########################
# install requirements #
########################
cd ../../
python -m pip install --upgrade pip
pip install -r requirements.txt
```


## 🍫 Quickstart
The example in main.py generates time-optimal trajectories according to the state-of-the-art method as well as of our basic generally valid version as well as our version with improved exploitation of kinematic properties. 

## Run tests
To run the test, execute the following from a terminal:
```shell
pytest -v tests.py
```

## 🏫 Affiliations
<p align="center">
    <img src="https://upload.wikimedia.org/wikipedia/de/thumb/4/44/Fzi_logo.svg/1200px-Fzi_logo.svg.png?raw=true" alt="FZI Logo" height="200"/>
</p>

## Citation

If you use top_uav_py in your research, please consider citing our original paper. 

```
@INPROCEEDINGS{10342270,
  author={Meyer, Fabian and Glock, Katharina and Sayah, David},
  booktitle={2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={TOP-UAV: Open-Source Time-Optimal Trajectory Planner for Point-Masses Under Acceleration and Velocity Constraints}, 
  year={2023},
  volume={},
  number={},
  pages={2838-2845},
  doi={10.1109/IROS55552.2023.10342270}}

```