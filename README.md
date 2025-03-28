# aaucns_rio
This software contains an implementation of Radar-Inertial Odometry using multi-state EKF and factor graphs.

# Compilation
`catkin_make --force-cmake -DCMAKE_BUILD_TYPE={Release, Debug}`.

`--force-cmake` is there only to force copying the yaml config file into the
`~/.ros` folder. If you copy it manually you do not need that option.

# Use
The software can be used either as a ROS node or to replay ROS bag files.
For the latter, modify the file in `nodes/rio_(fg)_replay_node.cpp` by giving the
bag filename (file must be in `~/.ros`) and modify the topic names. Acronym `fg` stands for "factor graph".
The default version uses EKF as the estimation backbone.

Within `src/rio_(fg)_replay.cpp` you might also have to modify the message types.

Then, run the software using one of the launch files in the `launch/` folder.

RIO estimator must be configured through a yaml config file. An example is in `config/` folder.
The software reads this config from the `~/.ros` folder (or your alternative ROS runtine folder if you changed it).

# Dependencies

Users are expected to have Eigen, PCL and GTSAM installed system-wide.

# Cite (BibTeX)

If you use this software please cite:
```
@INPROCEEDINGS{Michalczyk2024aaucns_rio,
  author={Michalczyk, Jan and Quell, Julius and Steidle, Florian and Müller, Marcus G. and Weiss, Stephan},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={Tightly-Coupled Factor Graph Formulation For Radar-Inertial Odometry},
  year={2024},
  volume={},
  number={},
  pages={3364-3370},
  doi={10.1109/IROS58592.2024.10801945}}
```

# Example dataset

You can download [here](https://cns-data.aau.at/rio_dataset/awr_6.bag) and [here](https://cns-data.aau.at/rio_dataset/awr_7.bag) example bagfiles to use with the rio.
Remember to place them in your `~/.ros` folder.

# Parameters

In the file `config/config.yaml` there are several parameters to control the behavior of the estimator.
Some parameters require at the moment the code to be recompiled. They are in the `include/aaucns_rio/config.h` file.

# Future work

It is planned to provide a docker container with the code.
