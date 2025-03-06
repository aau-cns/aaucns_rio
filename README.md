# aaucns_rio
MultiState-EKF Radar-Inertial Odometry

# Compilation
catkin_make --force-cmake -DCMAKE_BUILD_TYPE={Release, Debug}

`--force-cmake` is there only to force copying the yaml config file into the
`.ros` folder. If you copy it manually you do not need that option.

# Use
The software can be used wither as a ROS node or to replay ROS bag files.
For the latter, modify the file in `nodes/rio_(fg)_replay_node.cpp` by giving the
bag filename (file must be in `.ros) and modify the topic names.

Within `src/rio_(fg)_replay.cpp` you might also have to modify the message types.

Then, run the software using one of the launch files in the `launch/` folder.

# Dependencies

Users are expected to have Eigen, PCL and GTSAM installed system-wide.

# Cite

If you use this software please cite:

@INPROCEEDINGS{10801945,
  author={Michalczyk, Jan and Quell, Julius and Steidle, Florian and Müller, Marcus G. and Weiss, Stephan},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Tightly-Coupled Factor Graph Formulation For Radar-Inertial Odometry}, 
  year={2024},
  volume={},
  number={},
  pages={3364-3370},
  doi={10.1109/IROS58592.2024.10801945}}
