# aaucns_rio Docker image: build and run (EKF and FgRIO)

This builds a self-contained image from **this repo's own checked-out
source** that can run both backends:

- `aaucns_rio_node` — the original multi-state EKF backend ("EKF").
- `aaucns_rio_fg_node` — the factor-graph backend, with this checkout's FG
  bug-fix patch already applied in `src/fg/`, `include/aaucns_rio/fg/`, and
  `config/config.yaml`'s `noise_meas3`.

This file only covers *building the image* and *running the estimator*; it
assumes the fixes already in this checkout's git history, not anything you
need to apply yourself.

## 1. Build the image

From the **repo root** (the parent of this `docker/` directory):

```bash
docker build -f docker/Dockerfile -t aaucns_rio .
```

Takes a while — GTSAM 4.3 is built from source (`make -j$(nproc)`, no tests/
examples/unstable/python modules) and is the dominant cost (several
minutes). Everything else (Eigen header swap, CMake binary install, the two
sibling catkin packages, this repo's own source) is fast.

Requires network access during the build (clones GTSAM and the two sibling
catkin packages below, and downloads the Eigen 3.4.0 and CMake 3.28.3
release tarballs).

**Important**: the build uses whatever is in your working tree when you run
`docker build`, including uncommitted changes — it does not re-clone this
repo from a remote. Commit (or at least `git add`) what you want built
before running it if you're not sure.

### Sibling packages: ti_mmwave_rospkg and serial

This package depends on two catkin packages that live in separate repos,
not this one: `ti_mmwave_rospkg` (radar driver/messages) and `serial`
(serial-port library). The Dockerfile clones both at HEAD of their default
branch — **not pinned to a specific commit**. They're stable, rarely-changed
packages, but if you need byte-identical sources to a specific earlier
build, vendor your own copies instead:

```bash
mkdir -p docker/vendor
cp -r /path/to/your/ti_mmwave_rospkg docker/vendor/ti_mmwave_rospkg
cp -r /path/to/your/serial docker/vendor/serial
```

then replace the corresponding `RUN git clone ...` block in
`docker/Dockerfile` with:

```dockerfile
COPY docker/vendor/ti_mmwave_rospkg ${CATKIN_WS}/src/ti_mmwave_rospkg
COPY docker/vendor/serial ${CATKIN_WS}/src/serial
```

(`docker/vendor/` is not excluded by `.dockerignore`, so this works as-is —
only `docker/` itself is excluded from the `COPY .` that copies this
package's own source, not from being addressable by its own explicit
`COPY docker/vendor/...` lines.)

**Note**: upstream `ti_mmwave_rospkg`'s `CMakeLists.txt` targets C++11,
which fails to build against PCL 1.10 (`pcl/point_types.h` needs C++14+,
confirmed directly: building its C++11 setting fails with errors like
`'minusscalar' is not a member of 'pcl::traits'`). The Dockerfile
`sed`-patches this to C++14 right after cloning it — if you vendor your own
copy instead, make sure it has the same fix (or apply it yourself) rather
than dropping it.

### Why EKF and FG share a build

This package's `CMakeLists.txt` compiles `src/fg/rio_fg.cpp`,
`src/fg/imu_measurement.cpp` and `src/fg/marginalization.cpp` into the
**same static library** (`aaucns_rio_rio`) that `aaucns_rio_node` (EKF)
links against — the EKF binary never calls into any FG class at runtime,
but it does need GTSAM present and the FG headers compiling cleanly just to
build at all. This was verified directly during this patch's development: a
pristine (pre-fix) checkout fails to compile in a GTSAM-4.3 environment with
a `NoiseModelFactor3`/`NoiseModelFactorN` error from the FG factor headers —
confirming GTSAM-4.3 compatibility is a hard prerequisite for *either*
binary, not something FG-specific you can skip if you only want the EKF.

## 2. Start a container

```bash
docker run -it --name aaucns_rio --net host \
    -v /path/to/your/bags:/data/bags \
    aaucns_rio
```

`--net host` is the simplest way to let `roscore`/nodes/`rosbag` talk to each
other without extra port mapping; adjust if you have a reason not to use
host networking. Mount wherever your bag files live at `/data/bags` (or any
path you like — just use that path below).

Everything below runs **inside** this container (or via `docker exec -it
aaucns_rio bash` from another terminal — you need at least two shells: one
for `roscore`, and separate ones for the node, `rosbag play`, and the
`rosservice call`).

### Input format expected by both backends

Both nodes subscribe to:
- `/ti_mmwave/radar_scan_pcl` — a `sensor_msgs/PointCloud2` with named fields
  `x,y,z,intensity,velocity` (radar range/Doppler), NTNU point-cloud format.
- `/mavros/imu/data_raw` — a `sensor_msgs/Imu`. If your source topic is named
  something else (e.g. `/imu/data`), remap it at `rosbag play` time (shown
  below) rather than touching the node's hardcoded topic names.

If your data is ROS2 (`.mcap`), convert it to a ROS1 bag first, e.g. with the
`rosbags` Python library:

```bash
pip3 install rosbags
rosbags-convert --src your_sequence.mcap --dst your_sequence.bag
```

## 3. Run the EKF backend

In the container, put `config.yaml` next to the bag you're about to play —
the node loads `"config.yaml"` as a path **relative to its current working
directory** (`src/rio.cpp`: `YAML::LoadFile(config_file)`), so `cd` into a
directory containing it before launching:

```bash
mkdir -p /data/run_ekf && cd /data/run_ekf
cp ${CATKIN_WS}/src/aaucns_rio/config/config.yaml .
# edit calibration (q_riw/q_rix/.../POS_RtoI etc.) in config.yaml for your rig
# if it differs from the defaults baked into the package.
```

Terminal 1:
```bash
roscore
```

Terminal 2 (record the output):
```bash
cd /data/run_ekf
rosbag record -O pose_record.bag /pose /aaucns_rio_state
```

Terminal 3 (the estimator):
```bash
cd /data/run_ekf
rosrun aaucns_rio aaucns_rio_node
```
Wait for `No features found.` lines (normal at startup, before the radar has
enough points) — the node is up and waiting on `/ti_mmwave/radar_scan_pcl`
and `/mavros/imu/data_raw`.

Terminal 4 (playback — remap your IMU topic to the one the node expects):
```bash
rosbag play /data/bags/your_sequence.bag /imu/data:=/mavros/imu/data_raw -r 1
```

While the platform is stationary at the start of the bag (a few seconds in,
before any motion), initialize the filter:
```bash
rosservice call /rio_node/init_service "data: true"
```
You should see `Initialized filter trough ROS Service.` [sic, upstream's own
log message] in terminal 3, and `success: True` in terminal 4's shell.

When playback finishes (`rosbag play` prints `Done.`), stop recording
(`Ctrl-C` in terminal 2 — `SIGINT`, not `kill -9`, so the bag is finalized
rather than left as `pose_record.bag.active`) and stop the node (`Ctrl-C` in
terminal 3).

### Extract the trajectory (TUM format)

```python
import rosbag

out = []
for _, m, _ in rosbag.Bag("pose_record.bag").read_messages(topics=["/pose"]):
    p = m.pose.pose
    out.append((m.header.stamp.to_sec(), p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
out.sort()
with open("poses_tum.txt", "w") as f:
    for r in out:
        f.write("%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % r)
print("poses:", len(out))
```

## 4. Run the FG backend ("FgRIO")

Same pattern, with three differences: use `config_fg.yaml` (note the
filename — `rio_fg_node.cpp` hardcodes `"config_fg.yaml"`, not
`"config.yaml"`), the binary is `aaucns_rio_fg_node`, and the service is
`/rio_fg_node/init_service`.

```bash
mkdir -p /data/run_fg && cd /data/run_fg
cp ${CATKIN_WS}/src/aaucns_rio/config/config.yaml config_fg.yaml
```

Terminal 1: `roscore` (if not already running)

Terminal 2:
```bash
cd /data/run_fg
rosbag record -O pose_record.bag /pose /aaucns_rio_state
```

Terminal 3:
```bash
cd /data/run_fg
rosrun aaucns_rio aaucns_rio_fg_node
```

Terminal 4:
```bash
rosbag play /data/bags/your_sequence.bag /imu/data:=/mavros/imu/data_raw -r 1
```

Partway through the initial stationary period:
```bash
rosservice call /rio_fg_node/init_service "data: true"
```

Stop/extract exactly as in §3.

### Known FG-specific caveats

- **Run-to-run variance is real and not small.** The shared RANSAC front end
  (`velocity_provider.cpp`) seeds from `std::random_device` — genuine
  nondeterminism, not a fixed seed. Repeated runs of the *identical* binary
  on the *identical* bag can differ by several meters in final position on a
  50-second indoor sequence. Don't treat a single run's trajectory as
  reproducible to the centimeter; if you need a stable number, run several
  times and look at the spread, not one sample. This affects the EKF
  backend too (same shared front end), not just FG.
- **RTE across backends isn't comparable.** EKF publishes at IMU rate
  (~100 Hz), FG at radar rate (~10 Hz); `evo_rpe`'s default one-frame step
  measures drift over a different time delta for each, so only compare RTE
  within one backend's own rows, never EKF-vs-FG directly.

## 5. Deterministic replay (optional, for debugging)

Both backends also have a replay-mode binary (`aaucns_rio_replay_node`,
`aaucns_rio_fg_replay_node`) that reads a bag directly rather than via
`rosbag play` + live subscription — useful for isolating whether a problem
is related to message-arrival timing rather than the estimator's own logic:

```bash
cd /data/run_fg
rosrun aaucns_rio aaucns_rio_fg_replay_node /data/bags/your_sequence.bag config_fg.yaml /imu/data
```

(First two positional args are the bag path and the config filename; check
`nodes/rio_fg_replay_node.cpp`/`rio_replay_node.cpp` if your topic layout
differs, since the replay nodes' exact argv handling is less uniform than
the live nodes' constructor arguments.)

## Scoring

```bash
pip3 install evo --upgrade --no-binary evo
evo_ape tum reference_trajectory.txt poses_tum.txt -a --plot
evo_rpe tum reference_trajectory.txt poses_tum.txt -a --t_max_diff 0.05
```
