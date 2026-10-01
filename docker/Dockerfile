# aaucns_rio (EKF + factor-graph "FgRIO") build image.
#
# Builds this repo's own checked-out source (this Dockerfile uses the build
# context, not a fresh clone -- see ../docker/README.md for the exact build
# command) against: ROS Noetic (Ubuntu 20.04), Eigen 3.4.0 (system Eigen
# 3.3.7 is too old for GTSAM 4.3's headers), CMake 3.28.3 (system CMake
# 3.16.3 can't configure GTSAM 4.3), and GTSAM 4.3.0 built from source
# (system package is 4.2.0; GTSAM_WITH_TBB is disabled because the installed
# TBB predates the oneapi/tbb/ header layout GTSAM 4.3 expects).
#
# This repo's own FG bug-fix patch and GTSAM-4.3 header-compatibility patch
# (see REPRODUCE.md section 4c in the Camera-radar-odom project, if you have
# access to it) are already committed to this checkout's source -- this
# Dockerfile does not need to apply them. The one real patch it DOES apply is
# to GTSAM itself (its own GCC-9 incompatibility, unrelated to aaucns_rio)
# and to the sibling ti_mmwave_rospkg package (C++ standard bump) -- see the
# comments at each step below for why.
#
# Build (from the repo root, i.e. the parent of this docker/ directory):
#   docker build -f docker/Dockerfile -t aaucns_rio .
#
# See docker/README.md for how to run the resulting image against a bag,
# for both the EKF and FG backends.

FROM ros:noetic-perception

ENV DEBIAN_FRONTEND=noninteractive

# --- Base build tooling -----------------------------------------------------
# ros:noetic-perception already provides libeigen3-dev (3.3.7), libpcl-dev
# (which pulls boost), and CMake 3.16.3. We only need to add: git/curl (fetch
# GTSAM and the sibling packages), python3-dev (catkin builds it in anyway,
# made explicit here), and ros-noetic-nodelet (ti_mmwave_rospkg's
# build_depend, not guaranteed by the perception metapackage alone).
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        curl \
        ca-certificates \
        build-essential \
        python3-dev \
        python3-rosdep \
        ros-noetic-nodelet \
    && rm -rf /var/lib/apt/lists/*

# --- Eigen 3.4.0 (header-only swap) -----------------------------------------
# GTSAM 4.3's headers use a fixed-size-matrix brace-init constructor that
# Eigen 3.3.7 (Ubuntu 20.04's package) doesn't support. Eigen is header-only,
# so this is a safe, reversible swap; the original headers are kept at
# /usr/include/eigen3.orig_3.3.7 for reference.
RUN curl -fsSL -o /tmp/eigen340.tar.gz \
        https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz \
    && mkdir -p /tmp/eigen340_extract \
    && tar xzf /tmp/eigen340.tar.gz -C /tmp/eigen340_extract --strip-components=1 \
    && mv /usr/include/eigen3 /usr/include/eigen3.orig_3.3.7 \
    && mkdir -p /usr/include/eigen3 \
    && cp -r /tmp/eigen340_extract/Eigen /tmp/eigen340_extract/unsupported \
             /tmp/eigen340_extract/signature_of_eigen3_matrix_library \
             /usr/include/eigen3/ \
    && sed -i 's/3\.3\.7/3.4.0/' /usr/lib/cmake/eigen3/Eigen3ConfigVersion.cmake \
    && rm -rf /tmp/eigen340.tar.gz /tmp/eigen340_extract

# --- CMake 3.28.3 (prebuilt binary release) ---------------------------------
# Ubuntu 20.04's CMake (3.16.3) can't configure GTSAM 4.3's CMakeLists.txt.
# Installed to /opt/cmake-new rather than touching the system cmake package,
# and deliberately NOT added to PATH: catkin_make's own CMake invocation (for
# aaucns_rio, below) must keep using the system CMake 3.16.3 default. Newer
# CMake was confirmed to change how catkin's build resolves the C++ standard
# for aaucns_rio's PCL-dependent translation units (with /opt/cmake-new on
# PATH, those fail with "PCL requires C++14 or above" even though nothing
# about the actual compiler flags changed) -- so GTSAM's build block below
# invokes /opt/cmake-new/bin/cmake by its full path instead of relying on
# PATH.
RUN curl -fsSL -o /tmp/cmake.tar.gz \
        https://github.com/Kitware/CMake/releases/download/v3.28.3/cmake-3.28.3-linux-x86_64.tar.gz \
    && mkdir -p /opt/cmake-new \
    && tar xzf /tmp/cmake.tar.gz -C /opt/cmake-new --strip-components=1 \
    && rm -f /tmp/cmake.tar.gz

# --- GTSAM 4.3.0, built from source -----------------------------------------
# - GTSAM_USE_SYSTEM_EIGEN=ON: must use the 3.4.0 headers just installed,
#   not GTSAM's bundled (older) Eigen copy.
# - GTSAM_WITH_TBB=OFF: the installed Intel TBB predates GTSAM 4.3's expected
#   oneapi/tbb/ header layout; TBB is a performance option, not required for
#   correctness at this problem's scale.
# - GTSAM_BUILD_TESTS/EXAMPLES_ALWAYS/UNSTABLE/PYTHON=OFF: not needed, and
#   the unstable module's constrained/certifiable code is exactly where the
#   GCC-9-vs-abstract-ternary issue below lives, so it's fixed regardless of
#   this flag (aaucns_rio's own factor headers already need core GTSAM, and
#   fixing it once here keeps the option open to turn UNSTABLE back on).
# - GTSAM's own top-level CMakeLists.txt unconditionally adds a `timing/`
#   subdirectory (`gtsam_timing_utils`, internal benchmarking code, not
#   installed, not needed by aaucns_rio) with no CMake option to exclude it,
#   and CMake's generated Makefiles pull it into any `all`-rooted build
#   regardless of which named target you ask `make` for -- so there's no way
#   to skip it short of deleting the subdirectory. It's by far the slowest
#   thing to compile in this image but otherwise harmless; expect this step
#   to take several minutes.
# - Installed to /usr/local; the apt-get libgtsam-dev package (4.2.0, left
#   uninstalled here) would otherwise take precedence via /usr/lib, so do not
#   apt-get install libgtsam-dev/libgtsam-unstable-dev in this image.
#
# The QpCost.h fix below is a GTSAM-side patch, unrelated to aaucns_rio: GCC 9
# rejects a ternary (`cond ? *abstractBaseRef : throw ...`) in GTSAM's own
# gtsam/constrained/QpCost.h (no common non-abstract type for the two
# branches -- the standard permits it, GCC 9 doesn't). Confirmed directly by
# fetching GTSAM 4.3.0's real source: the fixer script matches and rewrites
# the exact constructor, no behavior change, idempotent if a future GTSAM
# release already avoids the pattern.
COPY docker/patches/fix_gtsam_qpcost_gcc9.py /tmp/fix_gtsam_qpcost_gcc9.py
RUN git clone --branch 4.3.0 --depth 1 https://github.com/borglab/gtsam.git /root/gtsam_src \
    && python3 /tmp/fix_gtsam_qpcost_gcc9.py \
        /root/gtsam_src/gtsam/constrained/QpCost.h \
    && mkdir -p /root/gtsam_src/build \
    && cd /root/gtsam_src/build \
    && /opt/cmake-new/bin/cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_BUILD_PYTHON=OFF \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && rm -rf /root/gtsam_src

# --- catkin workspace: sibling packages + this repo's own source -----------
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p ${CATKIN_WS}/src

# ti_mmwave_rospkg and serial are catkin-workspace siblings this package
# depends on (not part of this repo); cloned at HEAD of their default branch.
# If you need byte-identical sources to a specific prior build, vendor your
# own copies under docker/vendor/{ti_mmwave_rospkg,serial} and replace these
# RUN lines with `COPY docker/vendor/ti_mmwave_rospkg ${CATKIN_WS}/src/ti_mmwave_rospkg`
# / `COPY docker/vendor/serial ${CATKIN_WS}/src/serial` (see docker/README.md).
#
# ti_mmwave_rospkg upstream targets C++11 (`add_definitions(-std=c++11)` and
# `set(CMAKE_CXX_STANDARD 11)`), which is incompatible with PCL 1.10's own
# headers (Ubuntu 20.04's package: PCL 1.10 requires C++14+, confirmed
# directly -- building upstream's C++11 setting fails with GCC errors inside
# pcl/point_types.h such as "'minusscalar' is not a member of 'pcl::traits'",
# since PCL's point-type macros there rely on C++14 features). Bumped to
# C++14 here with a one-line sed.
RUN git clone --depth 1 https://github.com/radar-lab/ti_mmwave_rospkg.git \
        ${CATKIN_WS}/src/ti_mmwave_rospkg \
    && sed -i 's/-std=c++11/-std=c++14/; s/CMAKE_CXX_STANDARD 11/CMAKE_CXX_STANDARD 14/' \
        ${CATKIN_WS}/src/ti_mmwave_rospkg/CMakeLists.txt \
    && git clone --depth 1 https://github.com/wjwwood/serial.git \
        ${CATKIN_WS}/src/serial

# This repo's own source -- copied from the build context (see the `docker
# build` command at the top of this file), not re-cloned, so whatever is
# checked out when you run `docker build` (including any uncommitted changes
# in your working tree) is exactly what gets built. The build context's own
# docker/ directory and .git are excluded via ../.dockerignore so they don't
# end up inside the catkin package.
COPY . ${CATKIN_WS}/src/aaucns_rio

# --- Build ------------------------------------------------------------------
# Builds all four targets from one catkin workspace: aaucns_rio_node (EKF),
# aaucns_rio_fg_node (FG), and their deterministic bag-replay counterparts.
# rio_node links the fg/*.cpp translation units too (see docker/README.md,
# "Why EKF and FG share a build"), so GTSAM must be present even to build the
# EKF binary alone.
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
    && cd ${CATKIN_WS} \
    && catkin_make -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_PREFIX_PATH='/usr/local;/opt/ros/noetic'"

# --- Convenience: auto-source ROS + the workspace in interactive shells ----
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "source ${CATKIN_WS}/devel/setup.bash" >> /root/.bashrc

WORKDIR ${CATKIN_WS}
CMD ["/bin/bash"]
