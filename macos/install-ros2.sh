#!/bin/zsh
# Build the parts of ROS 2 Humble that the CacheCons simulation uses into
# ~/ros2_humble, following the ROS macOS instructions. Running it again
# continues an interrupted build. See README.md.
set -eo pipefail

ws=$HOME/ros2_humble
here=${0:A:h}

mkdir -p $ws/src
cd $ws

# A virtual environment, because Homebrew's Python refuses system-wide pip
# installs. It sees Homebrew's packages, which include the Gazebo bindings.
[ -d venv ] || $(brew --prefix python)/bin/python3 -m venv --system-site-packages venv
source venv/bin/activate
# opencv-python and protobuf must match Homebrew's opencv@4 and protobuf.
python -m pip install argcomplete catkin_pkg colcon-common-extensions empy==3.3.4 \
  importlib-metadata lark lxml netifaces psutil pydot pyparsing pyyaml rosdep rosdistro \
  'setuptools<80' vcstool transforms3d opencv-python==4.14.0.94 protobuf==7.36.2

vcs import --skip-existing --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
vcs import --skip-existing src < $here/extras.repos

export OPENSSL_ROOT_DIR=$(brew --prefix openssl)
export CMAKE_PREFIX_PATH=$(brew --prefix qt@5)
export PATH=$PATH:$(brew --prefix qt@5)/bin
# Homebrew's CMake 4 refuses packages that declare older minimum versions.
export CMAKE_POLICY_VERSION_MINIMUM=3.5
# Fast DDS does not build on macOS, so Cyclone DDS is the default middleware.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GZ_VERSION=harmonic

# The RPATH options embed the library locations, so SIP can stay enabled.
colcon build --symlink-install --merge-install \
  --packages-up-to \
    rclpy rclcpp rclcpp_components launch_ros ros2launch ros2run ros2topic ros2node \
    ros2pkg ros2param ros2service robot_state_publisher rmw_cyclonedds_cpp \
    rosidl_default_generators rosidl_default_runtime ament_cmake_python ament_index_python \
    std_msgs geometry_msgs sensor_msgs nav_msgs tf2_msgs rosgraph_msgs image_transport \
    cv_bridge tf_transformations ros_gz_bridge ros_gz_sim urdf \
  --packages-ignore \
    fastrtps rmw_fastrtps_cpp rmw_fastrtps_shared_cpp rmw_fastrtps_dynamic_cpp \
    google_benchmark_vendor ament_cmake_google_benchmark performance_test_fixture \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE=$VIRTUAL_ENV/bin/python -DPYTHON_EXECUTABLE=$VIRTUAL_ENV/bin/python \
    -DCMAKE_INSTALL_RPATH=$ws/install/lib -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
    -DOpenCV_DIR=$(brew --prefix opencv@4)/lib/cmake/opencv4
