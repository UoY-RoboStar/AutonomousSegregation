# Source this to use the ROS 2 Humble build from macos/install-ros2.sh:
#   source macos/setup.zsh
source ~/ros2_humble/venv/bin/activate
source ~/ros2_humble/install/setup.zsh
# Homebrew does not link Qt 5 into /opt/homebrew, so CMake would not find it
# when building cachecons_gazebo_gui and cachecons_state_viewer.
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$(brew --prefix qt@5)
