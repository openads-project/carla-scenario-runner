# CARLA Scenario Runner deployment

This directory contains the reusable Docker Compose service and Helm chart for
the ROS 2 wrapper around CARLA Scenario Runner. The files follow the structure
and conventions of the `openads-dev-environment` Compose and Helm generators,
but are maintained manually: this repository is not itself a ROS package and
adds `carla_ros_scenario_runner` from `carla-ros-bridge` while building the
`ros` image.
