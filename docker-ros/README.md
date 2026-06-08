# Scenario Runner ROS

This repository aims to provide standalone images of the [CARLA ROS Scenario Runner](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/carla-scenario-runner) for usage in CI pipelines, cluster deployments etc. by utilizing [docker-ros](https://gitlab.ika.rwth-aachen.de/fb-fi/ops/docker-ros).

**Note:**

The scenario runner used here is from a fork created by **ika**, which is integrated as a submodule. Additionally, some components of the [carla-ros-bridge](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/carla-ros-bridge) fork by **ika** (especially the scenario runner ROS node) are integrated by cloning and copying them over when building the image.

- [Nodes](#nodes)
  - [carla_ros_scenario_runner/carla_ros_scenario_runner_node.py](#carla_ros_scenario_runnercarla_ros_scenario_runnerpy)
- [Usage of docker-ros Images](#usage-of-docker-ros-images)
  - [Available Images](#available-images)
  - [Default Command](#default-command)
  - [Environment Variables](#environment-variables)
  - [Launch Files](#launch-files)
  - [Configuration Files](#configuration-files)
  - [Additional Remarks](#additional-remarks)
- [Building Locally](#building-locally)
  - [Requirements](#requirements)
  - [Steps](#steps)
- [Official Documentation](#official-documentation)


## Nodes

| Package | Node | Description |
| --- | --- | --- |
| `carla_ros_scenario_runner` | `carla_ros_scenario_runner.py` | Offers service to execute OpenScenarios on CARLA via ROS |

### carla_ros_scenario_runner/carla_ros_scenario_runner.py

This node, besides other components, comes from the ika fork of the [carla-ros-bridge](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/carla-ros-bridge) and is cloned into images of this project during the build.

Also see the [official documentation](https://carla.readthedocs.io/projects/ros-bridge/en/latest/carla_ros_scenario_runner/)

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/scenario_runner/status` | `carla_ros_scenario_runner_types.CarlaScenarioRunnerStatus` | The current status of the scenario runner execution (used by the [rviz_carla_plugin](https://carla.readthedocs.io/projects/ros-bridge/en/latest/rviz_plugin/)) |

#### Services

| Service | Type | Description |
| --- | --- | --- |
| `/scenario_runner/execute_scenario` | `carla_ros_scenario_runner_types.ExecuteScenario` | Execute a scenario. If another scenario is currently running, it gets stopped. |


## Official Documentation

- [Scenario Runner](https://github.com/carla-simulator/scenario_runner)
- [CARLA ROS Scenario Runner](https://carla.readthedocs.io/projects/ros-bridge/en/latest/carla_ros_scenario_runner/)


