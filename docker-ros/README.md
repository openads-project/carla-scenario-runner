# Scenario Runner ROS

These Docker ROS images package the CARLA Scenario Runner with the ROS scenario runner node used in OpenADS workflows.
They provide a reproducible container setup for running OpenSCENARIO scenarios in CI pipelines and cluster deployments.

## Nodes

| Package | Node | Description |
| --- | --- | --- |
| `carla_ros_scenario_runner` | `carla_ros_scenario_runner.launch.py` | Offers service to execute OpenScenarios on CARLA via ROS |

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


