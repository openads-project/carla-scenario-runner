# OpenADS specific changes

Based on ScenarioRunner v1.0 for CARLA 1.0.0 (Unreal Engine 5.8), including the fixes of the upstream `ue58-dev` branch up to `8ef09066`. Changes that upstream has adopted since are no longer listed here: the vehicle and blueprint defaults for Unreal Engine 5, which upstream now resolves through its own blueprint alias table.

## OpenSCENARIO and scenario execution

- Added realtime-factor control for synchronous scenario execution and fixed RT-factor computation.
- Added OpenSCENARIO 1.1 support with dedicated XSD schemas.
- Added support for executing folders containing multiple OpenSCENARIO files.
- Enabled world reloading for XODR maps.
- Changed `scenario_runner.py` to propagate scenario success or failure through the process exit code; a scenario without criteria counts as a success.
- Fixed `AddEntityAction` and `DeleteEntityAction` for entities added in the story: they are no longer spawned at scenario start, keep their entity name as role name, get a default controller and their initial velocity, and are resolved by name once they exist.
- Exported scenario evaluation summaries and connected the execution result to the metrics/output handling.
- Fixed distance, time headway and time-to-collision conditions without the optional `freespace` and `alongRoute` attributes.
- Left the weather unchanged for scenarios without an `EnvironmentAction`, instead of the daylight weather upstream sets: it stays the map's weather or the weather another client sets, e.g. the environment client of OpenADSim. A scenario that defines its weather still overrides it.
- Compared multi-level map names (e.g. `/Game/Carla/Maps/Town10HD_Opt`) by their last part when checking the loaded map.
- Saved recordings to the `--record` path itself instead of a path relative to `SCENARIO_RUNNER_ROOT`.
- Fixed loading and running OpenSCENARIO 1.x scenarios with py_trees 2.x: blackboard reads of unset variables, story element states, and the `criteria_*` conditions of the parser.
- Fixed scenarios without criteria ending on their first tick with py_trees 2.x.
- Kept the fallback to a default blueprint per actor category for models that are neither in the blueprint library nor in the upstream alias table.
- Limited the sensor cleanup after a scenario to the scenario's own sensors, so the sensors of an ego vehicle spawned by another client (`waitForEgo`) are kept for the next scenario.
- Fixed the cleanup when running several OpenSCENARIO files or routes; only the first scenario was cleaned up before.
- Fixed `FollowTrajectoryAction` in the `Init` section, which raised a `TypeError` since the trajectory parsing was extended for OpenSCENARIO 1.1.
- Fixed the `--scenario`, `--route` and `--openscenario2` modes, which failed with an `AttributeError` since folders of OpenSCENARIO files are supported. The files of a folder now run in alphabetical order.

## ROS and controller integration

- Added `ros_vehicle_control_route_action.py` to control OpenADStack through the route-planning action interface.
- Added `approaching_control.py` for waypoint following with TTC/THW-based braking behavior, including intersection handling.
- Improved `npc_vehicle_control.py` and `FollowTrajectoryAction` speed handling.
- Added new RTS and ARTS controller options and examples.
- Fixed RTS replay spinning actors for one or two ticks when a trajectory's heading crosses ±180°; angles are now interpolated using the smallest angle difference.
- Added new ROS 2 route controller using derived route goals from OpenSCENARIO definitions.
- Added `ScenarioState` to track whether a scenario is running, route completion, and route-action result callbacks; upstream removed the `CarlaDataProvider` that held these checks before.
- Sent intermediate route waypoints to the ROS 2 route planning module for RouteActions; trajectories still send only their destination.
- Removed the heading check when filtering interpolated route waypoints, so routes with sharp turns are no longer truncated.

## Build, CI and runtime environment

- Updated the Docker image to CARLA 1.0.0, Ubuntu 24.04, and Python 3.12.
- Installed the CARLA PythonAPI from the carla-simulator release `v1.0.0-1.0.0`, with only the requirements of the `carla` package plus `pygame`.
- Pinned the Python requirements, with `xmlschema` kept at 1.0.18 instead of the upstream 4.3.2.
- Added automatic Docker image CI builds, both standalone and ROS 2 aligned using docker-ros.
- Added a Docker Compose service and a Helm chart in `deployment/`, published as OCI artifacts by CI.
- Removed the upstream CI workflows `static_code_check.yml` and `unit_test.yml`, which only run on `master`.
- Added `scenario_runner_server.py`, a Flask-based HTTP wrapper for triggering `scenario_runner.py` from other containers and returning execution output.
- Fixed autopilot class naming to use `carla_autopilot`.
- Updated `perception_interfaces` and `planning_interfaces` to v1.2.0.
- Updated the openadservice Helm chart dependency to 1.3.0.
- Added the daily `ghcr-cleanup` workflow of `openads-dev-environment`, which also cleans the `ros` image.
- Release builds tag the standalone image with the release tag, for example `carla-scenario-runner:v1.0.0-1.2.0`, instead of `latest_<tag>_ci`, so the cleanup keeps them.

## Visualization and defaults

- Set the spectator view to the `ego_vehicle` at scenario start.
- Showed the realtime factor in the HUD of `manual_control.py`, which also moves the spectator to the ego vehicle and controls `ego_vehicle` by default.
