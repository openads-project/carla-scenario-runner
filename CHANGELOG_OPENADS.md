# OpenADS specific changes

## OpenSCENARIO and scenario execution

- Added realtime-factor control for synchronous scenario execution and fixed RT-factor computation.
- Added OpenSCENARIO 1.1 support with dedicated XSD schemas.
- Added support for executing folders containing multiple OpenSCENARIO files.
- Enabled world reloading for XODR maps.
- Changed `scenario_runner.py` to propagate scenario success or failure through the process exit code.
- Fixed controller assignmend for actors spawned during runtime (AddActor)
- Exported scenario evaluation summaries and connected the execution result to the metrics/output handling.

## ROS and controller integration

- Added `ros_vehicle_control_route_action.py` to control OpenADStack through the route-planning action interface.
- Added `approaching_control.py` for waypoint following with TTC/THW-based braking behavior, including intersection handling.
- Improved `npc_vehicle_control.py` and `FollowTrajectoryAction` speed handling.
- Added new RTS and ARTS controller options and examples.
- Added new ROS 2 route controller using derived route goals from OpenSCENARIO definitions.
- Improved data-provider checks for scenario state, route completion, and route-action result callbacks.

## Build, CI and runtime environment

- Updated the Docker image to CARLA 0.10.0, Ubuntu 24.04, and Python 3.12.
- Added automatic Docker image CI builds, both standalone and ROS 2 aligned using docker-ros.
- Added `scenario_runner_server.py`, a Flask-based HTTP wrapper for triggering `scenario_runner.py` from other containers and returning execution output.
- Fixed autopilot class naming to use `carla_autopilot`.
- Updated vehicle defaults for UE5.

## Visualization and defaults

- Set the spectator view to the `ego_vehicle` at scenario start.
- Update blueprint defaults for CARLA 0.10.0 and Unreal 5
