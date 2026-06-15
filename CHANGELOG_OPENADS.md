# OpenADS specific changes

## OpenSCENARIO and scenario execution

- Control the RT factor (!21)
- Support OpenSCENARIO 1.1 (!14)
- Option to execute folders with many OpenSCENARIO files (!6)
- Enable world reloading also for xodr maps (!6)
- Modified `scenario_runner.py` to return the result of scenario execution as exit code (!25)
- Fixed controller assignmend for actors spawned during runtime (AddActor) (!32)
- Export scenario evaluation summary (!6)

## ROS and controller integration

- `ros_vehicle_control_route_action.py` controller to control OpenADStack (!4)
- `approaching_control.py` controller for waypoint following with TTC/THW-based braking behavior
- Improve `npc_vehicle_control.py` controller (!11)
- Add ARTS and RTS controller options (!14, !23)
- Improve data provider checks for scenario state and route completion (!25)

## Build, CI and runtime environment

- Created GitHub workflow and Dockerfile to automatically build Docker images (!9)
- Updated Docker image to Ubuntu 24.04 with Python 3.12 and streamlined PythonAPI dependencies (!26)
- Fixed autopilot class naming (`carla_autopilot`) (!26)
- Updated vehicle defaults for UE5 (no dedicated MR found)
- `scenario_runner_server.py` HTTP wrapper to trigger `scenario_runner.py` from other containers (!10)

## Visualization and defaults

- Set spectator view to ego_vehicle (!6)
