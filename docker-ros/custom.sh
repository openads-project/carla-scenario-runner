# Install runtime dependencies for Scenario Runner and CARLA artifacts.
apt-get update
apt-get install -y --no-install-recommends \
    libtiff6 \
    unzip

# Copy over necessary ROS components from ros-bridge and remove the rest.
mkdir -p "$WORKSPACE/src/target"
if [[ -n "${GIT_HTTPS_USER:-}" && -n "${GIT_HTTPS_PASSWORD:-}" ]]; then
    git clone --recurse-submodules "https://${GIT_HTTPS_USER}:${GIT_HTTPS_PASSWORD}@gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/carla-ros-bridge.git"
else
    git clone --recurse-submodules "https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/carla-ros-bridge.git"
fi
mv carla-ros-bridge/carla_common "$WORKSPACE/src/target"
mv carla-ros-bridge/carla_msgs "$WORKSPACE/src/target"
mv carla-ros-bridge/carla_ros_scenario_runner "$WORKSPACE/src/target"
mv carla-ros-bridge/carla_ros_scenario_runner_types "$WORKSPACE/src/target"
mv carla-ros-bridge/ros_compatibility "$WORKSPACE/src/target"
rm -rf carla-ros-bridge

# Install missing ROS dependencies.
apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros2cli \
    ros-$ROS_DISTRO-ros2cli-common-extensions

# Remove dependencies that are installed via pip to avoid conflicts.
apt-get remove -y python3-transforms3d python3-psutil python3-blinker || true

export DOCKER_ROS_FILES_PATH=/docker-ros/additional-files
export SCENARIO_RUNNER_ROOT=$DOCKER_ROS_FILES_PATH

# docker-ros copies ADDITIONAL_FILES_DIR entries with Docker ADD. With './*',
# the srunner directory contents land directly in additional-files.
if [[ ! -e "$SCENARIO_RUNNER_ROOT/srunner" && -f "$SCENARIO_RUNNER_ROOT/__init__.py" ]]; then
    ln -s . "$SCENARIO_RUNNER_ROOT/srunner"
fi

# docker-ros flattens top-level directory contents into additional-files, so
# the shared install script can be either at the root or below docker/.
install_script="$SCENARIO_RUNNER_ROOT/install.sh"
if [[ ! -f "$install_script" ]]; then
    install_script="$SCENARIO_RUNNER_ROOT/docker/install.sh"
fi
if [[ ! -f "$install_script" ]]; then
    echo "Unable to find install.sh" >&2
    exit 1
fi

CARLA_ARTIFACTS_URL="https://gitlab.ika.rwth-aachen.de/api/v4/projects/1645/jobs/artifacts/main/download?job=provide-carla-artifacts&job_token=$GIT_HTTPS_PASSWORD" \
    bash "$install_script"

# .bashrc sources the setup script
echo "source /opt/carla/setup.bash" >> /root/.bashrc
