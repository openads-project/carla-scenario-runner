# Remove dependencies that are installed via pip to avoid conflicts.
apt-get remove -y python3-transforms3d python3-psutil python3-blinker || true

# Install runtime dependencies for Scenario Runner and CARLA artifacts.
apt-get update
apt-get install -y --no-install-recommends \
    libtiff6 \
    unzip

# Ensure libtiff5 compatibility for CARLA on Ubuntu 24.04.
if ! ldconfig -p | grep -q "libtiff.so.5"; then
    apt-get install -y libtiff5 && ldconfig || true
    if ! ldconfig -p | grep -q "libtiff.so.5"; then
        if [ -f /usr/lib/x86_64-linux-gnu/libtiff.so.6 ] && [ ! -e /usr/lib/x86_64-linux-gnu/libtiff.so.5 ]; then
            ln -sf /usr/lib/x86_64-linux-gnu/libtiff.so.6 /usr/lib/x86_64-linux-gnu/libtiff.so.5
            ldconfig
        else
            echo "Unable to provide libtiff.so.5 compatibility" >&2
            exit 1
        fi
    fi
fi

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

export DOCKER_ROS_FILES_PATH=/docker-ros/additional-files
export SCENARIO_RUNNER_ROOT=$DOCKER_ROS_FILES_PATH

# Install Scenario Runner Python dependencies from the copied repository.
python -m pip install -r "$SCENARIO_RUNNER_ROOT/requirements.txt"

# Install missing ROS dependencies after python3-psutil removal.
apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros2cli \
    ros-$ROS_DISTRO-ros2cli-common-extensions

# Download and install CARLA PythonAPI artifacts.
mkdir -p /opt/carla
curl --location --output artifacts.zip "https://gitlab.ika.rwth-aachen.de/api/v4/projects/1645/jobs/artifacts/main/download?job=provide-carla-artifacts&job_token=$GIT_HTTPS_PASSWORD"
unzip -q artifacts.zip
mv artifacts/PythonAPI /opt/carla
rm -rf artifacts artifacts.zip

# Install PythonAPI requirements, keeping the version of the first occurrence.
find /opt/carla/PythonAPI -type f -name "requirements.txt" -print0 | xargs -0 cat > /tmp/carla-pythonapi-requirements-raw.txt
awk -F '==' '{print $1}' /tmp/carla-pythonapi-requirements-raw.txt | awk '!visited[$1]++' > /tmp/carla-pythonapi-requirements.txt
python -m pip install -r /tmp/carla-pythonapi-requirements.txt

# Install the CARLA wheel that matches the current Python minor version.
pyver=$(python -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')")
wheel=$(echo /opt/carla/PythonAPI/carla/dist/*${pyver}*.whl)
python -m pip install --no-cache-dir "$wheel"

# Create a script to append necessary paths to PYTHONPATH
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla/agents" >> /opt/carla/setup.bash
echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla" >> /opt/carla/setup.bash
echo "export PYTHONPATH=\$PYTHONPATH:$SCENARIO_RUNNER_ROOT" >> /opt/carla/setup.bash

# Set the SCENARIO_RUNNER_ROOT environment variable
echo "export SCENARIO_RUNNER_ROOT=$SCENARIO_RUNNER_ROOT" >> /opt/carla/setup.bash

# Default file cache for CARLA client-side map files
echo "export CARLA_CACHE_DIR=/tmp/carlaCache" >> /opt/carla/setup.bash
echo "mkdir -p /tmp/carlaCache" >> /opt/carla/setup.bash

# .bashrc sources the setup script
echo "source /opt/carla/setup.bash" >> /root/.bashrc
