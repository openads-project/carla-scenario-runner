ARG UBUNTU_VERSION="24.04"
ARG ARTIFACTS_URL
ARG CI_JOB_TOKEN

############ dependencies ############
FROM "ubuntu:${UBUNTU_VERSION}"

USER root
SHELL ["/bin/bash", "-c"]

ARG CARLA_PATH=/opt/carla
ARG CARLA_API_PATH=/opt/carla/PythonAPI
ARG SCENARIO_RUNNER_PATH=/opt/carla/carla-scenario-runner

ARG ARTIFACTS_URL
ARG CI_JOB_TOKEN

# Install essentials
RUN apt-get update && \
    apt-get install -y \
      build-essential \
      curl \
      fontconfig \
      git \
      libgeos-dev \
      libjpeg8 \
      libomp5 \
      libpng16-16 \
      libtiff6 \
      libxerces-c-dev \
      unzip \
      x11-apps \
      python3-pip \
    && apt-get remove -y python3-blinker \
    && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 12

# Install scenario_runner
COPY . $SCENARIO_RUNNER_PATH
RUN CARLA_ARTIFACTS_URL="${ARTIFACTS_URL}&job_token=${CI_JOB_TOKEN}" \
    SCENARIO_RUNNER_ROOT="${SCENARIO_RUNNER_ROOT}" \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    bash "$SCENARIO_RUNNER_PATH/docker/install.sh" && \
    echo "source /opt/carla/setup.bash.sh" >> ~/.bashrc

RUN pip3 install Flask --break-system-packages

# Needed for (pygame based) scripts that have a GUI
ENV SDL_VIDEODRIVER=x11

USER root
SHELL ["/bin/bash", "-c"]

ARG WORKSPACE=${SCENARIO_RUNNER_PATH}
ENV WORKSPACE=${WORKSPACE}
WORKDIR ${WORKSPACE}

# Set entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

CMD [ "python3", "scenario_runner.py", "--help" ]
