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
      wget \
      x11-apps \
      python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 12

# Install scenario_runner
COPY . $SCENARIO_RUNNER_PATH
RUN pip3 install -r $SCENARIO_RUNNER_PATH/requirements.txt --break-system-packages
RUN pip3 install Flask --break-system-packages

# Download and extract artifacts
RUN curl --location --output artifacts.zip "${ARTIFACTS_URL}&job_token=${CI_JOB_TOKEN}" && \
    unzip artifacts.zip && \
    mkdir -p ${CARLA_PATH} && \
    mv artifacts/PythonAPI ${CARLA_PATH}

# Recursively install PythonAPI requirements, keep version of first occurrence
RUN cat $(find ${CARLA_API_PATH} -type f -name "requirements.txt") > /tmp/requirements_raw.txt \
    && awk -F '==' '{print $1}' /tmp/requirements_raw.txt | awk '!visited[$1]++' > /tmp/requirements.txt \
    && pip3 install -r /tmp/requirements.txt --break-system-packages

# Install CARLA PythonAPI wheel
RUN set -e; \
    pyver=$(python3 -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')"); \
    wheel=$(echo ${CARLA_API_PATH}/carla/dist/*${pyver}*.whl); \
    pip install --no-cache-dir "$wheel" --break-system-packages

# Create script that adds API to pythonpath and make .bashrc source it
RUN echo "export PYTHONPATH=\$PYTHONPATH:$CARLA_API_PATH/carla/agents" >> /setup_carla_env.sh; \
    echo "export PYTHONPATH=\$PYTHONPATH:$CARLA_API_PATH/carla" >> /setup_carla_env.sh; \
    echo "export CARLA_API_PATH=$CARLA_API_PATH" >> /setup_carla_env.sh; \
    echo "source /setup_carla_env.sh" >> ~/.bashrc

# Default file cache for CARLA client-side map files
ENV CARLA_CACHE_DIR=/tmp/carlaCache
RUN mkdir -p /tmp/carlaCache

# Needed for (pygame based) scripts that have a GUI
ENV SDL_VIDEODRIVER=x11

USER root
SHELL ["/bin/bash", "-c"]

ENV SCENARIO_RUNNER_ROOT=${SCENARIO_RUNNER_PATH}

ARG WORKSPACE=${SCENARIO_RUNNER_PATH}
ENV WORKSPACE=${WORKSPACE}
WORKDIR ${WORKSPACE}

# Set entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

CMD [ "python3", "scenario_runner.py", "--help" ]
