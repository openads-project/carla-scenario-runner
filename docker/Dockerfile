ARG UBUNTU_VERSION="22.04"

############ dependencies ############
FROM "ubuntu:${UBUNTU_VERSION}"

ARG ARTIFACTS_URL
ARG CI_JOB_TOKEN
ARG CI_REPOSITORY_URL

USER root
SHELL ["/bin/bash", "-c"]

ARG CARLA_PATH=/opt/carla
ARG CARLA_API_PATH=/opt/carla/PythonAPI
ARG SCENARIO_RUNNER_PATH=/opt/carla/carla-scenario-runner

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
      libtiff5 \
      libxerces-c-dev \
      unzip \
      wget \
      x11-apps \
      python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10

# Install scenario_runner
COPY . $SCENARIO_RUNNER_PATH
RUN pip3 install -r $SCENARIO_RUNNER_PATH/requirements.txt
RUN pip3 install Flask

# Download and extract artifacts
RUN curl --location --output artifacts.zip "${ARTIFACTS_URL}&job_token=${CI_JOB_TOKEN}" && \
    unzip artifacts.zip && \
    mkdir -p ${CARLA_PATH} && \
    mv artifacts/PythonAPI ${CARLA_PATH}

# Recursively install PythonAPI requirements, keep version of first occurrence
# NOTE: Installing open3d somehow leads to several unavoidable upgrades, especially to Numpy>=2.0.0
# This breaks Scenario Runner at the moment, thus open3d needs to be filtered out for the moment
RUN cat $(find ${CARLA_API_PATH} -type f -name "requirements.txt") > /tmp/requirements_raw.txt \
    && awk -F '==' '{print $1}' /tmp/requirements_raw.txt | awk '!visited[$1]++' > /tmp/requirements.txt \
    && sed -i '/^open3d$/d' /tmp/requirements.txt \
    && pip3 install --upgrade-strategy only-if-needed -r /tmp/requirements.txt

# Create script that adds API to pythonpath and make .bashrc source it
RUN echo "export PYTHONPATH=\$PYTHONPATH:$CARLA_API_PATH/carla/dist/$(ls $CARLA_API_PATH/carla/dist | grep py$(python --version | awk -F'[ .]' '{print $2"."$3}').)" >> /setup_carla_env.sh; \
    echo "export PYTHONPATH=\$PYTHONPATH:$CARLA_API_PATH/carla/agents" >> /setup_carla_env.sh; \
    echo "export PYTHONPATH=\$PYTHONPATH:$CARLA_API_PATH/carla" >> /setup_carla_env.sh; \
    echo "export CARLA_API_PATH=$CARLA_API_PATH" >> /setup_carla_env.sh; \
    echo "source /setup_carla_env.sh" >> ~/.bashrc

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
