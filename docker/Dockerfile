ARG UBUNTU_VERSION="22.04"

############ PythonAPI artifacts ############
FROM "ubuntu:${UBUNTU_VERSION}" as artifacts

USER root
SHELL ["/bin/bash", "-c"]
ARG ARTIFACTS_URL
ARG CI_TOKEN_USER
ARG CI_JOB_TOKEN

# Install curl + unzip
RUN apt-get update && \
    apt-get install -y \
      curl \
      unzip \
    && rm -rf /var/lib/apt/lists/*

# Download and extract artifacts
RUN curl --location --output artifacts.zip "${ARTIFACTS_URL}&job_token=${CI_JOB_TOKEN}" && \
    unzip artifacts.zip

ARG UBUNTU_VERSION

############ dependencies ############
FROM "ubuntu:${UBUNTU_VERSION}" as dependencies

USER root
SHELL ["/bin/bash", "-c"]

ARG CARLA_API_PATH=/opt/carla/PythonAPI
ARG SCENARIO_RUNNER_API_PATH=/opt/carla/scenario_runner

# Install essentials
RUN apt-get update && \
    apt-get install -y \
      fontconfig \
      x11-apps \
      python3-pip \
      libomp5 \
      libjpeg-turbo8 libtiff-dev \
      libgeos-dev \
      git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10

# Copy over PythonAPI
COPY --from=artifacts artifacts_ci/PythonAPI ${CARLA_API_PATH}

# Recursively install PythonAPI requirements, keep version of first occurrence
RUN cat $(find ${CARLA_API_PATH} -type f -name "requirements.txt") > /tmp/requirements_raw.txt \
    && awk -F '==' '{print $1}' /tmp/requirements_raw.txt | awk '!visited[$1]++' > /tmp/requirements.txt \
    && pip3 install -r /tmp/requirements.txt

# Create script that adds API to pythonpath and make .bashrc source it
RUN echo "export PYTHONPATH=\$PYTHONPATH:$CARLA_API_PATH/carla/dist/$(ls $CARLA_API_PATH/carla/dist | grep py$(python --version | awk -F'[ .]' '{print $2"."$3}').)" >> /setup_carla_env.sh; \
    echo "export PYTHONPATH=\$PYTHONPATH:$CARLA_API_PATH/carla/agents" >> /setup_carla_env.sh; \
    echo "export PYTHONPATH=\$PYTHONPATH:$CARLA_API_PATH/carla" >> /setup_carla_env.sh; \
    echo "export CARLA_API_PATH=$CARLA_API_PATH" >> /setup_carla_env.sh; \
    echo "source /setup_carla_env.sh" >> ~/.bashrc

# Install scenario_runner
RUn git clone --recurse-submodules https://${CI_TOKEN_USER}:${CI_JOB_TOKEN}@gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/scenario_runner.git $SCENARIO_RUNNER_API_PATH

RUN pip3 install -r $SCENARIO_RUNNER_API_PATH/requirements.txt

############ final ############
FROM dependencies as final

# Needed for (pygame based) scripts that have a GUI
ENV SDL_VIDEODRIVER=x11

USER root
SHELL ["/bin/bash", "-c"]

ARG WORKSPACE=/opt/carla/scenario_runner
ENV WORKSPACE=${WORKSPACE}
WORKDIR ${WORKSPACE}

# Set entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

CMD [ "python3", "../PythonAPI/util/test_connection.py --host carla-server" ]
