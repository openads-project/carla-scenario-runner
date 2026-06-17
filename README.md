# carla-scenario-runner

<p align="center">
  <a href="https://github.com/openads-project"><img src="https://img.shields.io/badge/OpenADS-f5ff01"/></a>
  <a href="https://github.com/openads-project/carla-scenario-runner/releases/latest"><img src="https://img.shields.io/github/v/release/openads-project/carla-scenario-runner"/></a>
  <a href="https://github.com/openads-project/carla-scenario-runner/blob/main/LICENSE"><img src="https://img.shields.io/github/license/openads-project/carla-scenario-runner"/></a>
  <br>
  <img src="https://img.shields.io/badge/Ubuntu-24.04-E95420"/>
  <img src="https://img.shields.io/badge/CARLA-0.10.0-blueviolet"/>
  <img src="https://img.shields.io/badge/Python-3.12-blueviolet"/>
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <a href="https://github.com/openads-project/carla-scenario-runner/actions/workflows/docker.yml"><img src="https://github.com/openads-project/carla-scenario-runner/actions/workflows/docker.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/carla-scenario-runner/actions/workflows/docker-ros.yml"><img src="https://github.com/openads-project/carla-scenario-runner/actions/workflows/docker-ros.yml/badge.svg"/></a>
</p>

**OpenSCENARIO execution for scenario-based testing and evaluation in CARLA**

> [!IMPORTANT]
> This repository is a fork of the official CARLA [scenario_runner](https://github.com/carla-simulator/scenario_runner). All initial and following modifications to the original repository are documented in [CHANGELOG_OPENADS.md](./CHANGELOG_OPENADS.md).

> [!IMPORTANT]
> This repository is part of [***OpenADS***](https://github.com/openads-project), the *Open Automated Driving Systems* project. *OpenADS* and its modules have been initiated and are currently being maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/de/).

> [!TIP]
> We recommend to use the *carla-scenario-runner* as **control actor** in our open, modular and scalable simulation framework <a href="https://github.com/openads-project/openadsim">**OpenADSim**.

---
---
## Original README

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![GitHub tag (latest SemVer)](https://img.shields.io/github/tag/carla-simulator/scenario_runner.svg)
[![Build Status](https://travis-ci.com/carla-simulator/scenario_runner.svg?branch=master)](https://travis-ci.com/carla/scenario_runner)

This repository contains traffic scenario definition and an execution engine
for CARLA. It also allows the execution of a simulation of the CARLA Challenge.
You can use this system to prepare your agent for the CARLA Challenge.

Scenarios can be defined through a Python interface, and with the newest version
the scenario_runner also the upcoming [OpenSCENARIO](http://www.openscenario.org/) standard and [OpenSCENARIO 2.0](https://www.asam.net/standards/detail/openscenario/v200/) standard is supported.

[Read the documentation here](https://scenario-runner.readthedocs.io/en/latest/).

[![Scenario_Runner for CARLA](Docs/img/scenario_runner_video.png)](https://youtu.be/ChmF8IFagpo?t=68)

Getting the ScenarioRunner
---------------------------

Use `git clone` or download the project from this page. Note that the master
branch contains the latest fixes and features, and may be required to use the latest features from CARLA.

It is important to also consider the release version that has to match the CARLA version.

* [Version 0.9.16](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.16) and the 0.9.16 Branch: Compatible with [CARLA 0.9.16](https://github.com/carla-simulator/carla/tree/0.9.16)
* [Version 0.9.15](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.15) and the 0.9.15 Branch: Compatible with [CARLA 0.9.15](https://github.com/carla-simulator/carla/releases/tag/0.9.15)
* [Version 0.9.13](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.13) and the 0.9.13 Branch: Compatible with [CARLA 0.9.13](https://github.com/carla-simulator/carla/releases/tag/0.9.13) and [CARLA 0.9.14](https://github.com/carla-simulator/carla/releases/tag/0.9.14)
* [Version 0.9.12](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.12) and the 0.9.12 Branch: Compatible with [CARLA 0.9.12](https://github.com/carla-simulator/carla/releases/tag/0.9.12)
* [Version 0.9.11](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.11) and the 0.9.11 Branch: Compatible with [CARLA 0.9.11](https://github.com/carla-simulator/carla/releases/tag/0.9.11)
* [Version 0.9.10](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.10) and the 0.9.10 Branch: Compatible with [CARLA 0.9.10](https://github.com/carla-simulator/carla/releases/tag/0.9.10)
* [Version 0.9.9](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.9) and the 0.9.9 Branch: Compatible with [CARLA 0.9.9](https://github.com/carla-simulator/carla/releases/tag/0.9.9). Use the 0.9.9 branch, if you use CARLA 0.9.9.4.
* [Version 0.9.8](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.8) and the 0.9.8 Branch: Compatible with [CARLA 0.9.8](https://github.com/carla-simulator/carla/releases/tag/0.9.8)
* [Version 0.9.7](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.7) and the 0.9.7 Branch: Compatible with [CARLA 0.9.7](https://github.com/carla-simulator/carla/releases/tag/0.9.7) but not with the later release patch versions.
* [Version 0.9.6](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.6) and the 0.9.6 Branch: Compatible with [CARLA 0.9.6](https://github.com/carla-simulator/carla/releases/tag/0.9.6)
* [Version 0.9.5](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.5) and [Version 0.9.5.1](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.5.1): Compatible with [CARLA 0.9.5](https://github.com/carla-simulator/carla/releases/tag/0.9.5)
* [Version 0.9.2](https://github.com/carla-simulator/scenario_runner/releases/tag/0.9.2): Compatible with [CARLA 0.9.2](https://github.com/carla-simulator/carla/releases/tag/0.9.2)

To use a particular version you can either download the corresponding tarball or simply checkout the version tag associated to the release (e.g. git checkout v0.9.5)

Currently no build is required, as all code is in Python.

Using the ScenarioRunner
------------------------

Please take a look at our [Getting started](https://scenario-runner.readthedocs.io/en/latest/getting_scenariorunner/)
documentation.

Challenge Evaluation
---------------------

The CARLA Challenge has moved to the [CARLA Autonomous Driving Leaderboard](https://leaderboard.carla.org/). Please see the [leaderboard repository](https://github.com/carla-simulator/leaderboard) and the [getting started guide](https://leaderboard.carla.org/get_started/) for more information.

Contributing
------------

Please take a look at our [Contribution guidelines](https://carla.readthedocs.io/en/latest/#contributing).

FAQ
------

If you run into problems, check our
[FAQ](http://carla.readthedocs.io/en/latest/faq/).

License
-------

ScenarioRunner specific code is distributed under MIT License.
