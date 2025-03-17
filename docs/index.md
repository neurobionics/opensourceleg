# opensourceleg

[![Build status](https://github.com/neurobionics/opensourceleg/workflows/build/badge.svg)](https://github.com/neurobionics/opensourceleg/actions?query=workflow%3Abuild)
[![Documentation Status](https://github.com/neurobionics/opensourceleg/actions/workflows/pages/pages-build-deployment/badge.svg)](https://neurobionics.github.io/opensourceleg/)
[![Python Version](https://img.shields.io/pypi/pyversions/opensourceleg.svg)](https://pypi.org/project/opensourceleg/)
[![Dependencies Status](https://img.shields.io/badge/dependencies-up%20to%20date-brightgreen.svg)](https://github.com/neurobionics/opensourceleg/pulls?utf8=%E2%9C%93&q=is%3Apr%20author%3Aapp%2Fdependabot)

[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Security: bandit](https://img.shields.io/badge/security-bandit-green.svg)](https://github.com/PyCQA/bandit)
[![Pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/neurobionics/opensourceleg/blob/main/.pre-commit-config.yaml)
[![License](https://img.shields.io/github/license/neurobionics/opensourceleg)](https://github.com/neurobionics/opensourceleg/blob/main/LICENSE)
![Coverage Report](https://github.com/neurobionics/opensourceleg/blob/main/assets/images/coverage.svg)

An open-source Software Development Kit (SDK) for robotics development, testing, and deployment. Originally developed for the [Open-Source Leg](https://www.opensourceleg.org/) project, this library provides a comprehensive framework for interfacing with various actuators and sensors in robotic systems. While initially designed for prosthetic leg applications, the `opensourceleg` library's modular architecture makes it versatile enough for any robotic platform utilizing similar components.

> We are currently testing the new version of the library (main branch), and the PyPI release will be updated soon. If you are looking for the source behind the existing PyPI release, please refer to the [legacy branch](https://github.com/neurobionics/opensourceleg/tree/legacy).

<img src="https://github.com/neurobionics/opensourceleg/blob/66ad4289ef9ba8701fac9337778f87b657286484/assets/images/banner.gif?raw=true" width="800" title="Open-Source Leg">

## 🎯 Key Features

This library solves common challenges in developing, testing, and deploying robotic algorithms:

| Feature | Description |
|---------|-------------|
| 📦 Standardized Interfaces | Provides consistent interfaces for common actuators and sensors |
| 🔄 Ready-to-Use Implementations | Offers ready-to-use implementations for popular hardware components |
| 🔍 Extensible Architecture | Allows for easy integration of custom components |
| 🧪 Comprehensive Benchmarks | Includes comprehensive benchmarks for popular hardware components |

## 👥 Ideal for Roboticists Who

- Want to develop robotic algorithms for the Open-Source Leg platform or any other robotic platform
- Need a reliable and extensible framework for interfacing with various actuators and sensors
- Are working on a robotic project and need a flexible and powerful software development kit
- Are looking for benchmarks to pick the best hardware for their robotic project

## Available Hardware Interfaces

The library currently supports the following hardware components:

| Actuators | Unit Tests | Hardware Tests | Benchmarks | Documentation |
|-----------|------------|-------------------|------------|---------------|
| Dephy Actpack | ✅ | ✅ | ⚠️ | ✅ |
| Moteus | ✅ | ⚠️ | ⚠️ | ✅ |
| TMotor | ✅ | ⚠️ | ❌ | ⚠️ |

| Sensors | Unit Tests | Hardware Tests | Benchmarks | Documentation |
|---------|------------|-------------------|------------|---------------|
| AS5048B Encoder | ✅ | ✅ | ❌ | ✅ |
| Lord Microstrain IMU | ✅ | ✅ | ❌ | ✅ |
| SRI Loadcell | ✅ | ✅ | ❌ | ✅ |

> Legend: ✅ Complete/Available; ⚠️ Partial/In Progress; ❌ Not Yet Available

> Hardware tests indicate successful testing on physical devices. Benchmarks include performance metrics such as response time and accuracy measurements. Documentation includes API reference and usage examples.

## Installation

The library is available on PyPI and can be installed using pip:

```bash
pip install opensourceleg
```

For more details on the installation process, please refer to the [installation guide](https://neurobionics.github.io/opensourceleg/installation).


## Usage

Once the library is installed, you can import it in your projects and start using the modules:

```python
from opensourceleg.actuators import ActuatorBase
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.sensors import SensorBase
from opensourceleg.sensors.encoder import AS5048B
...
```

For more details on available modules, tutorials, and examples, please refer to the [documentation](https://neurobionics.github.io/opensourceleg/tutorials/getting_started).


## License

The `opensourceleg` library is licensed under the terms of the [LGPL-v2.1 license](https://github.com/neurobionics/opensourceleg/raw/main/LICENSE). This license grants users a number of freedoms:

- You are free to use the `opensourceleg` library for any purpose.
- You are free to modify the `opensourceleg` library to suit your needs.
- You can study how the `opensourceleg` library works and change it.
- You can distribute modified versions of the `opensourceleg` library.

The LGPL license ensures that all these freedoms are protected, now and in the future, requiring everyone to share their modifications when they also share the library in public.

## Contributing

Contributions are welcome, and they are greatly appreciated! For more details, read our [contribution guidelines](https://github.com/neurobionics/opensourceleg/blob/main/CONTRIBUTING.md).