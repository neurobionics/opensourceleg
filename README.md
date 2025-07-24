<p align="center">
  <img src="https://raw.githubusercontent.com/neurobionics/opensourceleg/refs/heads/main/assets/images/banner.gif" width="100%">
</p>

<div align="center">

[![Build status](https://github.com/neurobionics/opensourceleg/actions/workflows/main.yml/badge.svg)](https://github.com/neurobionics/opensourceleg/actions/workflows/main.yml)
[![Documentation Status](https://github.com/neurobionics/opensourceleg/actions/workflows/pages/pages-build-deployment/badge.svg)](https://neurobionics.github.io/opensourceleg/)
[![Python Version](https://img.shields.io/pypi/pyversions/opensourceleg.svg)](https://pypi.org/project/opensourceleg/)
[![PyPI Downloads](https://img.shields.io/pepy/dt/opensourceleg?color=blue)](https://pepy.tech/projects/opensourceleg)


[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Security: bandit](https://img.shields.io/badge/security-bandit-green.svg)](https://github.com/PyCQA/bandit)
[![Pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/neurobionics/opensourceleg/blob/main/.pre-commit-config.yaml)
[![License](https://img.shields.io/github/license/neurobionics/opensourceleg)](https://github.com/neurobionics/opensourceleg/blob/main/LICENSE)
[![Coverage](https://raw.githubusercontent.com/neurobionics/opensourceleg/refs/heads/main/assets/images/coverage.svg)](https://github.com/neurobionics/opensourceleg/actions/workflows/main.yml)
<!-- ALL-CONTRIBUTORS-BADGE:START - Do not remove or modify this section -->
[![All Contributors](https://img.shields.io/badge/all_contributors-18-orange.svg?style=flat-square)](#contributors-)
<!-- ALL-CONTRIBUTORS-BADGE:END -->

</div>

**opensourceleg**: An open-source SDK for developing and testing algorithms on commonly used robotic hardware. Originally developed for the [Open-Source Leg](https://www.opensourceleg.org/) project, this library provides a comprehensive framework for interfacing with various actuators and sensors in robotic systems. While initially designed for prosthetic leg applications, the `opensourceleg` library's modular architecture makes it versatile for use with any robotic platform utilizing similar components.

> **Note:** Starting from version `opensourceleg 3.3.0`, only Python 3.10 and above are supported.
> If you need compatibility with Python 3.9, please install `opensourceleg 3.2.1`:
> ```bash
> pip install opensourceleg==3.2.1
> ```

## 🎯 Key Features

This library solves common challenges in developing, testing, and deploying robotic algorithms:

| Feature                         | Description                                                         |
| ------------------------------- | ------------------------------------------------------------------- |
| 📦 Standardized Interfaces      | Provides consistent interfaces for common actuators and sensors     |
| 🔄 Ready-to-Use Implementations | Offers ready-to-use implementations for popular hardware components |
| 🔍 Extensible Architecture      | Allows for easy integration of custom components                    |
| 🧪 Comprehensive Benchmarks     | Includes comprehensive benchmarks for popular hardware components   |

## 👥 Ideal for Roboticists Who

- Want to develop robotic algorithms for the Open-Source Leg platform or any other robotic platform
- Need a reliable and extensible framework for interfacing with various actuators and sensors
- Are working on a robotic project and need a flexible and powerful software development kit
- Are looking for benchmarks to pick the best hardware for their robotic project

<details>
<summary>🔄 Available Hardware Interfaces</summary>

The library currently supports the following hardware components:

| Sensors              | Unit Tests | Hardware Tests | Benchmarks | Documentation |
| -------------------- | ---------- | -------------- | ---------- | ------------- |
| AS5048B Encoder      | ✅         | ✅             | ❌         | ✅            |
| Lord Microstrain IMU | ✅         | ✅             | ❌         | ✅            |
| SRI Loadcell         | ✅         | ✅             | ❌         | ✅            |

| Actuators     | Unit Tests | Hardware Tests | Benchmarks | Documentation |
| ------------- | ---------- | -------------- | ---------- | ------------- |
| Dephy Actpack | ⚠️         | ✅             | ⚠️         | ✅            |
| Moteus        | ⚠️         | ⚠️             | ⚠️         | ✅            |
| TMotor        | ❌         | ⚠️             | ❌         | ❌            |

> Legend: ✅ Complete/Available; ⚠️ Partial/In Progress; ❌ Not Yet Available;

Hardware tests indicate successful testing on physical devices.
Benchmarks include performance metrics such as response time and accuracy measurements.
Documentation includes API reference and usage examples.

</details>

## 📦 Installation

The library is available on PyPI and can be installed using pip:

```bash
pip install opensourceleg
```

<details>
<summary>🔧 Hardware-Specific Dependencies</summary>

To keep your installation lightweight, you can install only the dependencies needed for your specific hardware:

```bash
# For Dephy actuators
pip install opensourceleg[dephy]

# For Moteus actuators
pip install opensourceleg[moteus]

# For I2C communication
pip install opensourceleg[communication]
```

| Extra           | Dependencies                         |
| --------------- | ------------------------------------ |
| `dephy`         | flexsea                              |
| `moteus`        | moteus, moteus-pi3hat                |
| `communication` | smbus2                               |
| `messaging`     | grpcio, grpcio-tools, types-protobuf |

</details>

For more details on the installation process, please refer to the [installation guide](https://neurobionics.github.io/opensourceleg/installation).

## 📚 Usage

Once the library is installed, you can import it in your projects and start using the modules:

```python
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.sensors.encoder import AS5048B
...
```

For more details on available modules, tutorials, and examples, please refer to the [documentation](https://neurobionics.github.io/opensourceleg/tutorials/sensors/getting_started).

## [![Repography logo](https://images.repography.com/logo.svg)](https://repography.com) / Community Activity [![Time period](https://images.repography.com/63788890/neurobionics/opensourceleg/recent-activity/iG94TwYB5IO_esvflL1mNk_1EL1bAD6JZ1guhH_p4ek/eOAVZwtHiffhksyJHZ7XihsPlw5JaT6XcoWMERSIhHs_badge.svg)](https://repography.com)

[![Timeline graph](https://images.repography.com/63788890/neurobionics/opensourceleg/recent-activity/iG94TwYB5IO_esvflL1mNk_1EL1bAD6JZ1guhH_p4ek/eOAVZwtHiffhksyJHZ7XihsPlw5JaT6XcoWMERSIhHs_timeline.svg)](https://github.com/neurobionics/opensourceleg/commits)
[![Issue status graph](https://images.repography.com/63788890/neurobionics/opensourceleg/recent-activity/iG94TwYB5IO_esvflL1mNk_1EL1bAD6JZ1guhH_p4ek/eOAVZwtHiffhksyJHZ7XihsPlw5JaT6XcoWMERSIhHs_issues.svg)](https://github.com/neurobionics/opensourceleg/issues)
[![Pull request status graph](https://images.repography.com/63788890/neurobionics/opensourceleg/recent-activity/iG94TwYB5IO_esvflL1mNk_1EL1bAD6JZ1guhH_p4ek/eOAVZwtHiffhksyJHZ7XihsPlw5JaT6XcoWMERSIhHs_prs.svg)](https://github.com/neurobionics/opensourceleg/pulls)
[![Top contributors](https://images.repography.com/63788890/neurobionics/opensourceleg/recent-activity/iG94TwYB5IO_esvflL1mNk_1EL1bAD6JZ1guhH_p4ek/eOAVZwtHiffhksyJHZ7XihsPlw5JaT6XcoWMERSIhHs_users.svg)](https://github.com/neurobionics/opensourceleg/graphs/contributors)
[![Trending topics](https://images.repography.com/63788890/neurobionics/opensourceleg/recent-activity/iG94TwYB5IO_esvflL1mNk_1EL1bAD6JZ1guhH_p4ek/eOAVZwtHiffhksyJHZ7XihsPlw5JaT6XcoWMERSIhHs_words.svg)](https://github.com/neurobionics/opensourceleg/commits)
[![Activity map](https://images.repography.com/63788890/neurobionics/opensourceleg/recent-activity/iG94TwYB5IO_esvflL1mNk_1EL1bAD6JZ1guhH_p4ek/eOAVZwtHiffhksyJHZ7XihsPlw5JaT6XcoWMERSIhHs_map.svg)](https://github.com/neurobionics/opensourceleg/commits)

## Contributors ✨

Thanks to all the wonderful people who have contributed to the project!

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="20%"><a href="https://senthurayyappan.com"><img src="https://avatars.githubusercontent.com/u/25511437?v=4?s=100" width="100px;" alt="Senthur Ayyappan"/><br /><sub><b>Senthur Ayyappan</b></sub></a><br /><a href="#infra-senthurayyappan" title="Infrastructure (Hosting, Build-Tools, etc)">🚇</a> <a href="#security-senthurayyappan" title="Security">🛡️</a> <a href="#maintenance-senthurayyappan" title="Maintenance">🚧</a> <a href="https://github.com/neurobionics/opensourceleg/commits?author=senthurayyappan" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="http://tkevinbest.github.io"><img src="https://avatars.githubusercontent.com/u/70407790?v=4?s=100" width="100px;" alt="tkevinbest"/><br /><sub><b>tkevinbest</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/issues?q=author%3Atkevinbest" title="Bug reports">🐛</a> <a href="https://github.com/neurobionics/opensourceleg/commits?author=tkevinbest" title="Tests">⚠️</a> <a href="#example-tkevinbest" title="Examples">💡</a> <a href="https://github.com/neurobionics/opensourceleg/commits?author=tkevinbest" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/jderosia"><img src="https://avatars.githubusercontent.com/u/134736683?v=4?s=100" width="100px;" alt="jderosia"/><br /><sub><b>jderosia</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/commits?author=jderosia" title="Documentation">📖</a> <a href="https://github.com/neurobionics/opensourceleg/commits?author=jderosia" title="Tests">⚠️</a> <a href="#example-jderosia" title="Examples">💡</a> <a href="https://github.com/neurobionics/opensourceleg/commits?author=jderosia" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/Robin0265"><img src="https://avatars.githubusercontent.com/u/69673450?v=4?s=100" width="100px;" alt="Yuanshao Yang"/><br /><sub><b>Yuanshao Yang</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/commits?author=Robin0265" title="Tests">⚠️</a> <a href="https://github.com/neurobionics/opensourceleg/issues?q=author%3ARobin0265" title="Bug reports">🐛</a> <a href="https://github.com/neurobionics/opensourceleg/commits?author=Robin0265" title="Documentation">📖</a> <a href="https://github.com/neurobionics/opensourceleg/commits?author=Robin0265" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/shreyhas"><img src="https://avatars.githubusercontent.com/u/68916446?v=4?s=100" width="100px;" alt="shreyhas"/><br /><sub><b>shreyhas</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/commits?author=shreyhas" title="Documentation">📖</a> <a href="https://github.com/neurobionics/opensourceleg/commits?author=shreyhas" title="Tests">⚠️</a> <a href="#example-shreyhas" title="Examples">💡</a></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="20%"><a href="https://github.com/jkotar3"><img src="https://avatars.githubusercontent.com/u/166853036?v=4?s=100" width="100px;" alt="jkotar3"/><br /><sub><b>jkotar3</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/commits?author=jkotar3" title="Tests">⚠️</a> <a href="https://github.com/neurobionics/opensourceleg/issues?q=author%3Ajkotar3" title="Bug reports">🐛</a> <a href="https://github.com/neurobionics/opensourceleg/commits?author=jkotar3" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/esharnow"><img src="https://avatars.githubusercontent.com/u/99085536?v=4?s=100" width="100px;" alt="esharnow"/><br /><sub><b>esharnow</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/commits?author=esharnow" title="Tests">⚠️</a> <a href="https://github.com/neurobionics/opensourceleg/issues?q=author%3Aesharnow" title="Bug reports">🐛</a> <a href="https://github.com/neurobionics/opensourceleg/commits?author=esharnow" title="Documentation">📖</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/JapmanGill"><img src="https://avatars.githubusercontent.com/u/82068921?v=4?s=100" width="100px;" alt="Japman Gill"/><br /><sub><b>Japman Gill</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/commits?author=JapmanGill" title="Tests">⚠️</a> <a href="https://github.com/neurobionics/opensourceleg/issues?q=author%3AJapmanGill" title="Bug reports">🐛</a> <a href="https://github.com/neurobionics/opensourceleg/commits?author=JapmanGill" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/unshrawal"><img src="https://avatars.githubusercontent.com/u/32817861?v=4?s=100" width="100px;" alt="unshrawal"/><br /><sub><b>unshrawal</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/issues?q=author%3Aunshrawal" title="Bug reports">🐛</a> <a href="https://github.com/neurobionics/opensourceleg/commits?author=unshrawal" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/anujtaosf"><img src="https://avatars.githubusercontent.com/u/123828257?v=4?s=100" width="100px;" alt="anujtaosf"/><br /><sub><b>anujtaosf</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/commits?author=anujtaosf" title="Documentation">📖</a> <a href="#example-anujtaosf" title="Examples">💡</a></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="20%"><a href="https://github.com/VarunSatyadevShetty"><img src="https://avatars.githubusercontent.com/u/62276853?v=4?s=100" width="100px;" alt="Varun Satyadev Shetty"/><br /><sub><b>Varun Satyadev Shetty</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/commits?author=VarunSatyadevShetty" title="Documentation">📖</a> <a href="#example-VarunSatyadevShetty" title="Examples">💡</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/Katharine-Walters"><img src="https://avatars.githubusercontent.com/u/111811694?v=4?s=100" width="100px;" alt="Katharine-Walters"/><br /><sub><b>Katharine-Walters</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/issues?q=author%3AKatharine-Walters" title="Bug reports">🐛</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/zachbons"><img src="https://avatars.githubusercontent.com/u/74107027?v=4?s=100" width="100px;" alt="zachbons"/><br /><sub><b>zachbons</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/issues?q=author%3Azachbons" title="Bug reports">🐛</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/anushkarathi"><img src="https://avatars.githubusercontent.com/u/98593597?v=4?s=100" width="100px;" alt="Anushka"/><br /><sub><b>Anushka</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/issues?q=author%3Aanushkarathi" title="Bug reports">🐛</a></td>
      <td align="center" valign="top" width="20%"><a href="https://nundinir.github.io/"><img src="https://avatars.githubusercontent.com/u/90918772?v=4?s=100" width="100px;" alt="Nundini Rawal"/><br /><sub><b>Nundini Rawal</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/issues?q=author%3Anundinir" title="Bug reports">🐛</a></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="20%"><a href="https://github.com/matteo-crotti"><img src="https://avatars.githubusercontent.com/u/32237604?v=4?s=100" width="100px;" alt="matteo-crotti"/><br /><sub><b>matteo-crotti</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/issues?q=author%3Amatteo-crotti" title="Bug reports">🐛</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/JerryGu-leiting"><img src="https://avatars.githubusercontent.com/u/178481044?v=4?s=100" width="100px;" alt="Jiarui Gu"/><br /><sub><b>Jiarui Gu</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/commits?author=JerryGu-leiting" title="Documentation">📖</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/gpranav162"><img src="https://avatars.githubusercontent.com/u/132157503?v=4?s=100" width="100px;" alt="Pranav G"/><br /><sub><b>Pranav G</b></sub></a><br /><a href="https://github.com/neurobionics/opensourceleg/commits?author=gpranav162" title="Documentation">📖</a> <a href="https://github.com/neurobionics/opensourceleg/commits?author=gpranav162" title="Code">💻</a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

Contributions of any kind are welcome! For more details, read our [contribution guidelines](https://neurobionics.github.io/opensourceleg/contributing/).

## 📝 License

The `opensourceleg` library is licensed under the terms of the [LGPL-v2.1 license](https://github.com/neurobionics/opensourceleg/raw/main/LICENSE). This license grants users a number of freedoms:

- You are free to use the `opensourceleg` library for any purpose.
- You are free to modify the `opensourceleg` library to suit your needs.
- You can study how the `opensourceleg` library works and change it.
- You can distribute modified versions of the `opensourceleg` library.

The LGPL license ensures that all these freedoms are protected, now and in the future, requiring everyone to share their modifications when they also share the library in public.

## 🐛 Issues

Found a bug or have a suggestion? Please [open an issue](https://github.com/neurobionics/opensourceleg/issues).
