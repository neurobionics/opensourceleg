<div align="center">

<h1>opensourceleg</h1>

[![Build status](https://github.com/neurobionics/opensourceleg/workflows/build/badge.svg?branch=master&event=push)](https://github.com/neurobionics/opensourceleg/actions?query=workflow%3Abuild)
[![Documentation Status](https://readthedocs.org/projects/opensourceleg/badge/?version=latest)](https://opensourceleg.readthedocs.io/en/latest/?badge=latest)
[![Python Version](https://img.shields.io/pypi/pyversions/opensourceleg.svg)](https://pypi.org/project/opensourceleg/)
[![Dependencies Status](https://img.shields.io/badge/dependencies-up%20to%20date-brightgreen.svg)](https://github.com/neurobionics/opensourceleg/pulls?utf8=%E2%9C%93&q=is%3Apr%20author%3Aapp%2Fdependabot)

[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Security: bandit](https://img.shields.io/badge/security-bandit-green.svg)](https://github.com/PyCQA/bandit)
[![Pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/neurobionics/opensourceleg/blob/main/.pre-commit-config.yaml)
[![License](https://img.shields.io/github/license/neurobionics/opensourceleg)](https://github.com/neurobionics/opensourceleg/blob/main/LICENSE)
![Coverage Report](https://github.com/neurobionics/opensourceleg/blob/6112694880e01a02307a81c44d49a74b58183b25/assets/images/coverage.svg)

An open-source software library for numerical computation, data acquisition, <br>and control of lower-limb robotic prostheses.

<img src="https://github.com/neurobionics/opensourceleg/blob/66ad4289ef9ba8701fac9337778f87b657286484/assets/images/banner.gif?raw=true" width="800" title="Open-Source Leg">

</div>

<br>

## Installation

The easiest and quickest way to install the *opensourceleg* library is via [pip](https://pip.pypa.io/en/stable/):

```bash
pip install opensourceleg
```

> If you plan on installing the *opensourceleg* library on a Raspberry Pi, we recommend using [opensourcelegpi](https://github.com/neurobionics/opensourcelegpi) tool, which is a cloud-based CI tool used to build an up-to-date OS for a [Raspberry Pi](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) that can be used headless/GUI-less to control autonomous/remote robotic systems. This tool bundles the *opensourceleg* library and its dependencies into a single OS image, which can be flashed onto a microSD card and used to boot a Raspberry Pi. For more information, click [here](https://github.com/neurobionics/opensourcelegpi/blob/main/README.md).

### Developing
To modify, develop, or contribute to the [opensourceleg](https://pypi.org/project/opensourceleg/) library, we encourage you to install [Poetry](https://python-poetry.org), which is a python packaging and dependency management tool. Once you have Poetry installed on your local machine, you can clone the repository and install the *opensourceleg* library by running the following commands:

```bash
git clone https://github.com/neurobionics/opensourceleg.git
cd opensourceleg

poetry install
poetry shell
```

## Documentation

You can find tutorials and API documentation at [opensourceleg.readthedocs.io](https://opensourceleg.readthedocs.io/en/latest/).

## License

The *opensourceleg* library is licensed under the terms of the [GPL-3.0 license](https://github.com/neurobionics/opensourceleg/raw/main/LICENSE). This license grants users a number of freedoms:

* You are free to use the *opensourceleg* library for any purpose.
* You are free to modify the *opensourceleg* library to suit your needs.
* You can study how the *opensourceleg* library works and change it.
* You can distribute modified versions of the *opensourceleg* library.

The GPL license ensures that all these freedoms are protected, now and in the future, requiring everyone to share their modifications when they also share the library in public.

## Contributing

Contributions are welcome, and they are greatly appreciated! For more details, read our [contribution guidelines](https://github.com/neurobionics/opensourceleg/blob/11765f7f7dd94e5d8699675149d5ff3596ea01b8/CONTRIBUTING.md).
