<div align="center">

<h1>opensourceleg</h1>

[![Build status](https://github.com/neurobionics/opensourceleg/workflows/build/badge.svg)](https://github.com/neurobionics/opensourceleg/actions?query=workflow%3Abuild)
[![Documentation Status](https://github.com/neurobionics/opensourceleg/actions/workflows/pages/pages-build-deployment/badge.svg)](https://neurobionics.github.io/opensourceleg/)
[![Python Version](https://img.shields.io/pypi/pyversions/opensourceleg.svg)](https://pypi.org/project/opensourceleg/)
[![Dependencies Status](https://img.shields.io/badge/dependencies-up%20to%20date-brightgreen.svg)](https://github.com/neurobionics/opensourceleg/pulls?utf8=%E2%9C%93&q=is%3Apr%20author%3Aapp%2Fdependabot)

[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Security: bandit](https://img.shields.io/badge/security-bandit-green.svg)](https://github.com/PyCQA/bandit)
[![Pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/neurobionics/opensourceleg/blob/main/.pre-commit-config.yaml)
[![License](https://img.shields.io/github/license/neurobionics/opensourceleg)](https://github.com/neurobionics/opensourceleg/blob/main/LICENSE)
![Coverage Report](assets/images/coverage.svg)

An open-source software library for numerical computation, data acquisition, <br>and control of lower-limb robotic prostheses.

> NOTE: We are currently testing the new version of the library (main branch), and the PyPI release will be updated soon. If you are looking for the source behind the existing PyPI release, please refer to the [legacy branch](https://github.com/neurobionics/opensourceleg/tree/legacy).

<img src="https://github.com/neurobionics/opensourceleg/blob/66ad4289ef9ba8701fac9337778f87b657286484/assets/images/banner.gif?raw=true" width="800" title="Open-Source Leg">

</div>

<br>

## Installation

The easiest and quickest way to install the _opensourceleg_ library is via [pip](https://pip.pypa.io/en/stable/):

```bash
pip install opensourceleg
```

You can now use the library in your projects! Please refer to the [documentation](https://opensourceleg.readthedocs.io/en/latest/) for tutorials, examples, and more.

### Developing and Contributing to the Library

If you'd like to modify or contribute to the [opensourceleg](https://pypi.org/project/opensourceleg/) library, we recommend following these steps:

1. **Fork the repository** by clicking the "Fork" button at the top right of this [page](https://github.com/neurobionics/opensourceleg).

2. **Clone your fork** to your local machine:

   ```bash
   git clone https://github.com/YOUR-USERNAME/opensourceleg.git
   cd opensourceleg
   ```

3. **Set up the upstream remote** to keep your fork in sync with the main repository:

   ```bash
   git remote add upstream https://github.com/neurobionics/opensourceleg.git
   ```

4. **Install Poetry** if you haven't already. [Poetry](https://python-poetry.org) is a python packaging and dependency management tool. We use poetry to manage dependencies and build the library.

5. **Install dependencies and activate the virtual environment**:

   ```bash
   poetry install
   poetry shell
   ```

6. **Install pre-commit hooks**:

   ```bash
   make install
   ```

7. **Create a new branch** for your feature or bugfix:

   ```bash
   git checkout -b feature-or-bugfix-name
   ```

8. **Make your changes** and commit them with descriptive messages.

9. **Run checks**. See [the contributing guidelines](https://github.com/neurobionics/opensourceleg/blob/main/CONTRIBUTING.md) for more information.

10. **Push your changes** to your fork:

   ```bash
   git push origin feature-or-bugfix-name
   ```

11. **Create a Pull Request** by navigating to your fork on GitHub and clicking `New Pull Request`.

Your changes will be reviewed by the maintainers, and if approved, they will be merged into the main repository.

## License

The _opensourceleg_ library is licensed under the terms of the [LGPL-v2.1 license](https://github.com/neurobionics/opensourceleg/raw/main/LICENSE). This license grants users a number of freedoms:

- You are free to use the _opensourceleg_ library for any purpose.
- You are free to modify the _opensourceleg_ library to suit your needs.
- You can study how the _opensourceleg_ library works and change it.
- You can distribute modified versions of the _opensourceleg_ library.

The GPL license ensures that all these freedoms are protected, now and in the future, requiring everyone to share their modifications when they also share the library in public.

## Contributing

Contributions are welcome, and they are greatly appreciated! For more details, read our [contribution guidelines](CONTRIBUTING.md).
