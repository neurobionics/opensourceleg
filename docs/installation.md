# Installation Guide

This guide will walk you through installing and setting up the opensourceleg library. Whether you're new to Python or an experienced developer, we'll cover everything you need to get started.

## Quick Start

```bash
pip install opensourceleg
```

If you're familiar with Python packages, the command above will get you started quickly. However, we recommend reading through the complete guide for a better setup.

## Understanding Installation Methods

We offer two ways to install opensourceleg:

- **Standard Installation**: For users who want to use the library as-is
- **Development Installation**: For contributors or users who need to modify the library

### 1. Standard Installation (Recommended for Most Users)

#### Prerequisites Explained

- **Python 3.9 or newer**: The library requires modern Python features. [Download Python here](https://www.python.org/downloads/)
  > 💡 When installing Python, make sure to check "Add Python to PATH" on Windows!
- **pip**: Python's package installer (comes with Python)
  > 💡 To verify your installation, open a terminal and run:
  >
  > ```bash
  > python --version
  > pip --version
  > ```

#### Virtual Environment Setup

A virtual environment helps avoid conflicts between different projects and their dependencies. It is generally a good idea to use one.

```bash
# Step 1: Create a virtual environment
# This creates a new folder named .venv with a fresh Python installation
python -m venv .venv

# Step 2: Activate the virtual environment
# On Linux/macOS:
source .venv/bin/activate
# On Windows:
.venv\Scripts\activate

# You'll know it's working when you see (.venv) at the start of your terminal line

# Step 3: Install opensourceleg
pip install opensourceleg
```
#### Hardware-Specific Dependencies

The opensourceleg library supports a variety of actuators and sensors, each with its own dependencies. Rather than installing all possible dependencies (which can be extensive), we recommend installing only what you need for your specific hardware configuration.

### Installing Hardware-Specific Packages

You can easily install hardware-specific dependencies using pip's "extras" feature:

```bash
# For Dephy actuators
pip install opensourceleg[dephy]

# For Moteus actuators
pip install opensourceleg[moteus]

# For I2C communication
pip install opensourceleg[communication]

# For gRPC messaging functionality
pip install opensourceleg[messaging]
```

### Available Extras

| Extra | Dependencies |
|-------|-------------|
| `dephy` | flexsea |
| `moteus` | moteus, moteus-pi3hat |
| `communication` | smbus2 |
| `messaging` | grpcio, grpcio-tools, types-protobuf |

This approach keeps your installation lightweight and focused on just the hardware you're working with.


💡 **Why Use a Virtual Environment?**
Keeps your projects isolated

- Prevents version conflicts
- Makes it easier to share your project
- Allows different Python versions for different projects

#### Basic Usage Example

```python
# Import the components you need
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.sensors.encoder import AS5048B

# Now you can use these components in your code
...
```

### 2. Development Installation

This section is for those who want to contribute to opensourceleg or need to modify its code.

#### Prerequisites in Detail

- **Python 3.9+**: Same as above
- **Poetry**: A modern dependency management tool, see [Poetry installation guide](https://python-poetry.org/docs/#installation)
   - The quickest way to install poetry is to use pipx, run the following command:
     ```bash
     pip install pipx
     ```
     Now add pipx to your PATH:
     ```bash
     nano ~/.bashrc
     ```
     Add the following line to the end of the file:
     ```bash
     export PATH="$HOME/.local/bin:$PATH"
     ```
     Save (CTRL+O) and exit (CTRL+X) the editor.
     Source the bashrc file to update your PATH:
     ```bash
     source ~/.bashrc
     ```
     Now you can install poetry using pipx:
     ```bash
     pipx install poetry
     ```

- **Git**: Version control system, see [Git installation guide](https://git-scm.com/downloads)

#### Detailed Development Setup

1. **Fork & Clone the Repository**

   ```bash
   # First, fork the repository on GitHub (click the 'Fork' button)

   # Then clone your fork (replace YOUR-USERNAME with your GitHub username)
   git clone https://github.com/YOUR-USERNAME/opensourceleg.git

   # Move into the project directory
   cd opensourceleg
   ```

2. **Configure Development Environment**

   ```bash
   # Connect your fork to the main repository for updates
   git remote add upstream https://github.com/neurobionics/opensourceleg.git

   # Install project dependencies using Poetry
   poetry install  # This might take a few minutes

   # Activate the Poetry environment
   poetry shell # for older versions of poetry
   eval $(poetry env activate)

   # Install pre-commit hooks for code quality
   make install
   ```

   > **Windows Users**: You'll need to install `make` first: Install [chocolatey](https://chocolatey.org/install) and run: `choco install make`. Alternatively, download from [GnuWin32](https://gnuwin32.sourceforge.net/packages/make.htm)

3. **Development Workflow Guide**

   ```bash
   # Create a new branch for your changes
   git checkout -b feature-name
   # Use a descriptive name like 'add-new-sensor' or 'fix-motor-bug'

   # After making changes, run the quality checks
   make check    # Runs linting, type checking, and tests
   make format   # Automatically formats your code to match our style

   # Commit your changes
   git add .
   git commit -m "Detailed description of your changes"
   git push origin feature-name
   ```

4. **Contributing to opensourceleg**
   If you want to contribute your modifications to the `opensourceleg` library so that others can use them, please read our [contribution guidelines](contributing.md).

## Troubleshooting Common Issues

### 1. System Package Warnings

If you see warnings about breaking system packages (common in Python 3.10+):

```bash
# Not recommended, but works if you're in a hurry:
pip install opensourceleg --break-system-packages

# Better solution: Use a virtual environment as described above
```

### 2. Virtual Environment Problems

If your virtual environment isn't activating:

```bash
# Check where Python is running from
which python  # on Unix/macOS
where python  # on Windows

# Should show path to your virtual environment
# Example: .venv/bin/python
```

### 3. Poetry Installation Issues

If Poetry isn't working:

```bash
# Verify Poetry installation
poetry --version

# If not found, try adding to PATH or reinstalling
```

## Getting Help

Everyone was a beginner once! Don't hesitate to ask questions if you get stuck. We have several ways to get help:

1. **Bug Reports**: Open an issue on [GitHub](https://github.com/neurobionics/opensourceleg/issues)
2. **Questions**: Check our [discussions](https://www.opensourceleg.org/community)

## Next Steps

1. **Read the Documentation**: [Full Documentation](https://neurobionics.github.io/opensourceleg/)
2. **Try Tutorials**: Start with our [getting started guide](https://neurobionics.github.io/opensourceleg/tutorials/getting_started)
3. **Join the Community**: Connect with other users on our [community forum](https://www.opensourceleg.org/)
4. **Contribute**: Check our [contribution guidelines](https://github.com/neurobionics/opensourceleg/blob/main/CONTRIBUTING.md)
