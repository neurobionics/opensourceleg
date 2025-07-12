# Deployment Guide

This guide covers building and deploying opensourceleg across different architectures, with special focus on Raspberry Pi deployment.

## Supported Architectures

| Architecture | Platform | Examples |
|--------------|----------|----------|
| `x86_64` | Intel/AMD 64-bit | Most desktop/server systems |
| `aarch64` | ARM 64-bit | Raspberry Pi 4/5, Apple Silicon |
| `armv7` | ARM 32-bit | Raspberry Pi 3/older |

## Build Methods

### 1. Native Development Build

For local development on your current architecture:

```bash
# Development build (debug mode)
make build-rust

# Release build (optimized)
make build-rust-release
```

### 2. Cross-Compilation (Linux Host)

#### Setup Cross-Compilation Environment

```bash
# One-time setup (Linux only)
make setup-cross
```

#### Build for Specific Targets

```bash
# ARM64 (Raspberry Pi 4/5)
make build-rust-arm64

# ARMhf (Raspberry Pi 3/older)
make build-rust-armhf

# All targets
make build-rust-all
```

### 3. Docker Cross-Compilation (All Platforms)

Works on macOS, Windows, and Linux:

```bash
# Build for all ARM targets using Docker
make build-rust-docker
```

This creates wheels in `dist/` for:
- Native platform (x86_64)
- ARM64 (aarch64)
- ARMhf (armv7)

### 4. GitHub Actions (Automated)

Automatic builds are triggered on:
- **Pull Requests**: Build and test on all platforms
- **Releases**: Build and publish to PyPI

## Raspberry Pi Deployment

### Quick Install from PyPI

```bash
# Install pre-built wheel (if available for your architecture)
pip install opensourceleg

# Verify Rust backend
python -c "import opensourceleg; print('Rust backend:', opensourceleg.HAS_RUST_BACKEND)"
```

### Build from Source on Raspberry Pi

If no pre-built wheel is available:

```bash
# Install build dependencies
sudo apt update
sudo apt install -y build-essential pkg-config libssl-dev

# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env

# Install Python dependencies
pip install uv

# Clone and build
git clone https://github.com/neurobionics/opensourceleg.git
cd opensourceleg
uv sync
make build-rust-release
```

### Cross-Compile for Raspberry Pi

From a Linux development machine:

```bash
# Setup cross-compilation (one-time)
make setup-cross

# Build ARM64 wheel for Raspberry Pi 4/5
make build-rust-arm64

# Copy wheel to Raspberry Pi
scp dist/*.whl pi@raspberrypi.local:

# Install on Raspberry Pi
ssh pi@raspberrypi.local
pip install opensourceleg-*.whl
```

## Performance Considerations

### Raspberry Pi 4/5 (ARM64)
- **Recommended**: Use ARM64 builds for best performance
- **Memory**: 4GB+ RAM recommended for compilation
- **Storage**: Use fast SD card (Class 10+) or SSD

### Raspberry Pi 3/older (ARMhf)
- **Limitation**: 32-bit architecture, limited memory
- **Recommendation**: Use pre-built wheels when possible
- **Compilation**: May require swap space increase

### Development Machine
- **Cross-compilation**: 5-10x faster than building on Pi
- **Docker**: Universal solution, works on any platform
- **CI/CD**: Automatic builds ensure compatibility

## Troubleshooting

### Common Issues

#### Missing Cross-Compilation Tools
```bash
# Error: linker `aarch64-linux-gnu-gcc` not found
sudo apt install gcc-aarch64-linux-gnu

# Error: linker `arm-linux-gnueabihf-gcc` not found
sudo apt install gcc-arm-linux-gnueabihf-gcc
```

#### Python Version Mismatch
```bash
# Ensure compatible Python version on target
python --version  # Should be 3.10+
```

#### Import Errors on Raspberry Pi
```bash
# Check if wheel matches architecture
python -c "import platform; print(platform.machine())"

# Reinstall with correct architecture wheel
pip uninstall opensourceleg
pip install opensourceleg --force-reinstall
```

### Platform-Specific Notes

#### macOS Development
- Cross-compilation not directly supported
- Use Docker: `make build-rust-docker`
- GitHub Actions for ARM builds

#### Windows Development
- WSL2 recommended for cross-compilation
- Docker alternative: `make build-rust-docker`
- GitHub Actions for ARM builds

#### Linux Development
- Full cross-compilation support
- Native performance
- All build methods available

## CI/CD Integration

### Automated Builds

Every push/PR triggers:
- Quality checks (linting, tests)
- Multi-platform builds (Linux, macOS, Windows)
- Cross-compilation for ARM targets

### Release Process

1. **Tag Release**: `git tag v3.1.0 && git push --tags`
2. **Automatic Build**: GitHub Actions builds all architectures
3. **PyPI Upload**: Wheels uploaded automatically
4. **Verification**: Test installation on target platforms

### Architecture Matrix

| Platform | x86_64 | ARM64 | ARMhf |
|----------|--------|-------|-------|
| Linux | ✅ | ✅ | ✅ |
| macOS | ✅ | ✅ | ❌ |
| Windows | ✅ | ❌ | ❌ |

## Advanced Configuration

### Custom Rust Features

Enable specific optimizations in `Cargo.toml`:

```toml
[target.aarch64-unknown-linux-gnu]
rustflags = ["-C", "target-cpu=cortex-a72"]  # Pi 4 specific

[target.armv7-unknown-linux-gnueabihf]
rustflags = ["-C", "target-cpu=cortex-a53"]  # Pi 3 specific
```

### Environment Variables

Control build behavior:

```bash
# Enable debug symbols in release builds
export CARGO_PROFILE_RELEASE_DEBUG=true

# Optimize for size instead of speed
export CARGO_PROFILE_RELEASE_OPT_LEVEL=s

# Use specific linker
export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc
```

## Distribution Strategy

### PyPI Wheels

Automatically built and published:
- `opensourceleg-3.1.0-cp310-cp310-linux_x86_64.whl`
- `opensourceleg-3.1.0-cp310-cp310-linux_aarch64.whl`
- `opensourceleg-3.1.0-cp310-cp310-linux_armv7l.whl`
- `opensourceleg-3.1.0-cp310-cp310-macosx_*.whl`
- `opensourceleg-3.1.0-cp310-cp310-win_amd64.whl`

### Installation Fallback

1. **Pre-built wheel** (fastest)
2. **Source build with Rust** (full features)
3. **Pure Python fallback** (no Rust backend)

This ensures opensourceleg works everywhere, with optimal performance where possible.
