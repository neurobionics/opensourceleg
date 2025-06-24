#!/bin/bash
# Setup cross-compilation environment for ARM targets (Raspberry Pi)

set -e

echo "🚀 Setting up cross-compilation environment for ARM targets"

# Check if running on Linux (required for cross-compilation)
if [[ "$OSTYPE" != "linux-gnu"* ]]; then
    echo "❌ Cross-compilation is currently only supported on Linux"
    echo "   For other platforms, use GitHub Actions or Docker"
    exit 1
fi

# Install Rust if not present
if ! command -v rustc &> /dev/null; then
    echo "📦 Installing Rust..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source ~/.cargo/env
fi

# Add ARM targets
echo "📦 Adding ARM compilation targets..."
rustup target add aarch64-unknown-linux-gnu     # ARM64 (Raspberry Pi 4/5)
rustup target add armv7-unknown-linux-gnueabihf # ARMhf (Raspberry Pi 3/older)

# Install cross-compilation tools
echo "📦 Installing cross-compilation tools..."
sudo apt-get update
sudo apt-get install -y \
    gcc-aarch64-linux-gnu \
    gcc-arm-linux-gnueabihf \
    pkg-config \
    libssl-dev

# Create cargo config for cross-compilation
mkdir -p ~/.cargo
cat > ~/.cargo/config.toml << EOF
[target.aarch64-unknown-linux-gnu]
linker = "aarch64-linux-gnu-gcc"

[target.armv7-unknown-linux-gnueabihf]
linker = "arm-linux-gnueabihf-gcc"
EOF

echo "✅ Cross-compilation environment setup complete!"
echo ""
echo "Now you can build for ARM targets:"
echo "  make build-rust-arm64   # For Raspberry Pi 4/5 (ARM64)"
echo "  make build-rust-armhf   # For Raspberry Pi 3/older (ARMhf)"
echo "  make build-rust-all     # For all targets"
