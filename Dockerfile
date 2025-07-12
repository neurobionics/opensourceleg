# syntax=docker/dockerfile:1

FROM python:3.11-slim-bookworm

# Install system dependencies and Rust
RUN apt-get update && apt-get install -y \
    curl \
    build-essential \
    && curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y \
    && . ~/.cargo/env \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Add Rust to PATH
ENV PATH="/root/.cargo/bin:${PATH}"

# Install UV
RUN pip install uv

# Copy only requirements to cache them in docker layer
WORKDIR /code
COPY uv.lock pyproject.toml Cargo.toml /code/

# Copy Rust source code
COPY src /code/src/

# Project initialization:
RUN uv sync --frozen --no-dev

# Build Rust extension
RUN uv run maturin develop --release

# Copy Python code to the Docker image
COPY opensourceleg /code/opensourceleg/

CMD [ "uv", "run", "python", "-c", "import opensourceleg; print('OpenSourceLeg with Rust backend loaded')"]
