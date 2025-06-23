# syntax=docker/dockerfile:1

FROM python:3.11-slim-bookworm

# Install UV
RUN pip install uv

# Copy only requirements to cache them in docker layer
WORKDIR /code
COPY uv.lock pyproject.toml /code/

# Project initialization:
RUN uv sync --frozen --no-dev

# Copy Python code to the Docker image
COPY opensourceleg /code/opensourceleg/

CMD [ "uv", "run", "python", "opensourceleg/foo.py"]
