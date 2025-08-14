"""
An open-source software library for numerical computation, data acquisition,
and control of lower-limb robotic prostheses.
"""

from importlib import metadata as importlib_metadata


def get_version() -> str:
    try:
        return importlib_metadata.version(__name__)
    except importlib_metadata.PackageNotFoundError:  # pragma: no cover
        return "unknown"


__version__: str = get_version()

# Import high-performance Rust backend if available
try:
    from . import rust  # noqa: F401

    HAS_RUST_BACKEND = True
except ImportError:
    HAS_RUST_BACKEND = False

__all__ = ["__version__", "HAS_RUST_BACKEND"]
