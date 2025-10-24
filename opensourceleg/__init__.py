"""
An open-source software library for numerical computation, data acquisition,
and control of lower-limb robotic prostheses.
"""

from importlib import metadata as importlib_metadata

from opensourceleg.logging.sentry_config import init_sentry


def get_version() -> str:
    try:
        return importlib_metadata.version(__name__)
    except importlib_metadata.PackageNotFoundError:  # pragma: no cover
        return "unknown"


__version__: str = get_version()

# Initialize Sentry (logging is already initialized in config.py module import)
init_sentry()
