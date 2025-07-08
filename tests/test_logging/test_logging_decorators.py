from unittest.mock import Mock

from opensourceleg.logging import LOGGER
from opensourceleg.logging.decorators import deprecated, deprecated_with_routing, deprecated_with_suggestion


# test deprecated & decorator
def test_deprecated_decorator():
    LOGGER.original_warning = LOGGER.warning
    LOGGER.warning = Mock()

    @deprecated
    def testing(x):
        return x

    assert testing(2) == 2
    LOGGER.warning.assert_called_once_with(f"Function `{testing.__name__}` is deprecated.")

    LOGGER.warning = LOGGER.original_warning
    del LOGGER.original_warning


# test deprecated with suggestion & decorator
def test_deprecated_with_suggestion():
    LOGGER.original_warning = LOGGER.warning
    LOGGER.warning = Mock()

    def alternate(y):
        return y

    assert alternate(4) == 4

    @deprecated_with_suggestion(alternate)
    def testing(x):
        return x

    assert testing("here") == "here"
    LOGGER.warning.assert_called_once_with(
        f"Function `{testing.__name__}` is deprecated. Please use `{alternate.__name__}` instead."
    )

    LOGGER.warning = LOGGER.original_warning
    del LOGGER.original_warning


# test deprecated with routing & decorator
def test_deprecated_with_routing():
    LOGGER.original_warning = LOGGER.warning
    LOGGER.warning = Mock()

    def alternate(y):
        return y * 3

    assert alternate(4) == 12

    @deprecated_with_routing(alternate)
    def testing(x):
        return x * 2

    assert testing(2) == 6

    LOGGER.warning.assert_called_once_with(
        f"Function `{testing.__name__}` is deprecated. "
        f"Please use `{alternate.__name__}` instead, "
        f"which will be called automatically now."
    )

    LOGGER.warning = LOGGER.original_warning
    del LOGGER.original_warning
