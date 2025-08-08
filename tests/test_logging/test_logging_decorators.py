from unittest.mock import Mock

from observable import Logger

from opensourceleg.logging.decorators import deprecated, deprecated_with_routing, deprecated_with_suggestion


# test deprecated & decorator
def test_deprecated_decorator():
    Logger.original_warning = Logger.warning
    Logger.warning = Mock()

    @deprecated
    def testing(x):
        return x

    assert testing(2) == 2
    Logger.warning.assert_called_once_with(f"Function `{testing.__name__}` is deprecated.")

    Logger.warning = Logger.original_warning
    del Logger.original_warning


# test deprecated with suggestion & decorator
def test_deprecated_with_suggestion():
    Logger.original_warning = Logger.warning
    Logger.warning = Mock()

    def alternate(y):
        return y

    assert alternate(4) == 4

    @deprecated_with_suggestion(alternate)
    def testing(x):
        return x

    assert testing("here") == "here"
    Logger.warning.assert_called_once_with(
        f"Function `{testing.__name__}` is deprecated. Please use `{alternate.__name__}` instead."
    )

    Logger.warning = Logger.original_warning
    del Logger.original_warning


# test deprecated with routing & decorator
def test_deprecated_with_routing():
    Logger.original_warning = Logger.warning
    Logger.warning = Mock()

    def alternate(y):
        return y * 3

    assert alternate(4) == 12

    @deprecated_with_routing(alternate)
    def testing(x):
        return x * 2

    assert testing(2) == 6

    Logger.warning.assert_called_once_with(
        f"Function `{testing.__name__}` is deprecated. "
        f"Please use `{alternate.__name__}` instead, "
        f"which will be called automatically now."
    )

    Logger.warning = Logger.original_warning
    del Logger.original_warning
