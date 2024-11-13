from io import StringIO
from unittest.mock import patch

import pytest

from opensourceleg.safety.safety import *


# Created class for testing purposes
class Sample:
    def __init__(self):
        self._test = 10
        self._test2 = 5
        self.test_proxy_att_name = None

    @property
    def test(self):
        return self._test

    @property
    def test2(self):
        return self._test2

    def notatt(self):
        return 2

    @test.setter
    def test(self, value):
        self._test = value


# Test class ThermalLimitException
def test_Thermal_Limit_Exception_initialization():
    thermal_limit_exception = ThermalLimitException("test_ThermalLimitException")
    assert thermal_limit_exception.message == "test_ThermalLimitException"


def test_Thermal_Limit_Exception_initialization_with_default():
    thermal_limit_exception = ThermalLimitException()
    assert thermal_limit_exception.message == "Software thermal limit exceeded. Exiting."


# Test def is_changing & decorator
def test_is_changing_single_point():
    # Test single point, where len(history) < max_points (default 10)
    att = "test_att"
    instance = Sample()

    @is_changing(att)
    def test_changing_point1(instance):
        return 8

    assert test_changing_point1(instance) == 8


def test_is_changing_four_points_not_less_than_threshold():
    # Test case of max points reached, but stddev > default threshold
    # Stddev of 6,7,8,9 ~ 1.118, much greater than default threshold
    att = "test_att"
    max_points = 4
    instance = Sample()

    for x in range(6, 10):

        @is_changing(att, max_points)
        def test_changing_point(instance):
            return x

        assert test_changing_point(instance) == x


def test_is_changing_four_points_less_than_threshold():
    # Test case of max points reached, with stddev < defined threshold
    # Stddev of 6,7,8,9 ~ 1.118, with defined threshold = 2
    att = "test_att"
    max_points = 4
    test_threshold = 2
    instance = Sample()
    for x in range(6, 9):

        @is_changing(att, max_points, test_threshold)
        def test_changing_point(instance):
            return x

        assert test_changing_point(instance) == x

    @is_changing(att, max_points, test_threshold)
    def test_changing_point1(instance):
        return 9

    with pytest.raises(ValueError, match=f"{att} is unstable"):
        test_changing_point1(instance)


def test_is_changing_four_points_less_than_threshold_with_proxy_att_name():
    # Test case of max points reached, with stddev < defined threshold
    # Stddev of 6,7,8,9 ~ 1.118, with defined threshold = 2
    # For this case, proxy attribute name is not None
    att = "test_att"
    max_points = 4
    test_threshold = 2
    proxy_att_name = "test_proxy_att_name"
    instance = Sample()
    for x in range(6, 9):

        @is_changing(att, max_points, test_threshold, proxy_att_name)
        def test_changing_point(instance):
            return x

        assert test_changing_point(instance) == x

    @is_changing(att, max_points, test_threshold, proxy_att_name)
    def test_changing_point1(instance):
        return 9

    with patch("sys.stdout", new=StringIO()) as temp_out:
        test_changing_point1(instance)
        assert temp_out.getvalue() == f"{att} isn't stable, returning {proxy_att_name}\n"


def test_is_changing_parameters():
    # Test without parameters in is_changing call
    with pytest.raises(TypeError):

        @is_changing()
        def test_for_type_error(instance: object):
            return 0

        test_for_type_error({})


def test_is_changing_parameters2():
    # Test without parameter instance in func def
    with pytest.raises(TypeError):
        att = "test_att"

        @is_changing(att)
        def test_for_type_error():
            return 0

        test_for_type_error()


# Test def is_negative & decorator
def test_is_negative_def_neg():
    # Test default clamp and property value negative = -1 (clamp only relevant if value >=0)
    @is_negative()
    def test_negative_with_default_clamp(instance: object):
        return -1

    assert test_negative_with_default_clamp({}) == -1


def test_is_negative_def_pos():
    # Test clamp default = False, with property value positive = 2
    @is_negative()
    def test_positive_with_default_clamp(instance: object):
        return 2

    with pytest.raises(ValueError, match="Value must be negative"):
        test_positive_with_default_clamp({})


def test_is_negative_false_pos():
    # Test clamp explicitly set to False, with property value positive = 2
    @is_negative(False)
    def test_positive_with_false_clamp(instance: object):
        return 2

    with pytest.raises(ValueError, match="Value must be negative"):
        test_positive_with_false_clamp({})


def test_is_negative_true_pos():
    # Test clamp explicitly set to True, with property value positive = 2
    @is_negative(True)
    def test_positive_with_true_clamp(instance: object):
        return 2

    assert test_positive_with_true_clamp({}) == 0


def test_is_negative_parameter():
    # Test without parameter instance
    with pytest.raises(TypeError):

        @is_negative()
        def test_for_type_error():
            return 1

        test_for_type_error()


# Test def is_positive & decorator
def test_is_positive_def_pos():
    # Test default clamp and property value positive = 1 (clamp only relevant if value <=0)
    @is_positive()
    def test_positive_with_default_clamp(instance: object):
        return 1

    assert test_positive_with_default_clamp({}) == 1


def test_is_positive_def_neg():
    # Test clamp default = False, with property value negative = -2
    @is_positive()
    def test_negative_with_default_clamp(instance: object):
        return -2

    with pytest.raises(ValueError, match="Value must be positive"):
        test_negative_with_default_clamp({})


def test_is_positive_false_neg():
    # Test clamp explicitly set to False, with property value negative = -2
    @is_positive(False)
    def test_negative_with_false_clamp(instance: object):
        return -2

    with pytest.raises(ValueError, match="Value must be positive"):
        test_negative_with_false_clamp({})


def test_is_positive_true_neg():
    # Test clamp explicitly set to True, with property value negative = -2
    @is_positive(True)
    def test_negative_with_true_clamp(instance: object):
        return -2

    assert test_negative_with_true_clamp({}) == 0


def test_is_positive_parameter():
    # Test without parameter instance
    with pytest.raises(TypeError):

        @is_positive()
        def test_for_type_error():
            return 1

        test_for_type_error()


# Test def is_zero & decorator
def test_is_zero_def_zero():
    # Test default clamp and property value equal to zero (clamp only relevant if value !=0)
    @is_zero()
    def test_zero_with_default_clamp(instance: object):
        return 0

    assert test_zero_with_default_clamp({}) == 0


def test_is_zero_def_neg():
    # Test clamp default = False, with property value negative = -2
    @is_zero()
    def test_negative_with_default_clamp(instance: object):
        return -2

    with pytest.raises(ValueError, match="Value must be zero"):
        test_negative_with_default_clamp({})


def test_is_zero_false_pos():
    # Test clamp explicitly set to False, with property value positive = 2
    @is_zero(False)
    def test_positive_with_false_clamp(instance: object):
        return 2

    with pytest.raises(ValueError, match="Value must be zero"):
        test_positive_with_false_clamp({})


def test_is_zero_true_neg():
    # Test clamp explicitly set to True, with property value negative = -2
    @is_zero(True)
    def test_negative_with_true_clamp(instance: object):
        return -2

    assert test_negative_with_true_clamp({}) == 0


def test_is_zero_parameter():
    # Test without parameter instance
    with pytest.raises(TypeError):

        @is_zero()
        def test_for_type_error():
            return 0

        test_for_type_error()


# Test def is_within_range & decorator
def test_is_within_range_def_in():
    # Test clamp default = False, with property value = 2 in range
    min = 1
    max = 4

    @is_within_range(min, max)
    def test_within_range_with_default_clamp(instance: object):
        return 2

    assert test_within_range_with_default_clamp({}) == 2


def test_is_within_range_def_out():
    # Test clamp default = False, with property value = 5 out of range
    min = 1
    max = 4

    @is_within_range(min, max)
    def test_out_of_range_with_default_clamp(instance: object):
        return 5

    with pytest.raises(ValueError, match=f"Value must be within {min} and {max}"):
        test_out_of_range_with_default_clamp({})


def test_is_within_range_false_out():
    # Test clamp explicitly set to False, with property value = 0 out of range
    min = 1
    max = 4

    @is_within_range(min, max, False)
    def test_out_of_range_with_false_clamp(instance: object):
        return 0

    with pytest.raises(ValueError, match=f"Value must be within {min} and {max}"):
        test_out_of_range_with_false_clamp({})


def test_is_within_range_true_smaller():
    # Test clamp explicitly set to True, with property value = -2 lower than min
    min = 1
    max = 4

    @is_within_range(min, max, True)
    def test_lower_with_true_clamp(instance: object):
        return -2

    assert test_lower_with_true_clamp({}) == min


def test_is_within_range_true_larger():
    # Test clamp explicitly set to True, with property value = 6 higher than max
    min = 1
    max = 4

    @is_within_range(min, max, True)
    def test_higher_with_true_clamp(instance: object):
        return 5

    assert test_higher_with_true_clamp({}) == max


def test_is_within_range_parameter():
    # Test without parameters in is_within_range call
    with pytest.raises(TypeError):

        @is_within_range()
        def test_for_type_error(instance: object):
            return 0

        test_for_type_error({})


def test_is_within_range_parameter2():
    # Test without parameter instance in func def
    with pytest.raises(TypeError):
        min = 1
        max = 4

        @is_within_range(min, max)
        def test_for_type_error():
            return 0

        test_for_type_error()


def test_is_within_range_max_less_min():
    # Test maximum value greater than minimum value for range
    min = 4
    max = 1
    with pytest.raises(ValueError, match="Maximum value must be greater than minimum value of range"):

        @is_within_range(min, max)
        def test_for_min_greater_than_max(instance: object):
            return 0

        test_for_min_greater_than_max({})


# Test def is_greater_than & decorator
def test_is_greater_def_in():
    # Test clamp default = False, with value greater than min
    min = 2

    @is_greater_than(min)
    def test_greater_with_default_clamp(instance: object):
        return 4

    assert test_greater_with_default_clamp({}) == 4


def test_is_greater_def_out():
    # Test clamp default = False, with value less than min
    min = 2

    @is_greater_than(min)
    def test_less_with_default_clamp(instance: object):
        return 1

    with pytest.raises(ValueError, match=f"Value must be greater than {min}"):
        test_less_with_default_clamp({})


def test_is_greater_false_out():
    # Test clamp explicitly set to False, with value less than min
    min = 2

    @is_greater_than(min, False)
    def test_less_with_false_clamp(instance: object):
        return 1

    with pytest.raises(ValueError, match=f"Value must be greater than {min}"):
        test_less_with_false_clamp({})


def test_is_greater_true_out():
    # Test clamp explicitly set to True, with value less than min
    min = 2

    @is_greater_than(min, True)
    def test_less_with_true_clamp(instance: object):
        return -2

    assert test_less_with_true_clamp({}) == min


def test_is_greater_equality_false_equal():
    # Test clamp explicitly set to False, with value equal to min and equality set to True
    min = 2

    @is_greater_than(min, False, True)
    def test_equal_with_false_clamp(instance: object):
        return 2

    assert test_equal_with_false_clamp({}) == min


def test_is_greater_equality_false_out():
    # Test clamp explicitly set to False, with value less than min and equality set to True
    min = 2

    @is_greater_than(min, False, True)
    def test_less_with_false_clamp(instance: object):
        return 1

    with pytest.raises(ValueError, match=f"Value must be greater than or equal to {min}"):
        test_less_with_false_clamp({})


def test_is_greater_equality_true_out():
    # Test clamp explicitly set to True, with value less than min and equality set to True
    min = 2

    @is_greater_than(min, True, True)
    def test_less_with_true_clamp(instance: object):
        return 1

    assert test_less_with_true_clamp({}) == min


def test_is_greater_parameter():
    # Test without parameters in is_greater_than call
    with pytest.raises(TypeError):

        @is_greater_than()
        def test_for_type_error(instance: object):
            return 0

        test_for_type_error({})


def test_is_greater_parameter2():
    # Test without parameter instance in func def
    with pytest.raises(TypeError):
        min = 2

        @is_greater_than(min)
        def test_for_type_error():
            return 0

        test_for_type_error()


# Test def is_less_than & decorator
def test_is_less_def_in():
    # Test clamp default = False, with value less than max
    max = 4

    @is_less_than(max)
    def test_less_with_default_clamp(instance: object):
        return 2

    assert test_less_with_default_clamp({}) == 2


def test_is_less_def_out():
    # Test clamp default = False, with value greater than max
    max = 2

    @is_less_than(max)
    def test_greater_with_default_clamp(instance: object):
        return 4

    with pytest.raises(ValueError, match=f"Value must be less than {max}"):
        test_greater_with_default_clamp({})


def test_is_less_false_out():
    # Test clamp explicitly set to False, with value greater than max
    max = 2

    @is_less_than(max, False)
    def test_greater_with_false_clamp(instance: object):
        return 4

    with pytest.raises(ValueError, match=f"Value must be less than {max}"):
        test_greater_with_false_clamp({})


def test_is_less_true_out():
    # Test clamp explicitly set to True, with value greater than max
    max = 2

    @is_less_than(max, True)
    def test_greater_with_true_clamp(instance: object):
        return 5

    assert test_greater_with_true_clamp({}) == max


def test_is_less_equality_false_equal():
    # Test clamp explicitly set to False, with value equal to max and equality set to True
    max = 2

    @is_less_than(max, False, True)
    def test_equal_with_false_clamp(instance: object):
        return 2

    assert test_equal_with_false_clamp({}) == max


def test_is_less_equality_false_out():
    # Test clamp explicitly set to False, with value greater than max and equality set to True
    max = 2

    @is_less_than(max, False, True)
    def test_less_with_false_clamp(instance: object):
        return 3

    with pytest.raises(ValueError, match=f"Value must be less than or equal to {max}"):
        test_less_with_false_clamp({})


def test_is_less_equality_true_out():
    # Test clamp explicitly set to True, with value greater than max and equality set to True
    max = 2

    @is_less_than(max, True, True)
    def test_less_with_true_clamp(instance: object):
        return 3

    assert test_less_with_true_clamp({}) == max


def test_is_less_parameter():
    # Test without parameters in is_less_than call
    with pytest.raises(TypeError):

        @is_less_than()
        def test_for_type_error(instance: object):
            return 0

        test_for_type_error({})


def test_is_less_parameter2():
    # Test without parameter instance in func def
    with pytest.raises(TypeError):
        max = 2

        @is_less_than(max)
        def test_for_type_error():
            return 0

        test_for_type_error()


# Test def custom_criteria & decorator. Will return true if x > 2
def custom_func(x: float):
    return x ^ 2 > 4


def test_custom_criteria_def_in():
    # Test value meeting criteria
    @custom_criteria(custom_func)
    def test_custom_with_default_clamp(instance: object):
        return 4

    assert test_custom_with_default_clamp({}) == 4


def test_custom_criteria_def_out():
    # Test value not meeting the criteria
    @custom_criteria(custom_func)
    def test_custom_with_default_clamp(instance: object):
        return 2

    with pytest.raises(ValueError, match="Value does not meet custom criteria"):
        test_custom_with_default_clamp({})


def test_custom_criteria_parameter():
    # Test without parameters in custom_criteria call
    with pytest.raises(TypeError):

        @custom_criteria()
        def test_for_type_error(instance: object):
            return 0

        test_for_type_error({})


def test_custom_criteria_parameter2():
    # Test without parameter instance in func def
    with pytest.raises(TypeError):

        @custom_criteria(custom_func)
        def test_for_type_error():
            return 0

        test_for_type_error()


# Test class SafetyDecorators
def test_safetys_init_default():
    safetys = SafetyDecorators()
    assert all(
        callable(func)
        for func in [
            safetys.is_changing,
            safetys.is_negative,
            safetys.is_positive,
            safetys.is_within_range,
            safetys.is_greater_than,
            safetys.is_less_than,
            safetys.custom_criteria,
        ]
    )


# Test init
def test_safety_manager_init():
    test_manager = SafetyManager()
    assert type(test_manager._safe_objects) == dict
    assert test_manager._safe_objects == {}


# Test add safety
@patch("builtins.print")
def test_add_safety_att_doesnt_exist(mock_print):
    test_manager = SafetyManager()
    samp = Sample()

    attribute = "wrong"
    test_manager.add_safety(samp, attribute, SafetyDecorators.is_positive())
    mock_print.assert_called_once_with(f"Error: The attribute '{attribute}' does not exist in the given object.")


@patch("builtins.print")
def test_add_safety_att_not_prop(mock_print):
    test_manager = SafetyManager()
    samp = Sample()

    attribute = "notatt"
    test_manager.add_safety(samp, attribute, SafetyDecorators.is_positive())
    mock_print.assert_called_once_with(
        f"Warning: The attribute '{attribute}' is not a property. The SafetyManager only works on properties."
    )


def test_add_safety():
    test_manager = SafetyManager()
    samp = Sample()

    attribute = "test"
    decorator = SafetyDecorators.is_positive()
    assert samp not in test_manager._safe_objects
    test_manager.add_safety(samp, attribute, decorator)
    assert samp in test_manager._safe_objects
    assert test_manager._safe_objects[samp] == {attribute: [decorator]}

    decorator2 = SafetyDecorators.is_negative()
    test_manager.add_safety(samp, attribute, decorator2)
    assert test_manager._safe_objects[samp] == {attribute: [decorator, decorator2]}

    attribute2 = "test2"
    test_manager.add_safety(samp, attribute2, decorator)
    assert test_manager._safe_objects[samp] == {
        attribute: [decorator, decorator2],
        attribute2: [decorator],
    }


# Test start
def test_start():
    test_manager = SafetyManager()
    samp = Sample()
    attribute = "test"

    decorator = SafetyDecorators.is_negative()
    test_manager.add_safety(samp, attribute, decorator)
    assert test_manager.safe_objects == {samp: {attribute: [decorator]}}

    test_manager.start()
    with pytest.raises(ValueError, match="Value must be negative"):
        pass


# Test update & safe objects
def test_update_and_safe_objects():
    test_manager = SafetyManager()
    samp = Sample()
    attribute = "test"

    decorator = SafetyDecorators.is_positive()
    test_manager.add_safety(samp, attribute, decorator)
    assert test_manager.safe_objects == {samp: {attribute: [decorator]}}
    test_manager.start()

    samp.test = -10

    with pytest.raises(ValueError, match="Value must be positive"):
        test_manager.update()
