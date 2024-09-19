import numpy as np
import pytest


from opensourceleg.safety.safety import *

#test class ThermalLimitException
def test_Thermal_Limit_Exception_initialization():
    thermal_limit_exception = ThermalLimitException("test_ThermalLimitException")
    assert thermal_limit_exception.message == "test_ThermalLimitException"

#test def is_changing & decorator


#test def is_negative & decorator
def test_is_negative_decorator():

    # Test clamp default = False, with property value negative = -1
    @is_negative()
    def test_negative_with_default_clamp(instance: object):
        return -1
    assert test_negative_with_default_clamp({}) == -1

    # Test clamp default = False, with property value positive = 2
    @is_negative()
    def test_positive_with_default_clamp(instance: object):
        return 2
    with pytest.raises(ValueError):   
        test_positive_with_default_clamp({})

    # Test clamp explicitly set to False, with property value positive = 2
    @is_negative(False)
    def test_positive_with_false_clamp(instance: object):
        return 2
    with pytest.raises(ValueError):
        test_positive_with_false_clamp({})
    
    # Test clamp explicitly set to True, with property value positive = 2
    @is_negative(True)
    def test_positive_with_true_clamp(instance: object):
        return 2
    assert test_positive_with_true_clamp({}) == 0

    # Test without parameter instance 
    with pytest.raises(TypeError):
        @is_negative()
        def test_for_type_error():
            return 1
        test_for_type_error()
        

#test def is_positive & decorator
def test_is_positive_decorator():

    # Test clamp default = False, with property value positive = 1
    @is_positive()
    def test_positive_with_default_clamp(instance: object):
        return 1
    assert test_positive_with_default_clamp({}) == 1

    # Test clamp default = False, with property value negative = -2
    @is_positive()
    def test_negative_with_default_clamp(instance: object):
        return -2
    with pytest.raises(ValueError):   
        test_negative_with_default_clamp({})

    # Test clamp explicitly set to False, with property value negative = -2
    @is_positive(False)
    def test_negative_with_false_clamp(instance: object):
        return -2
    with pytest.raises(ValueError):
        test_negative_with_false_clamp({})
    
    # Test clamp explicitly set to True, with property value negative = -2
    @is_positive(True)
    def test_negative_with_true_clamp(instance: object):
        return -2
    assert test_negative_with_true_clamp({}) == 0

    # Test without parameter instance 
    with pytest.raises(TypeError):
        @is_positive()
        def test_for_type_error():
            return 1
        test_for_type_error()


#test def is_zero & decorator
def test_is_zero_decorator():

    # Test clamp default = False, with property value = 0
    @is_zero()
    def test_zero_with_default_clamp(instance: object):
        return 0
    assert test_zero_with_default_clamp({}) == 0

    # Test clamp default = False, with property value negative = -2
    @is_zero()
    def test_negative_with_default_clamp(instance: object):
        return -2
    with pytest.raises(ValueError):   
        test_negative_with_default_clamp({})

    # Test clamp explicitly set to False, with property value positive = 2
    @is_zero(False)
    def test_positive_with_false_clamp(instance: object):
        return 2
    with pytest.raises(ValueError):
        test_positive_with_false_clamp({})
    
    # Test clamp explicitly set to True, with property value negative = -2
    @is_zero(True)
    def test_negative_with_true_clamp(instance: object):
        return -2
    assert test_negative_with_true_clamp({}) == 0

    # Test without parameter instance 
    with pytest.raises(TypeError):
        @is_zero()
        def test_for_type_error():
            return 0
        test_for_type_error()


#test def is_within_range & decorator
def test_is_within_range_decorator():
    
    # Test clamp default = False, with property value = 2 in range
    @is_within_range(1,4)
    def test_within_range_with_default_clamp(instance: object):
        return 2
    assert test_within_range_with_default_clamp({}) == 2

    # Test clamp default = False, with property value = 5 out of range
    @is_within_range(1,4)
    def test_out_of_range_with_default_clamp(instance: object):
        return 5 
    with pytest.raises(ValueError):   
        test_out_of_range_with_default_clamp({})

    # Test clamp explicitly set to False, with property value = 0 out of range
    @is_within_range(1,4,False)
    def test_out_of_range_with_false_clamp(instance: object):
        return 0
    with pytest.raises(ValueError):
        test_out_of_range_with_false_clamp({})
    
    # Test clamp explicitly set to True, with property value = -2 lower than min
    @is_within_range(1,4,True)
    def test_lower_with_true_clamp(instance: object):
        return -2
    assert test_lower_with_true_clamp({}) == 1

    # Test clamp explicitly set to True, with property value = 6 higher than max
    @is_within_range(1,4,True)
    def test_higher_with_true_clamp(instance: object):
        return 5
    assert test_higher_with_true_clamp({}) == 4

    # Test without parameters in is_within_range call
    with pytest.raises(TypeError):
        @is_within_range()
        def test_for_type_error(instance: object):
            return 0
        test_for_type_error({})

    # Test without parameter instance in func def
    with pytest.raises(TypeError):
        @is_within_range(1,4)
        def test_for_type_error():
            return 0
        test_for_type_error()


#test def is_greater_than & decorator
# Test clamp default = False, with property value = 2 in range
    """@is_greater_than(2)
    def test_greater_with_default_clamp(instance: object):
        return 4
    assert test_greater_with_default_clamp({}) == 4

    # Test clamp default = False, with property value = 5 out of range
    @is_greater_than(2)
    def test_less_with_default_clamp(instance: object):
        return 1 
    with pytest.raises(ValueError):   
        test_less_with_default_clamp({})

    # Test clamp explicitly set to False, with property value = 0 out of range
    @is_greater_than(2,False)
    def test_less_with_false_clamp(instance: object):
        return 1
    with pytest.raises(ValueError):
        test_less_with_false_clamp({})
    
    # Test clamp explicitly set to True, with property value = -2 lower than min
    @is_greater_than(2,True)
    def test_lower_with_true_clamp(instance: object):
        return -2
    assert test_lower_with_true_clamp({}) == 1

    # Test clamp explicitly set to True, with property value = 6 higher than max
    @is_greater_than(2,True)
    def test_higher_with_true_clamp(instance: object):
        return 5
    assert test_higher_with_true_clamp({}) == 4

    # Test without parameters in is_within_range call
    with pytest.raises(TypeError):
        @is_greater_than()
        def test_for_type_error(instance: object):
            return 0
        test_for_type_error({})

    # Test without parameter instance in func def
    with pytest.raises(TypeError):
        @is_greater_than(2)
        def test_for_type_error():
            return 0
        test_for_type_error()"""


#test def is_less_than & decorator


#test def custom_criteria & decorator


#test class SafetyDecorators
def test_safety_decorators_init_default():
    safety_decorators = SafetyDecorators()
    assert callable(safety_decorators.is_changing)
    assert callable(safety_decorators.is_negative)
    assert callable(safety_decorators.is_positive)
    assert callable(safety_decorators.is_within_range)
    assert callable(safety_decorators.is_greater_than)
    assert callable(safety_decorators.is_less_than)
    assert callable(safety_decorators.custom_criteria)


#test class SafetyManager
    #init
    #add_safety
    #start
    #update


#test class Sensor in main