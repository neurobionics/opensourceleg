import math

import pytest

from opensourceleg.generators.expression_evaluator import ExpressionEvaluator


def test_init_simple_expression():
    """Test initialization with a simple expression."""
    evaluator = ExpressionEvaluator("1 + 2")
    assert evaluator.expression == "1 + 2"
    assert "pi" in evaluator._available_vars
    assert "e" in evaluator._available_vars


def test_init_with_variables():
    """Test initialization with custom variables."""
    variables = ["x", "y"]
    evaluator = ExpressionEvaluator("x + y", variables)
    assert evaluator._variable_names == ["x", "y"]
    assert "pi" in evaluator._available_vars  # Built-ins still present
    assert "e" in evaluator._available_vars
    assert "x" in evaluator._available_vars
    assert "y" in evaluator._available_vars


def test_init_variables_override_builtins():
    """Test that custom variables can override built-in constants."""
    variables = ["pi", "e"]
    evaluator = ExpressionEvaluator("pi + e", variables)
    # Built-ins are still available for validation
    assert "pi" in evaluator._available_vars
    assert "e" in evaluator._available_vars


def test_evaluate_constants():
    """Test evaluation of constant expressions."""
    evaluator = ExpressionEvaluator("42")
    assert evaluator.evaluate() == 42.0

    evaluator = ExpressionEvaluator("3.14159")
    assert evaluator.evaluate() == 3.14159


def test_evaluate_builtin_constants():
    """Test evaluation using built-in constants."""
    evaluator = ExpressionEvaluator("pi")
    assert evaluator.evaluate() == math.pi

    evaluator = ExpressionEvaluator("e")
    assert evaluator.evaluate() == math.e


def test_evaluate_variables():
    """Test evaluation with variables."""
    evaluator = ExpressionEvaluator("x", ["x"])
    assert evaluator.evaluate(7.5) == 7.5

    # Test different runtime value
    assert evaluator.evaluate(10.0) == 10.0


def test_evaluate_binary_operations():
    """Test binary arithmetic operations."""
    test_cases = [
        ("1 + 2", 3.0),
        ("5 - 3", 2.0),
        ("4 * 6", 24.0),
        ("15 / 3", 5.0),
        ("2 ** 3", 8.0),
        ("7 % 3", 1.0),
        ("7 // 2", 3.0),
    ]

    for expr, expected in test_cases:
        evaluator = ExpressionEvaluator(expr)
        assert evaluator.evaluate() == expected


def test_evaluate_unary_operations():
    """Test unary operations."""
    evaluator = ExpressionEvaluator("-5")
    assert evaluator.evaluate() == -5.0

    evaluator = ExpressionEvaluator("+10")
    assert evaluator.evaluate() == 10.0


def test_evaluate_comparison_operations():
    """Test comparison operations."""
    test_cases = [
        ("5 == 5", True),
        ("5 != 3", True),
        ("5 < 10", True),
        ("5 <= 5", True),
        ("10 > 5", True),
        ("5 >= 5", True),
        ("5 == 3", False),
        ("5 != 5", False),
    ]

    for expr, expected in test_cases:
        evaluator = ExpressionEvaluator(expr)
        result = evaluator.evaluate()
        assert result == (1.0 if expected else 0.0)  # Boolean converted to float


def test_evaluate_function_calls():
    """Test mathematical function calls."""
    test_cases = [
        ("abs(-5)", 5.0),
        ("min(3, 7, 1)", 1.0),
        ("max(3, 7, 1)", 7.0),
        ("round(3.7)", 4.0),
        ("sin(0)", 0.0),
        ("cos(0)", 1.0),
        ("tan(0)", 0.0),
        ("exp(0)", 1.0),
        ("log(1)", 0.0),
        ("log10(10)", 1.0),
        ("sqrt(16)", 4.0),
        ("ceil(3.2)", 4.0),
        ("floor(3.8)", 3.0),
    ]

    for expr, expected in test_cases:
        evaluator = ExpressionEvaluator(expr)
        result = evaluator.evaluate()
        assert abs(result - expected) < 1e-10


def test_evaluate_trigonometric_functions():
    """Test trigonometric functions with known values."""
    # Test pi/2 for sin and cos
    evaluator = ExpressionEvaluator("sin(pi/2)")
    assert abs(evaluator.evaluate() - 1.0) < 1e-10

    evaluator = ExpressionEvaluator("cos(pi)")
    assert abs(evaluator.evaluate() - (-1.0)) < 1e-10

    # Test inverse trig functions
    evaluator = ExpressionEvaluator("asin(1)")
    assert abs(evaluator.evaluate() - math.pi / 2) < 1e-10


def test_evaluate_hyperbolic_functions():
    """Test hyperbolic functions."""
    evaluator = ExpressionEvaluator("sinh(0)")
    assert abs(evaluator.evaluate() - 0.0) < 1e-10

    evaluator = ExpressionEvaluator("cosh(0)")
    assert abs(evaluator.evaluate() - 1.0) < 1e-10

    evaluator = ExpressionEvaluator("tanh(0)")
    assert abs(evaluator.evaluate() - 0.0) < 1e-10


def test_evaluate_conditional_expressions():
    """Test conditional (ternary) expressions."""
    evaluator = ExpressionEvaluator("10 if 5 > 3 else 20")
    assert evaluator.evaluate() == 10.0

    evaluator = ExpressionEvaluator("10 if 5 < 3 else 20")
    assert evaluator.evaluate() == 20.0

    evaluator = ExpressionEvaluator("x if x > 0 else -x", ["x"])
    assert evaluator.evaluate(-5) == 5.0
    assert evaluator.evaluate(3) == 3.0


def test_evaluate_complex_expressions():
    """Test complex mathematical expressions."""
    # Quadratic formula components
    evaluator = ExpressionEvaluator("(-b + sqrt(b**2 - 4*a*c)) / (2*a)", ["a", "b", "c"])
    result = evaluator.evaluate(1, -5, 6)  # a=1, b=-5, c=6
    assert abs(result - 3.0) < 1e-10  # One root of x^2 - 5x + 6 = 0

    # Trigonometric identity
    evaluator = ExpressionEvaluator("sin(x)**2 + cos(x)**2", ["x"])
    assert abs(evaluator.evaluate(math.pi / 4) - 1.0) < 1e-10


def test_evaluate_with_multiple_variables():
    """Test evaluation with multiple variables in order."""
    evaluator = ExpressionEvaluator("2 * x + y", ["x", "y"])

    # Test with different values in the correct order (x, y)
    assert evaluator.evaluate(1, 2) == 4.0  # 2*1 + 2 = 4
    assert evaluator.evaluate(3, 1) == 7.0  # 2*3 + 1 = 7
    assert evaluator.evaluate(5, 2) == 12.0  # 2*5 + 2 = 12


def test_error_invalid_syntax():
    """Test error handling for invalid expression syntax."""
    with pytest.raises(ValueError, match="Invalid expression syntax"):
        ExpressionEvaluator("(1 + 2")  # Missing closing parenthesis

    with pytest.raises(ValueError, match="Invalid expression syntax"):
        ExpressionEvaluator("1 +")  # Incomplete expression


def test_error_undefined_variable():
    """Test error handling for undefined variables."""
    with pytest.raises(ValueError, match="Undefined variable"):
        ExpressionEvaluator("x + 1")  # x not defined

    # Variables are checked at compile time, not evaluation time
    with pytest.raises(ValueError, match="Expression compilation failed.*Undefined variable"):
        ExpressionEvaluator("x + y", ["x"])  # y not defined at compile time


def test_error_unused_variables():
    """Test error handling for declared but unused variables."""
    with pytest.raises(ValueError, match="Declared variables not used in expression: f, t"):
        ExpressionEvaluator("pi", ["f", "t"])  # f and t declared but not used

    with pytest.raises(ValueError, match="Declared variables not used in expression: z"):
        ExpressionEvaluator("x + y", ["x", "y", "z"])  # z declared but not used

    # Should not raise error if all variables are used
    evaluator = ExpressionEvaluator("x + y", ["x", "y"])  # All variables used
    assert evaluator.evaluate(1, 2) == 3

    # Should allow declaring built-in constants as variables (to override them)
    evaluator = ExpressionEvaluator("pi + e", ["pi", "e"])  # Overriding built-ins
    assert evaluator.evaluate(3.0, 2.0) == 5.0


def test_error_wrong_argument_count():
    """Test error handling for wrong number of arguments."""
    # Test too few arguments
    evaluator = ExpressionEvaluator("x + y", ["x", "y"])
    with pytest.raises(ValueError, match="Expected 2 arguments for variables \\(x, y\\), got 1"):
        evaluator.evaluate(5)  # Missing argument for y

    # Test too many arguments
    with pytest.raises(ValueError, match="Expected 2 arguments for variables \\(x, y\\), got 3"):
        evaluator.evaluate(1, 2, 3)  # Too many arguments

    # Test no-variable expression called with arguments
    evaluator_no_vars = ExpressionEvaluator("pi + e")
    with pytest.raises(ValueError, match="No variables expected"):
        evaluator_no_vars.evaluate(1.0)  # Should be called without arguments


def test_error_unsupported_function():
    """Test error handling for unsupported functions."""
    with pytest.raises(ValueError, match="Unsupported function"):
        ExpressionEvaluator("print(5)")

    with pytest.raises(ValueError, match="Unsupported function"):
        ExpressionEvaluator("eval('1+1')")


def test_error_unsupported_operations():
    """Test error handling for unsupported operations."""
    # Bitwise operations not supported
    with pytest.raises(ValueError, match="Unsupported operator"):
        ExpressionEvaluator("5 & 3")

    with pytest.raises(ValueError, match="Unsupported operator"):
        ExpressionEvaluator("5 | 3")


def test_error_complex_function_calls():
    """Test error handling for complex function calls."""
    with pytest.raises(ValueError, match="Expression compilation failed.*Only simple function calls are supported"):
        ExpressionEvaluator("obj.method()")


def test_error_division_by_zero():
    """Test error handling for division by zero."""
    evaluator = ExpressionEvaluator("1 / 0")
    with pytest.raises(ValueError, match="Evaluation error"):
        evaluator.evaluate()


def test_error_math_domain_errors():
    """Test error handling for mathematical domain errors."""
    evaluator = ExpressionEvaluator("sqrt(-1)")
    with pytest.raises(ValueError, match="Evaluation error"):
        evaluator.evaluate()

    evaluator = ExpressionEvaluator("log(0)")
    with pytest.raises(ValueError, match="Evaluation error"):
        evaluator.evaluate()


def test_multiple_comparisons():
    """Test chained comparison operations."""
    evaluator = ExpressionEvaluator("1 < 2 < 3")
    assert evaluator.evaluate() == 1.0  # True

    evaluator = ExpressionEvaluator("1 < 3 < 2")
    assert evaluator.evaluate() == 0.0  # False

    evaluator = ExpressionEvaluator("x <= y <= z", ["x", "y", "z"])
    assert evaluator.evaluate(1, 2, 3) == 1.0  # x=1, y=2, z=3
    assert evaluator.evaluate(2, 1, 3) == 0.0  # x=2, y=1, z=3 (False)


def test_operator_precedence():
    """Test that operator precedence is correctly handled."""
    evaluator = ExpressionEvaluator("2 + 3 * 4")
    assert evaluator.evaluate() == 14.0  # Not 20

    evaluator = ExpressionEvaluator("2 ** 3 ** 2")
    assert evaluator.evaluate() == 512.0  # Right associative: 2^(3^2) = 2^9

    evaluator = ExpressionEvaluator("10 - 6 / 2")
    assert evaluator.evaluate() == 7.0  # Not 2


def test_parentheses():
    """Test parentheses for grouping operations."""
    evaluator = ExpressionEvaluator("(2 + 3) * 4")
    assert evaluator.evaluate() == 20.0

    evaluator = ExpressionEvaluator("2 * (3 + 4)")
    assert evaluator.evaluate() == 14.0


def test_nested_function_calls():
    """Test nested function calls."""
    evaluator = ExpressionEvaluator("sin(cos(0))")
    expected = math.sin(math.cos(0))
    assert abs(evaluator.evaluate() - expected) < 1e-10

    evaluator = ExpressionEvaluator("max(min(5, 10), 3)")
    assert evaluator.evaluate() == 5.0


def test_expression_recompilation():
    """Test that expressions are properly compiled only once."""
    evaluator = ExpressionEvaluator("x * 2", ["x"])

    # First evaluation should compile
    result1 = evaluator.evaluate(5)
    assert result1 == 10.0

    # Second evaluation should use compiled version
    result2 = evaluator.evaluate(7)
    assert result2 == 14.0

    # Verify the compiled function exists
    assert evaluator._compiled is not None


def test_variable_order_matters():
    """Test that variables are evaluated in the order they were specified."""
    evaluator = ExpressionEvaluator("a - b", ["a", "b"])
    assert evaluator.evaluate(10, 3) == 7.0  # a=10, b=3: 10-3=7
    assert evaluator.evaluate(3, 10) == -7.0  # a=3, b=10: 3-10=-7

    # Different order in list should maintain the same argument order
    evaluator2 = ExpressionEvaluator("x / y + z", ["x", "y", "z"])
    assert evaluator2.evaluate(12, 3, 1) == 5.0  # x=12, y=3, z=1: 12/3+1=5
    assert evaluator2.evaluate(6, 2, 10) == 13.0  # x=6, y=2, z=10: 6/2+10=13
