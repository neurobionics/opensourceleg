import ast
import math
import operator as op
from typing import Any, Callable, Optional

from .base import SignalGenerator

# Supported operations for safe evaluation
SAFE_OPERATORS = {
    ast.Add: op.add,
    ast.Sub: op.sub,
    ast.Mult: op.mul,
    ast.Div: op.truediv,
    ast.Pow: op.pow,
    ast.USub: op.neg,
    ast.UAdd: op.pos,
    ast.Mod: op.mod,
    ast.FloorDiv: op.floordiv,
    ast.Eq: op.eq,
    ast.NotEq: op.ne,
    ast.Lt: op.lt,
    ast.LtE: op.le,
    ast.Gt: op.gt,
    ast.GtE: op.ge,
}

SAFE_FUNCTIONS = {
    "abs": abs,
    "min": min,
    "max": max,
    "round": round,
    "sin": math.sin,
    "cos": math.cos,
    "tan": math.tan,
    "asin": math.asin,
    "acos": math.acos,
    "atan": math.atan,
    "atan2": math.atan2,
    "sinh": math.sinh,
    "cosh": math.cosh,
    "tanh": math.tanh,
    "exp": math.exp,
    "log": math.log,
    "log10": math.log10,
    "sqrt": math.sqrt,
    "ceil": math.ceil,
    "floor": math.floor,
    "radians": math.radians,
    "degrees": math.degrees,
}


class ExpressionEvaluator:
    """Safely evaluate mathematical expressions with variables"""

    def __init__(self, expression: str, variables: Optional[dict[str, Any]] = None):
        self.expression = expression
        self.variables = {"pi": math.pi, "e": math.e, **(variables or {})}
        self._compiled = None
        self._compile_expression()

    def _compile_expression(self) -> None:
        """Compile the expression into a safe function"""
        try:
            node = ast.parse(self.expression, mode="eval")
            self._compiled = self._compile_node(node.body)
        except SyntaxError as e:
            raise ValueError(f"Invalid expression syntax: {e}") from e
        except Exception as e:
            raise ValueError(f"Expression compilation failed: {e}") from e

    def _compile_node(self, node) -> Callable[[dict[str, Any]], float]:  # noqa: C901
        """Recursively compile AST nodes"""

        if isinstance(node, ast.Constant):
            return lambda env: node.value

        elif isinstance(node, ast.Name):
            name = node.id
            if name not in self.variables:
                raise ValueError(f"Undefined variable: {name}")
            return lambda env: env[name]

        elif isinstance(node, ast.UnaryOp):
            operand = self._compile_node(node.operand)
            op_func = SAFE_OPERATORS.get(type(node.op))
            if op_func is None:
                raise ValueError(f"Unsupported operator: {type(node.op).__name__}")
            return lambda env: op_func(operand(env))

        elif isinstance(node, ast.BinOp):
            left = self._compile_node(node.left)
            right = self._compile_node(node.right)
            op_func = SAFE_OPERATORS.get(type(node.op))
            if op_func is None:
                raise ValueError(f"Unsupported operator: {type(node.op).__name__}")
            return lambda env: op_func(left(env), right(env))

        elif isinstance(node, ast.Compare):
            left = self._compile_node(node.left)
            comparators = [self._compile_node(c) for c in node.comparators]
            ops = [SAFE_OPERATORS.get(type(op)) for op in node.ops]
            if any(op_func is None for op_func in ops):
                raise ValueError("Unsupported comparison operator")
            return lambda env: self._evaluate_compare(left, ops, comparators, env)

        elif isinstance(node, ast.Call):
            if not isinstance(node.func, ast.Name):
                raise TypeError("Only simple function calls are supported")

            func_name = node.func.id
            if func_name not in SAFE_FUNCTIONS:
                raise ValueError(f"Unsupported function: {func_name}")

            args = [self._compile_node(arg) for arg in node.args]
            func = SAFE_FUNCTIONS[func_name]
            return lambda env: func(*(arg(env) for arg in args))

        elif isinstance(node, ast.IfExp):
            test = self._compile_node(node.test)
            body = self._compile_node(node.body)
            orelse = self._compile_node(node.orelse)
            return lambda env: body(env) if test(env) else orelse(env)

        else:
            raise TypeError(f"Unsupported node type: {type(node).__name__}")

    def _evaluate_compare(self, left, ops, comparators, env):
        """Evaluate comparison operators"""
        lval = left(env)
        for op_func, comp in zip(ops, comparators):
            rval = comp(env)
            if not op_func(lval, rval):
                return False
            lval = rval
        return True

    def evaluate(self, variables: Optional[dict[str, Any]] = None) -> float:
        """Evaluate the expression with given variables"""
        if self._compiled is None:
            self._compile_expression()

        env = {**self.variables, **(variables or {})}
        try:
            return self._compiled(env)
        except KeyError as e:
            raise ValueError(f"Missing variable: {e}") from e
        except Exception as e:
            raise ValueError(f"Evaluation error: {e}") from e


class CustomGenerator(SignalGenerator):
    def __init__(self, expression: str, variables: Optional[dict[str, Any]] = None, **kwargs):
        """
        Custom signal generator from mathematical expression.

        Args:
            expression: Mathematical expression using variables
            variables: Dictionary of variables used in expression
            **kwargs: Base class parameters

        Example:
            gen = CustomGenerator(
                expression="A * sin(2*pi*f*t) + noise",
                variables={'A': 1.0, 'f': 0.5},
                noise_amplitude=0.1
            )
        """
        super().__init__(**kwargs)
        self.expression = expression
        self.variables = variables or {}

        # Add time to variables
        self.variables["t"] = 0.0

        # Compile expression
        self._evaluator = ExpressionEvaluator(expression, self.variables)

    def _generate(self) -> float:
        # Update time variable
        self.variables["t"] = self._time
        return self._evaluator.evaluate(self.variables)
