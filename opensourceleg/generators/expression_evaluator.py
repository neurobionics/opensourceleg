import ast
import math
import operator as op
from typing import Any, Callable, Optional, cast

UNARY_OPERATORS: dict[type[ast.unaryop], Callable[[Any], Any]] = {
    ast.USub: op.neg,
    ast.UAdd: op.pos,
}

BINARY_OPERATORS: dict[type[ast.operator], Callable[[Any, Any], Any]] = {
    ast.Add: op.add,
    ast.Sub: op.sub,
    ast.Mult: op.mul,
    ast.Div: op.truediv,
    ast.Pow: op.pow,
    ast.Mod: op.mod,
    ast.FloorDiv: op.floordiv,
}

COMPARISON_OPERATORS: dict[type[ast.cmpop], Callable[[Any, Any], bool]] = {
    ast.Eq: op.eq,
    ast.NotEq: op.ne,
    ast.Lt: op.lt,
    ast.LtE: op.le,
    ast.Gt: op.gt,
    ast.GtE: op.ge,
}

SAFE_FUNCTIONS: dict[str, Callable[..., Any]] = {
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
}


class ExpressionEvaluator:
    """Safely evaluate mathematical expressions with variables"""

    def __init__(self, expression: str, variables: Optional[list[str]] = None):
        self.expression = expression
        self._variable_names = variables or []
        self._expected_arg_count = len(self._variable_names)

        # Create set of available variables for validation (much more efficient)
        self._available_vars = {"pi", "e"} | set(self._variable_names)

        self._var_positions = {name: i for i, name in enumerate(self._variable_names)}

        self._compiled: Optional[Callable[..., float]] = None
        self._compile_expression()

        if self._compiled is None:
            raise RuntimeError("Expression compilation failed")
        self._compiled_func: Callable[..., float] = self._compiled

    def _compile_expression(self) -> None:
        """Compile the expression into a safe function"""
        # Track variables used during compilation for validation
        self._used_variables: set[str] = set()

        try:
            node = ast.parse(self.expression, mode="eval")
            self._compiled = self._compile_node(node.body)

            # Validate that all declared variables are actually used
            if self._variable_names:
                declared_vars = set(self._variable_names)
                unused_vars = declared_vars - self._used_variables

                if unused_vars:
                    unused_list = ", ".join(sorted(unused_vars))
                    raise ValueError(f"Declared variables not used in expression: {unused_list}")  # noqa: TRY301

        except SyntaxError as e:
            raise ValueError(f"Invalid expression syntax: {e}") from e
        except Exception as e:
            raise ValueError(f"Expression compilation failed: {e}") from e

    def _compile_node(self, node: ast.AST) -> Callable[..., float]:
        """Compile AST nodes by dispatching to specific node compilers based on node type"""

        if isinstance(node, ast.Constant):
            return self._compile_constant(node)

        elif isinstance(node, ast.Name):
            return self._compile_name(node)

        elif isinstance(node, ast.UnaryOp):
            return self._compile_unaryop(node)

        elif isinstance(node, ast.BinOp):
            return self._compile_binop(node)

        elif isinstance(node, ast.Compare):
            return self._compile_compare(node)

        elif isinstance(node, ast.Call):
            return self._compile_call(node)

        elif isinstance(node, ast.IfExp):
            return self._compile_ifexp(node)

        else:
            raise TypeError(f"Unsupported node type: {type(node).__name__}")

    def _compile_constant(self, node: ast.Constant) -> Callable[..., float]:
        """Compile constant value node"""
        if isinstance(node.value, (int, float, str)):
            value = cast(int | float | str, node.value)
            return lambda *args: float(value)
        else:
            raise TypeError(f"Unsupported constant type: {type(node.value).__name__}")

    def _compile_name(self, node: ast.Name) -> Callable[..., float]:
        """Compile variable name node"""
        name = node.id
        if name not in self._available_vars:
            raise ValueError(f"Undefined variable: {name}")

        # Track usage of declared variables (not built-in constants unless overridden)
        if name in self._variable_names:
            self._used_variables.add(name)

        # Handle built-in constants
        if name == "pi" and name not in self._variable_names:
            return lambda *args: math.pi
        elif name == "e" and name not in self._variable_names:
            return lambda *args: math.e
        else:
            # Get position of variable in argument list
            pos = self._var_positions[name]
            return lambda *args: args[pos]

    def _compile_unaryop(self, node: ast.UnaryOp) -> Callable[..., float]:
        """Compile unary operation node"""
        operand = self._compile_node(node.operand)
        op_func: Optional[Callable[[Any], Any]] = UNARY_OPERATORS.get(type(node.op))
        if op_func is None:
            raise ValueError(f"Unsupported operator: {type(node.op).__name__}")
        return lambda *args: op_func(operand(*args))

    def _compile_binop(self, node: ast.BinOp) -> Callable[..., float]:
        """Compile binary operation node"""
        left = self._compile_node(node.left)
        right = self._compile_node(node.right)
        opr_func: Optional[Callable[[Any, Any], Any]] = BINARY_OPERATORS.get(type(node.op))
        if opr_func is None:
            raise ValueError(f"Unsupported operator: {type(node.op).__name__}")
        return lambda *args: opr_func(left(*args), right(*args))

    def _compile_compare(self, node: ast.Compare) -> Callable[..., float]:
        """Compile comparison node"""
        left = self._compile_node(node.left)
        comparators = [self._compile_node(c) for c in node.comparators]
        ops: list[Optional[Callable[[Any, Any], bool]]] = [COMPARISON_OPERATORS.get(type(op)) for op in node.ops]
        if any(op_func is None for op_func in ops):
            raise ValueError("Unsupported comparison operator")
        return lambda *args: self._evaluate_compare_args(left, ops, comparators, args)

    def _evaluate_compare_args(
        self,
        left: Callable[..., Any],
        ops: list[Optional[Callable[[Any, Any], bool]]],
        comparators: list[Callable[..., Any]],
        args: tuple[Any, ...],
    ) -> bool:
        """Evaluate comparison operators with positional arguments"""
        lval = left(*args)
        for op_func, comp in zip(ops, comparators):
            rval = comp(*args)
            if op_func is None:
                raise ValueError("Comparison operator is None")
            if not op_func(lval, rval):
                return False
            lval = rval
        return True

    def _compile_call(self, node: ast.Call) -> Callable[..., float]:
        """Compile function call node"""
        if not isinstance(node.func, ast.Name):
            raise TypeError("Only simple function calls are supported")

        func_name = node.func.id
        if func_name not in SAFE_FUNCTIONS:
            raise ValueError(f"Unsupported function: {func_name}")

        args = [self._compile_node(arg) for arg in node.args]
        func: Callable[..., Any] = SAFE_FUNCTIONS[func_name]
        return lambda *call_args: func(*(arg(*call_args) for arg in args))

    def _compile_ifexp(self, node: ast.IfExp) -> Callable[..., float]:
        """Compile conditional expression node"""
        test = self._compile_node(node.test)
        body = self._compile_node(node.body)
        orelse = self._compile_node(node.orelse)
        return lambda *args: body(*args) if test(*args) else orelse(*args)

    def evaluate(self, *args: Any) -> float:
        """
        Evaluate the expression with given variable values.

        Args:
            *args: Variable values in the same order as specified in the variables list
                  during initialization. Built-in constants (pi, e) are automatically included.

        Returns:
            The evaluated expression result as a float

        Raises:
            ValueError: If wrong number of arguments provided or evaluation fails
        """
        # Fast path: check argument count using cached value
        if len(args) != self._expected_arg_count:
            self._raise_arg_count_error(len(args))

        try:
            return self._compiled_func(*args)
        except Exception as e:
            raise ValueError(f"Evaluation error: {e}") from e

    def _raise_arg_count_error(self, received_count: int) -> None:
        """Helper method to raise argument count error with detailed message."""
        if self._expected_arg_count == 0:
            raise ValueError("No variables expected. Call evaluate() without arguments.")
        else:
            var_names = ", ".join(self._variable_names)
            raise ValueError(
                f"Expected {self._expected_arg_count} arguments for variables ({var_names}), got {received_count}"
            )


if __name__ == "__main__":
    evaluator = ExpressionEvaluator("4 * cos(2*pi*f*t)", ["f", "t"])
    print(evaluator.evaluate(1.0, 0.0))  # f=1.0, t=0.0
