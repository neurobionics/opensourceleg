import ast
import math
import operator as op
from typing import Any, Callable, Optional, cast

# Supported operations for safe evaluation
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
    "radians": math.radians,
    "degrees": math.degrees,
}


class ExpressionEvaluator:
    """Safely evaluate mathematical expressions with variables"""

    def __init__(self, expression: str, variables: Optional[dict[str, Any]] = None):
        self.expression = expression
        self.variables = {"pi": math.pi, "e": math.e, **(variables or {})}
        self._compiled: Optional[Callable[[dict[str, Any]], float]] = None
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

    def _compile_node(self, node: ast.AST) -> Callable[[dict[str, Any]], float]:
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

    def _compile_constant(self, node: ast.Constant) -> Callable[[dict[str, Any]], float]:
        """Compile constant value node"""
        return lambda env: node.value

    def _compile_name(self, node: ast.Name) -> Callable[[dict[str, Any]], float]:
        """Compile variable name node"""
        name = node.id
        if name not in self.variables:
            raise ValueError(f"Undefined variable: {name}")
        return lambda env: env[name]

    def _compile_unaryop(self, node: ast.UnaryOp) -> Callable[[dict[str, Any]], float]:
        """Compile unary operation node"""
        operand = self._compile_node(node.operand)
        op_func: Optional[Callable[[Any], Any]] = UNARY_OPERATORS.get(type(node.op))
        if op_func is None:
            raise ValueError(f"Unsupported operator: {type(node.op).__name__}")
        return lambda env: op_func(operand(env))

    def _compile_binop(self, node: ast.BinOp) -> Callable[[dict[str, Any]], float]:
        """Compile binary operation node"""
        left = self._compile_node(node.left)
        right = self._compile_node(node.right)
        opr_func: Optional[Callable[[Any, Any], Any]] = BINARY_OPERATORS.get(type(node.op))
        if opr_func is None:
            raise ValueError(f"Unsupported operator: {type(node.op).__name__}")
        return lambda env: opr_func(left(env), right(env))

    def _compile_compare(self, node: ast.Compare) -> Callable[[dict[str, Any]], float]:
        """Compile comparison node"""
        left = self._compile_node(node.left)
        comparators = [self._compile_node(c) for c in node.comparators]
        ops: list[Optional[Callable[[Any, Any], bool]]] = [COMPARISON_OPERATORS.get(type(op)) for op in node.ops]
        if any(op_func is None for op_func in ops):
            raise ValueError("Unsupported comparison operator")
        return lambda env: self._evaluate_compare(left, ops, comparators, env)

    def _compile_call(self, node: ast.Call) -> Callable[[dict[str, Any]], float]:
        """Compile function call node"""
        if not isinstance(node.func, ast.Name):
            raise TypeError("Only simple function calls are supported")

        func_name = node.func.id
        if func_name not in SAFE_FUNCTIONS:
            raise ValueError(f"Unsupported function: {func_name}")

        args = [self._compile_node(arg) for arg in node.args]
        func: Callable[..., Any] = SAFE_FUNCTIONS[func_name]
        return lambda env: func(*(arg(env) for arg in args))

    def _compile_ifexp(self, node: ast.IfExp) -> Callable[[dict[str, Any]], float]:
        """Compile conditional expression node"""
        test = self._compile_node(node.test)
        body = self._compile_node(node.body)
        orelse = self._compile_node(node.orelse)
        return lambda env: body(env) if test(env) else orelse(env)

    def _evaluate_compare(
        self,
        left: Callable[[dict[str, Any]], Any],
        ops: list[Optional[Callable[[Any, Any], bool]]],
        comparators: list[Callable[[dict[str, Any]], Any]],
        env: dict[str, Any],
    ) -> bool:
        """Evaluate comparison operators"""
        lval = left(env)
        for op_func, comp in zip(ops, comparators):
            rval = comp(env)
            if op_func is None:
                raise ValueError("Comparison operator is None")
            if not op_func(lval, rval):
                return False
            lval = rval
        return True

    def evaluate(self, variables: Optional[dict[str, Any]] = None) -> float:
        """Evaluate the expression with given variables"""
        if self._compiled is None:
            self._compile_expression()

        self._compiled = cast(Callable[[dict[str, Any]], float], self._compiled)
        env = {**self.variables, **(variables or {})}
        try:
            return float(self._compiled(env))
        except KeyError as e:
            raise ValueError(f"Missing variable: {e}") from e
        except Exception as e:
            raise ValueError(f"Evaluation error: {e}") from e
