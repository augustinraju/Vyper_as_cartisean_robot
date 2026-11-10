"""Read numeric #define values from the Marlin firmware configuration headers.

Only what the host side needs: active (not commented-out) `#define NAME value` lines,
with values that are numbers or arithmetic on other defines, e.g.
`#define Y_MAX_POS Y_BED_SIZE+MANUAL_Y_HOME_POS`. Preprocessor conditionals (#if)
are not evaluated; if a name is defined more than once, the first definition wins.
"""

from __future__ import annotations

import ast
import operator
import os
import re

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
CONFIG_FILES = (
    os.path.join(REPO_ROOT, "Marlin", "Configuration.h"),
    os.path.join(REPO_ROOT, "Marlin", "Configuration_adv.h"),
)

_DEFINE = re.compile(r"^\s*#define\s+([A-Za-z_]\w*)[ \t]+(.+?)\s*$")
_OPS = {
    ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
    ast.Div: operator.truediv, ast.USub: operator.neg, ast.UAdd: operator.pos,
}


def read_defines(paths=CONFIG_FILES) -> dict[str, str]:
    """{name: raw value text} for every active #define with a value."""
    defines: dict[str, str] = {}
    for path in paths:
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                m = _DEFINE.match(line.split("//", 1)[0])
                if m:
                    defines.setdefault(m.group(1), m.group(2).strip())
    return defines


def _eval(node: ast.AST, defines: dict[str, str], seen: tuple[str, ...]) -> float:
    if isinstance(node, ast.Expression):
        return _eval(node.body, defines, seen)
    if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
        return float(node.value)
    if isinstance(node, ast.UnaryOp) and type(node.op) in _OPS:
        return _OPS[type(node.op)](_eval(node.operand, defines, seen))
    if isinstance(node, ast.BinOp) and type(node.op) in _OPS:
        return _OPS[type(node.op)](_eval(node.left, defines, seen), _eval(node.right, defines, seen))
    if isinstance(node, ast.Name):
        return _value(node.id, defines, seen)
    raise ValueError(f"unsupported expression in {seen[-1]}: {ast.dump(node)}")


def _value(name: str, defines: dict[str, str], seen: tuple[str, ...] = ()) -> float:
    if name in seen:
        raise ValueError(f"recursive #define: {' -> '.join(seen + (name,))}")
    if name not in defines:
        raise KeyError(f"{name} is not #defined in {', '.join(CONFIG_FILES)}")
    return _eval(ast.parse(defines[name], mode="eval"), defines, seen + (name,))


def numeric_defines(*names: str, paths=CONFIG_FILES) -> dict[str, float]:
    """Evaluate the given #define names to floats, e.g. numeric_defines("X_MIN_POS")."""
    defines = read_defines(paths)
    return {name: _value(name, defines) for name in names}
