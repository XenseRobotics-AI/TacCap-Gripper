#!/usr/bin/env python3
"""Check SDK documentation without importing the native SDK or using hardware."""
import ast
import re
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs" / "sdk"


def parameters(function):
    args = function.args
    positional = [a.arg for a in args.args if a.arg != "self"]
    defaults = {a.arg: ast.unparse(v) for a, v in zip(args.args[-len(args.defaults):], args.defaults)}
    defaults.update({a.arg: ast.unparse(v) for a, v in zip(args.kwonlyargs, args.kw_defaults) if v is not None})
    return positional, [a.arg for a in args.kwonlyargs], defaults


def main():
    pages = sorted(DOCS.rglob("*.md"))
    errors = []
    snippets = 0
    for page in pages:
        content = page.read_text()
        for block in re.findall(r"```python\n(.*?)\n```", content, re.DOTALL):
            try:
                ast.parse(block, filename=str(page))
                snippets += 1
            except SyntaxError as exc:
                errors.append(f"{page.relative_to(ROOT)}: {exc}")
        for target in re.findall(r"\]\(([^)]+)\)", content):
            url = urlsplit(target)
            if url.scheme or not url.path:
                continue
            if not (page.parent / unquote(url.path)).exists():
                errors.append(f"{page.relative_to(ROOT)}: missing link {target}")

    module = ast.parse((ROOT / "python/xense/taccap/gripper.py").read_text())
    gripper = next(n for n in module.body if isinstance(n, ast.ClassDef) and n.name == "Gripper")
    reference = (DOCS / "reference/gripper.md").read_text()
    signatures = {}
    for block in re.findall(r"```text\n(.*?)\n```", reference, re.DOTALL):
        for name, args in re.findall(r"(?:Gripper\.)?(\w+)\((.*?)\)", block, re.DOTALL):
            function = ast.parse(f"def {name}({args}):\n    pass").body[0]
            signatures["__init__" if name == "Gripper" else name] = parameters(function)
    methods = 0
    for function in gripper.body:
        if not isinstance(function, ast.FunctionDef):
            continue
        if any(isinstance(d, ast.Name) and d.id == "property" for d in function.decorator_list):
            if f"`{function.name}`" not in reference:
                errors.append(f"Missing property: {function.name}")
            continue
        if function.name.startswith("_") and function.name != "__init__":
            continue
        methods += 1
        if signatures.get(function.name) != parameters(function):
            errors.append(f"Missing or outdated signature: Gripper.{function.name}")

    if errors:
        raise SystemExit("\n".join(errors))
    print(f"OK: {len(pages)} pages, {snippets} Python snippets, {methods} API signatures, local links")


if __name__ == "__main__":
    main()
