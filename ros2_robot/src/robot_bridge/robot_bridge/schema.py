"""可选的入站帧校验。

以 config/schemas/frames.schema.json 为唯一事实来源，校验外部发来的
command / request 帧。默认关闭；仅当 server.validate_incoming=true 且
安装了 jsonschema 时启用，未安装则安静跳过（不引入硬依赖）。
"""

import json
import os

try:
    import jsonschema
except ImportError:  # pragma: no cover
    jsonschema = None


def make_validator(schema_path: str):
    """返回一个 validate(frame)->错误信息|None 的函数；不可用时返回 None。"""
    if jsonschema is None or not schema_path or not os.path.exists(schema_path):
        return None
    with open(schema_path, "r", encoding="utf-8") as f:
        schema = json.load(f)
    validator = jsonschema.Draft7Validator(schema)

    def validate(frame: dict):
        errors = sorted(validator.iter_errors(frame), key=lambda e: e.path)
        if not errors:
            return None
        e = errors[0]
        loc = "/".join(str(p) for p in e.path) or "<root>"
        return f"{loc}: {e.message}"

    return validate
