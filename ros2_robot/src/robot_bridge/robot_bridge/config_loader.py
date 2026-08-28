"""集中式配置加载。

接口名 / Action 名 / launch 命令 / 关节名 / 限位 全部来自 config/bridge_config.yaml，
本模块只负责把它读成 dict，供各模块用 get 链路访问。
文件缺失或格式非法直接抛错——配置就是唯一事实来源，不做内置兜底。
"""

from pathlib import Path

import yaml


def _deep_merge(base: dict, overlay: dict) -> dict:
    """Return a recursive merge without mutating either source mapping."""
    merged = dict(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_config(path: str) -> dict:
    if not path:
        raise ValueError("必须提供配置文件路径（config_file 参数）")
    config_path = Path(path).resolve()
    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ValueError(f"配置文件根节点必须是映射: {path}")
    extends = cfg.pop("extends", None)
    if extends:
        parent_path = (config_path.parent / extends).resolve()
        with parent_path.open("r", encoding="utf-8") as f:
            parent = yaml.safe_load(f)
        if not isinstance(parent, dict):
            raise ValueError(f"基础配置文件根节点必须是映射: {parent_path}")
        cfg = _deep_merge(parent, cfg)
    return cfg
