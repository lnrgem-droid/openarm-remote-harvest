"""集中式配置加载。

接口名 / Action 名 / launch 命令 / 关节名 / 限位 全部来自 config/bridge_config.yaml，
本模块只负责把它读成 dict，供各模块用 get 链路访问。
文件缺失或格式非法直接抛错——配置就是唯一事实来源，不做内置兜底。
"""

import yaml


def load_config(path: str) -> dict:
    if not path:
        raise ValueError("必须提供配置文件路径（config_file 参数）")
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ValueError(f"配置文件根节点必须是映射: {path}")
    return cfg
