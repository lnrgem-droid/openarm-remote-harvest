#!/usr/bin/env python3
"""Verify source snapshot and locally compiled overlay; never opens CAN."""
import argparse
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''): h.update(block)
    return h.hexdigest()


def verify(root=ROOT, built=True):
    manifests = ['release.sha256'] + (['installed.sha256'] if built else [])
    for manifest in manifests:
        lines = (root/manifest).read_text().splitlines()
        if not lines: raise RuntimeError('empty manifest: '+manifest)
        for line in lines:
            expected, name = line.split('  ', 1)
            path = (root/name).resolve()
            if not path.is_relative_to(root.resolve()) or digest(path) != expected:
                raise RuntimeError('版本校验失败: '+name)
    if built and (root/'built_from.sha256').read_text().strip() != digest(root/'release.sha256'):
        raise RuntimeError('安装源码与已编译版本不一致，请重新安装')
    return digest(root/'release.sha256')

if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--source-only', action='store_true')
    a = p.parse_args()
    print(verify(built=not a.source_only))
