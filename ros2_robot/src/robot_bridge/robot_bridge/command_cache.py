"""最新命令缓存。

纯逻辑、线程安全：外部 50Hz 写入最新目标值，桥接按 control_rate 读取。
没有新数据时 get() 仍返回上一次的值（Zero-Order Hold），
直到超过 ttl_ms 判定为过期，由调用方决定安全策略。
"""

import threading
import time


class CachedValue:
    __slots__ = ("value", "stamp", "ttl")

    def __init__(self, value, stamp: float, ttl_ms: float):
        self.value = value
        self.stamp = stamp
        self.ttl = ttl_ms / 1000.0

    def expired(self, now: float) -> bool:
        return (now - self.stamp) > self.ttl


class CommandCache:
    """按 key 保存最新命令值，每个 key 独立时间戳和 TTL。"""

    def __init__(self, default_ttl_ms: float = 100.0):
        self._lock = threading.Lock()
        self._store: dict[str, CachedValue] = {}
        self._default_ttl = default_ttl_ms

    def set(self, key: str, value, ttl_ms: float | None = None) -> None:
        with self._lock:
            self._store[key] = CachedValue(
                value, time.monotonic(), ttl_ms if ttl_ms is not None else self._default_ttl
            )

    def get(self, key: str):
        """返回 (value, expired)；key 不存在返回 (None, True)。"""
        now = time.monotonic()
        with self._lock:
            cv = self._store.get(key)
            if cv is None:
                return None, True
            return cv.value, cv.expired(now)

    def clear(self, key: str | None = None) -> None:
        with self._lock:
            if key is None:
                self._store.clear()
            else:
                self._store.pop(key, None)
