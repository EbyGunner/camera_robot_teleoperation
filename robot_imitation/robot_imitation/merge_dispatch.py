"""Batching of arm targets into single planning goals.

move_group executes ONE motion goal at a time, so per-arm goals can only
ever alternate. This dispatcher instead merges the freshest target of each
arm into one batch: when both arms have fresh targets, the batch covers
both (planned as the combined SRDF group and executed simultaneously);
when only one hand is active, a short merge window is allowed for the
other hand to join before a single-arm batch goes out.

Pure Python (threading only) so it is unit-testable without ROS. Replaces
the earlier SerialDispatcher (dispatch.py), which took turns.
"""

import threading
from typing import Any, Dict, List, Optional


class MergeDispatcher:
    def __init__(self, keys: List[str], merge_window: float):
        self._keys = list(keys)
        self._window = float(merge_window)
        self._lock = threading.Lock()
        self._latest: Dict[str, Any] = {}       # freshest payload per key
        self._dirty_since: Dict[str, float] = {}  # key -> first-dirty time
        self._busy = False
        self._busy_since: Optional[float] = None
        self._busy_keys: List[str] = []

    # ------------------------------------------------------------------
    def update(self, key: str, payload: Any, now: float) -> None:
        """Record the freshest target for *key* (marks it pending)."""
        with self._lock:
            self._latest[key] = payload
            self._dirty_since.setdefault(key, now)

    def clear(self, key: str) -> None:
        """Forget a pending target (e.g. the hand disappeared)."""
        with self._lock:
            self._latest.pop(key, None)
            self._dirty_since.pop(key, None)

    # ------------------------------------------------------------------
    def take_batch(self, now: float) -> Optional[Dict[str, Any]]:
        """Return {key: payload} to send now, or None.

        A batch is released when nothing is in flight and either every
        key has a pending target (no reason to wait) or the oldest
        pending target has already waited out the merge window.
        """
        with self._lock:
            if self._busy or not self._dirty_since:
                return None
            all_dirty = len(self._dirty_since) == len(self._keys)
            oldest = min(self._dirty_since.values())
            if not all_dirty and (now - oldest) < self._window:
                return None
            batch = {k: self._latest[k] for k in self._keys
                     if k in self._dirty_since}
            self._dirty_since.clear()
            self._busy = True
            self._busy_since = now
            self._busy_keys = list(batch.keys())
            return batch

    def complete(self, now: float) -> None:
        """Mark the in-flight batch finished (success, failure or cancel)."""
        with self._lock:
            self._busy = False
            self._busy_since = None
            self._busy_keys = []

    # ------------------------------------------------------------------
    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy

    @property
    def busy_keys(self) -> List[str]:
        with self._lock:
            return list(self._busy_keys)

    def active_age(self, now: float) -> Optional[float]:
        with self._lock:
            if not self._busy or self._busy_since is None:
                return None
            return now - self._busy_since
