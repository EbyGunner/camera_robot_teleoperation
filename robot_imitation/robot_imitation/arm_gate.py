"""Flow control for a single arm's planning goals.

Pure Python (threading only) so it stays unit-testable without ROS.
"""

import threading
from typing import Any, Optional


class ArmGate:
    """One goal in flight per arm; the newest pending target wins."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pending: Any = None
        self._has_pending = False
        self._busy = False
        self._busy_since: Optional[float] = None

    # ------------------------------------------------------------------
    def update(self, payload: Any) -> None:
        """Record the freshest target, replacing any queued one."""
        with self._lock:
            self._pending = payload
            self._has_pending = True

    def clear(self) -> None:
        """Forget the queued target (e.g. the hand disappeared).

        Does not touch a goal already in flight - cancelling that is the
        planner's job, and letting the current motion finish is the less
        surprising behaviour when a hand flickers out for a few frames.
        """
        with self._lock:
            self._pending = None
            self._has_pending = False

    # ------------------------------------------------------------------
    def take(self, now: float) -> Optional[Any]:
        """Return the pending target and mark the arm busy, or None."""
        with self._lock:
            if self._busy or not self._has_pending:
                return None
            payload = self._pending
            self._pending = None
            self._has_pending = False
            self._busy = True
            self._busy_since = now
            return payload

    def complete(self) -> None:
        """Mark the in-flight goal finished (success, failure or cancel)."""
        with self._lock:
            self._busy = False
            self._busy_since = None

    # ------------------------------------------------------------------
    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy

    @property
    def has_pending(self) -> bool:
        with self._lock:
            return self._has_pending

    def active_age(self, now: float) -> Optional[float]:
        """Seconds the current goal has been in flight, or None if idle."""
        with self._lock:
            if not self._busy or self._busy_since is None:
                return None
            return now - self._busy_since