#!/usr/bin/env python3
"""Časové přiřazení asynchronních měření LiDAR/IMU/CSI.

Senzory sdílejí time.monotonic() (software timestamp při přijetí).
Hodnoty lze interpolovat pouze uvnitř stejné kalibrační session.
Při zastaralém měření není extrapolace: vracíme None.
Nejde o hardwarovou synchronizaci ani de-skew jednotlivých paprsků.
"""
from collections import deque
import bisect
import math
import threading


class TimedHistory:
    """Omezená a thread-safe historie malých slovníků měření."""

    def __init__(self, capacity=512):
        if capacity < 2:
            raise ValueError("capacity must be >= 2")
        self.lock = threading.RLock()
        self.rows = deque(maxlen=capacity)

    def clear(self):
        with self.lock:
            self.rows.clear()

    def add(self, timestamp, values):
        stamp = float(timestamp)
        if not math.isfinite(stamp):
            raise ValueError("timestamp must be finite")
        data = dict(values)
        with self.lock:
            if self.rows and stamp <= self.rows[-1][0]:
                # MonoClock must never go backward / repeat in one session.
                return False
            self.rows.append((stamp, data))
        return True

    def at(self, timestamp, max_age=0.15, max_gap=0.25):
        """Vrátí senzorem naměřená data pro čas dokončení LiDAR skenu.

        Interpolace mezi vzorky jen při stejné session. Jinak pouze
        nejbližší předchozí vzorek bez extrapolace; staré vzorky odmítá.
        """
        target = float(timestamp)
        if not math.isfinite(target) or max_age <= 0 or max_gap <= 0:
            return None
        with self.lock:
            if not self.rows:
                return None
            rows = tuple(self.rows)
        stamps = [row[0] for row in rows]
        idx = bisect.bisect_right(stamps, target)
        if idx == 0:
            # Žádné vzorky před cílovým časem: neextrapolovat do minulosti.
            return None
        previous_t, previous = rows[idx - 1]
        age = target - previous_t
        if age > max_age:
            return None
        result = dict(previous)
        result["sync_ms"] = round(age * 1000.0, 2)
        if idx < len(rows):
            next_t, nxt = rows[idx]
            gap = next_t - previous_t
            if (gap <= max_gap and nxt.get("session") == previous.get("session")
                    and next_t - target <= max_age):
                factor = (target - previous_t) / gap
                for key, old in previous.items():
                    new = nxt.get(key)
                    if (key != "session" and isinstance(old, (int, float))
                            and not isinstance(old, bool)
                            and isinstance(new, (int, float))
                            and not isinstance(new, bool)
                            and math.isfinite(old) and math.isfinite(new)):
                        result[key] = old + (new - old) * factor
                result["sync_ms"] = round(min(age, next_t - target) * 1000.0, 2)
        return result
