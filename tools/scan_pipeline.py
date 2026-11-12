#!/usr/bin/env python3
"""Provozní diagnostika: kolik úplných skenů doopravdy dorazí a zpracuje SLAM.

Čtečku UART nikdy neblokujeme drahým scan matcherem. Pokud bounded
fronta přeteče, zahodí se nejstarší čekající sken a ztráta se počítá.
Žádný pevný časový gate na přijímané otáčky již neexistuje.
"""
from collections import deque
from queue import Empty, Full
import statistics
import threading
import time


class PipelineStats:
    def __init__(self):
        self.lock = threading.Lock()
        self.received = 0
        self.enqueued = 0
        self.processed = 0
        self.dropped = 0
        self.errors = 0
        self.rx_times = deque(maxlen=160)
        self.done_times = deque(maxlen=160)
        self.processing_ms = deque(maxlen=30)
        self.lag_ms = deque(maxlen=30)
        self.imu_sync_ms = deque(maxlen=30)
        self.camera_sync_ms = deque(maxlen=30)

    def received_scan(self, timestamp=None, imu=None, camera=None):
        now = time.monotonic() if timestamp is None else timestamp
        with self.lock:
            self.received += 1
            self.rx_times.append(now)
            if imu is not None:
                self.imu_sync_ms.append(imu["sync_ms"])
            if camera is not None:
                self.camera_sync_ms.append(camera["sync_ms"])

    def queued(self):
        with self.lock:
            self.enqueued += 1

    def lost_overload(self):
        with self.lock:
            self.dropped += 1

    def finished(self, duration_ms, lag_ms, failed=False):
        with self.lock:
            self.processed += 1
            self.done_times.append(time.monotonic())
            self.processing_ms.append(float(duration_ms))
            self.lag_ms.append(float(lag_ms))
            if failed:
                self.errors += 1

    def snapshot(self, queue_size=0):
        now = time.monotonic()
        with self.lock:
            def rate(times):
                recent = sum(t >= now - 3 for t in times)
                # Okno 3 s; nevydávat po rozběhu přehnané FPS.
                return round(recent / 3.0, 1)

            def avg(values):
                return round(statistics.mean(values), 1) if values else None

            return {
                "received": self.received,
                "enqueued": self.enqueued,
                "processed": self.processed,
                "dropped_overload": self.dropped,
                "worker_errors": self.errors,
                "input_hz": rate(self.rx_times),
                "processed_hz": rate(self.done_times),
                "pending": int(queue_size),
                "processing_ms": avg(self.processing_ms),
                "latency_ms": avg(self.lag_ms),
                "imu_sync_ms": avg(self.imu_sync_ms),
                "camera_sync_ms": avg(self.camera_sync_ms),
            }


def queue_scan(scan_queue, item, stats):
    """Předat každý sken; bounded fronta chrání real-time režim."""
    try:
        scan_queue.put_nowait(item)
    except Full:
        try:
            scan_queue.get_nowait()
            scan_queue.task_done()
            stats.lost_overload()
        except Empty:
            pass
        try:
            scan_queue.put_nowait(item)
        except Full:
            # Frontu mezitím mohl naplnit jiný producent.
            stats.lost_overload()
            return False
    stats.queued()
    return True
