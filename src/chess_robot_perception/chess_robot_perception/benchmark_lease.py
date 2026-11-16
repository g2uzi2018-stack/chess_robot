"""Cooperative production inference lease; offline tests may own the outer lock."""
import os
import time


class BenchmarkLease:
    def __init__(self):
        self.path = os.environ.get('XIANGQI_BENCHMARK_LOCK')
        self.file = None
        self.until = 0.

    def can_submit(self, busy):
        if not self.path:
            return True
        import fcntl
        if self.file is not None and time.monotonic() >= self.until:
            if busy:
                return False  # drain all owned slots before handing over
            self.close()
            return False
        if self.file is None:
            handle = open(self.path, 'a')
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                handle.close()
                return False
            self.file = handle
            self.until = time.monotonic() + 1.
        return True

    def close(self):
        if self.file is not None:
            import fcntl
            fcntl.flock(self.file, fcntl.LOCK_UN)
            self.file.close()
            self.file = None
