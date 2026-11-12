"""Propagate a failed ROS spin worker to its owning foreground node process."""
import threading


class SpinGuard:
    def __init__(self, target):
        self.errors = []
        def run():
            try:
                target()
            except BaseException as error:
                self.errors.append(repr(error))
        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def ensure_alive(self):
        if self.errors or not self.thread.is_alive():
            raise RuntimeError('ROS spin worker failed: '+(self.errors[0] if self.errors else 'unexpected exit'))

    def join(self, timeout=2):
        self.thread.join(timeout=timeout)
