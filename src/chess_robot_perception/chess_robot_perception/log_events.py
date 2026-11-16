"""ROS diagnostics for repeated failures; never selects or filters business data."""
import json
import time


class RepeatedFault:
    def __init__(self, logger, event, interval_s=3.):
        self.logger = logger
        self.event = event
        self.interval_s = interval_s
        self.key = None
        self.count = 0

    def _summary(self, stage, level):
        details = dict(stage=stage, reason=self.key[0], object=self.key[1],
                       impact=self.key[2], count=self.count,
                       first_unix_s=self.first, last_unix_s=self.last)
        getattr(self.logger, level)(
            self.event+' '+json.dumps(details, ensure_ascii=False, separators=(',', ':')))

    def failure(self, reason, object, impact):
        key = (reason, object, impact)
        now = time.monotonic()
        if self.key != key:
            if self.key is not None:
                self._summary('changed', 'error')
            self.key = key
            self.count = 0
            self.first = time.time()
        self.count += 1
        self.last = time.time()
        if self.count == 1 or now-self.reported >= self.interval_s:
            self._summary('first' if self.count == 1 else 'ongoing', 'error')
            self.reported = now

    def recover(self):
        if self.key is not None:
            self._summary('recovered', 'info')
            self.key = None
            self.count = 0
