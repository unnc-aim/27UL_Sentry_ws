"""Stop first; resume only after continuously clear, healthy observations."""
import math


class ObstacleHold:
    def __init__(self):
        self.started = None
        self.clear_since = None

    def update(self, now, nearest, ready):
        if not math.isfinite(now) or not math.isfinite(nearest) or nearest < 0:
            raise ValueError('Invalid obstacle observation')
        if self.started is None:
            if nearest >= .4:
                return False
            self.started = now
        if now - self.started >= 10.:
            raise RuntimeError('Near obstacle did not clear within 10 s at zero velocity')
        if nearest >= .45 and ready:
            if self.clear_since is None:
                self.clear_since = now
            if now - self.clear_since >= 1.:
                self.started = self.clear_since = None
                return False
        else:
            self.clear_since = None
        return True
