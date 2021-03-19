"""Utilities."""

import threading


class StoppableThread(threading.Thread):
    """Stoppable Thread."""

    def __init__(self):
        """Initialize."""
        super().__init__()
        self._is_stopped = threading.Event()

    def stop(self):
        """Stop the thread."""
        self._is_stopped.set()

    def is_stopped(self):
        """Get running status."""
        return self._is_stopped.isSet()
