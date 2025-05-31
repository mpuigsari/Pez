# core/scheduler.py

import time
from threading import Thread, Event
from typing import Callable, Optional


class TransmissionStep:
    """
    A single step in a transmission sequence.
    - name: a short string identifier (for logging or debugging)
    - action: a callable to invoke when this step is reached
    - duration: if not None, seconds to wait after action
    - wait_for: if not None, a callable that returns True once this step is complete;
                the scheduler will busy‐poll wait_for() until it returns True, then move on.
    """

    def __init__(
        self,
        name: str,
        action: Optional[Callable[[], None]],
        *,
        duration: Optional[float] = None,
        wait_for: Optional[Callable[[], bool]] = None
    ):
        if duration is not None and wait_for is not None:
            raise ValueError("Cannot specify both duration and wait_for")
        self.name = name
        self.action = action
        self.duration = duration
        self.wait_for = wait_for

    def __repr__(self):
        return f"<TransmissionStep {self.name}>"


class TransmissionScheduler(Thread):
    """
    Runs a list of TransmissionSteps in sequence, optionally looping forever.

    Example:
        steps = [
            TransmissionStep('step1', lambda: print("Hello"), duration=1.0),
            TransmissionStep('step2', None, wait_for=lambda: some_flag),
            TransmissionStep('step3', lambda: print("Bye"), duration=0.5),
        ]
        sched = TransmissionScheduler(steps, loop=True)
        sched.start()
        # Sched will call step1.action(), sleep 1.0 s, then wait until some_flag==True,
        # then call step3.action(), sleep 0.5 s, then loop back to step1.
    """

    def __init__(self, steps, loop: bool = False):
        super().__init__(daemon=True)
        self.steps = steps
        self.loop = loop
        self._stop_event = Event()

    def run(self):
        while not self._stop_event.is_set():
            for step in self.steps:
                if self._stop_event.is_set():
                    return

                # 1) Execute action if provided
                if step.action:
                    step.action()

                # 2) If duration is specified, sleep that amount
                if step.duration is not None:
                    end_t = time.monotonic() + step.duration
                    while time.monotonic() < end_t:
                        if self._stop_event.is_set():
                            return
                        time.sleep(0.01)

                # 3) If wait_for is specified, busy‐wait until it returns True
                elif step.wait_for is not None:
                    while not step.wait_for():
                        if self._stop_event.is_set():
                            return
                        time.sleep(0.01)

                # 4) Otherwise, immediately move to next step

            if not self.loop:
                return

    def stop(self):
        """Signal the scheduler to cease after finishing the current step."""
        self._stop_event.set()
