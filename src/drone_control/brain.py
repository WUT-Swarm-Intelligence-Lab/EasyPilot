from typing import Callable
from drone_control.nerves import Nerves

class SearchArucoBrain(Nerves):
    def __init__(self) -> None:
        self.events = {}
        super().__init__()

    def add_spike_event(self, name: str, callback : Callable) -> None:
        self.events[name] = callback

    def do(self, msg, id) -> None:
        if msg is not None:
            for name, callback in self.events.items():
                if callback is not None:
                    if msg["msg"] == name:
                        callback(msg, id=id)