import functools


def requires_flight_ready(func):
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self.is_ready():
            raise RuntimeError("Call remove_before_flight() before flying.")
        return func(self, *args, **kwargs)
    return wrapper


class FlightReady:
    global_ready: list["FlightReady"] = []

    def __init__(self):
        FlightReady.global_ready.append(self)
        self.ready = False

    def remove_before_flight(self) -> None:
        self.ready = True

    def wait_for_flight_ready(self) -> None:
        while not self.is_ready():
            pass

    def is_ready(self) -> bool:
        return bool(FlightReady.global_ready) and all(d.ready for d in FlightReady.global_ready)
