import queue

class Nerves:

    id = 0
    def __init__(self) -> None:
        self.id = Nerves.id
        Nerves.id += 1
        self.on_spike = {}
        self.qs = {}

    def spike(self, msg: dict) -> None:
        if self.on_spike is not None:
            for id, callback in self.on_spike.items():
                if callback is not None:
                    callback(msg, id=id)

    def connect(self, other_nerves: "Nerves") -> None:
        other_nerves.on_spike[other_nerves.id] = self.on_spike_cb
        self.qs[other_nerves.id] = queue.Queue()

    def on_spike_cb(self, msg, id) -> None:
        self.qs[id].put(msg)

    def listen(self) -> None:
        for id, q in self.qs.items():
            if not q.empty():
                msg = q.get()
                self.do(msg, id)

    def do(self, msg: dict, id: int) -> None:
        pass