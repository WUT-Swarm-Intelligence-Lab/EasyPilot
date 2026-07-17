# EasyPilot

## About Us

TBA

## How to

1. Clone the repository:

```bash
git clone https://github.com/WUT-Swarm-Intelligence-Lab/EasyPilot.git
cd EasyPilot
```

2. Install dependencies:

```bash
uv sync
```

3. Run the example:

```bash
uv run example.py
```

## Build your own

```python
import logging
import time

import cv2

from drone_control import Drone

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")


def main() -> None:
    def on_frame(frame) -> None:
        cv2.imshow("drone", frame)
        cv2.waitKey(1)

    drone = Drone(camera_ip="192.168.4.1")
    drone.find(0)

    # Start camera feed and wait for connection
    drone.camera_feed(on_frame)
    drone.camera_wait_until_ready()

    # Enable forward-flight mode (drone faces direction of travel)
    drone.set_forward_fly(True)

    drone.takeoff(height=0.5)

    for wp in [
        [1.5, 0.0, 0.5],
        [1.5, 1.5, 0.5],
        [0.0, 1.5, 0.5],
        [0.0, 0.0, 0.5],
    ]:
        drone.goto(wp)
        while drone.is_moving():
            time.sleep(0.05)
        print(f"  Reached {wp}")

    drone.land()
    drone.camera_stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
```

## Contributors

<a href="https://github.com/KlaNiedz"><img src="https://github.com/KlaNiedz.png" width="100" style="border-radius: 50%;" alt="KlaNiedz" /></a>
<a href="https://github.com/WUT-Swarm-Intelligence-Lab/EasyPilot/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=WUT-Swarm-Intelligence-Lab/EasyPilot" />
</a>

The code building that repository was originaly designed by: [KlaNiedz](https://github.com/KlaNiedz)

<a href="https://github.com/KlaNiedz"><img src="https://github.com/KlaNiedz.png?size=100" width="100" style="border-radius: 50%;" alt="KlaNiedz" /></a>


## StarTracker

[![Star History Chart](https://api.star-history.com/svg?repos=WUT-Swarm-Intelligence-Lab/EasyPilot&type=Date)](https://star-history.com/#WUT-Swarm-Intelligence-Lab/EasyPilot&Date)
