# DJITelloPy
This library makes it simpler to control Tello.
## Install using pip
If you have not yet installed this library, perform the following commands.

```bash
pip install djitellopy
```
## Take-off and Landing

```python
from djitellopy import Tello

tello = Tello()

tello.connect()
tello.takeoff()
tello.land()
```

## Left, Right, Forward or Back
```python
tello = Tello()

tello.connect()
tello.takeoff()

tello.move_up(100)
tello.move_down(100)

tello.land()



## Left, Right, Forward or Back
```python
tello = Tello()

tello.connect()
tello.takeoff()

tello.move_up(100)
tello.move_left(100)
tello.move_right(100)
tello.move_forward(100)
tello.move_back(100)
tello.land()

```
