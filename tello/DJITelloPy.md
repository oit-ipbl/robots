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
tello.end()
```

## Up,Down
```python
tello = Tello()

tello.connect()
tello.takeoff()

tello.move_up(100)
tello.move_down(100)

tello.land()
tello.end()
```


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
tello.end()
```
## move
move( up, 100)

```python
tello = Tello()

tello.connect()
tello.takeoff()

tello.move( up, 100)
tello.move( left, 100)
tello.move( right, 100)
tello.move( forward, 100)
tello.move( back, 100)
tello.land()
tello.end()
```
