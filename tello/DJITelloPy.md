# DJITelloPy
This library makes it simpler to control Tello.
## Install using pip
If you have not yet installed this library, perform the following commands.

```bash
pip install djitellopy
```
## Take-off and Landing
### tello_sample01.py
The following codes are for takeoffs and landings in Tello.
```python
from djitellopy import Tello

tello = Tello()

tello.connect()
tello.takeoff()
tello.land()
tello.end()
```

## Up,Down
### tello_sample02.py
The following code is for vertical movement in Tello.
#### move_up(self, x),move_down(self, x)
Fly x cm up or down.
```python
from djitellopy import Tello

tello = Tello()

tello.connect()
tello.takeoff()

tello.move_up(100)
tello.move_down(100)

tello.land()
tello.end()
```


##  Forward, Back, Left or Right
### tello_sample03.py
The following codes are for forward/backward/left/right movement in Tello.
#### move_forward(self, x),move_back(self, x),move_left(self, x),move_right(self, x)
Fly x cm forward, back, left or right.

```python
from djitellopy import Tello

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
## Move
### tello_sample04.py
You can also control tello by using the Move method.
#### move(self, direction, x)
Tello fly up, down, left, right, forward or back with distance x cm. 
```python
from djitellopy import Tello

tello = Tello()

tello.connect()
tello.takeoff()

tello.move( "up", 100)
tello.move( "left", 100)
tello.move( "right", 100)
tello.move( "forward", 100)
tello.move( "back", 100)
tello.land()
tello.end()
```
## Flip
###  tello_sample05.py
You can also control tello by using the Flip method.
#### flip(self, direction)　or flip_x (self)
Do a flip maneuver. Users would normally call one of the flip_x functions instead.
```python
from djitellopy import Tello

tello = Tello()

tello.connect()
tello.takeoff()

tello.flip_back()
tello.flip_forward()
tello.flip_forward()
tello.flip_left()
tello.flip_right()
tello.flip("f")
tello.land()
tello.end()
```
## Rotate
### tello_sample06.py
```python
from djitellopy import Tello
import time

tello = Tello()

tello.connect()
tello.takeoff()

tello.rotate_clockwise( 180)
tello.rotate_clockwise(-180)

tello.rotate_counter_clockwise(180)
tello.rotate_counter_clockwise(-180)

tello.land()
tello.end()
```
## Getter
### tello_sample07.py
```python
from djitellopy import Tello
import time

tello = Tello()

tello.connect()
tello.takeoff()

print(f"Battery: {tello.get_battery()}%")
print(f"flight_time:{tello.get_flight_time()}s")
print(f"height:{tello.get_height()}cm")
print(f"temperature:{tello.get_highest_temperature()}")
print(f"Tof:{tello.get_distance_tof()}cm")

tello.land()
tello.end()
```
