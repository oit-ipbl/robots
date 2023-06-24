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
