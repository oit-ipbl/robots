# Simple Control

Learn how to simply control tello here.
The sdk is used to control tello!
## Take-off and Landing

```python
import socket
import time

#Create a UDP socket
socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tello_address = ('192.168.10.1' , 8889)

#command-mode : 'command'
print ('start')
socket.sendto('command'.encode('utf-8'),tello_address)
response, ip = socket.recvfrom(1024)
print('Response:', response.decode('utf-8'))
time.sleep(1)

print ('takeoff')
socket.sendto('takeoff'.encode('utf-8'),tello_address)
response, ip = socket.recvfrom(1024)
print('Response:', response.decode('utf-8'))
time.sleep(1)

print ('up')
socket.sendto('up 30'.encode('utf-8'),tello_address)
response, ip = socket.recvfrom(1024)
print('Response:', response.decode('utf-8'))
time.sleep(1)

print ('land')
socket.sendto('land'.encode('utf-8'),tello_address)
response, ip = socket.recvfrom(1024)
print('Response:', response.decode('utf-8'))
time.sleep(1)

socket.close()
```
However, there are simpler libraries available, which we will use in this PBL.

[DJITelloPy](./DJITelloPy.md)
