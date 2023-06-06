# Tello
The Tello is a drone made by the Chinese company DJI. It can be controlled by a smartphone or laptop. This PBL is controlled automatically using python.

https://www.ryzerobotics.com/tello

## Tello SDK
The tello has an SDK, which can be used to control it.

SDK 1.3

https://terra-1-g.djicdn.com/2d4dce68897a46b19fc717f3576b7c6a/Tello%20%E7%BC%96%E7%A8%8B%E7%9B%B8%E5%85%B3/For%20Tello/Tello%20SDK%20Documentation%20EN_1.3_1122.pdf

SDK 2.0

https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf

## Important factor

### Sending Commands and Receiving Responses
Tello IP: 192.168.10.1 UDP PORT: 8889 << - - >> PC 

### Receive Tello State
Tello IP: 192.168.10.1 - >> PC / Mac / Mobile UDP Server: 0.0.0.0 UDP PORT:8890

### Receiving Tello Video Streams
Tello IP: 192.168.10.1 - >> PC / Mac / Mobile UDP Server: 0.0.0.0 UDP PORT:11111
