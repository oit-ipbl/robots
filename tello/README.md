# Tello
The Tello is a drone manufactured by the Chinese company DJI. It can be controlled using a smartphone or laptop. This PBL (Project-Based Learning) is programmed to be controlled automatically using Python.
https://www.ryzerobotics.com/tello

## SSID



## Firmware Update
To run Tello with python3, the formware needs to be updated, which can be done from an app on your phone.

[Google plya](https://play.google.com/store/apps/details?id=com.ryzerobotics.tello&hl=ja-JP)

[App Store](https://apps.apple.com/jp/app/tello/id1330559633)

## Tello SDK
The Tello has an SDK (Software Development Kit) that allows users to control it.




### SDK 2.0
You can find the user guide for SDK 2.0 at the following link:

https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf

## Important factor

### Sending Commands and Receiving Responses
Tello IP: 192.168.10.1 UDP PORT: 8889 << - - >> PC 

### Receive Tello State
Tello IP: 192.168.10.1 - >> PC / Mac / Mobile UDP Server: 0.0.0.0 UDP PORT:8890

### Receiving Tello Video Streams
Tello IP: 192.168.10.1 - >> PC / Mac / Mobile UDP Server: 0.0.0.0 UDP PORT:11111

Now let's actually use these SDKs to control tello!
[Simple Control](https://github.com/oit-ipbl/robots/blob/main/tello/simple%20control.md)
