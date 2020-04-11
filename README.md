# esp-idf-GPS-Recorder
GPS NMEA Recorder for M5Stack.   
You can record & play NMEA messages like a voice recorder.   

---

# Hardware requirements
M5Stack.  
GPS module like NEO-6M.

---

# Software requirements
esp-idf ver4.1 or later.   
Because uart_enable_pattern_det_intr() has been changed to uart_enable_pattern_det_baud_intr().

---

# Install
```
git clone https://github.com/nopnop2002/esp-idf-GPS-Recorder
cd esp-idf-GPS-Recorder
make menuconfig
make install
```

---

# Configure
You can configure using menuconfig.

![config-1](https://user-images.githubusercontent.com/6020549/79033541-e488fa00-7be9-11ea-8550-f17f6dcfaaca.jpg)

![config-2](https://user-images.githubusercontent.com/6020549/79033544-e783ea80-7be9-11ea-9719-27720609e9a4.jpg)

---

# Recording   
NMEA messages are recorded in SPIFFS.   

## Wireing to GPS   
You can use GROVE port as UART RXD.   
SCL of GROVE(Yellow) is GPIO22.   
SDA of GROVE(White) is GPIO21.   
You can choice GPS RX GPIO using menuconfig.   

## Operation   
Displays NMEA messages and signal status.   
![View](https://user-images.githubusercontent.com/6020549/79033551-f66a9d00-7be9-11ea-8d75-149feb918670.JPG)

Press button A(Left button) to start recording.   
Press button A(Left button) again to stop the recording.   
![Recording](https://user-images.githubusercontent.com/6020549/79033552-f9658d80-7be9-11ea-8ba0-e7617401619b.JPG)


# Playback   
Outputs the NMEA message recorded in SPIFFS to UART TXD.   

## Wireing to TARGET   
You can use GROVE port as UART TXD.   
SCL of GROVE(Yellow) is GPIO22.   
SDA of GROVE(White) is GPIO21.   
You can choice PLAYBACK TX GPIO using menuconfig.   

## Operation   
Press button B(Center button) to start playback in repeat mode.   
Press button C(Right button) to stop the playback.   
![Playback-Repeat](https://user-images.githubusercontent.com/6020549/79033596-4f3a3580-7bea-11ea-84ad-9b37d2dbf7b8.JPG)


Press button B(Center button) for 2 seconds or more to start playback once.   
Press button C(Right button) to stop the playback.   
![Playback-Once](https://user-images.githubusercontent.com/6020549/79033601-5a8d6100-7bea-11ea-987d-875087a71c2d.JPG)



