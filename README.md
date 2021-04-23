# miniDAQ
miniDAQ is a race data acquisition device based on a teensy 3.2 / 3.6

- **Kalman Filter** is used to fuse GPS and IMU data to achieve acurate localization.
to learn more about [Kalman filter](http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies).<br>
- Got to [How kalman filter Works](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/).

<img alt="Kalman" width=350 src="https://www.bzarg.com/wp-content/uploads/2015/08/kalflow.png" /></a>
<img alt="Kalman" width=575 src="https://github.com/SasaKuruppuarachchi/miniDAQ/blob/main/misc/Kalman.png" />

- Recorded data currently saves into a kml txt file whick can be imported to [Google Earth](https://www.google.com/earth/download/gep/agree.html?hl=en-GB).
- Files are written into the onboard SD card driver


### Component list
- [Teensy 3.2, 3.6](https://www.sparkfun.com/products/14057)
- [AbsOrient IMU](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview)
- [Adafruit Ultimate GPS featherwing](https://learn.adafruit.com/adafruit-ultimate-gps-featherwing)
- [Monochrome 128x64 OLED Display Module](https://learn.adafruit.com/1-5-and-2-4-monochrome-128x64-oled-display-module/overview)<br><br>
Optional
* [Dual CAN-Bus adapter for Teensy 3.5, 3.6](https://www.tindie.com/products/Fusion/dual-can-bus-adapter-for-teensy-35-36/)
* [XBee3](https://www.sparkfun.com/products/15126)

 Teensy microcontroller is picked due to the processing power and the capability to run threads.

### High level Scematic (Does not include the display connections)
![alt text](https://github.com/SasaKuruppuarachchi/miniDAQ/blob/main/misc/Scematic.png "High level Scematic")
