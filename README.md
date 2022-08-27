# HATino_Improved
An improved version of [HATino](https://github.com/juanmcasillas/HATino), an Arduino head tracker made for FaceTrackNoIR and [OpenTrack](https://github.com/opentrack/opentrack). Based on [Hatire, by Furax49](https://sourceforge.net/projects/hatire/).  
Tested on OpenTrack with an Arduino Nano, but will work with every Arduino and on every Hatire-supporting head tracking program.

HATino utilizes the GY-521 MPU6050 accelerometer/gyro module, and supports a serial bluetooth module such as HC-05.

### Required Libraries:

- I2CDev: https://github.com/jrowberg/i2cdevlib.git
- Wire (Built into Arduino IDE)
- SoftwareSerial (Built into Arduino IDE)
