# control_module

Robot Control Module under construction. The code was initially on beaglebone
black and now runs on raspberry pi 2 with the following hardware:

* atmega24u4 encoder reader on i2c
* motor controller from Adafruit on i2c
* 10 DOH sensor from Adafruit on i2c
* raspberry pi 2 as master computer

The code is currently in POC/testing state. Incorporating a stereo
camera module to be used in mapping/localization.


## I2C Addresses

```
i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- 19 -- -- -- -- 1e -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: 60 -- -- -- -- -- -- -- -- -- -- 6b -- -- -- -- 
70: 70 -- -- -- -- -- -- 77                         
```

where 
```
	19 -> Accelator (LSM303)
	1e -> Magnetometer (LSM303)
	60 -> Motor Controller
	6b -> Gyro (L3GD20)
	70 -> RPI on board MUX
	77 -> Barometer (BMP085)
```

## TODO for short term

* Move poller to the I/O thread and add a listener socket for vision module

* Read battery voltage levels from AtMega and safety shutdown the robot

* Remove any beaglebone black related code.

* Tests for components

## References / Credits

* Madgwick's implementation of Mayhony's AHRS algorithm
* Adafruit for original Gyro/Accelator/Magnetometer/Barometer/Motor controller reading code
* Arduino-IDE sources for timer code
