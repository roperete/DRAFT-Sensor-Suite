# DRAFT-Sensor-Suite

## DRAFT Air Quality Sensors

The DRAFT Experiment will read the air quality across an air filter.
This experiment has been designed by The Spring Institute for Forest on the Moon.

Engineering: Jorge Galván Lobo \
Science: Patrick Grove\
Electronics: Álvaro Ropero López

## Technical Specs
1x Raspberry PI 3 (Sensor Computer)\
2x Arduino Nano (Sensor controller)\
2x SGP30 Gas Sensor\
2x MQ135 Gas Sensor\
2x MQ3 Gas Sensor\
2x MQ7 Gas Sensor

A Sensor Suite is composed of one of each sensor, connected to one Arduino Nano controller. \
One Sensor Suite will be place at each end of the Air filter, to measure the difference in Air Quality after filtering.

## Installation
The MQ sensors are operated in Arduino using the library MQUnifiedSensor\ https://www.arduino.cc/reference/en/libraries/mqunifiedsensor/ 

The SGP30 sensor is operated in Arduino using the library SparkFun_SGP30\ https://github.com/sparkfun/SparkFun_SGP30_Arduino_Library

## Usage

- Install Arduino IDE.
- Open the _Arduino_SensorSuite.ino_ file.
```bash
arduino
```
- Upload the file into both Arduino Nano, on its respective port.
- Calibrate the MQ Sensors (see below).
>
- Execute _SensorReading_  python script on Raspberry Pi.
- It will appear a file named _sensor_data.csv_ which contains the data logged from the sensors.
- Once the data is collected, the python script _Plot_sensor_data_ can be used to visualize the data.

### Calibration

#### MQ sensors 
MQ Sensors require calibration. 
Before starting any work, you should power your sensor and let it burn for at least 24 hours. This step is very important because MQ sensors come polluted, and burning helps them to clean themselves. You have to power the sensors with 5V and put them In a clean place.


Since the heater of the sensor draws a lot of current and the sensor needs 5V to work properly, it is better if you use an external power supply that assures you a voltage input of 5V with enough current for the heater (The Arduino provided a little less than 4.6V when tested). The Arduino Mini Pro has an internal 5V regulator, so you can connect an external power supply (no more than 12V) to the Raw pin. Alternatively, this tutorial shows you how to setup a 5V regulated power supply:
https://www.jaycon.com/understanding-a-gas-sensor/

For Calibration, open _Arduino_SensorSuite.ino_ and follow the instructions
You can do so by placing your cursor in front of the second line in this snippet and pressing Ctrl+/
```arduino
//While calibrating Your sensor Uncomment this calibration portion and calibrate for R0.
  /*****************************  MQ CAlibration ********************************************
```
Then upload the file to the Arduino Nano controller and open the Serial Monitor (Ctrl+Shift+M) and wait until calibration is complete.
Repeat this process for both Sensor Suites.



#### SGP30 Sensor:
The first 10-20 readings will always be eCO2 400 ppm  TVOC 0 ppb . That's because the sensor is warming up, so it will have 'null' readings.
The SensorReading script will omit the first 20 lines of readings for this reason.

## License

The Spring Institute for Forest on the Moon
