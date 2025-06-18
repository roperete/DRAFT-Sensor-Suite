//#include <Wire.h>
// #include <SPI.h>
// #include <Adafruit_BMP280.h> // https://github.com/adafruit/Adafruit_BMP280_Library
// #include <DFRobot_LWLP.h> // https://github.com/DFRobot/DFRobot_LWLP
// #include "SparkFun_SGP30_Arduino_Library.h" // https://github.com/sparkfun/SparkFun_SGP30_Arduino_Library
#include "PMS.h" // https://github.com/fu-hsi/PMS/tree/master
//#include <AltSoftSerial.h>
#include <SoftwareSerial.h>


// #define placa "Arduino UNO"
// #define Voltage_Resolution 5
// #define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO

// #define MQ4R0 96  // in k ohms
// #define MQ4RL 20 // in k ohms
// #define MQ4A 1012.7 // CH4
// #define MQ4B -2.786
// #define VpinMQ4 A3
// #define pinMQ4 A0 //Analog input 0 of your arduino
// #define typeMQ4 "MQ-4" //MQ135

// #define MQ135R0 24.45
// #define MQ135RL 1.98
// #define MQ135A 110.47 // CO2
// #define MQ135B -2.862
// #define VpinMQ135 A6
// #define pinMQ135 A1 //Analog input 0 of your arduino
// #define typeMQ135 "MQ-135" //MQ135

// // #define MQ7R0 0.14
// #define MQ7RL 1 
// #define MQ7R0 0.13 
// #define MQ7A 99.042 //CO
// #define MQ7B -1.518
// #define pinMQ7 A2 //Analog input 0 of your arduino
// #define typeMQ7 "MQ-7" //MQ135
// #define RatioMQ7CleanAir 27.5 //RS / R0 = 27.5 ppm 
// #define PWMPIN 10 // Pin connected to mosfet

// #define Average_sampling 10

//Declare Sensors
SoftwareSerial pmsSerial(8,9); // in this order Rx , Tx (D2 = Rx ,D3 = Tx)
//AltSoftSerial altSerial;
PMS pms(pmsSerial);

// Adafruit_BMP280 bmp; // I2C 0X77
// SGP30 mySensor; //create an object of the SGP30 class I2C 0X58
// DFRobot_LWLP lwlp; // 0x00


PMS::DATA datapms;

float readMQ(int analogPin, float RL, float R0, float a, float b, int voltagePin = 0);
float calculateR0(int analogPin, float RL, float a, float b, float ppm, int voltagePin = 0);
float calculateR0FromRatio(int analogPin, float RL, float ratioRS_R0);
float calculateR0FromRatio(int analogPin, float RL, float ratioRS_R0,int voltagePin = 0);
float getAirVelocity(float pressureDiff, float airDensity = 1.225);

void setup() {
  Serial.begin(9600); //Init serial port
  pmsSerial.begin(9600);
  delay(1000);

  // pinMode(PWMPIN, OUTPUT);
  //Wire.begin();

  // BMP SETUP
//   if (!bmp.begin()) {
//     Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
//     while (1);
//  }
  delay(1000);

  // PITOT SETUP
  // while (lwlp.begin() != 0) {
  //   Serial.println("Failed to initialize the chip, please confirm the chip connection");
  //   delay(1000);
  // }
  delay(1000);

  //SGP30 SETUP
  // mySensor.begin();
  // mySensor.initAirQuality();

  // delay(100);
  // if (pms.read(datapms))
  //   {
  //   Serial.print("PM 1.0 (ug/m3): ");
  //   Serial.println(datapms.PM_AE_UG_1_0);

  //   Serial.print("PM 2.5 (ug/m3): ");
  //   Serial.println(datapms.PM_AE_UG_2_5);

  //   Serial.print("PM 10.0 (ug/m3): ");
  //   Serial.println(datapms.PM_AE_UG_10_0);

  //   Serial.println();

  //   }
}

void loop() {
  // float R0=0;
  // // DFRobot_LWLP::sLwlp_t data;

  // Serial.println(F("Starting 90s heating cycle for the MQ7"));
  // // 90s heating cycle = 180 loops (90000ms / 500ms = 120)
  // for(int i = 0; i < 10 ; i++) {
  //   //Serial.print(F("Heating loop: ")); Serial.println(i);
  //   // analogWrite(PWMPIN, 255); // 5V heating
  //   delay(500);
  // }

  // Serial.println(F("SWITCH TO SENSING"));
  // Serial.println(F("SWITCH TO SENSING -- MQ7"));
  // Serial.println(F("SWITCH TO SENSING -- 90s"));
  // // analogWrite(PWMPIN, 71);
  // for(int i = 0; i < 180; i++) {

    
    // Serial.print(F(" Sensing loop: ")); Serial.println(i);
    //  // 1.4V sensing for the MQ7

    // Serial.print(F(" MQ7 READING : "));
    // Serial.print(readMQ(pinMQ7,MQ7RL,MQ7R0,MQ7A,MQ7B));
    // Serial.print(F(" ppm"));

    // Serial.print(F("\n MQ4 READING : "));
    // Serial.print(readMQ(pinMQ4,MQ4RL,MQ4R0,MQ4A,MQ4B,VpinMQ4));
    // Serial.print(F(" ppm"));

    // Serial.print(F("\n MQ135 READING : "));
    // Serial.print(readMQ(pinMQ135,MQ135RL,MQ135R0,MQ135A,MQ135B,VpinMQ135));
    // Serial.print(F(" ppm"));
    //R0 = calculateR0(pinMQ135,MQ135RL,MQ135A,MQ135B,850);

    // Serial.print(F("\n SGP30 READING :"));
    // mySensor.measureAirQuality();
    // Serial.print(F("CO2: "));
    // Serial.print(mySensor.CO2);
    // Serial.print(F(" ppm\tTVOC: "));
    // Serial.print(mySensor.TVOC);
    // Serial.println(F(" ppb"));

    // Serial.print(F(" BMP280 READING :"));
    // Serial.print(F(" Pressure = "));
    // Serial.print(bmp.readPressure());
    // Serial.println(F(" Pa"));

    // Serial.print(F(" PITOT READING :"));
    // data = lwlp.getData();
    // Serial.print(F(" Air speed : "));
    // Serial.print(getAirVelocity(data.presure));
    // Serial.println(F(" m/s"));
    // delay(100); 

    if (pms.read(datapms))
    {
    unsigned long startTime = millis();
    
    Serial.print("PM 1.0 (ug/m3): ");
    Serial.println(datapms.PM_AE_UG_1_0);

    Serial.print("PM 2.5 (ug/m3): ");
    Serial.println(datapms.PM_AE_UG_2_5);

    Serial.print("PM 10.0 (ug/m3): ");
    Serial.println(datapms.PM_AE_UG_10_0);

    Serial.println();
    unsigned long endTime = millis();
    unsigned long duration = endTime - startTime;
    Serial.print(" Loop duration: ");
    Serial.print(duration);
    Serial.println(" ms\n");
    
    unsigned long startTime = millis();
    }


 
    // delay(max(500-duration, 0));

  // }


}

// this functions reads the analog input of the mq and the voltage input of the sensor if desired to output the ppm concentration of the gaz chosen using the a and b variables

// float readMQ(int analogPin, float RL, float R0, float a, float b, int voltagePin = 0) {

//   int rawADC = analogRead(analogPin);
//   if (rawADC == 0) return -1; // éviter division par zéro

//   // Read the input voltage of the sensor to make the correction
//   float supplyVoltage = 5.0; //default voltage
//   if (voltagePin != 0) {
//     int rawVoltageADC = analogRead(voltagePin);
//     if (rawVoltageADC == 0) return -1; // division by zero ?
//     supplyVoltage = rawVoltageADC * (5.0 / 1023.0);
//   }

//   float voltage = rawADC * (supplyVoltage / 1023.0);

//   float RS = (supplyVoltage - voltage) * RL / voltage;

//   float ratio = RS / R0;

//   float ppm = a * pow(ratio, b);

//   return ppm;
// }

// // this code is used to calibrate the MQ135 using the known CO2 ppm
// float calculateR0(int analogPin, float RL, float a, float b, float ppm, int voltagePin = 0) {
//   const int samples = 5;
//   float totalRS = 0;

//   for (int i = 0; i < samples; i++) {
//     int rawADC = analogRead(analogPin);
//     if (rawADC == 0) continue; // skip invalid reading

//     float supplyVoltage = 5.0;
//     if (voltagePin != 0) {
//       int rawVoltageADC = analogRead(voltagePin);
//       if (rawVoltageADC == 0) return -1; 
//       supplyVoltage = rawVoltageADC * (5.0 / 1023.0);
//     }

//     float voltage = rawADC * (supplyVoltage / 1023.0);
//     float RS = (supplyVoltage - voltage) * RL / voltage;
//     totalRS += RS;

//     delay(30); // short delay between readings
//   }

//   float avgRS = totalRS / samples;
//   float ratio = pow(ppm / a, 1.0 / b); // Invert ppm = a * (RS/R0)^b
//   float R0 = avgRS / ratio;
//   Serial.print(F("\n Calculated R0 is : "));
//   Serial.print(R0,6);
//   return R0;
// }

// // this is to calibrate the MQ4 and MQ7 sensors using the clean air RS/R0 ratio
// float calculateR0FromRatio(int analogPin, float RL, float ratioRS_R0,int voltagePin = 0) { 
//   float RS_sum = 0;

//   for (int i = 0; i < 5; i++) {
//     int raw = analogRead(analogPin);
//     if (raw == 0) return -1; // avoid divide by zero

//     float supplyVoltage = 5.0; 
//     if (voltagePin != 0) {
//       int rawVoltageADC = analogRead(voltagePin);
//       if (rawVoltageADC == 0) return -1; 
//       supplyVoltage = rawVoltageADC * (5.0 / 1023.0);
//     }

//     float voltage = raw * (supplyVoltage / 1023.0);
//     float RS = (supplyVoltage - voltage) * RL / voltage;
//     RS_sum += RS;
//     delay(20); // small delay between readings
//   }

//   float RS_avg = RS_sum / 5.0;
//   float R0 = RS_avg / ratioRS_R0;
//   return R0;
// }

// float getAirVelocity(float pressureDiff, float airDensity = 1.225) {

//   pressureDiff = abs(pressureDiff); 
//   float velocity = sqrt((2 * pressureDiff) / airDensity);
//   return velocity; // in meters per second

// }
