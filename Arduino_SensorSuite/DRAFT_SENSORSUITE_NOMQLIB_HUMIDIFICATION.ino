#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h> // https://github.com/adafruit/Adafruit_BMP280_Library https://github.com/adafruit/Adafruit_Sensor
#include <DFRobot_LWLP.h> // https://github.com/DFRobot/DFRobot_LWLP
#include "SparkFun_SGP30_Arduino_Library.h" // https://github.com/sparkfun/SparkFun_SGP30_Arduino_Library
#include "PMS.h" // https://github.com/fu-hsi/pms
#include <SoftwareSerial.h>

// DD Set it to 1 for human readable serial write and to 0 to export CSV                
#define HUMAN_READABLE 0

#define placa "Arduino UNO"
#define Voltage_Resolution 5
#define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO

#define MQ4R0 2  // in k ohms
#define MQ4RL 20 // in k ohms
#define MQ4A 1012.7 // CH4
#define MQ4B -2.786
#define pinMQ4 A0 //Analog input 0 of your arduino
#define typeMQ4 "MQ-4" //MQ135

#define MQ135R0 14
#define MQ135RL 1.98
#define MQ135A 110.47 // CO2
#define MQ135B -2.862
#define pinMQ135 A1 //Analog input 0 of your arduino
#define typeMQ135 "MQ-135" //MQ135

// #define MQ7R0 0.14
#define MQ7RL 1 
#define MQ7R0 0.030796
#define MQ7A 99.042 //CO
#define MQ7B -1.518
#define pinMQ7 A2 //Analog input 0 of your arduino
#define typeMQ7 "MQ-7" //MQ135
#define RatioMQ7CleanAir 27.5 //RS / R0 = 27.5 ppm 
#define PWMPIN 10 // Pin connected to mosfet

#define Average_sampling 10

///////////////////// IRRIGATION PARAMETERS /////////////////////////
const float overShoot = 5.0;       // Humidity surplus (5% more)
const float hysteresis = 10;      // Reduced hysteresis
const float waterFlowRate = 0.01416;   // ml/second of water
const unsigned long propagationDelay = 1800; // Propagation delay (30min = 1800s)
const unsigned long measureInterval = 30;    // Measure every 10 loops (10s)
int lastActiveFilterIndex = -1; 
// Robust sensor detection
const float disconnectedRangeMin = 850  ;  // Min value for disconnected sensor
const float disconnectedRangeMax = 1025;  // Max value for disconnected sensor

// Simplified calibration per filter (2 points)
struct FilterCalibration {
  int rawValueDry;        // ~970 (auto calibration)
  int rawValueTarget;     // Your measurement (ex: 450)
  float targetMl;         // Your measurement (ex: 120ml)
};

// Simplified structure for each filter
struct FilterState {
  int humidityPin;
  float currentHumidity;
  float currentMl;
  unsigned long irrigationStartTime;
  bool humidifierRunning;
  bool waitingForPropagation;
  bool isActive;
};

// Specific calibrations per filter (adjust according to your measurements)
FilterCalibration filterCal[3] = {
  {880, 415, 580.0},  // Filter 1: dry=970, target=302 (580ml)
  {880, 338, 980.0},  // Filter 2: dry=970, target=353 (980ml) BIO REGOLITH
  {880, 740, 170.0}   // Filter 3: dry=970, target=790 (170ml)
};

// Filter configuration - SINGLE humidifier at pin 7
FilterState filters[3] = {
  {A7, 0, 0, 0, false, false, false},  // Filter 1
  {A6, 0, 0, 0, false, false, false},  // Filter 2
  {A3, 0, 0, 0, false, false, false}   // Filter 3
};

const int numFilters = 3;
unsigned long globalLoopCounter = 0;
int activeFilterIndex = -1; // Index of currently active filter (-1 = none)

//Declare Sensors
SoftwareSerial pmsSerial(8,9); // in this order Tx , Rx 
PMS pms(pmsSerial);

Adafruit_BMP280 bmp; // I2C 0X77
SGP30 mySensor; //create an object of the SGP30 class I2C 0X58
PMS::DATA datapms;

float Calibration;

//////////////// BEGIN UTILITY FUNCTIONS ///////////////////////////////////

// function to print a CSV message
void printCSV(String sensor, String gas, float value){
  Serial.println( sensor + "," + gas + "," + String(value));  
}

// this functions reads the analog input of the mq and the voltage input of the sensor if desired to output the ppm concentration of the gaz chosen using the a and b variables

float readMQ(int analogPin, float RL, float R0, float a, float b, int voltagePin = 0) {

  int rawADC = analogRead(analogPin);
  int rawVoltageADC = analogRead(voltagePin);
  if (rawADC == 0) return -1; // avoid division by zero

  // Read the input voltage of the sensor to make the correction
  float supplyVoltage = 5.0; //default voltage
  if (voltagePin != 0) {
    if (rawVoltageADC == 0) return -1; // division by zero ?
    supplyVoltage = rawVoltageADC * (5.0 / 1023.0);
  }


  float voltage = rawADC * (supplyVoltage / 1023.0);

  float RS = (supplyVoltage - voltage) * RL / voltage;

  float ratio = RS / R0;

  float ppm = a * pow(ratio, b);

  return ppm;
}

// this code is used to calibrate the MQ135 using the known CO2 ppm
float calculateR0(int analogPin, float RL, float a, float b, float ppm, int voltagePin = 0) {
  const int samples = 1;
  float totalRS = 0;

  for (int i = 0; i < samples; i++) {
    int rawADC = analogRead(analogPin);
    if (rawADC == 0) continue; // skip invalid reading
    int rawVoltageADC = analogRead(voltagePin);
    float supplyVoltage = 5.0;
    if (voltagePin != 0) {
      
      if (rawVoltageADC == 0) return -1; 
      supplyVoltage = rawVoltageADC * (5.0 / 1023.0);
    }
    Serial.print(supplyVoltage);
    Serial.print("V  ");
    supplyVoltage = 5.2;
    float voltage = rawADC * (supplyVoltage / 1023.0);
    float RS = (supplyVoltage - voltage) * RL / voltage;
    totalRS += RS;

    delay(30); // short delay between readings
  }

  float avgRS = totalRS / samples;
  float ratio = pow(ppm / a, 1.0 / b); // Invert ppm = a * (RS/R0)^b
  float R0 = avgRS / ratio;
#if HUMAN_READABLE == 1  
  Serial.print(F("\n Calculated R0 is : "));
  Serial.print(R0,6);
#endif  
  return R0;
}

// this is to calibrate the MQ4 and MQ7 sensors using the clean air RS/R0 ratio
float calculateR0FromRatio(int analogPin, float RL, float ratioRS_R0,int voltagePin = 0) { 
  float RS_sum = 0;

  for (int i = 0; i < 5; i++) {
    int raw = analogRead(analogPin);
    int rawVoltageADC = analogRead(voltagePin);
    if (raw == 0) return -1; // avoid divide by zero

    float supplyVoltage = 5.0; 
    if (voltagePin != 0) {

      if (rawVoltageADC == 0) return -1; 
      supplyVoltage = rawVoltageADC * (5.0 / 1023.0);
    }

    float voltage = raw * (supplyVoltage / 1023.0);
    float RS = (supplyVoltage - voltage) * RL / voltage;
    RS_sum += RS;
    delay(20); // small delay between readings
  }

  float RS_avg = RS_sum / 5.0;
  float R0 = RS_avg / ratioRS_R0;
  #if HUMAN_READABLE == 1  
  Serial.print(F("\n Calculated R0 is : "));
  Serial.print(R0,6);
  #endif  
  return R0;
}

//////////////// IRRIGATION FUNCTIONS ///////////////////////////////////

// Test if a sensor is physically connected
bool isSensorConnected(int rawValue) {
  return !(rawValue >= disconnectedRangeMin && rawValue <= disconnectedRangeMax);
}

// Convert sensor value → ml of water (linear interpolation)
float rawToMl(int rawValue, int filterIndex) {
  // If drier than dry calibration, then 0ml
  if (rawValue >= filterCal[filterIndex].rawValueDry) return 0;
  
  // If more humid than target, limit to target
  if (rawValue <= filterCal[filterIndex].rawValueTarget) return filterCal[filterIndex].targetMl;
  
  // Linear interpolation between dry and target
  return map(rawValue, filterCal[filterIndex].rawValueDry, filterCal[filterIndex].rawValueTarget, 0, filterCal[filterIndex].targetMl);
}

// Function to read MH sensor humidity
float readHumidity(int pin, int sensorIndex) {
  int rawValue = analogRead(pin);
  
  // Physical connection test
  if (!isSensorConnected(rawValue)) {
    return -1; // Sensor not connected
  }
  
  // Convert to ml then to percentage relative to target
  filters[sensorIndex].currentMl = rawToMl(rawValue, sensorIndex);
#if HUMAN_READABLE == 1    
  Serial.print("Reading : ");
  Serial.print(rawValue);

  Serial.print(" / Pin  : ");
  Serial.print(sensorIndex);
#endif
  float humidity = (filters[sensorIndex].currentMl / filterCal[sensorIndex].targetMl) * 100.0;
  
  return constrain(humidity, 0, 100);
}

// Active filter detection
int detectActiveFilter() {
  int activeFilter = -1;
  
  for (int i = 0; i < numFilters; i++) {
    float humidity = readHumidity(filters[i].humidityPin, i);
    
    if (humidity >= 0) { // Sensor connected
      if (activeFilter == -1) {
        activeFilter = i;
        filters[i].isActive = true;
        lastActiveFilterIndex = i; // Save last active filter
      } else {
        // Multiple active sensors - keep first detected
        filters[i].isActive = false;
      }
    } else {
      filters[i].isActive = false;
    }
  }
  
  return activeFilter;
}

// Irrigation algorithm for active filter only
void irrigationControl() {
  // Active filter detection
  int newActiveFilter = detectActiveFilter();
  
  if (newActiveFilter != activeFilterIndex) {
    // Active filter change
    if (activeFilterIndex >= 0) {
      // Stop irrigation of previous filter
      // Timestamp initialization
      filters[activeFilterIndex].humidifierRunning = false;
#if HUMAN_READABLE == 1      
      Serial.print(F("Stop irrigation - Filter change ("));
      Serial.print(activeFilterIndex + 1);
      Serial.print(F(" -> "));
      Serial.print(newActiveFilter >= 0 ? newActiveFilter + 1 : 0);
      Serial.println(F(")"));
#endif      
    }
    activeFilterIndex = newActiveFilter;
  }
  
  // === DEGRADED MODE - SAFETY IRRIGATION ===
  if (activeFilterIndex < 0) {
    digitalWrite(7, LOW);  
    return;    
#if HUMAN_READABLE == 1 
    Serial.print("FailSafe");
#endif
  } else {
    digitalWrite(7, HIGH);

    
  }
    
  
  FilterState &activeFilter = filters[activeFilterIndex];
  
  // Periodic humidity reading (every 10 loops = 10s)
  if (globalLoopCounter % measureInterval == 0) {
    activeFilter.currentHumidity = readHumidity(activeFilter.humidityPin, activeFilterIndex);
    
    if (activeFilter.currentHumidity < 0) {
      // Sensor disconnected during use
#if HUMAN_READABLE == 1      
      Serial.println(F("WARNING: Sensor disconnected during irrigation!"));
#endif      
      activeFilterIndex = -1;
      digitalWrite(7, HIGH); // Safety stop
      return;
    }
    
#if HUMAN_READABLE == 1    
    // Debug display
    Serial.print(F("Filter "));
    Serial.print(activeFilterIndex + 1);
    Serial.print(F(" - "));
    Serial.print(activeFilter.currentMl, 1);
    Serial.print(F("ml/"));
    Serial.print(filterCal[activeFilterIndex].targetMl, 1);
    Serial.print(F("ml ("));
    Serial.print(activeFilter.currentHumidity, 1);
    Serial.println(F("%)"));
#endif    
  }
  
  // If waiting for propagation, check if delay has elapsed
  if (activeFilter.waitingForPropagation) {
    unsigned long elapsedSeconds = (millis() - activeFilter.irrigationStartTime) / 1000;
    if (elapsedSeconds >= propagationDelay) {
      activeFilter.waitingForPropagation = false;
#if HUMAN_READABLE == 1      
      Serial.print(F("Filter "));
      Serial.print(activeFilterIndex + 1);
      Serial.println(F(" - End of propagation wait (30min)"));
#endif      
    } else {
      digitalWrite(7, LOW);
      return; // Continue waiting
    }
  }
  
  // Calculate deficit in ml
  float targetWithOvershoot = filterCal[activeFilterIndex].targetMl * (1.0 + overShoot / 100.0);
  float deficitMl = targetWithOvershoot - activeFilter.currentMl;
  
  // Irrigation decision
  if (!activeFilter.humidifierRunning && !activeFilter.waitingForPropagation) {
    // Start condition with hysteresis
    float startThreshold = filterCal[activeFilterIndex].targetMl * (1.0 - hysteresis / 100.0);
    
    if (activeFilter.currentMl < startThreshold) {
      // Start humidifier
      digitalWrite(7, LOW);
      activeFilter.humidifierRunning = true;
      activeFilter.irrigationStartTime = millis();
      
#if HUMAN_READABLE == 1      
      Serial.print(F("START irrigation Filter "));
      Serial.print(activeFilterIndex + 1);
      Serial.print(F(" - Deficit: "));
      Serial.print(deficitMl, 1);
      Serial.println(F(" ml"));
#endif      
    }
  }
  
  // Stop management by estimation or by sensor
  if (activeFilter.humidifierRunning) {
    unsigned long runTimeSeconds = (millis() - activeFilter.irrigationStartTime) / 1000;
    float waterAdded = runTimeSeconds * waterFlowRate;
    digitalWrite(7, LOW);
    bool stopByEstimation = waterAdded >= deficitMl;
    bool stopBySensor = activeFilter.currentMl >= targetWithOvershoot;
    
    if (stopByEstimation || stopBySensor) {
      // Stop humidifier
      digitalWrite(7, HIGH);
      activeFilter.humidifierRunning = false;
      activeFilter.waitingForPropagation = true;
      
#if HUMAN_READABLE == 1      
      Serial.print(F("STOP irrigation Filter "));
      Serial.print(activeFilterIndex + 1);
      Serial.print(F(" - Water added: "));
      Serial.print(waterAdded, 1);
      Serial.print(F(" ml | Reason: "));
      Serial.println(stopBySensor ? F("Sensor OK") : F("Estimation reached"));
      Serial.println(F("Waiting for propagation 30min..."));
#else 
      printCSV("Humidity","%",activeFilter.currentHumidity);
#endif      
    }
  }
}
//////////////// END UTILITY FUNCTIONS ///////////////////////////////////  

void setup() {
  Serial.begin(9600); //Init serial port
  pmsSerial.begin(9600);
  delay(1000);
  pms.passiveMode();
  delay(1000);
  pinMode(PWMPIN, OUTPUT);
  // Irrigation pin initialization (single humidifier)
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  
  Wire.begin();
  
  // BMP SETUP
  if (!bmp.begin()) {
#if HUMAN_READABLE == 1
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
# endif    
    
 }
  delay(1000);

  //SGP30 SETUP
  mySensor.begin();
  mySensor.initAirQuality();
  
#if HUMAN_READABLE == 1  
  Serial.println(F("=== Sensor + precise irrigation system ==="));
  Serial.println(F("Filter calibrations (dry → target) :"));
  for (int i = 0; i < numFilters; i++) {
    Serial.print(F("Filter "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(filterCal[i].rawValueDry);
    Serial.print(F(" → "));
    Serial.print(filterCal[i].rawValueTarget);
    Serial.print(F(" ("));
    Serial.print(filterCal[i].targetMl);
    Serial.println(F("ml)"));
  }
  Serial.println(F("Humidifier: Pin 7"));
  Serial.println(F("MH sensors: A7, A6, A3"));
  Serial.println(F("==========================================="));
#endif
}

///////////////////////////////// LOOP /////////////////////////

void loop() {
  float R0=0;
#if HUMAN_READABLE == 1  
  Serial.println(F("Starting 60s heating cycle for the MQ7"));
#endif  
  // 60s heating cycle = 60 loops

  for(int i = 0; i < 60 ; i++) {
#if HUMAN_READABLE == 1    
    Serial.print(F("Heating loop: ")); Serial.println(i);
#endif    
    analogWrite(PWMPIN, 255); // 5V heating
    
    // =============== IRRIGATION DURING HEATING ===============
    globalLoopCounter++;
    irrigationControl();
    
    delay(1000);
  }

#if HUMAN_READABLE == 1   
  Serial.println(F("SWITCH TO SENSING"));
  Serial.println(F("SWITCH TO SENSING -- MQ7"));
  Serial.println(F("SWITCH TO SENSING -- 90s"));
#endif  
  analogWrite(PWMPIN, 71);

  for(int i = 0; i < 180; i++) {
    unsigned long startTime = millis();    

// TODO DD: find what sensor & gas  to putr in CSV for MQ sensors

#if HUMAN_READABLE == 1  
    Serial.print(F(" Sensing loop: ")); Serial.println(i);
#endif    
     // 1.4V sensing for the MQ7
    float mq7 = readMQ(pinMQ7,MQ7RL,MQ7R0,MQ7A,MQ7B);

#if HUMAN_READABLE == 1 
    Serial.print(F(" MQ7 READING : "));
    Serial.print(mq7);
    Serial.print(F(" ppm"));
    R0 = calculateR0FromRatio(pinMQ7,MQ7RL,28);
#else
    printCSV("MQ7", "MQ7", mq7 );
#endif    

    float mq4 = readMQ(pinMQ4,MQ4RL,MQ4R0,MQ4A,MQ4B);  

#if HUMAN_READABLE == 1     
    Serial.print(F("\n MQ4 READING : "));
    Serial.print(mq4);
    Serial.print(F(" ppm"));
    Serial.print(calculateR0FromRatio(pinMQ4,MQ4RL,4.5));
    
#else
    printCSV("MQ4", "MQ4", mq4 );
#endif    

    float mq135 = readMQ(pinMQ135,MQ135RL,MQ135R0,MQ135A,MQ135B);

#if HUMAN_READABLE == 1   
    Serial.print(F("\n MQ135 READING : "));
    Serial.print(mq135);;
    Serial.print(F(" ppm"));
    Serial.print(F(" R0 : "));
    Serial.print(calculateR0(pinMQ135,MQ135RL,MQ135A,MQ135B,1416));
    //R0 = calculateR0(pinMQ135,MQ135RL,MQ135A,MQ135B,850);
#else
    printCSV("MQ135", "MQ135", mq135 );
#endif    

  mySensor.measureAirQuality();
#if HUMAN_READABLE == 1  
    Serial.print(F("\n SGP30 READING :"));  
    Serial.print(F("CO2: "));
    Serial.print(mySensor.CO2);
#else
    printCSV("SGP30", "CO2", mySensor.CO2);
#endif

#if HUMAN_READABLE == 1
    Serial.print(F(" ppm\tTVOC: "));
    Serial.print(mySensor.TVOC);
#else
    printCSV("SGP30", "TVOC", mySensor.TVOC);
#endif

#if HUMAN_READABLE == 1
    Serial.println(F(" ppb"));
    Serial.print(F(" BMP280 READING :"));
    Serial.print(F(" Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(F(" Pa"));
#else
    printCSV("BMP280", "Pressure", 0);
#endif    

#if HUMAN_READABLE == 1
    Serial.print(F(" Venturi :"));
    Serial.print(F(" Flow rate : "));
    Serial.print("NaN");
    Serial.println(F(" m/s"));
#else
  printCSV("Venturi", "FlowRate", 0);
  
#endif

    pms.requestRead();
    pms.readUntil(datapms);

#if HUMAN_READABLE == 1    
    Serial.print("PM 1.0 (ug/m3): ");
    Serial.println(datapms.PM_AE_UG_1_0);
    Serial.print("PM 2.5 (ug/m3): ");
    Serial.println(datapms.PM_AE_UG_2_5);
    Serial.print("PM 10.0 (ug/m3): ");
    Serial.println(datapms.PM_AE_UG_10_0);

#else
  printCSV( "PM", "PM1", datapms.PM_AE_UG_1_0);
  printCSV( "PM", "PM2.5", datapms.PM_AE_UG_2_5);
  printCSV( "PM", "PM10", datapms.PM_AE_UG_10_0);
  
#endif    

    // =============== IRRIGATION CONTROL ===============
    // Control active filter only
    globalLoopCounter++;
    irrigationControl();

    // Irrigation status display (every 60 loops = 60 seconds)
#if HUMAN_READABLE == 1    
    if (globalLoopCounter % 60 == 0) {
      Serial.println(F("=== IRRIGATION STATUS ==="));
      
      if (activeFilterIndex >= 0) {
        FilterState &activeFilter = filters[activeFilterIndex];
        
        Serial.print(F("ACTIVE Filter "));
        Serial.print(activeFilterIndex + 1);
        Serial.print(F(": "));
        
        if (activeFilter.humidifierRunning) {
          unsigned long runTime = (millis() - activeFilter.irrigationStartTime) / 1000;
          float waterAdded = runTime * waterFlowRate;
          Serial.print(F("IRRIGATION ("));
          Serial.print(waterAdded, 1);
          Serial.print(F("ml)"));
        } else if (activeFilter.waitingForPropagation) {
          unsigned long elapsed = (millis() - activeFilter.irrigationStartTime) / 1000;
          unsigned long remaining = propagationDelay - elapsed;
          Serial.print(F("WAITING ("));
          Serial.print(remaining / 60);
          Serial.print(F("min)"));
        } else {
          Serial.print(F("STANDBY"));
        }
        
        Serial.print(F(" - "));
        Serial.print(activeFilter.currentMl, 1);
        Serial.print(F("ml/"));
        Serial.print(filterCal[activeFilterIndex].targetMl, 1);
        Serial.print(F("ml ("));
        Serial.print(activeFilter.currentHumidity, 1);
        Serial.println(F("%)"));
      } else {
        Serial.println(F("No active filter"));
      }
      Serial.println(F("========================"));
    }
#endif

    unsigned long endTime = millis();
    unsigned long duration = endTime - startTime;
#if HUMAN_READABLE == 1     
    Serial.print(F(" Loop duration: "));
    Serial.print(duration);
    Serial.println(F(" ms\n"));
#endif    
    // No need for delay as PMS7003 already enforces 1s per loop

  }

}