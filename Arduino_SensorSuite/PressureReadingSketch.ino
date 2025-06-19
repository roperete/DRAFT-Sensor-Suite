/*!
 * @file readData.ino
 * @brief read the pressure difference
 * @details Read pressure difference and temperature value of the differential pressure sensor 
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2020-05-14
 * @url https://github.com/DFRobot/DFRobot_LWLP
 */
#include <DFRobot_LWLP.h>
float calculateAirSpeed(float deltaPressure,float T);
/*! 
 * @brief Construct the function
 * @param pWire IC bus pointer object and construction device, can both pass or not pass parameters, Wire in default.
 * @param address Chip IIC address, addresses 0x0 
 */
DFRobot_LWLP lwlp;
float Calibration;
DFRobot_LWLP::sLwlp_t data;
void setup() {

  Serial.begin(9600);
  //Init Chip
  while (lwlp.begin() != 0) {
  Serial.println("Failed to initialize the chip, please confirm the chip connection");
  delay(1000);
  }
  // Calibration
  
  for(int i=0;i<100;i++){
    data = lwlp.getData();
    delay(20);
    Calibration+=data.presure;
  }
  Calibration=Calibration/100;

}
void loop(void){
  
  //Get data of single measurement 
  data = lwlp.getData();
  //Get data processed by the filter 
  //data = lwlp.getfilterData();
  //Get temperature in unit centigrade degree
  Serial.print("Temperature: ");
  Serial.print(data.temperature);
  Serial.println(" C");
  Serial.print("Flow Rate : ");
  //Get pressure difference in unit pa 
  float FlowRate=calculateAirSpeed(data.presure-Calibration,data.temperature);
  Serial.print(FlowRate,5);
  Serial.println(" m3/s");
  Serial.print("Air Speed : ");
  float AirSpeed=FlowRate/(3.1416*pow(((100e-3)/2),2));
  Serial.print(AirSpeed,5);
  Serial.println(" m/s");
  Serial.print(data.presure);Serial.println(" Pa");
  delay(500);

}

float calculateAirSpeed(float deltaPressure, float T) {
  float D1=100.5e-3;
  float D2=75e-3;

  const float airDensity = 95900 / ( (T+273.15) * 287.05); // kg/m³ (masse volumique de l'air à 15°C et 1 atm)
  float dp = abs(deltaPressure); // on s'assure que la pression soit positive

  // Formule de Bernoulli : v = sqrt(2 * dp / ρ)
  //float airSpeed = sqrt(2.0 * dp / airDensity); 

  float airSpeed = sqrt((pow(3.1416,2)*abs(deltaPressure))/(8 * airDensity*(1/pow(D2,4)-1/pow(D1,4))));
  
  return airSpeed; // en m/s

}
