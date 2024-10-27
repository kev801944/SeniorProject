#include <OneWire.h>
#include <DallasTemperature.h>

#define TdsSensorPin A0
#define VREF 5.0              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point
#define ONE_WIRE_BUS 4
#define motorPin1 6
#define motorPin2 7

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature;       // current temperature for compensation

void setup(void) {
  // Start serial communication for debugging purposes
  Serial.begin(115200);
  // Start up the library
  tempSensor.begin();
  pinMode(TdsSensorPin,INPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
}

void loop() {

  tempSensor.requestTemperatures();
  temperature = tempSensor.getTempCByIndex(0);

  static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U){     //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT){ 
      analogBufferIndex = 0;
    }
  }   
  
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 800U){
    printTimepoint = millis();
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0;
      
      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
      float compensationCoefficient = 1.0+0.02*(temperature-25.0);
      //temperature compensation
      float compensationVoltage = averageVoltage/compensationCoefficient;
      
      //convert voltage value to tds value
      tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
      
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");

      if (temperature <= 30.0) {
        digitalWrite(motorPin1,LOW);
        digitalWrite(motorPin2,LOW);
      } else {
        digitalWrite(motorPin1,HIGH);
        digitalWrite(motorPin2,LOW);
      }
      
      Serial.print("Temperature in Celsius: ");
      Serial.print(temperature);
      Serial.print(" ");

      Serial.print("TDS Value: ");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
    }
  }
}

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

// void setup(){
//   Serial.begin(115200);
//   pinMode(TdsSensorPin,INPUT);
// }

// #include <OneWire.h>
// #include <DallasTemperature.h>

// // Data wire is conntec to the Arduino digital pin 4
// #define ONE_WIRE_BUS 4

// // Setup a oneWire instance to communicate with any OneWire devices
// OneWire oneWire(ONE_WIRE_BUS);

// // Pass our oneWire reference to Dallas Temperature sensor 
// DallasTemperature sensors(&oneWire);

// void setup(void)
// {
//   // Start serial communication for debugging purposes
//   Serial.begin(9600);
//   // Start up the library
//   sensors.begin();
// }

// void loop(void){ 
//   // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
//   sensors.requestTemperatures();

//   float temperature = sensors.getTempCByIndex(0);

//   Serial.print("Celsius temperature: ");
//   // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
//   Serial.print(temperature); 
//   delay(1000);
// }

// Original source code: https://wiki.keyestudio.com/KS0429_keyestudio_TDS_Meter_V1.0#Test_Code
// Project details: https://RandomNerdTutorials.com/arduino-tds-water-quality-sensor/

// #define TdsSensorPin A0
// #define VREF 5.0              // analog reference voltage(Volt) of the ADC
// #define SCOUNT  30            // sum of sample point

// int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
// int analogBufferTemp[SCOUNT];
// int analogBufferIndex = 0;
// int copyIndex = 0;

// float averageVoltage = 0;
// float tdsValue = 0;
// float temperature = 16;       // current temperature for compensation

// // median filtering algorithm
// int getMedianNum(int bArray[], int iFilterLen){
//   int bTab[iFilterLen];
//   for (byte i = 0; i<iFilterLen; i++)
//   bTab[i] = bArray[i];
//   int i, j, bTemp;
//   for (j = 0; j < iFilterLen - 1; j++) {
//     for (i = 0; i < iFilterLen - j - 1; i++) {
//       if (bTab[i] > bTab[i + 1]) {
//         bTemp = bTab[i];
//         bTab[i] = bTab[i + 1];
//         bTab[i + 1] = bTemp;
//       }
//     }
//   }
//   if ((iFilterLen & 1) > 0){
//     bTemp = bTab[(iFilterLen - 1) / 2];
//   }
//   else {
//     bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
//   }
//   return bTemp;
// }

// void setup(){
//   Serial.begin(115200);
//   pinMode(TdsSensorPin,INPUT);
// }

// void loop(){
//   static unsigned long analogSampleTimepoint = millis();
//   if(millis()-analogSampleTimepoint > 40U){     //every 40 milliseconds,read the analog value from the ADC
//     analogSampleTimepoint = millis();
//     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
//     analogBufferIndex++;
//     if(analogBufferIndex == SCOUNT){ 
//       analogBufferIndex = 0;
//     }
//   }   
  
//   static unsigned long printTimepoint = millis();
//   if(millis()-printTimepoint > 800U){
//     printTimepoint = millis();
//     for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
//       analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
//       // read the analog value more stable by the median filtering algorithm, and convert to voltage value
//       averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0;
      
//       //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
//       float compensationCoefficient = 1.0+0.02*(temperature-25.0);
//       //temperature compensation
//       float compensationVoltage=averageVoltage/compensationCoefficient;
      
//       //convert voltage value to tds value
//       tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
      
//       //Serial.print("voltage:");
//       //Serial.print(averageVoltage,2);
//       //Serial.print("V   ");
//       Serial.print("TDS Value:");
//       Serial.print(tdsValue,0);
//       Serial.println("ppm");
//     }
//   }
// }




