#include <Arduino.h>
#include <Wire.h>
#include <heartRate.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>

#define MAX_BRIGHTNESS 255

//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data

MAX30105 particleSensor;

const int tempPin = A3; 
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
int32_t prevHR, prevO2;

bool once = true;

float readTemp();
void readHrSpO2();

void setup() {
  pinMode(tempPin, INPUT);

  Serial.begin(115200);   //initialize the serial communication
  Serial.println("Serial on");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  particleSensor.setup(); //Configure sensor with these settings    
}

void loop() {
  float temperature = 0;

  if(once){
    bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

    //read the first 100 samples, and determine the signal range
    for (byte i = 0 ; i < bufferLength ; i++){
     while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
    }
  
    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    once = false;
  }

  temperature = readTemp();
  readHrSpO2();

  if(heartRate < 200 && heartRate >= 0 && spo2 < 101 && spo2 >= 0){
    Serial.print(temperature, 2); //insert what needs to be written in here
    Serial.print(",");
    Serial.print(heartRate, DEC); 
    Serial.print(",");
    Serial.print(spo2, DEC);
    Serial.println(";");
    prevHR = heartRate;
    prevO2 = spo2;
  }

  else{
    Serial.print(temperature, 2); //insert what needs to be written in here
    Serial.print(",");
    Serial.print(prevHR, DEC); 
    Serial.print(",");
    Serial.print(prevO2, DEC);
    Serial.println(";");
  }

}

float readTemp(){
  float rawTemp; // Variable to store raw temperature
  float voltage;  // Variable to store voltage calculation
  float celsius;  // Variable to store Celsius value
  //Serial.println("readTemp");
  rawTemp = analogRead(tempPin);  // Read the raw 0-1023 value of temperature into a variable
  //Serial.println(rawTemp);
  voltage = rawTemp * (5 / 1023.0);
  //Serial.println(voltage);
  celsius = (voltage - 0.5) * 100;  // Calculate the celsius temperature, based on that voltage
  //Serial.println(celsius);

  return(celsius);
}

void readHrSpO2(){
  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 25; i < 100; i++){
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 75; i < 100; i++){
   while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    
    /*Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.print(irBuffer[i], DEC);

    Serial.print(F(", HR="));
    Serial.print(heartRate, DEC);

    Serial.print(F(", HRvalid="));
    Serial.print(validHeartRate, DEC);

    Serial.print(F(", SPO2="));
    Serial.print(spo2, DEC);

    Serial.print(F(", SPO2Valid="));
    Serial.println(validSPO2, DEC);*/
  }
  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}
