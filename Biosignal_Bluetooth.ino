
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "algorithm_by_RF.h"
#include "max30102.h"

const int tempPin = A3; 
const int oxiPin = 10; //pin connected to MAX30102 INT (interrupt pin)
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;     
uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data
float old_n_spo2;  // Previous SPO2 value
uint8_t uch_dummy;

float readTemp();
void readHrSpO2(float* heartRate, float* oxygenSat);

void setup() {
  pinMode(tempPin, INPUT);
  pinMode(oxiPin, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102

  Wire.begin();
  Serial.begin(115200);   //initialize the serial communication
  Serial.print("Serial on");
  maxim_max30102_reset(); //resets the MAX30102
  delay(1000);
  Serial.print("Maxim reset");
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  maxim_max30102_init();  //initialize the MAX30102
  Serial.print("Maxim initialized");
  old_n_spo2=0.0;

  previousMillis = millis();
  Serial.print("Timer set");
}

void loop() {
  float temperature = 0;
  float HR = 0;
  float SpO2 = 0;

  currentMillis = millis();

  if((currentMillis - previousMillis >= 1000)){
    temperature = readTemp();
    readHrSpO2(&HR, &SpO2);
    previousMillis = millis();
  
    //if(Serial.available()){
        Serial.print(temperature, 2); //insert what needs to be written in here
        Serial.print(",");
        Serial.print(HR, 2); 
        Serial.print(",");
        Serial.print(SpO2, 2);
        Serial.println(";");
    //}
  }
}

float readTemp(){
  float rawTemp; // Variable to store raw temperature
  float voltage;  // Variable to store voltage calculation
  float celsius;  // Variable to store Celsius value

  rawTemp = analogRead(tempPin);  // Read the raw 0-1023 value of temperature into a variable
  voltage = rawTemp * (5 / 1023.0);
  celsius = (voltage - 0.5) * 100;  // Calculate the celsius temperature, based on that voltage

  return(celsius);
}

void readHrSpO2(float* heartRate, float* oxygenSat){
  float n_spo2,ratio,correl;  //SPO2 value
  int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate; //heart rate value
  int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
  int32_t i;
  char hr_str[10];
     
  //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  //read BUFFER_SIZE samples, and determine the signal range
  for(i = 0; i < BUFFER_SIZE; i++){
    while(digitalRead(oxiPin)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
  }

  //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
  rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl); 

  if(ch_hr_valid && ch_spo2_valid) { 
    old_n_spo2 = n_spo2;
    *heartRate = n_heart_rate;
    *oxygenSat = n_spo2;
  }
}
