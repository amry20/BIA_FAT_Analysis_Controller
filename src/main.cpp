#include <Arduino.h>
#include <FlashStorage_STM32.h>
#include <ADS1115_WE.h> 
#include <MD_AD9833.h>
#include <Wire.h>
#include <SPI.h>
#include "main.h"

#define I2C_ADDRESS 0x48
#define F_SEL_1 PB_12
#define F_SEL_2 PB_13
#define F_SEL_3 PB_14
#define F_SEL_4 PB_15
#define HEART_PIN PA_11

const uint32_t PIN_FSYNC = PA4;	///< SPI Load pin number (FSYNC in AD9833 usage)
const uint32_t DRDY_PIN = PB8;

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);
MD_AD9833	DDS(PIN_FSYNC);  // Hardware SPI
bool ADC_Exists = false;
volatile bool ADCDoneConvertion = false;
float AverageVoltage = 0.0;
float AccumulateVoltage = 0.0;
int AccumulateCount = 0;
uint32_t SamplingInterval = 10;
int n_sample = 10;
void setup() {
  // put your setup code here, to run once:
  init_app();

}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t time_sampling = 0;
  if (ADC_Exists &&(millis() - time_sampling >= SamplingInterval)){
    time_sampling = millis();
    adc.startSingleMeasurement();
  }
  if (ADCDoneConvertion && ADC_Exists){
    ADCDoneConvertion = false;
    AccumulateCount++;
    AccumulateVoltage += adc.getResult_V();
    if (AccumulateCount >= n_sample)
    {
      AverageVoltage = (AccumulateVoltage / (float)AccumulateCount);
      AccumulateCount = 0;
      AccumulateVoltage = 0.0;
    }
  }
  heartbeat();
}

void init_app(){
  Serial.begin(115200);
  Wire.begin();
  //Init ADC chip
  if (adc.init()){
    ADC_Exists = true;
    adc.setVoltageRange_mV(ADS1115_RANGE_4096);
    adc.setCompareChannels(ADS1115_COMP_0_GND);
    adc.setAlertPinMode(ADS1115_ASSERT_AFTER_1);
    adc.setConvRate(ADS1115_250_SPS);
    adc.setMeasureMode(ADS1115_SINGLE);
    adc.setAlertPinToConversionReady();
    attachInterrupt(DRDY_PIN,adcConvDone,FALLING);
    Serial.println("ADC Initialized Succesfully");
  }
  pinMode(F_SEL_1,INPUT_PULLUP);
  pinMode(F_SEL_2,INPUT_PULLUP);
  pinMode(F_SEL_3,INPUT_PULLUP);
  pinMode(F_SEL_4,INPUT_PULLUP);
  pinMode(HEART_PIN,OUTPUT);
  digitalWriteFast(HEART_PIN,LOW);
  //Init DDS chip
  Serial.println("Initializing DDS...");
  DDS.begin();
  DDS.setMode(DDS.MODE_SINE);
  DDS.setPhase(DDS.CHAN_0,0);
  DDS.setActivePhase(DDS.CHAN_0);
  Serial.println("Done.");
}

void adcConvDone(){
  ADCDoneConvertion = true;
}

void heartbeat(){
  static uint32_t beat_time = 0;
  if (millis() - beat_time >= 500){
    beat_time = millis();
    digitalToggleFast(HEART_PIN);
  }
}

void handleCommand(){

}