#include <Arduino.h>
#include <FlashStorage_STM32.h>
#include "ADS1X15.h"
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
#define ValidIDTag 0x7F

const uint32_t PIN_FSYNC = PA_4;	///< SPI Load pin number (FSYNC in AD9833 usage)
const uint32_t DRDY_PIN = PB_8;

typedef struct EEPROM_DATA{
  uint8_t IDTag;
  float custom_freq;
  uint32_t sampling_interval;
  uint16_t n_sample;

}__attribute__((packed)) EEPROM_DATA;

ADS1115 adc(I2C_ADDRESS);
MD_AD9833	DDS(PA7,PA5,PIN_FSYNC);  // Hardware SPI
bool ADC_Exists = false;
volatile bool ADCDoneConvertion = false;
float AverageVoltage = 0.0;
float AccumulateVoltage = 0.0;
uint16_t AccumulateCount = 0;
uint32_t SamplingInterval = 10;
uint16_t n_sample = 10;
float DDSCustomFreq = 200000.0;
EEPROM_DATA APP_EEPROM_DATA;
void setup() {
  // put your setup code here, to run once:
  init_app();
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t time_sampling = 0;
  AverageVoltage = readChannel(0) * 2.0;
  Serial.println(AverageVoltage,3);
  handleCommand();
  heartbeat();
}

void init_app(){
  Serial.begin(115200);
  //Read EEPROM data
  Wire.begin();
  if(adc.begin()){
    adc.setGain(0);      //  6.144 volt
    adc.setDataRate(7);  //  0 = slow   4 = medium   7 = fast
    adc.setMode(0);      //  continuous mode
    adc.readADC(0);      //  first read to trigger
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
  if (!digitalReadFast(F_SEL_1) && digitalReadFast(F_SEL_2) && digitalReadFast(F_SEL_3) && digitalReadFast(F_SEL_4))
    DDS.setFrequency(DDS.CHAN_0,25000.0);
  else if (digitalReadFast(F_SEL_1) && !digitalReadFast(F_SEL_2) && digitalReadFast(F_SEL_3) && digitalReadFast(F_SEL_4))
    DDS.setFrequency(DDS.CHAN_0,50000.0);
  else if (digitalReadFast(F_SEL_1) && digitalReadFast(F_SEL_2) && !digitalReadFast(F_SEL_3) && digitalReadFast(F_SEL_4))
    DDS.setFrequency(DDS.CHAN_0,100000.0);
  else if (digitalReadFast(F_SEL_1) && digitalReadFast(F_SEL_2) && digitalReadFast(F_SEL_3) && !digitalReadFast(F_SEL_4))
    DDS.setFrequency(DDS.CHAN_0,DDSCustomFreq);
  else DDS.setFrequency(DDS.CHAN_0,50000);
  DDS.setActiveFrequency(DDS.CHAN_0);
  Serial.print("DDS Frequency: ");
  Serial.println(DDS.getFrequency(DDS.CHAN_0));
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
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

}
float readChannel(int channel) {
  float voltage = 0.0;
  int adc0 = adc.getValue();
  voltage =adc.toVoltage(adc0);
  return voltage;
}