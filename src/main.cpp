#include <Arduino.h>
#include <FlashStorage_STM32.h>
#include<ADS1115_WE.h>
#include <MD_AD9833.h>
#include <Wire.h>
#include <SPI.h>
#include "main.h"
#include <HardwareSerial.h>

#define I2C_ADDRESS 0x48
#define F_SEL_1 PB_12
#define F_SEL_2 PB_13
#define F_SEL_3 PB_14
#define F_SEL_4 PB_15
#define DRDY_PIN PE8
#define HEART_PIN PA_11
#define ValidIDTag 0x7F
#define FW_VERSION "1.0.0DBG"

const uint32_t PIN_FSYNC = PA_4;	///< SPI Load pin number (FSYNC in AD9833 usage)
const uint32_t DRDY_PIN = PB_8;

typedef struct EEPROM_DATA{
  uint8_t IDTag;
  float custom_freq;
  uint32_t sampling_interval;
  uint16_t n_sample;

}__attribute__((packed)) EEPROM_DATA;

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);
MD_AD9833	DDS(PA7,PA5,PIN_FSYNC);  // Hardware SPI
HardwareSerial SerialComm(PA3, PA2);

bool ADC_Exists = false;
bool UseCustomFreq = false;
volatile bool ADCDoneConvertion = false;
float AverageVoltage1 = 0.0;
float AverageVoltage2 = 0.0;
float AccumulateVoltage1 = 0.0;
float AccumulateVoltage2 = 0.0;
float ModImpedance = 0.0;
float PointImpedance = 0.0;
uint16_t AccumulateCount = 0;
uint32_t SamplingInterval = 10;
uint16_t n_sample = 85;
float DDSCustomFreq = 10000.0;
EEPROM_DATA APP_EEPROM_DATA;
void setup() {
  // put your setup code here, to run once:
  init_app();
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t sample_count = 0;
  static bool get_adc = true;
  static int channel_count = 0;
  static bool wait_done = false;
  if (channel_count == 0)
  {
    if (!wait_done)
    {
      adc.setCompareChannels(ADS1115_COMP_0_GND);
      adc.startSingleMeasurement();
      wait_done = true;
    }
    if (wait_done){
      if (ADCDoneConvertion){
        ADCDoneConvertion = false;
        AccumulateVoltage1 += adc.getResult_V() * 2.055;
        channel_count = 1;
        wait_done = false;
      }
    }
  }
  else if (channel_count == 1){
    if (!wait_done)
    {
      adc.setCompareChannels(ADS1115_COMP_1_GND);
      adc.startSingleMeasurement();
      wait_done = true;
    }
    if (wait_done){
      if (ADCDoneConvertion){
        ADCDoneConvertion = false;
        AccumulateVoltage2 += adc.getResult_V() * 2.055;
        channel_count = 0;
        wait_done = false;
        sample_count++;
      }
    }
  }
  
  if (sample_count >= n_sample)
  {
    AverageVoltage1 = AccumulateVoltage1 / sample_count;
    AverageVoltage2 = AccumulateVoltage2 / sample_count;
    PointImpedance = AverageVoltage1 / (1.09585 / 2000.0); 
    ModImpedance = AverageVoltage2 / (1.09585 / 2000.0);
    AccumulateVoltage1 = 0;
    AccumulateVoltage2 = 0;
    sample_count = 0;
    char data[256];
    memset(data,0,sizeof(data));
    sprintf(data,"D,%0.5f,%0.5f,%0.5f,%0.5f\n",AverageVoltage1,AverageVoltage2,PointImpedance,ModImpedance);
    Serial.write(data,strlen(data));
    SerialComm.write(data,strlen(data));
  }
  
  handleCommand();
  heartbeat();
}
void convReadyAlert(){
   ADCDoneConvertion = true;
}
void init_app(){
  Serial.begin(115200);
  SerialComm.begin(115200);
  //Read EEPROM data
  Wire.begin();
  Wire.setClock(400000);
  if(adc.init()){
    adc.setVoltageRange_mV(ADS1115_RANGE_6144);      //  6.144 volt
    adc.setConvRate(ADS1115_250_SPS);
    adc.setCompareChannels(ADS1115_COMP_0_GND); //comment line/change parameter to change channels
    adc.setAlertPinMode(ADS1115_ASSERT_AFTER_1);
    adc.setAlertPinToConversionReady(); //needed for this sketch
    attachInterrupt(DRDY_PIN, convReadyAlert, FALLING);
    adc.startSingleMeasurement();
    while (!ADCDoneConvertion)
    {
      /* code */
    }
    ADCDoneConvertion = false;
    Serial.println("ADC Initialized Succesfully");
    SerialComm.println("ADC Initialized Succesfully");
  }
  
  pinMode(F_SEL_1,INPUT_PULLUP);
  pinMode(F_SEL_2,INPUT_PULLUP);
  pinMode(F_SEL_3,INPUT_PULLUP);
  pinMode(F_SEL_4,INPUT_PULLUP);
  pinMode(HEART_PIN,OUTPUT);
  digitalWriteFast(HEART_PIN,LOW);
  //Init DDS chip
  Serial.println("Initializing DDS...");
  SerialComm.println("Initializing DDS...");
  DDS.begin();
  DDS.setMode(DDS.MODE_SINE);
  DDS.setPhase(DDS.CHAN_0,0);
  DDS.setFrequency(DDS.CHAN_0,0);
  delay(1000);
  if (!digitalReadFast(F_SEL_1) && digitalReadFast(F_SEL_2) && digitalReadFast(F_SEL_3) && digitalReadFast(F_SEL_4))
    DDS.setFrequency(DDS.CHAN_0,25000.0);
  else if (digitalReadFast(F_SEL_1) && !digitalReadFast(F_SEL_2) && digitalReadFast(F_SEL_3) && digitalReadFast(F_SEL_4))
    DDS.setFrequency(DDS.CHAN_0,50000.0);
  else if (digitalReadFast(F_SEL_1) && digitalReadFast(F_SEL_2) && !digitalReadFast(F_SEL_3) && digitalReadFast(F_SEL_4))
    DDS.setFrequency(DDS.CHAN_0,100000.0);
  else if (digitalReadFast(F_SEL_1) && digitalReadFast(F_SEL_2) && digitalReadFast(F_SEL_3) && !digitalReadFast(F_SEL_4))
  {
    DDS.setFrequency(DDS.CHAN_0,DDSCustomFreq);
    UseCustomFreq = true;
  }
  else DDS.setFrequency(DDS.CHAN_0,50000);
  DDS.setActiveFrequency(DDS.CHAN_0);
  Serial.print("DDS Frequency: ");
  SerialComm.print("DDS Frequency: ");
  Serial.println(DDS.getFrequency(DDS.CHAN_0));
  SerialComm.println(DDS.getFrequency(DDS.CHAN_0));
  Serial.println("Done.");
  SerialComm.println("Done.");
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
  if (Serial.available() > 0)
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("FIRMWARE")) {
      Serial.printf("FW,%s\n",FW_VERSION);
    }
    else  if (cmd.startsWith("FQ ")) {
      String freqStr = cmd.substring(3); // Mengambil angka setelah "FQ "
      float frequency = freqStr.toFloat(); // Mengonversi ke float
      if (UseCustomFreq){
        DDSCustomFreq = frequency;
        DDS.setFrequency(DDS.CHAN_0,DDSCustomFreq);
        Serial.printf("Successfuly to set the DDS frequency to %0.3f\n",DDS.getFrequency(DDS.CHAN_0));
      }
      else{
        Serial.println("Can't set DDS the frequency, please select custom frequency on the jumper pin!");
      }
    }
  }
  if (SerialComm.available() > 0)
  {
    String cmd = SerialComm.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("FIRMWARE")) {
      SerialComm.printf("FW,%s\n",FW_VERSION);
    }
    else  if (cmd.startsWith("FQ ")) {
      String freqStr = cmd.substring(3); // Mengambil angka setelah "FQ "
      float frequency = freqStr.toFloat(); // Mengonversi ke float
      if (UseCustomFreq){
        DDSCustomFreq = frequency;
        DDS.setFrequency(DDS.CHAN_0,DDSCustomFreq);
        SerialComm.printf("Successfuly to set the DDS frequency to %0.3f\n",DDS.getFrequency(DDS.CHAN_0));
      }
      else{
        SerialComm.println("Can't set the DDS frequency, please select custom frequency on the jumper pin!");
      }
    }
  }
}
float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  return voltage;
}
