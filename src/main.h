#ifndef MAIN_H
#define MAIN_H
void init_app();
void adcConvDone();
void heartbeat();
void handleCommand();
float readChannel(ADS1115_MUX channel);
void convReadyAlert();
#endif