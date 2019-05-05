#include <Arduino.h>
#include <esp32-hal-log.h>
//#include <Wire.h>                     // Uncomment if passing a TwoWire (Wire.h) reference

#include "VEML6075.h"

#define SCLPIN 22                     // I2C SCL GPIO
#define SDAPIN 21                     // I2C SDA GPIO

VEML6075 VML;                         // Below shows the default parameters passed if not entered
//VEML6075 VML(0x10, Wire);           // Optional arguments shown

void setup() 
{
    VML.begin(SDAPIN, SCLPIN);        // Pass your I2C numbers (defaults 21, 22 if not specified)
}

void loop() 
{
    /* Library equests new relevant readings (see update curbing to perform manually) */
    ESP_LOGI("MAIN", "UVA: %.2f  UVB: %.2f  UVINDEX: % .2f", VML.getUVA(), VML.getUVB(), VML.getUVIndex());         
    vTaskDelay(1000);
}  

