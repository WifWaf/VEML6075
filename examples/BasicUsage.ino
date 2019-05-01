#include <Arduino.h>
#include <esp32-hal-log.h>

#include "VEML6075.h"

#define DEBUG 0

#define SCLPIN GPIO_NUM_22      // gpio number for I2C master clock
#define SDAPIN GPIO_NUM_21      // gpio number for I2C master data

VEML6075 VML(0x10);             // Address here is optional, default is shown

void setup() 
{
    VML.begin(SDAPIN, SCLPIN);  // Pass your I2C numbers in GPI_NUM_X format as C library causes issues in C++ - see driver/gpio.h.
}

void loop() 
{
    /* Function automatically requests new relevant readings (see poll curbing to perform manually) */
    ESP_LOGI("MAIN", "UVA: %.2f  UVB: %.2f  UVINDEX: % .2f", VML.getUVA(), VML.getUVB(), VML.getUVIndex());         
    vTaskDelay(1000);
}  

