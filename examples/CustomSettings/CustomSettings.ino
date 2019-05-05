#include <Arduino.h>
#include <esp32-hal-log.h>

#include "VEML6075.h"

#define SCLPIN 22 
#define SDAPIN 21

VEML6075 VML; 

void setVMLConfig();                        // Pre-declaration for non Arduino IDE environments

void setup() 
{
    VML.begin(SDAPIN, SCLPIN);              // Pass your I2C numbers

    setVMLConfig();                         // Call function below
}

void loop() 
{
    ESP_LOGI("MAIN", "UVA: %.2f  UVB: %.2f  UVINDEX: % .2f", VML.getUVA(), VML.getUVB(), VML.getUVIndex());         
    vTaskDelay(1000);
}  

void setVMLConfig()
{
    vml_Config myConfig;                    // Create instance of vml_config

    myConfig.shutdown = UV_CONF_SD_OFF;     // Set the vml_config variables, options/defintions are shown in the header file
    myConfig.forced = UV_CONF_AF_OFF;
    myConfig.trigg = UV_CONF_TRIG_OFF;
    myConfig.dynamic = UV_CONF_HD_NORM;
    myConfig.int_time = UV_CONF_T_100MS;

    VML.setConfig(&myConfig);               // Pass a vml_config reference into setConfig function
}