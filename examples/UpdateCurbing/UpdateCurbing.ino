/* 
While the sensor can be in "Force Mode (takes a reading when you request a value, see settings)",
an option has been added to request new individual readings only, or what is neccesary to calculate 
a variable.

Additionaly, you can "poll" all sensors at once with the function update();

--- Bool force -----
True: Requests sensor upate
False, Does not request sensor update
Default: True
*/

#include <Arduino.h>
#include <esp32-hal-log.h>

#include "VEML6075.h"

#define SCLPIN 22 
#define SDAPIN 21 

VEML6075 VML;

void setup()
{
    VML.begin(SDAPIN, SCLPIN);
}

void loop() 
{
  /* Update sensor */
  VML.update(); 

  /* request readings using getUVX(false) - keeping all calculations synchronised to update() */
  ESP_LOGI("MAIN", "UVA: %.2f  UVB: %.2f  UVINDEX: % .2f", VML.getUVA(false), VML.getUVB(false), VML.getUVIndex(false));      
  vTaskDelay(1000);
}  

