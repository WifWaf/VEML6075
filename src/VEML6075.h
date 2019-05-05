/*************************************************** 
  Author: Jonathan Dempsey JDWifWaf@gmail.com
  
  Version: 1.0.2

  License: Apache 2.0

  ESP32 Arduino library for the VEML6075

 ****************************************************/
 
#ifndef _VEML6075_H
#define _VEML6075_H

#include <Arduino.h>
#include <Wire.h>

/* Default Address used if not specified with constructor */
#define VEML6075_ADDR 0x10 

/*UV_Config Register -------------------------------------------*/
#define UV_CONF_HD_NORM   (0x00)
#define UV_CONF_HD_HIGH   (0x08)
#define UV_CONF_TRIG_ON   (0x04)
#define UV_CONF_TRIG_OFF  (0x00)
#define UV_CONF_AF_OFF    (0x00)
#define UV_CONF_AF_ON     (0x02)
#define UV_CONF_SD_OFF    (0x00)
#define UV_SD_ON          (0x01)

#define UV_CONF_T_50MS  (0x00)
#define UV_CONF_T_100MS (0x10)
#define UV_CONF_T_200MS (0x20)
#define UV_CONF_T_400MS (0x30)
#define UV_CONF_T_800MS (0x40)
#define UV_CONF_T_MASK  (0x8F)

/* Variables --------------------------------------------------*/
/* enum defintions for intergration times */
typedef enum
{
  T_50MS = UV_CONF_T_50MS,
  T_100MS = UV_CONF_T_100MS,
  T_200MS = UV_CONF_T_200MS,
  T_400MS = UV_CONF_T_400MS,
  T_800MS = UV_CONF_T_800MS,
  MASK = UV_CONF_T_MASK, 
} vml_IntegrationTime;

/* Config structure for custom configuration, defaults are loaded */
typedef struct 
{
  uint8_t shutdown = UV_CONF_SD_OFF;
  uint8_t forced = UV_CONF_AF_OFF;
  uint8_t trigg = UV_CONF_TRIG_OFF;
  uint8_t dynamic = UV_CONF_HD_NORM;
  uint8_t int_time = UV_CONF_T_100MS; 
} vml_Config;

/* Class -------------------------------------------------------*/
class VEML6075
{
  public:
    VEML6075(byte addr = 0x10, TwoWire &inWire = Wire);

    bool begin(uint8_t sda = 21, uint8_t sdl = 22, vml_Config *inConfig = NULL);   

    void setConfig(vml_Config *inConfig);
    void setIntegration(vml_IntegrationTime inTime);
    
    void update();
    float getUVA(bool force = true);
    float getUVB(bool force = true);
    float getUVIndex(bool force = true);
    unsigned int getID();
    unsigned int  getRawUVA(bool force = true);
    unsigned int  getRawUVB(bool force = true);
    unsigned int  getRawDark(bool force = true);
    unsigned int  getRawVisComp(bool force = true);
    unsigned int  getRawIRComp(bool force = true);

  private:
    unsigned int readings[6]; // Index follows enum convention   

    uint8_t config = 0;
    byte _addr;

    TwoWire* myWire;          // I2C stream reference
    
    typedef enum 
    {
      DARK = 0,
      VIS = 1,
      IR = 2,
      UVA = 3,
      UVB = 4,
      ID = 5,
      CONF = 6,
    } vml_CommandID; 

    vml_Config settings;    
    uint8_t _sda, _sdl;

    void configure();    

    void requestReading(vml_CommandID alias);    
    float calculate(vml_CommandID alias, bool force);

    unsigned int read(uint8_t inCommand);
    void write(uint8_t inCommand, uint8_t inByte);
};

#endif