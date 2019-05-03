/*************************************************** 
  ESP32 Arduino library for the VEML6075
  
  Author: Jonathan Dempsey JDWifWaf@gmail.com
  
  Version: 1.0.1

  License: Apache 2.0

 ****************************************************/

#include "VEML6075.h"
#include <esp32-hal-log.h>
#include <driver/i2c.h>

/*Enable/Disable Debug Prints -----------------------------------------*/
#define DEBUG 0

/*ESP_LOG Naming ------------------------------------------------------*/
#define TAG_VEM "VEM6075 I2C"

/*I2C Config ----------------------------------------------------------*/
#define I2C_MASTER_FREQ_HZ 400000   // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define I2C_NUM I2C_NUM_0           // I2C number

/*I2C Communication ---------------------------------------------------*/
#define ACK_CHECK_EN  (0x1)               // master will check for acknolwedgement
#define ACK_CHECK_DIS (0x0)               // master will not check for acknolwedgement
#define ACK_VAL       (I2C_MASTER_ACK)    // master will acknowledge (the translation from C to C++ is finicky, so driver define used)
#define NACK_VAL      (I2C_MASTER_NACK)   // master will not acknowledge  

/* Coefficients for UV Index translation ------------------------------*/
#define COEF_UVA_VIS (2.22)
#define COEF_UVA_IR  (1.33)
#define COEF_UVB_VIS (2.95)
#define COEF_UVB_IR  (1.74)
#define RECIEVE_UVA  (0.001461)
#define RECIEVE_UVB  (0.002591)

/* Relevant commands --------------------------------------------------*/
#define COM_CONF (0x00)
#define COM_UVA  (0x07)
#define COM_DARK (0x08)
#define COM_UVB  (0x09)
#define COM_VIS  (0x0A)
#define COM_IR   (0x0B)
#define COM_ID   (0x0C)

VEML6075::VEML6075(byte addr) : _addr(addr) {}; 

bool VEML6075::begin(gpio_num_t sda, gpio_num_t sdl, vml_IntegrationTime inTime)
{
  this->_sda = sda; 
  this->_sdl = sdl;

  if(I2CSetup() != ESP_OK)
    return false;
  
  if(inTime != T_100MS)
    this->settings.int_time = inTime;

  if(!wasConfigured)
    configure();

  return true;
}

void VEML6075::setConfig(vml_Config *inConfig)
{
  memcpy(&this->settings, inConfig, sizeof(this->settings));

  this->wasConfigured = true;

  configure();
}

void VEML6075::setIntegration(vml_IntegrationTime inTime)
{
 this->settings.int_time |= inTime;
 configure();
}

void VEML6075::update()
{
  requestReading(DARK);
  requestReading(VIS);
  requestReading(IR);
  requestReading(UVA);
  requestReading(UVB);
}

unsigned int VEML6075::getRawUVA(bool force)
{
  if(force)
    requestReading(UVA);

  return this->readings[UVA];
}

unsigned int VEML6075::getRawUVB(bool force)
{
  if(force)
    requestReading(UVB);

  return this->readings[UVB];
}

unsigned int VEML6075::getRawDark(bool force)
{
  if(force)
    requestReading(DARK);

  return this->readings[DARK];
}

unsigned int VEML6075::getRawVisComp(bool force)
{
  if(force)
    requestReading(VIS);

  return this->readings[VIS];
}

unsigned int VEML6075::getRawIRComp(bool force)
{
  if(force)
    requestReading(IR);

  return this->readings[IR];
}

unsigned int VEML6075::getID()
{
  return read(ID);
}

float VEML6075::getUVA(bool force)
{
  return calculate(UVA, force);
}

float VEML6075::getUVB(bool force)
{
  return calculate(UVB, force);
}

float VEML6075::getUVIndex(bool force)
{
  float calcUVA, calcUVB;

  calcUVA = RECIEVE_UVA * getUVA(force);  
  calcUVB = RECIEVE_UVB * getUVB(force);

  return (calcUVA + calcUVB)/2;
}

void VEML6075::requestReading(vml_CommandID alias)
{  
  switch(alias)
  {
    case UVA:
      readings[alias] = read(COM_UVA);
    break;
    case UVB:
      readings[alias] = read(COM_UVB);
    break;
    case DARK:
      readings[alias] = read(COM_DARK);
    break;
    case VIS:
      readings[alias] = read(COM_VIS);
    break;
    case IR:
      readings[alias] = read(COM_IR);
    break;
    default:
    break;
  }; 
}
 
float VEML6075::calculate(vml_CommandID alias, bool force) 
{
  float calc[4]; 
  float product = 0;

  if(force)
  {
    requestReading(DARK);
    requestReading(VIS);
    requestReading(IR);
    requestReading(alias);
  }
  
  memset(calc, 0, 4);

  for(uint8_t x = 0; x < 3; x++)
  {
    calc[x] = this->readings[x];
  }
  
  product = this->readings[alias];

  for(uint8_t x = 0; x < 4; x++)
  {
    calc[x] -= _max(this->readings[DARK], 0);
  }
  
  switch(alias)
  {
    case UVA:
    product -= (COEF_UVA_VIS * calc[VIS]) - (COEF_UVA_IR * calc[IR]);
    break;
    case UVB:
    product -= (COEF_UVB_VIS * calc[VIS]) - (COEF_UVB_IR * calc[IR]);
    break;
    default:
    return 0;
  };

  #if DEBUG
  ESP_LOGD(TAG_VEM, "Dark %d Vis %.2f IR %.2f Product %.2f", this->readings[DARK],
  this->readings[VIS] ,this->readings[IR], this->readings[alias]);
  #endif
  
  if(product < 0)
    product = -1;

  return product;
}

unsigned int VEML6075::read(uint8_t inCommand)
{
  uint8_t LByte, *pLByte, HByte, *pHByte;
  pLByte = &LByte;
  pHByte = &HByte;
  
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  ESP_ERROR_CHECK(i2c_master_start(cmd));

  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (VEML6075_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, inCommand, ACK_CHECK_EN));
 
  ESP_ERROR_CHECK(i2c_master_start(cmd));

  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (VEML6075_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN));
  ESP_ERROR_CHECK(i2c_master_read_byte(cmd, pLByte, I2C_MASTER_ACK));
  ESP_ERROR_CHECK(i2c_master_read_byte(cmd, pHByte, I2C_MASTER_ACK));

  ESP_ERROR_CHECK(i2c_master_stop(cmd));

  ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS));

  i2c_cmd_link_delete(cmd);

  uint16_t ret = (HByte << 8) | LByte;

  #if DEBUG
  ESP_LOGD(TAG_VEM,"Reg %#03x  Low Byte %d  *pLowByte %d  High Byte %d  *pHByte %d  Return %d",inCommand, LByte, *pLByte, HByte, *pHByte, ret);
  #endif

  return ret;
}

void VEML6075::write(uint8_t inCommand, uint8_t inByte)
{
   
  #if DEBUG
  ESP_LOGD(TAG_VEM,"Writing to VEML6075");
  #endif

  uint8_t LSB = (0xFF & (inByte >> 0)); // LSB
  uint8_t MSB = (0xFF & (inByte >> 8)); // MSB

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  ESP_ERROR_CHECK(i2c_master_start(cmd));

  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (VEML6075_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, inCommand, ACK_CHECK_EN));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, LSB, ACK_CHECK_EN));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MSB, ACK_CHECK_EN));

  ESP_ERROR_CHECK(i2c_master_stop(cmd));
  ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, (1000 / portTICK_RATE_MS)));

  i2c_cmd_link_delete(cmd);
}

byte VEML6075::I2CSetup()
{ 
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = _sda;
  conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
  conf.scl_io_num = _sdl;
  conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;  
  i2c_param_config(I2C_NUM , &conf);

  return i2c_driver_install(I2C_NUM, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0); 
}

void VEML6075::configure()
{
  /* Place bits into config byte (defaults are shown in header) */
  this->config |= this->settings.shutdown;
  this->config |= this->settings.forced;
  this->config |= this->settings.trigg;
  this->config |= this->settings.dynamic;
  this->config |= this->settings.int_time;
  
  /* Call write function */
  write(COM_CONF, this->config);
}
