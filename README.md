![Version](https://img.shields.io/badge/Version-v1.0.1-green.svg)

# VEML6075 for ESP32

An Arduino VEML6075 library for specifically the Espressif ESP32. 

The ESP32 experiences difficulties with current available libraries due to the way Wire.h handles the communication. This library coordinates the communication required for the sensor, overcoming the issue.

NOTE: This has not been tested in conjunction with the Wire.h library, therefore changes might be required to the I2C setup (see VEML6075.cpp).

### Features
---
* Pass a custom configuration
* Software "force mode" for individual reading control

### Commands
---
Contains all available commands as per the datasheet.

### Usage
---
The library will be found in the IDE/IO library manager. Alternatively, simply clone this library to your working library folder and include "VEML6075.h" in your main sketch.

If you are having difficulties, please raise an issue.

### A Bit About the Sensor
---
VEML67057 is a true UVA/UVB sensor. 

This library uses the coefficients for sunlight and therefore will not provide accurate (and often out of range - shown as "-1") readings for LED lighting etc. To do this, calculations must be made as per the datasheets (see extras).

### Authors
---
Myself, you can find my contact details below.

### License
---
This project is licensed under Apache 2.0 - see the LICENSE.md file for details

### Acknowledgments
----
This library was originally inspired by schizobovine's work! https://github.com/schizobovine/VEML6075;

### Feedback
---
Feel free to improve on the project and propose appropriate changes and even coding advice as I'm still new to this world.
