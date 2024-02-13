#include "Modulino.h"

/*
Buttons buttons(62);
Tone buzzer(30);
LEDS leds(54);
Encoder encoder(58);
*/
BoschSensorClass imu(Wire1);
VL53L1X tof_sensor;
APDS9960 color(Wire1, -1);  // TODO: need to change to APDS9999 https://docs.broadcom.com/doc/APDS-9999-DS
LPS22HBClass barometer(Wire1);
HS300xClass humidity(Wire1);

Color RED(255, 0, 0);
Color BLUE(0, 0, 255);
Color GREEN(0, 255, 0);
Color VIOLET(255, 0, 255);