/*
 * led.h
 *
 *  Created on: Dec 7, 2020
 *      Author: patrick
 */

#ifdef __cplusplus
extern "C" {
#endif


#ifndef INC_LED_H_
#define INC_LED_H_

//#include "tim.h"
#include "gpio.h"
//#include "stepper.h"
//#include "adc.h"
#include "stdint.h"


void toggleLed(uint8_t red, uint8_t green, uint8_t blue);


#endif /* INC_LED_H_ */



#ifdef __cplusplus
}
#endif
