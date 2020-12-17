/*
 * calibration.h
 *
 *  Created on: Dec 7, 2020
 *      Author: patrick
 */



#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "cmsis_os2.h"
#include "ip6.h"
#include "spotlight_config.h"

#ifndef SOLAR_SENSOR_NODE
#include "stepper.h"
#endif

#ifndef SOLAR_SENSOR_NODE
void startCal(float angle_base_min, float angle_base_max,
		float angle_led_min, float angle_led_max,
				Stepper& motor_base, Stepper& motor_led);
#endif

void addNodeID(uint32_t UID, uint32_t *arr);
int valueinarray(float val, float arr[]);
void broadcastCalStart(struct CalMsg* msg);
void getMeasurementsFromNodes(struct CalMsg* msg, int32_t angle_1, int32_t angle_2);
void broadcastCalComplete(struct CalMsg* msg);

#ifdef __cplusplus
}
#endif

#endif /* INC_CALIBRATION_H_ */




