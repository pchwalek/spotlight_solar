/*
 * calibration.cpp
 *
 *  Created on: Dec 7, 2020
 *      Author: patrick
 */

#include "calibration.h"
#include "app_thread.h"
#include "usbd_cdc_if.h"
#include "myMain.h"

#define SPOTLIGHT_UID			0x18

#define CAL_INIT_CAL_TIMEOUT	2000
#define CAL_POS_TIMEOUT  		1000 // 1 second





uint32_t calibration_table[20] = {0};
struct CalMsg calMsg;



int32_t led_step;
int32_t base_step;

//otIp6Address multicastAddr;
//const char nodeCalResource[15];


/**
 * @brief starts calibration process for solar-powered systems within proximity of Spotlight.
 *
 * @param[in]  angle_base_min   min angle of base (represented as a fraction of 360-degrees from start)
 * @param[in]  angle_base_max   max angle of base (represented as a fraction of 360-degrees from start)
 * @param[in]  angle_led_min    min angle of led (represented as a fraction of 360-degrees from start)
 * @param[in]  angle_led_max    max angle of led (represented as a fraction of 360-degrees from start)
 *
 * @retval none.
 */
#ifndef SOLAR_SENSOR_NODE

void startCal(float angle_base_min, float angle_base_max,
		float angle_led_min, float angle_led_max,
				Stepper& motor_base, Stepper& motor_led){

	// configure calMsg packet
	calMsg.UID = SPOTLIGHT_UID;
	calMsg.cal_code = CALIBRATION_START_COMMAND;

	/* (1) hone axis */
	motor_base.honeAxis();
	motor_led.honeAxis();

	/* (2) multicast that calibration will begin */

	// clear current calibration table
	memset(calibration_table,0,sizeof(calibration_table));

	broadcastCalStart(&calMsg); //TODO: implement

	/* (3) wait for nodes to ACK and populate a table with the UIDs */

	// note: nodes will be replying to a Coap server (code exists in app_thread.c for populating of table)

	// wait for nodes to reply with their IDs
	osDelay(CAL_INIT_CAL_TIMEOUT);


	/* (4) Run through each position, sending an msg to each node indicating the angle is set
	 * and waiting for said nodes to respond with their solar power measurements. If node does
	 * not reply within CAL_POS_TIMEOUT, re-send msg and wait one more cycle before continuing.
	 */
		// calculate step bounds for each axis
		int32_t min_led_axis_step = (((angle_led_min) * LED_STEP_RANGE));
		int32_t max_led_axis_step = (((angle_led_max) * LED_STEP_RANGE));
		int32_t min_base_axis_step = (((angle_led_min) * BASE_STEP_RANGE));
		int32_t max_base_axis_step = (((angle_led_max) * BASE_STEP_RANGE));

		// move to desired angle
		uint8_t flip_dir = 0;
		for(base_step = min_base_axis_step; base_step <= max_base_axis_step; base_step += (BASE_CAL_STEP_MULTI * BASE_MIN_ANGLE_STEP)){
			motor_base.setAbsPos(base_step);

			if(flip_dir == 0){
				for(led_step = min_led_axis_step; led_step <= max_led_axis_step; led_step += (LED_CAL_STEP_MULTI * LED_MIN_ANGLE_STEP)){
					motor_led.setAbsPos(led_step);
					getMeasurementsFromNodes(&calMsg, base_step, led_step);

					// wait CAL_POS_TIMEOUT milliseconds for nodes to reply with power values
					osThreadFlagsWait (CAL_THREAD_FLAG, osFlagsWaitAny, CAL_POS_TIMEOUT);
				}
				flip_dir = 1;
			}
			else{
				for(int32_t led_step = max_led_axis_step; led_step >= min_led_axis_step; led_step -= (LED_CAL_STEP_MULTI * LED_MIN_ANGLE_STEP)){
					motor_led.setAbsPos(led_step);
					getMeasurementsFromNodes(&calMsg, base_step, led_step);

					// wait CAL_POS_TIMEOUT milliseconds for nodes to reply with power values
					osThreadFlagsWait (CAL_THREAD_FLAG, osFlagsWaitAny, CAL_POS_TIMEOUT);
				}
				flip_dir = 0;
			}
		}


//	/* (5) send calibration complete message to all nodes */
		broadcastCalComplete(&calMsg);

}
#endif


void broadcastCalStart(CalMsg* msg){

	msg->cal_fcn = CALIBRATION_START_COMMAND;
	msg->angle_1 = 0;
	msg->angle_2 = 0;

	APP_THREAD_SendCoapMsg(msg, sizeof(CalMsg),
			&multicastAddr, nodeCalResource,
			REQUEST_ACK, OT_COAP_CODE_PUT, 1U);

	return;
}

void broadcastCalComplete(CalMsg* msg){

	msg->cal_fcn = CALIBRATION_START_COMMAND;
	msg->angle_1 = 0;
	msg->angle_2 = 0;

	APP_THREAD_SendCoapMsg(msg, sizeof(CalMsg),
			&multicastAddr, nodeCalResource,
			REQUEST_ACK, OT_COAP_CODE_PUT, 1U);

	return;
}

void getMeasurementsFromNodes(CalMsg* msg, int32_t angle_1, int32_t angle_2){

	msg->cal_fcn = CALIBRATION_GET_MEAS;
	msg->angle_1 = angle_1;
	msg->angle_2 = angle_2;

	APP_THREAD_SendCoapMsg(msg, sizeof(CalMsg),
			&multicastAddr, nodeCalResource,
			REQUEST_ACK, OT_COAP_CODE_GET, 1U);


	return;
}




void addNodeID(uint32_t UID, uint32_t *arr){
	int i;
	for(i = 0; i < sizeof(arr) / sizeof(arr[0]); i++)
	{
		if(arr[i] == UID)
			return;
		else if(arr[i] == 0){
			arr[i] = UID;
			return;
		}
	}
}

int valueinarray(float val, float arr[])
{
    int i;
    for(i = 0; i < sizeof(arr) / sizeof(arr[0]); i++)
    {
        if(arr[i] == val)
            return i;
        else if(arr[i] == 0)
        	return 0;
    }
    return -1;
}



