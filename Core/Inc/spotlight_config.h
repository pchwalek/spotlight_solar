/*
 * spotlight_config.h
 *
 *  Created on: Dec 10, 2020
 *      Author: patrick
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef INC_SPOTLIGHT_CONFIG_H_
#define INC_SPOTLIGHT_CONFIG_H_

#include "stdio.h"
#include "ip6.h"

#define SOLAR_SENSOR_NODE				1
//#define SOLAR_SENSOR_NODE_I2C_DISABLE	1
//#define DEBUG_SERIAL				1

#define CALIBRATION_CODE
#define CALIBRATION_START_COMMAND	0x01
#define CALIBRATION_GET_MEAS	0x02
#define CALIBRATION_RESEND_MEAS		0x03
#define CALIBRATION_STOP_COMMAND	0x04

#define CAL_THREAD_FLAG 	0x00000003U


#define POWER_MEAS_ADDR	0x44
#define CURRENT_RES 0.12158 // uA/bit
#define BUS_VOLT_RES 1.25 // mV/bit
#define SHUNT_VOLT_RES 2.5 // uV/bit
#define POWER_RES CURRENT_RES*25 // uW/bit




struct CalMsg {
	uint8_t UID;
	uint8_t cal_code;
	uint8_t cal_fcn;
	int32_t angle_1;
	int32_t angle_2;
};

struct MeasMsg{
	uint8_t UID;
	uint8_t command;
	int32_t angle_1;
	int32_t angle_2;
	uint16_t power;
	uint16_t current;
	uint16_t shuntVoltage;
	uint16_t busVoltage;
};

void measMsgReceivedFromNode(struct MeasMsg* msg);
void sendPowerMeasurement(otIp6Address peerAddr, int32_t angle_1, int32_t angle_2);

void powerMeasSetup(void);
uint16_t getPower();
uint16_t getCurrent();
uint16_t getBusVoltage();
uint16_t getShuntVoltage();

extern volatile struct MeasMsg measMsg;

#endif /* INC_SPOTLIGHT_CONFIG_H_ */

#ifdef __cplusplus
}
#endif
