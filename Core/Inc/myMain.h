/*
 * myMain.h
 *
 *  Created on: Nov 22, 2020
 *      Author: patrick
 */

#ifndef INC_MYMAIN_H_
#define INC_MYMAIN_H_

#include "main.h"
#include "cmsis_os2.h"

int myMain(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Reset_Device( void );
void Reset_IPCC( void );
void Init_Exti( void );
void Reset_BackupDomain( void );
void Config_HSE(void);
void initClk (void);

extern osThreadId_t defaultTaskHandle;

#endif /* INC_MYMAIN_H_ */
