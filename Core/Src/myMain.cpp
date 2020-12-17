/*
 * myMain.cpp
 *
 *  Created on: Nov 22, 2020
 *      Author: patrick
 */

#include "myMain.h"

#include "gpio.h"

#include "otp.h"
#include "stm32_lpm.h"
#include "dbg_trace.h"
#include "hw_conf.h"

#include "calibration.h"
#include "spotlight_config.h"

#ifndef SOLAR_SENSOR_NODE
#include "tim.h"
#include "stepper.h"
#include "adc.h"
#endif

#define STEPS 2052
#define STEPS_LED_RANGE 31011
volatile uint16_t timer;

struct ADC_DATA{
	uint16_t mag_base;
	uint16_t mag_led;
	uint16_t temp_main;
	uint16_t temp_led;
};

struct ADC_DATA adc_data;

//uint16_t adc_data[4];
volatile uint16_t temp;

int myMain(void){

#ifdef SOLAR_SENSOR_NODE
	#ifndef SOLAR_SENSOR_NODE_I2C_DISABLE

//		HAL_Delay(500);
		powerMeasSetup();
		HAL_Delay(100);

//		while(1){
//			temp = getBusVoltage();
//			HAL_Delay(1000);
//		}

	#endif
#else
	HAL_TIM_Base_Start_IT(&htim2);

	HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Start(&htim17);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

	Stepper motor_base(STEPS, AIN1_1_GPIO_Port, AIN1_1_Pin,
			AIN2_1_GPIO_Port, AIN2_1_Pin,
			BIN1_1_GPIO_Port, BIN1_1_Pin,
			BIN2_1_GPIO_Port, BIN2_1_Pin, &adc_data.mag_base, NON_INVERTED);
	motor_base.setSpeed(12);
	motor_base.setStepBound(BASE_STEP_RANGE);

	Stepper motor_led(STEPS, AIN1_2_GPIO_Port, AIN1_2_Pin,
				AIN2_2_GPIO_Port, AIN2_2_Pin,
				BIN1_2_GPIO_Port, BIN1_2_Pin,
				BIN2_2_GPIO_Port, BIN2_2_Pin,  &adc_data.mag_led, INVERTED);
	motor_led.setSpeed(16);
	motor_led.setStepBound(LED_STEP_RANGE);

	uint32_t counter = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adc_data, 4);
	HAL_Delay(100);

	while (1)
		  {
			counter += 1;
//			HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adc_data, 4);
//			HAL_Delay(100);

//			while(1){
//				motor_led.honeAxis();
//				motor_led.step(-STEPS_LED_RANGE);
//			}
//			while(1){
//				motor_base.honeAxis();
//				motor_base.step(-3078);
//				motor_base.step(-3078);
//			}
			startCal(0, 0.5, 0, 0.3, motor_base, motor_led);

			while(1){};

//			motor_led.step(-STEPS_LED_RANGE);
//			motor_led.step(STEPS_LED_RANGE);



//			if(adc_data.mag_led != 0){
////				while(1);
//				motor_led.step(50);
//			}else{
//				while(1);
//			}
			//
			//motor_base.step(-31011);
//			motor_base.step(-10);
//			motor_base.step(STEPS_LED_RANGE);
//			while(1){};
//			motor_base.step(-1);
//			if(adc_data.mag_led == 0){
//				temp = 0;
//			}
//			else{
//				temp=1;
//			}


//			motor_led.step(-STEPS);
//			HAL_Delay(500);
//			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
//			HAL_Delay(500);
//			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
//			HAL_Delay(500);
//			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
//			HAL_Delay(500);
//			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
		  }
#endif
}

//uint32_t test_cnt = 0;
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  // Check which version of the timer triggered this callback and toggle LED
//  if (htim == &htim17 )
//  {
//	  test_cnt += 1;
//	  if( (test_cnt % 1000000) == 0){
//		  HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
//
//	  }
//
//  }
//}




void Reset_BackupDomain( void )
{
  if ((LL_RCC_IsActiveFlag_PINRST() != FALSE) && (LL_RCC_IsActiveFlag_SFTRST() == FALSE))
  {
    HAL_PWR_EnableBkUpAccess(); /**< Enable access to the RTC registers */

    /**
     *  Write twice the value to flush the APB-AHB bridge
     *  This bit shall be written in the register before writing the next one
     */
    HAL_PWR_EnableBkUpAccess();

    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();
  }

  return;
}

void Reset_Device( void )
{
#if ( CFG_HW_RESET_BY_FW == 1 )
  Reset_BackupDomain();

  Reset_IPCC();
#endif

  return;
}

void Reset_IPCC( void )
{
  LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_IPCC);

  LL_C1_IPCC_ClearFlag_CHx(
                           IPCC,
                           LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
                               | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C2_IPCC_ClearFlag_CHx(
                           IPCC,
                           LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
                               | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C1_IPCC_DisableTransmitChannel(
                                    IPCC,
                                    LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
                                        | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C2_IPCC_DisableTransmitChannel(
                                    IPCC,
                                    LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
                                        | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C1_IPCC_DisableReceiveChannel(
                                   IPCC,
                                   LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
                                       | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C2_IPCC_DisableReceiveChannel(
                                   IPCC,
                                   LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
                                       | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  return;
}

void Init_Exti( void )
{
  /**< Disable all wakeup interrupt on CPU1  except IPCC(36), HSEM(38) */
  LL_EXTI_DisableIT_0_31(~0);
  LL_EXTI_DisableIT_32_63( (~0) & (~(LL_EXTI_LINE_36 | LL_EXTI_LINE_38)) );

  return;
}


void Config_HSE(void)
{
    OTP_ID0_t * p_otp;

  /**
   * Read HSE_Tuning from OTP
   */
  p_otp = (OTP_ID0_t *) OTP_Read(0);
  if (p_otp)
  {
    LL_RCC_HSE_SetCapacitorTuning(p_otp->hse_tuning);
  }

  return;
}

void initClk (void)
{
  RCC_OscInitTypeDef st_oscInit = {0};

  /* Get mutex */
  LL_HSEM_1StepLock(HSEM, 5);

  st_oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  st_oscInit.HSI48State     = RCC_HSI48_ON; /* Used by USB */
  HAL_RCC_OscConfig(&st_oscInit);
}


