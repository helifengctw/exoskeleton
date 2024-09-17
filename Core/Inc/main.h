/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void Read_RTC_Time(void);
void Read_CAN_Single_Angle(void);
void Read_CAN_Multi_Angle(void);
void Read_CAN_Status(void);
void Read_CAN_PID(void);
void Read_ADC_Pressure(void);
void Read_ADC_fb_Force(void);
void Read_UART_ankle_angle(void);

void Speed_control(uint32_t _speed);
void Angle_control_increment(uint32_t _angle, uint16_t _time);
void Angle_control_destination_multi(uint32_t _angle, uint16_t _speed); 
void Angle_control_destination_single(uint8_t _dire, uint16_t _angle, uint16_t _speed); 
void Motor_stop(void);
void Write_current_point_as_zero(void);
void Clear_all_to_zero(void);

long map(long x, long in_min, long in_max, long out_min, long out_max);

void protection_check(void);
void send_msg_2_cp(void);
	
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
