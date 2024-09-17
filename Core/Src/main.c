/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "crc_1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRESS_MIN	(20)
#define PRESS_MAX	(6000)
#define VOLTAGE_MIN (100)
#define VOLTAGE_MAX (3300)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// motor
int64_t multi_angle_value = 0, slack_point = 0, force_point = 0, peak_point = 0;
int64_t min_protect_point = 0, max_protect_point = 0;
uint32_t single_angle_value = 0;
uint16_t encoder_position_value = 0, motor_speed_value = 0, motor_current_value = 0;
uint8_t motor_temperatrue = 0;
uint8_t pos_P = 0, pos_I = 0, spd_P = 0, spd_I = 0, trq_P = 0, trq_I = 0;

uint8_t run_mode = 1, command = 0, param_1 = 0, param_2 = 0;
uint8_t flag_trigger_once = 0x01, flag_mode_2_suspend = 0x00;
uint8_t flag_mode_3_suspend = 0x00, mode_3_hit_count = 0, mode_3_hit_once = 0;
uint16_t mode_2_wait_start_time = 0, mode_3_wait_start_time = 0;
uint32_t mode_3_last_dist = 0;
static uint8_t motorId = 1, Ratio = 9;

// pressure
uint16_t press_value_AD = 0;
long PRESS_AO = 0, press_valve = 4500;
long VOLTAGE_AO = 0;

uint16_t fb_force_value_AD = 0;
long FORCE_AO = 0, force_valve = 4500;


// ankle angle 
uint8_t ankle_angle_read_cmd_high[8] = {0x00, 0x03, 0x00, 0xD5, 0x00, 0x02, 0xD4, 0x22};
uint8_t ankle_angle_read_cmd_low[8] = {0x00, 0x03, 0x00, 0xD6, 0x00, 0x01, 0x23, 0x64};
int32_t ankle_angle_value = 0, last_ankle_angle_value = 0;
uint8_t ankle_angle_loops = 0;
const int32_t zero_ankle_angle_value = 190, ankle_angle_threshold = 200;
int64_t aat_point = 0, anti_aat_point = 0;

// control
uint8_t if_show = 0;
long sys_time_ms = 0;
uint8_t system_self_inspect_status = 6;
const uint8_t read_data_len = 10, write_data_len = 8;
uint16_t ctl_params[read_data_len / 2];
uint8_t data2sent[write_data_len];
static uint32_t txMailBox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
	printf("Init success!!!\r\n");
	while (1)
	{
		Read_CAN_Multi_Angle();
		Read_CAN_Single_Angle();
		Read_CAN_Status();
//		Read_CAN_PID();
		Read_RTC_Time();
		Read_ADC_Pressure();
		Read_ADC_fb_Force();
		Read_UART_ankle_angle();
//		protection_check();
		
		if(system_self_inspect_status == 0) {
			if (PRESS_AO >= press_valve) {
				system_self_inspect_status = 1;
			}
		} else if(system_self_inspect_status == 1) {
			HAL_Delay(300);
			if (ankle_angle_value < ankle_angle_threshold) {
				command = 4;
				param_1 = 0;
				param_2 = 8;
				flag_trigger_once = 1;
			} else if (ankle_angle_value >= ankle_angle_threshold) {
				aat_point = multi_angle_value;
				system_self_inspect_status = 2;
				command = 5;
				param_1 = 0;
				param_2 = 80;
				flag_trigger_once = 1;
			} 
		} else if (system_self_inspect_status == 2) {
			HAL_Delay(300);
			if (ankle_angle_value < ankle_angle_threshold) {
				system_self_inspect_status = 3;
			}
		} else if (system_self_inspect_status == 3) {
			HAL_Delay(300);
			if (ankle_angle_value < ankle_angle_threshold) {
				command = 5;
				param_1 = 0;
				param_2 = 8;
				flag_trigger_once = 1;
			} else if (ankle_angle_value >= ankle_angle_threshold) {
				anti_aat_point = multi_angle_value;
				system_self_inspect_status = 4;
				slack_point = (aat_point + anti_aat_point) / 2;
			}
		} else if (system_self_inspect_status == 4) {
			Angle_control_destination_multi(slack_point, 360);
			system_self_inspect_status = 5;
		} else if (system_self_inspect_status == 5) {
			HAL_Delay(1000);
			run_mode = 3;
			system_self_inspect_status = 6;
		}
		
		if (run_mode == 0) { // 强制停止模式
			Motor_stop();
			run_mode = 1;
		} else if (run_mode == 1) { // 待机模式
			if (flag_trigger_once == 0x01) { // 执行命令
				if (command == 0) { // 0-停止
					Motor_stop();
					flag_trigger_once = 0x00;
				} else if (command == 1) { // 1-切换模式
					run_mode = param_2;
					flag_trigger_once = 0x00;
				} else if (command == 2) { // 2-匀速转动（param: 速度）
					uint16_t speed_dps = (param_1 << 8) + param_2;
					if (speed_dps <= 0) Speed_control(30);
					else Speed_control(speed_dps);
					flag_trigger_once = 0x00;
				} else if (command == 3) { // 3-归零
					Angle_control_destination_single(0, 0, 180);
					flag_trigger_once = 0x00;
				} else if (command == 4) { // 4-前进一定角度（param: 角度）
					uint16_t cmd_angle = (param_1 << 8) + param_2;
					if (cmd_angle <= 0) Angle_control_destination_multi(multi_angle_value + 90, 180);
					else Angle_control_destination_multi(multi_angle_value + cmd_angle, 180); // pos_multi, speed
					flag_trigger_once = 0x00;
				} else if (command == 5) { // 5-后退一定角度（param: 角度）
					uint16_t cmd_angle = (param_1 << 8) + param_2;
					if (cmd_angle <= 0) Angle_control_destination_multi(multi_angle_value - 90, 180);
					else Angle_control_destination_multi(multi_angle_value - cmd_angle, 180); // pos_multi, speed
					flag_trigger_once = 0x00;
				} else if (command == 6) { // 6-读取或跑到特定角度（param_1: 读或跑, parma_2: 角度编码）
					if (param_1 == 0) {
						switch (param_2) {
							case 0 : {
								slack_point = multi_angle_value;
								break;
							}
							case 1 : {
								force_point = multi_angle_value;
								break;
							}
							case 2 : {
								peak_point = multi_angle_value;
								break;
							}
							case 3 : {
								min_protect_point = multi_angle_value;
								break;
							}
							case 4 : {
								max_protect_point = multi_angle_value;
								break;
							}
						}
						flag_trigger_once = 0x00;
					} else if (param_1 == 1) {
						uint32_t cmd_point = 0;
						switch (param_2) {
							case 0 : {
								cmd_point = slack_point;
								break;
							}
							case 1 : {
								cmd_point = force_point;
								break;
							}
							case 2 : {
								cmd_point = peak_point;
								break;
							}
							case 3 : {
								cmd_point = min_protect_point;
								break;
							}
							case 4 : {
								cmd_point = max_protect_point;
								break;
							}
							default : cmd_point = multi_angle_value;
						}
						Angle_control_destination_multi(cmd_point, 180);
						while (1) {
							HAL_Delay(50);
							Read_CAN_Multi_Angle();
							if (multi_angle_value - cmd_point < 2 && multi_angle_value - cmd_point > -2) break;
						}
					}
				} else if (command == 7) { // 7-正常跑一次流程
					Angle_control_destination_multi(force_point, 180);
					while (1) {
						HAL_Delay(50);
						Read_CAN_Multi_Angle();
						if (multi_angle_value - force_point < 2 && multi_angle_value - force_point > -2) break;
					}
					Angle_control_destination_multi(peak_point, 180);
					while (1) {
						HAL_Delay(50);
						Read_CAN_Multi_Angle();
						if (multi_angle_value - peak_point < 2 && multi_angle_value - peak_point > -2) break;
					}
					Angle_control_destination_multi(slack_point, 180);
					while (1) {
						HAL_Delay(50);
						Read_CAN_Multi_Angle();
						if (multi_angle_value - slack_point < 2 && multi_angle_value - slack_point > -2) break;
					}
					flag_trigger_once = 0x00;
				} else if (command == 8) { // 8-清楚所有角度数据
					Clear_all_to_zero();
					flag_trigger_once = 0x00;
				}
			}
		} else if (run_mode == 2) {
			if (flag_mode_2_suspend == 0x00) {
				Angle_control_increment(8, 1); //d, run_time
				flag_mode_2_suspend = 0x01;
				mode_2_wait_start_time = sys_time_ms;
			} else if (flag_mode_2_suspend == 0x01 && sys_time_ms - mode_2_wait_start_time > 2000) { //keep suspending for more than 3s
				flag_mode_2_suspend = 0x00;
			}
		} else if (run_mode == 3) {
			if (flag_mode_3_suspend == 0x00) {
				Angle_control_increment(8, 1); //d, run_time
				flag_mode_3_suspend = 0x01;
				mode_3_hit_once = 0;
				mode_3_wait_start_time = sys_time_ms;
			} else if (flag_mode_3_suspend == 0x01) {
				if (sys_time_ms - mode_3_wait_start_time <= 3000) { //keep suspending for more than 3s
					 if (mode_3_hit_once != 1) {
						 if (PRESS_AO >= press_valve) {
							 switch (mode_3_hit_count){
								 case 0: {
									 force_point = multi_angle_value;
									 mode_3_hit_count++;
									 break;
								 }
								 case 1: {
									 if (multi_angle_value - force_point >= 200) {
										 peak_point = multi_angle_value;
										 mode_3_hit_count++;
									 }
									 break;
								 }
								 case 2: {
									 run_mode = 1;
									 command = 7;
									 flag_trigger_once = 1;
									 mode_3_hit_count = 0;
								 }
							 }
							 flag_mode_3_suspend = 0;
							 mode_3_hit_once = 1;
						 }
					 }
				} else flag_mode_3_suspend = 0x00;
			}
		}else if (run_mode == 4) {
			if (PRESS_AO >= press_valve) {
				HAL_Delay(10);
				Angle_control_destination_multi(force_point, 360);
				while (1) {
					Read_CAN_Multi_Angle();
					HAL_Delay(10);
					if (multi_angle_value - force_point < 2 && multi_angle_value - force_point > -2) break;
				}
				Angle_control_destination_multi(peak_point, 270);
				while (1) {
					HAL_Delay(50);
					Read_CAN_Multi_Angle();
					if (multi_angle_value - peak_point < 2 && multi_angle_value - peak_point > -2) break;
				}
				Angle_control_destination_multi(slack_point, 720);
				while (1) {
					HAL_Delay(10);
					Read_CAN_Multi_Angle();
					if (multi_angle_value - slack_point < 2 && multi_angle_value - slack_point > -2) break;
				}
			}
		}
		
		
		if (if_show) {
			printf("t[%08ld], mode[%02u], ma-sa-spd[%lld:%d:%u], p-f[%6ld:%6ld]\r\n", 
				sys_time_ms, run_mode, multi_angle_value, 
				ankle_angle_value, motor_speed_value, PRESS_AO, FORCE_AO);
//			printf("time[%u:%u], mode[%u], crc_r[%d]\r\n", 
//			RTC_Time[1], RTC_Time[2], run_mode, crc_16_result);
//			printf("%ld\r\n", FORCE_AO);
		}
		send_msg_2_cp();

	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRxHeader, canRxData) != HAL_OK) {
	Error_Handler();    /* Reception Error */
	}
	if ((canRxHeader.StdId == (DEVICE_STD_ID + motorId)) && (canRxHeader.DLC == 8)) {
		if (canRxData[0] == 0x92) {
			multi_angle_value = 0;
			for (int i = 7; i >= 1; i--) {
				multi_angle_value = multi_angle_value << 8;
				multi_angle_value |= canRxData[i];
			}
			multi_angle_value /= Ratio * 100;
		} else if (canRxData[0] == 0x94) {
			for (int i = 7; i >= 4; i--) {
				single_angle_value = single_angle_value << 8;
				single_angle_value |= canRxData[i];
			}
			single_angle_value /= Ratio * 100;
		} else if (canRxData[0] == 0x30) {
			pos_P = canRxData[2]; 
			pos_I = canRxData[3]; 
			spd_P = canRxData[4]; 
			spd_I = canRxData[5]; 
			trq_P = canRxData[6]; 
			trq_I = canRxData[7];
		} else if ( canRxData[0] == 0x9C || (canRxData[0] >= 0xA0 && canRxData[0] <= 0xA8) ) {
			encoder_position_value = (canRxData[7] << 8) + canRxData[6];
			motor_speed_value = ((canRxData[5] << 8) + canRxData[4]) / Ratio;
			motor_current_value = (canRxData[3] << 8) + canRxData[2];
			motor_temperatrue = canRxData[1];
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART1) {
		command = *(cmd_1 + 0);
		param_1 = *(cmd_1 + 1);
		param_2 = *(cmd_1 + 2);
		flag_trigger_once = 0x01;
		printf("receive message: %d:%d:%d\r\n", command, param_1, param_2);
		
		HAL_StatusTypeDef return_status;
		return_status = HAL_UART_Receive_IT(&huart1, cmd_1, 3);
		if (return_status == HAL_BUSY) {
			__HAL_UART_CLEAR_OREFLAG(&huart1);
			huart1.RxState = HAL_UART_STATE_READY;
			huart1.Lock = HAL_UNLOCKED;
			printf("ORE!!!");
			return_status = HAL_UART_Receive_IT(&huart1, cmd_1, 3);
		}
		
	} else if(huart->Instance == USART2) {
		uint16_t crc_16_result = (*(cmd_2 - 2 + read_data_len) << 8) + *(cmd_2 - 1 + read_data_len);
		if (Crc16(cmd_2, read_data_len-2) == crc_16_result) {
			printf("receive control message: ");
			for (int i = 0; i < (read_data_len / 2); i++) {
				ctl_params[i] = *(cmd_2+i*2+1) + (*(cmd_2+i*2) << 8);
				printf("%d:", ctl_params[i]);
			}
			printf("\r\n");
		} else {
			printf("CONTROL MESSAGE READ ERRO!!!\r\n");
		}
		HAL_UART_Receive_IT(&huart2, cmd_2, read_data_len);
	} else if(huart->Instance == USART3) {
		last_ankle_angle_value = ankle_angle_value;
		uint16_t crc_16_result = (*(cmd_3 + 8) << 8) + *(cmd_3 + 7);
		if (Crc16(cmd_3, 9-2) == crc_16_result) {
			ankle_angle_value = (*(cmd_3 + 3) << 24) + 
				(*(cmd_3 + 4) << 16) + (*(cmd_3 + 5) << 8) + *(cmd_3 + 6);
			ankle_angle_value *= 360;
			ankle_angle_value /= 262144;
			if (last_ankle_angle_value%360 >= 270 && ankle_angle_value <= 90) {
				ankle_angle_loops += 1;
			}
			ankle_angle_value += ankle_angle_loops * 360;
		} else {
			printf("ANKLE ANGLE READ ERRO!!!\r\n");
		}
		HAL_UART_Receive_IT(&huart3, cmd_3, 9);
	}
}

void Read_RTC_Time(void) {
	sys_time_ms = HAL_GetTick();
}

void Read_CAN_Single_Angle(void) {
	canTxHeader.StdId = DEVICE_STD_ID + motorId;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
	
	canTxData[0] = 0x94;
	canTxData[1] = 0x00;
	canTxData[2] = 0x00;
	canTxData[3] = 0x00;
	canTxData[4] = 0x00;
	canTxData[5] = 0x00;
	canTxData[6] = 0x00;
	canTxData[7] = 0x00;
	
	HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &txMailBox);
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3);
}

void Read_CAN_Multi_Angle(void) {
	canTxHeader.StdId = DEVICE_STD_ID + motorId;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
	
	canTxData[0] = 0x92;
	canTxData[1] = 0x00;
	canTxData[2] = 0x00;
	canTxData[3] = 0x00;
	canTxData[4] = 0x00;
	canTxData[5] = 0x00;
	canTxData[6] = 0x00;
	canTxData[7] = 0x00;
	
	HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &txMailBox);
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3);
}

void Read_CAN_Status(void) {
	canTxHeader.StdId = DEVICE_STD_ID + motorId;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
	
	canTxData[0] = 0x9C;
	canTxData[1] = 0x00;
	canTxData[2] = 0x00;
	canTxData[3] = 0x00;
	canTxData[4] = 0x00;
	canTxData[5] = 0x00;
	canTxData[6] = 0x00;
	canTxData[7] = 0x00;
	
	HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &txMailBox);
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3);
}

void Read_CAN_PID(void) {
	canTxHeader.StdId = DEVICE_STD_ID + motorId;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
	
	canTxData[0] = 0x30;
	canTxData[1] = 0x00;
	canTxData[2] = 0x00;
	canTxData[3] = 0x00;
	canTxData[4] = 0x00;
	canTxData[5] = 0x00;
	canTxData[6] = 0x00;
	canTxData[7] = 0x00;
	
	HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &txMailBox);
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3);
}

void Read_ADC_Pressure(void) {
	press_value_AD = get_adc_average(hadc1, 3);	//3次平均值
	VOLTAGE_AO = map(press_value_AD, 0, 4095, 0, 3300); //12位分辨率，3.3V
	if(VOLTAGE_AO < VOLTAGE_MIN) {
		PRESS_AO = 0;
	} else if(VOLTAGE_AO > VOLTAGE_MAX) {
		PRESS_AO = PRESS_MAX;
	} else {
		PRESS_AO = map(VOLTAGE_AO, VOLTAGE_MIN, VOLTAGE_MAX, PRESS_MIN, PRESS_MAX);
	}
}

void Read_ADC_fb_Force(void) {
	fb_force_value_AD = get_adc_average(hadc2, 3);	//3次平均值
	VOLTAGE_AO = map(fb_force_value_AD, 0, 4095, 0, 3300); //12位分辨率，3.3V
	if(VOLTAGE_AO < VOLTAGE_MIN) {
		FORCE_AO = 0;
	} else if(VOLTAGE_AO > VOLTAGE_MAX) {
		FORCE_AO = PRESS_MAX;
	} else {
		FORCE_AO = map(VOLTAGE_AO, VOLTAGE_MIN, VOLTAGE_MAX, PRESS_MIN, PRESS_MAX);
	}
}

void Read_UART_ankle_angle(void) {
	HAL_UART_Transmit(&huart3, ankle_angle_read_cmd_high, 8, 10);
//	HAL_Delay(1);
//	HAL_UART_Transmit(&huart3, ankle_angle_read_cmd_low, 8, 10);
}

void Speed_control(uint32_t _speed) {
	canTxHeader.StdId = DEVICE_STD_ID + motorId;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
	
	_speed *= Ratio * 100;
	canTxData[0] = 0xA2;
	canTxData[1] = 0x00;
	canTxData[2] = 0x00;
	canTxData[3] = 0x00;
	canTxData[4] = *((uint8_t *)&_speed + 0);
	canTxData[5] = *((uint8_t *)&_speed + 1);
	canTxData[6] = *((uint8_t *)&_speed + 2);
	canTxData[7] = *((uint8_t *)&_speed + 3);
	
	if (if_show) {
		printf(" sending message: [0xA2] ==> ");
	}
	
	HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &txMailBox);
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3);
	if (if_show) {
		printf(" success \r\n");
	}
}

void Angle_control_increment(uint32_t _angle, uint16_t _time) {
	canTxHeader.StdId = DEVICE_STD_ID + motorId;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
	
	uint16_t _speed = _angle / _time * 9;
	_angle *= Ratio * 100;
	canTxData[0] = 0xA8;
	canTxData[1] = 0x00;
	canTxData[2] = *((uint8_t *)&_speed + 0); // speed limit:1 rps
	canTxData[3] = *((uint8_t *)&_speed + 1);
	canTxData[4] = *((uint8_t *)&_angle + 0);
	canTxData[5] = *((uint8_t *)&_angle+ 1);
	canTxData[6] = *((uint8_t *)&_angle + 2);
	canTxData[7] = *((uint8_t *)&_angle + 3);
	
	if (if_show) {
		printf(" sending message: [0xA8] ==> ");
	}
	
	HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &txMailBox);
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3);
	if (if_show) {
		printf(" success \r\n");
	}
}

void Angle_control_destination_multi(uint32_t _angle, uint16_t _speed) {
	canTxHeader.StdId = DEVICE_STD_ID + motorId;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
		
	_speed *= Ratio;
	_angle *= Ratio * 100;
	canTxData[0] = 0xA4;
	canTxData[1] = 0x00;
	canTxData[2] = *((uint8_t *)&_speed + 0);
	canTxData[3] = *((uint8_t *)&_speed + 1);
	canTxData[4] = *((uint8_t *)&_angle + 0);
	canTxData[5] = *((uint8_t *)&_angle + 1);
	canTxData[6] = *((uint8_t *)&_angle + 2);
	canTxData[7] = *((uint8_t *)&_angle + 3);
	
	if (if_show) {
		printf(" sending message: [0xA4] ==> ");
	}
	HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &txMailBox);
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3);
	if (if_show) {
		printf(" success \r\n");
	}
}

void Angle_control_destination_single(uint8_t _dire, uint16_t _angle, uint16_t _speed) {
	canTxHeader.StdId = DEVICE_STD_ID + motorId;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
		
	_speed *= Ratio;
	_angle *= Ratio * 100;
	canTxData[0] = 0xA6;
	canTxData[1] = _dire; // direction : 0x00=>clockwise, 0x01=>anti
	canTxData[2] = *((uint8_t *)&_speed + 0);
	canTxData[3] = *((uint8_t *)&_speed + 1);
	canTxData[4] = *((uint8_t *)&_angle + 0);
	canTxData[5] = *((uint8_t *)&_angle + 1);
	canTxData[6] = 0x00;
	canTxData[7] = 0x00;
	
	if (if_show) {
		printf(" sending message: [0xA6] ==> ");
	}
	
	HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &txMailBox);
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3);
	if (if_show) {
		printf(" success \r\n");
	}
}

void Motor_stop(void) {
	canTxHeader.StdId = DEVICE_STD_ID + motorId;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
	
	canTxData[0] = 0x81;
	canTxData[1] = 0x00;
	canTxData[2] = 0x00;
	canTxData[3] = 0x00;
	canTxData[4] = 0x00;
	canTxData[5] = 0x00;
	canTxData[6] = 0x00;
	canTxData[7] = 0x00;
	
	if (if_show) {
		printf(" sending message: [0x81] ==> ");
	}
	
	HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &txMailBox);
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3);
	if (if_show) {
		printf(" success \r\n");
	}
}

void Write_current_point_as_zero(void) {
	canTxHeader.StdId = DEVICE_STD_ID + motorId;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
	
	canTxData[0] = 0x19;
	canTxData[1] = 0x00;
	canTxData[2] = 0x00;
	canTxData[3] = 0x00;
	canTxData[4] = 0x00;
	canTxData[5] = 0x00;
	canTxData[6] = 0x00;
	canTxData[7] = 0x00;
	
	if (if_show) {
		printf(" sending message: [0x19] ==> ");
	}
	
	HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &txMailBox);
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3);
	if (if_show) {
		printf(" success \r\n");
	}
}

void Clear_all_to_zero(void) {
	canTxHeader.StdId = DEVICE_STD_ID + motorId;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
	
	canTxData[0] = 0x95;
	canTxData[1] = 0x00;
	canTxData[2] = 0x00;
	canTxData[3] = 0x00;
	canTxData[4] = 0x00;
	canTxData[5] = 0x00;
	canTxData[6] = 0x00;
	canTxData[7] = 0x00;
	
	if (if_show) {
		printf(" sending message: [0x95] ==> ");
	}
	
	HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &txMailBox);
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3);
	if (if_show) {
		printf(" success \r\n");
	}
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void protection_check(void) {
	if (ankle_angle_value >= 100 && ankle_angle_value <= 350) {
		run_mode = 0;
		if (if_show) {
			printf("danger!!!, stop");
		}
	}
}

void send_msg_2_cp(void) {
	data2sent[0] = 0xFF & (PRESS_AO >> 8);
	data2sent[1] = 0xFF & PRESS_AO;
	data2sent[2] = 0xFF & (FORCE_AO >> 8);
	data2sent[3] = 0xFF & FORCE_AO;
	data2sent[4] = 0xFF & (ankle_angle_value >> 8);
	data2sent[5] = 0xFF & ankle_angle_value;
	
	uint16_t crc_16_result = Crc16(data2sent, write_data_len-2);
	data2sent[write_data_len-1] = crc_16_result & 0xFF;
	data2sent[write_data_len-2] = (crc_16_result >> 8) & 0xFF;
	HAL_UART_Transmit(&huart2, data2sent, write_data_len, 10);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
