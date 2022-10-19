/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "tusb.h"

#include "vl53l5cx_api.h"
#include "wave.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ASCII_ESC 27
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
VL53L5CX_Configuration TOF_Dev; /* Sensor configuration */
GestureRecognizer_RES_X__RES_Y__HISTORY_SIZE gesture_recognizer;
static int tof_ready_int;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

static uint8_t tof_init(void);
static void tof_results_print(VL53L5CX_ResultsData *TOF_Results);
static SensorMeasurement_RES_X__RES_Y tof_convert_measurement(
		VL53L5CX_ResultsData *measurement, uint32_t time_ms);
static void recognizer_hand_state_print(RecognizerResult *result);
static void recognizer_gestures_print(RecognizerResult *result);
static void hid_report_recognized_gestures(RecognizerResult *result);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile int button_pressed;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int status = 0;

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

	tusb_init();

	status = tof_init();
	if (status) {
		printf("tof init failed. code: %i\r\n", status);
		return status;
	}

	RecognizerParams recognizer_params = recognizer_params_default();
	SensorParams sensor_params = sensor_params_default_vl53l5cx();

	gesture_recognizer = gesture_recognizer_new(recognizer_params, sensor_params);

	// Move cursor to top left corner
	printf("%c[f", ASCII_ESC);
	// Clear screen
	printf("%c[2J", ASCII_ESC);

	printf("init done.\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		tud_task();

		if (button_pressed) {
			button_pressed = 0;
			// For debugging any actions that need an activation
			printf("Button pressed.\r\n");
		}

		if (tof_ready_int) {
			tof_ready_int = 0;

			VL53L5CX_ResultsData TOF_Results = { 0 };
			RecognizerResult recognizer_result = recognizer_result_default();

			vl53l5cx_get_ranging_data(&TOF_Dev, &TOF_Results);

			SensorMeasurement_RES_X__RES_Y converted_measurement =
					tof_convert_measurement(&TOF_Results, HAL_GetTick());

			gesture_recognizer_update(&gesture_recognizer,
					converted_measurement, &recognizer_result);

			hid_report_recognized_gestures(&recognizer_result);

			// Move cursor to top left corner
			//printf("%c[f", ASCII_ESC);
			// Clear screen
			//printf("%c[2J", ASCII_ESC);

			//tof_results_print(&TOF_Results);
			recognizer_hand_state_print(&recognizer_result);
			recognizer_gestures_print(&recognizer_result);
		}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TOF_RST_Pin|TOF_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|TOF_LPN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOF_RST_Pin TOF_PWR_EN_Pin */
  GPIO_InitStruct.Pin = TOF_RST_Pin|TOF_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_INT_Pin */
  GPIO_InitStruct.Pin = TOF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOF_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin TOF_LPN_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|TOF_LPN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
	return HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
}

static uint8_t tof_init(void) {
	uint8_t status = 0;
	uint8_t isAlive = 0;
	TOF_Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;

	status = Reset_Sensor(&TOF_Dev.platform);

	status = vl53l5cx_is_alive(&TOF_Dev, &isAlive);
	if (!isAlive || status) {
		printf("VL53L5CX not detected at requested address\r\n");
		return status;
	}

	/* (Mandatory) Init VL53L5CX sensor */
	status = vl53l5cx_init(&TOF_Dev);
	if (status) {
		printf("VL53L5CX ULD Loading failed\r\n");
		return status;
	}

	status = vl53l5cx_set_resolution(&TOF_Dev, VL53L5CX_RESOLUTION_8X8);
	if (status) {
		printf("VL53L5CX ULD set resolution failed\r\n");
		return status;
	}

	// max 60Hz for resolution 4x4, max 15Hz for 8x8
	status = vl53l5cx_set_ranging_frequency_hz(&TOF_Dev, 15);
	if (status) {
		printf("VL53L5CX ULD set ranging frequency failed\r\n");
		return status;
	}

	status = vl53l5cx_start_ranging(&TOF_Dev);
	if (status) {
		printf("VL53L5CX ULD Start ranging failed\r\n");
		return status;
	}

	return 0;
}

static void tof_results_print(VL53L5CX_ResultsData *TOF_Results) {
	printf("TOF Sensor:\r\n");
	printf("data no : %3u\r\n", TOF_Dev.streamcount);

	// Print status as matrix
	printf("Status:\r\n");
	for (int y = 0; y < RES_Y; y++) {
		for (int x = 0; x < RES_X; x++) {
			printf("[ %3u ] ",
					TOF_Results->target_status[VL53L5CX_NB_TARGET_PER_ZONE
							* ((y * RES_Y) + x)]);

		};
		printf("\r\n");
	}
	printf("\r\n\r\n");

	// print dist as matrix
	printf("Distance:\r\n");
	for (int y = 0; y < RES_Y; y++) {
		for (int x = 0; x < RES_X; x++) {

			printf("[ %4d ] ",
					TOF_Results->distance_mm[VL53L5CX_NB_TARGET_PER_ZONE
							* ((y * RES_Y) + x)]);

		};
		printf("\r\n");
	}
	printf("\r\n");
}

static SensorMeasurement_RES_X__RES_Y tof_convert_measurement(
		VL53L5CX_ResultsData *measurement, uint32_t time_ms) {
	SensorMeasurement_RES_X__RES_Y converted = sensor_measurement_invalid();

	for (size_t y = 0; y < RES_Y; y++) {
		for (size_t x = 0; x < RES_X; x++) {
			uint8_t status = measurement->target_status[(y * RES_Y) + x];

			// only use valid measurements (valid are status 5: confidence 100%, status 6, 9: confidence > 50%.
			// see um2884 - page 14 )
			if (status == 5 || status == 6 || status == 9) {
				converted.zone_dist[y][x] =
						measurement->distance_mm[VL53L5CX_NB_TARGET_PER_ZONE
								* ((y * RES_Y) + x)];
			}
		}
	}

	converted.time_ms = time_ms;

	return converted;
}

static void recognizer_hand_state_print(RecognizerResult *result) {
	if (result->hand_state.tag == HandFound) {
		/*
		// Print in sphercial coordinates

		printf("hand_r=%02d, hand_theta=%02d, hand_phi=%02d,",
				(int) roundf(result->hand_state.hand_found.hand_pos.r),
				(int) roundf(result->hand_state.hand_found.hand_pos.theta * 57.29),
				(int) roundf(result->hand_state.hand_found.hand_pos.phi * 57.29));
		 */

		// Print in cartesian coordinates
		CoordsCartesian hand_pos_cart = coords_cartesian_from_spherical(
				result->hand_state.hand_found.hand_pos);

		printf("t=%lu, hand_x=%04d, hand_y=%04d, hand_z=%04d\r\n",
				HAL_GetTick(),
				(int) roundf(hand_pos_cart.x),
				(int) roundf(hand_pos_cart.y),
				(int) roundf(hand_pos_cart.z));

	}
}

static void recognizer_gestures_print(RecognizerResult *result) {
	switch (result->gesture) {
	case GestureNone: {
		break;
	}
	case GestureStaticHold: {
		printf("Gesture detected: Static hold\r\n");

		break;
	}
	case GestureSwipeRight: {
		printf("Gesture detected: Swipe right\r\n");

		break;
	}
	case GestureSwipeLeft: {
		printf("Gesture detected: Swipe left\r\n");

		break;
	}
	case GestureSwipeUp: {
		printf("Gesture detected: Swipe up\r\n");

		break;
	}
	case GestureSwipeDown: {
		printf("Gesture detected: Swipe down\r\n");

		break;
	}
	default:
		break;
	}

	return;
}

// The loop calling this function must not block, or multiple keypresses are reported.
static void hid_report_recognized_gestures(RecognizerResult *result) {
	// use to avoid send multiple consecutive zero report for keyboard
	static bool has_key = false;

	if (tud_hid_n_ready(ITF_KEYBOARD)) {
		switch (result->gesture) {
		case GestureNone: {
			// send empty key report if previously a gesture was recognized
			if (has_key)
				tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, NULL);
			has_key = false;

			break;
		}
		case GestureStaticHold: {
			uint8_t keycode[6] = { 0 };
			keycode[0] = HID_KEY_SPACE;

			tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, keycode);
			has_key = true;
			break;
		}
		case GestureSwipeRight: {
			uint8_t keycode[6] = { 0 };
			keycode[0] = HID_KEY_ARROW_RIGHT;

			tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, keycode);
			has_key = true;

			break;
		}
		case GestureSwipeLeft: {
			uint8_t keycode[6] = { 0 };
			keycode[0] = HID_KEY_ARROW_LEFT;

			tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, keycode);
			has_key = true;

			break;
		}
		case GestureSwipeUp: {
			uint8_t keycode[6] = { 0 };
			keycode[0] = HID_KEY_ARROW_UP;

			tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, keycode);
			has_key = true;

			break;
		}
		case GestureSwipeDown: {
			uint8_t keycode[6] = { 0 };
			keycode[0] = HID_KEY_ARROW_DOWN;

			tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, keycode);
			has_key = true;

			break;
		}
		default:
			break;
		}
	}

	return;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == TOF_INT_Pin) {
		tof_ready_int++;
	} else if (GPIO_Pin == B1_Pin) {
		button_pressed = 1;
	}
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id,
		hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
// TODO not Implemented
	(void) itf;
	(void) report_id;
	(void) report_type;
	(void) buffer;
	(void) reqlen;

	return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id,
		hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
// This example doesn't use multiple report and report ID
	(void) itf;
	(void) report_id;
	(void) report_type;

// echo back anything we received from host
	tud_hid_report(0, buffer, bufsize);
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
	while (1) {
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
