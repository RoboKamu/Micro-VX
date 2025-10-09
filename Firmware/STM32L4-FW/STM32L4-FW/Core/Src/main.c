/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

#include "LIS2DW12.h"       /* Custom driver accelerometer */
#include "ssd1306.h"        /* oled i2c module */
#include "ssd1306_fonts.h"  /* oled fonts */
#include <stdarg.h>         /* varadic functions */
#include <string.h>         /* dbg include for uart print */
#include <stdio.h>

#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * taken from stm32l4xx_ll_adc.h for definition of vrefint calibraion
 * ( datasheet p. 29, reference manual p. 311 )
 * Used for calculating the VDDA voltage
 */

#define VREFINT_CAL_ADDR                    ((uint16_t*) (0x1FFF75AAUL)) /* Internal voltage reference, address of parameter VREFINT_CAL: VrefInt ADC raw data acquired at temperature 30 DegC (tolerance: +-5 DegC), Vref+ = 3.0 V (tolerance: +-10 mV). */
#define VREFINT_CAL_VALUE                   (*VREFINT_CAL_ADDR)         /* dereferenced to use in calculations */
#define VDDA_CHARAC                         3000                        /* VDDA voltage characteristic from datasheet */
#define BUFFER_LEN 							256 						/* total size of the dma buffer */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */

volatile uint8_t dma_ftf_flag = 0;
volatile uint8_t dma_htf_flag = 0;

static uint16_t samples[BUFFER_LEN];

#define N 				BUFFER_LEN					/* used for FFT array and looping */
#define SAMPLE_RATE		50000.0f					/* effective sampling freq (64 Mhz / 640/2) */

float fft_buf_in[N];
float fft_buf_out[N];
arm_rfft_fast_instance_f32 fft_handler;

volatile GPIO_PinState state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
static void oled_print(uint8_t x, uint8_t y, const SSD1306_Font_t Font,
		SSD1306_COLOR color, const char *s, ...);
static void run_audio_visualizer();
static void run_fft_visualizer();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == SWITCH_EXTI_Pin) {

		/* Update for new state */
		state = HAL_GPIO_ReadPin(SWITCH_EXTI_GPIO_Port, SWITCH_EXTI_Pin);

	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	if (hadc->Instance == ADC1) {

		dma_ftf_flag = 1;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	}

}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {

	if (hadc->Instance == ADC1) {

		dma_htf_flag = 1;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	}

}

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  /* initialize FFT */
  arm_rfft_fast_init_f32(&fft_handler, N);

	ssd1306_Init();

	/* calibrate and start ADC with interrupts */

	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
		Error_Handler();

	if (HAL_ADC_Start_DMA(&hadc1, samples, BUFFER_LEN) != HAL_OK)
		Error_Handler();

	if (HAL_TIM_Base_Start_IT(&htim15) != HAL_OK)
		Error_Handler();

	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		if (dma_htf_flag) {
			dma_htf_flag = 0;
			for (uint8_t i = 0; i < N/2; i++) {
				fft_buf_in[i] = (float)samples[i];
			}
		}

		if (dma_ftf_flag) {
			dma_ftf_flag = 0;
			for (uint16_t i = N/2; i < N; i++) {
				fft_buf_in[i] = (float)samples[i];
			}

			if (state == GPIO_PIN_SET) 	run_fft_visualizer();
			else 						run_audio_visualizer();
		}

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T15_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_1;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 2284;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x20B5546F;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 640-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 2-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SWITCH_EXTI_Pin */
  GPIO_InitStruct.Pin = SWITCH_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWITCH_EXTI_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief wrapper for printing string to OLED
 */
static void oled_print(uint8_t x, uint8_t y, const SSD1306_Font_t Font,
		SSD1306_COLOR color, const char *s, ...) {

	/*
	 * This is a varadic function that wraps the printing sequence to the OLED.
	 * Can then be used like printf after defining location, font and color.
	 * More on varadic functions here: https://man7.org/linux/man-pages/man3/stdarg.3.html
	 * More on vsnprintf here: https://man7.org/linux/man-pages/man3/vsnprintf.3.html
	 */

	int n = 0;
	char buffer[20];
	if (strlen(s) > 20)
		return;

	va_list args;

	va_start(args, s);
	n = vsnprintf(buffer, sizeof(buffer), s, args);
	va_end(args);

	if (n < 0)
		return;

	ssd1306_SetCursor(x, y);
	ssd1306_WriteString(buffer, Font, color);

}

/**
 * audio analyzer, prints half of the dma buffer (128 samples) on oled
 * Can use relative scaling for better display if values are above threshold
 */
static void run_audio_visualizer() {

	ssd1306_Fill(Black);

	float sum = 0.0f, max=1.0f;
	for (uint8_t i = 0; i < BUFFER_LEN/2; i++) {
		if (fft_buf_in[i] > max) max = fft_buf_in[i];
		sum += fft_buf_in[i];
	}

	float mean = sum/((float)BUFFER_LEN/2);
	const int threshold = 7;
	float scale = (max-mean) > threshold ? (15.0f)/(max-mean) : 15.0f / 2048.0f;

	for (uint8_t i = 0; i < BUFFER_LEN/2; i++) {
		/* only display first half that is already converted */
		float y = 31 - (fft_buf_in[i]-mean) * scale;
		if (y >= 64) y=63;
		if (y < 0) y = 0;
		ssd1306_DrawPixel(i, (uint8_t)(y+0.5), White);
	}

	ssd1306_UpdateScreen();
}

/**
 * @breif showing FFT visualizer on OLED
 *
 */
static void run_fft_visualizer() {

	ssd1306_Fill(Black);

	float sum = 0;
	for (uint16_t i = 0; i < N; i++) sum += fft_buf_in[i];
	float mean = (float)sum/(float)N;

	for (uint16_t i = 0; i < N; i++) {
		float hann = 0.5 * (1 - cos(2*PI*i/(N-1))); /* hann window multiplier */
		float val = (float)fft_buf_in[i] - mean;
		fft_buf_in[i] = hann * val;
	}

	arm_rfft_fast_f32(&fft_handler, fft_buf_in, fft_buf_out, 0);

	float max = 1.0f;
	float mags[N/2];
	uint8_t bin_index = 0; /* keep track of the index for fft bin number */

	for (uint16_t i = 0; i < N; i+=2) {
		/* calculate the magnitude of the bin */
		float x = fft_buf_out[i];
		float y = fft_buf_out[i+1];
		mags[bin_index] = sqrtf(x*x + y*y);

		if (max < mags[bin_index]) {
			max = mags[bin_index];
//			peakHz = (uint16_t) ((float)bin_index * SAMPLE_RATE / ((float)N));
		}

		bin_index++;
	}

	/* display the transform */
	for (uint8_t i = 0; i < N/2; i++) {
		float mag = mags[i];
		if (mag < 100) mag = 0;
		/* scale magnitude to oled, default divisor = 1 / (64 / (128*2048)) */
		mag = mag / max;
		mag = mag * 63.0;
		mag = mag >= 64.0 ? 63 : mag;
		mag = mag < 0 ? 0 : mag;
		ssd1306_DrawPixel(i, 63-(uint8_t)mag, White);

	}

//	oled_print(0, 32, Font_7x10, White, "freq= %u", peakHz);

	ssd1306_UpdateScreen();
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
#ifdef USE_FULL_ASSERT
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
