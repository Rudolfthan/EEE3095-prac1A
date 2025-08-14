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
#include <stdint.h>
#include "stm32f0xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define and initialise the global varibales required
// Max iterations
#define MAX_ITER  100
#define LED_START GPIO_PIN_0
#define LED_END   GPIO_PIN_1
#define LED_PORT  GPIOB

// Test dimensions
const uint16_t image_dimensions[] = {128, 160, 192, 224, 256};
const uint8_t array_size  = sizeof(image_dimensions) / sizeof(image_dimensions[0]);

// Global variables
volatile uint32_t start_time, end_time,execution_time;

volatile uint32_t exec_time[5];      // Execution times
volatile uint64_t checksum[5];       // checksums


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);


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
  //int count =1;

  /* USER CODE BEGIN 2 */
  for (int i=0; i<array_size;i++){
	  int width = image_dimensions[i];
	  int height = width; //square

	  //TODO: Turn on LED 0 to signify the start of the operation
	  //GPIOB->ODR |= LED;// set PB0
      HAL_GPIO_WritePin(LED_PORT, LED_START , GPIO_PIN_SET);

	  //TODO: Record the start time
	  start_time = HAL_GetTick();

	  //TODO: Call the Mandelbrot Function and store the output in the checksum variable defined initially
	  checksum[i] = calculate_mandelbrot_double(width, height, MAX_ITER);

	  //TODO: Record the end time
	  end_time = HAL_GetTick();

	  //TODO: Calculate the execution time
	   exec_time[i] = end_time - start_time;  // Store per-image time

	  //TODO: Turn on LED 1 to signify the end of the operation
      HAL_GPIO_WritePin(LED_PORT, LED_START, GPIO_PIN_RESET); // clear LED_START
      HAL_GPIO_WritePin(LED_PORT, LED_END, GPIO_PIN_SET);// set LED_END


	  //TODO: Hold the LEDs on for a 1s delay
	  HAL_Delay(1000);

	  //TODO: Turn off the LEDs
      HAL_GPIO_WritePin(LED_PORT, LED_END, GPIO_PIN_RESET);// clear LEDs

  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Mandelbroat using variable type integers and fixed point arithmetic
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations) {
    uint64_t mandelbrot_sum = 0;
    const uint64_t scale = 1 << 16;
    const uint64_t scale_3_5 = 3.5 * scale;
    const uint64_t scale_2_5 = 2.5 * scale;
    const uint64_t scale_2_0 = 2.0 * scale;
    const uint64_t scale_1_0 = 1.0 * scale;
    const uint64_t escape_radius_sq = 4 * scale * scale;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Compute x0 and y0 in fixed-point
            uint64_t x0 = (x * scale_3_5) / width - scale_2_5;
            uint64_t y0 = (y * scale_2_0) / height - scale_1_0;

            uint64_t xi = 0;
            uint64_t yi = 0;
            uint64_t iteration = 0;

            while (iteration < max_iterations) {

                uint64_t xi_sq = (xi * xi) / scale;
                uint64_t yi_sq = (yi * yi) / scale;

                if (xi_sq + yi_sq > escape_radius_sq) break;
                uint64_t temp = xi_sq - yi_sq;
                yi = (2 * ((xi * yi) / scale)) + y0;
                xi = temp + x0;

                iteration++;
            }

            mandelbrot_sum += iteration;
        }
    }

    return mandelbrot_sum;
}

//TODO: Mandelbroat using variable type double
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations){
    uint64_t mandelbrot_sum = 0;
    //TODO: Complete the function implementation


    const double scale_3_5 = 3.5;
    const double scale_2_5 = 2.5;
    const double scale_2_0 = 2.0;
    const double scale_1_0 = 1.0;
    const double escape_radius_sq = 4.0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {

            double x0 = ((double)x * scale_3_5) / (double)width - scale_2_5;
            double y0 = ((double)y * scale_2_0) / (double)height - scale_1_0;

            double xi = 0.0;
            double yi = 0.0;
            uint64_t iteration = 0;

            while (iteration < max_iterations) {
                // xi² and yi²
                double xi_sq = xi * xi;
                double yi_sq = yi * yi;

                // Check escape condition
                if (xi_sq + yi_sq > escape_radius_sq) break;


                double temp = xi_sq - yi_sq;
                yi = 2.0 * xi * yi + y0;
                xi = temp + x0;

                iteration++;
            }

            mandelbrot_sum += iteration;
        }
    }

    return mandelbrot_sum;

    return mandelbrot_sum;
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
