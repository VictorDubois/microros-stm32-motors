/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM15|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);


void StartDefaultTask(void *argument)
{

	setup();


  // micro-ROS configuration

  rmw_uros_set_custom_transport(
    true,
    (void *) &huart2,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);



  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;


  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	  // Fails => probably because lack of RAM?
	  // Todo: regenerate with less pub/sub/history
	  // Then with no server
	  // Last resort: try to use CCMRAM??
      HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
      while(true)
      {
      HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
      	HAL_Delay(1000);
      	HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET); // Turn Off LED
      	HAL_Delay(1000);
      }

      printf("Error on default allocators (line %d)\n", __LINE__);
  }
	loop(&htim3, &htim15);


	// Never called!
  // micro-ROS app

  rcl_publisher_t publisher;
  std_msgs__msg__Int32 msg;
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;

  while(rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
  		{
  			HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
  			HAL_Delay(100);
  			HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET); // Turn Off LED
  			HAL_Delay(300);

  			}

  allocator = rcl_get_default_allocator();

  //create init_options
  rcl_ret_t ret = RCL_RET_OK;
  //rcl_ret_t ret =
		  rclc_support_init(&support, 0, NULL, &allocator);//@todo comprendre pourquoi ça fail
  if (ret != RCL_RET_OK)
  {
	  HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
	  while(true)
	        {
	        HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
	        	HAL_Delay(2000);
	        	HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET); // Turn Off LED
	        	HAL_Delay(500);
	        }
	//printf("Error publishing (line %d)\n", __LINE__);
  }
  // create node
  //ret =
		  rclc_node_init_default(&node, "cubemx_node", "", &support);
  if (ret != RCL_RET_OK)
  {
	  HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
	  while(true)
	        {
	        HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
	        	HAL_Delay(100);
	        	HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET); // Turn Off LED
	        	HAL_Delay(1000);
	        }
	//printf("Error publishing (line %d)\n", __LINE__);
  }
  // create publisher
  ret = rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "cubemx_publisher");
  if (ret != RCL_RET_OK)
  {
	  HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
	  while(true)
	        {
	        HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
	        	HAL_Delay(1000);
	        	HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET); // Turn Off LED
	        	HAL_Delay(100);
	        }
	//printf("Error publishing (line %d)\n", __LINE__);
  }

  msg.data = 0;
  //

  for(;;)
  {
    ret = rcl_publish(&publisher, &msg, NULL);
    if (ret != RCL_RET_OK)
    {
    	while(true)
    	      {
    	      HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
    	      	HAL_Delay(100);
    	      	HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET); // Turn Off LED
    	      	HAL_Delay(100);
    	      }
    	HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
      printf("Error publishing (line %d)\n", __LINE__);
    }

    HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED

    msg.data++;
    osDelay(10);
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM15) {

	  update_motor_board();


  	}

  /* USER CODE END Callback 1 */
}

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
    HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
    HAL_Delay(100);
    HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET); // Turn Off LED
    HAL_Delay(100);
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
