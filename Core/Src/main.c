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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "stm32f0xx.h"
#include <stdlib.h>
#include <time.h>
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
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
// TODO: Define input variables
typedef enum {
    MODE_OFF = 0,
    MODE_1_BACK_FORTH = 1,
    MODE_2_INVERSE_BACK_FORTH = 2,
    MODE_3_SPARKLE = 3
} LED_Mode;

// Global variables
LED_Mode current_mode = MODE_OFF;
int current_led_index = 0;
int direction = 1; // 1 for forward, -1 for backward
int fast_mode = 0; // 0 for 1 second, 1 for 0.5 second

// Mode 3 (Sparkle) variables - FIXED
uint8_t sparkle_pattern = 0;
int sparkle_state = 0; // 0 = show pattern, 1 = turning off LEDs
uint32_t sparkle_delay_counter = 0;
uint32_t sparkle_target_delay = 0;
int leds_to_turn_off[8];
int sparkle_led_count = 0;  // FIXED: Store the count of LEDs that were on
int turn_off_index = 0;
uint32_t turn_off_timer = 0;
uint32_t turn_off_target_delay = 0;  // FIXED: Store the target delay for turning off

// Button debounce variables
uint32_t last_button_press[4] = {0, 0, 0, 0};
uint32_t debounce_delay = 200; // 200ms debounce

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
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
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  // TODO: Start timer TIM16
  // Start the timer in interrupt mode
  HAL_TIM_Base_Start_IT(&htim16);

  // Initialize random seed
  srand(HAL_GetTick());

  // Ensure all LEDs are off at startup using LL functions
  LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
  LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
  LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
  LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // TODO: Check pushbuttons to change timer delay
    // Handle Button0 press to toggle delay timing
    static uint32_t last_button0_press = 0;
    if (LL_GPIO_IsInputPinSet(Button0_GPIO_Port, Button0_Pin) == 0) {  // Button pressed (LOW)
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_button0_press > debounce_delay) {
            last_button0_press = current_time;
            fast_mode = !fast_mode;
            if (fast_mode) {
                // Set ARR for 0.5 second
                __HAL_TIM_SET_AUTORELOAD(&htim16, 499);
            } else {
                // Set ARR for 1 second
                __HAL_TIM_SET_AUTORELOAD(&htim16, 999);
            }
        }
    }

    HAL_Delay(10); // Small delay to prevent excessive polling

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */
  NVIC_EnableIRQ(TIM16_IRQn);
  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  /**/
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Function prototypes
void Set_LED_Pattern(uint8_t pattern);
void Handle_Mode_1(void);
void Handle_Mode_2(void);
void Handle_Mode_3(void);
uint8_t Generate_Random_Pattern(void);
uint32_t Generate_Random_Delay(uint32_t min_ms, uint32_t max_ms);
int Is_Button_Pressed_Debounced(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, int button_index);

// Timer interrupt handler
void TIM16_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim16);
}

// Timer interrupt callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM16) {
        // Check for mode change button presses (with debouncing)
        // Note: Using Button1_Pin, Button2_Pin, Button3_Pin instead of GPIO_PIN_1, etc.
        if (Is_Button_Pressed_Debounced(Button1_GPIO_Port, Button1_Pin, 1)) {
            current_mode = MODE_1_BACK_FORTH;
            current_led_index = 0;
            direction = 1;
        }

        if (Is_Button_Pressed_Debounced(Button2_GPIO_Port, Button2_Pin, 2)) {
            current_mode = MODE_2_INVERSE_BACK_FORTH;
            current_led_index = 0;
            direction = 1;
        }

        if (Is_Button_Pressed_Debounced(Button3_GPIO_Port, Button3_Pin, 3)) {
            current_mode = MODE_3_SPARKLE;
            sparkle_state = 0;
            sparkle_delay_counter = 0;
            turn_off_timer = 0;
        }

        // Execute current mode
        switch (current_mode) {
            case MODE_1_BACK_FORTH:
                Handle_Mode_1();
                break;
            case MODE_2_INVERSE_BACK_FORTH:
                Handle_Mode_2();
                break;
            case MODE_3_SPARKLE:
                Handle_Mode_3();
                break;
            default:
                Set_LED_Pattern(0x00); // All LEDs off
                break;
        }
    }
}

// FIXED: Set LED pattern function using LL functions
void Set_LED_Pattern(uint8_t pattern)
{
    // LED0 (bit 0)
    if (pattern & 0x01) {
        LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
    } else {
        LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
    }

    // LED1 (bit 1)
    if (pattern & 0x02) {
        LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
    } else {
        LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
    }

    // LED2 (bit 2)
    if (pattern & 0x04) {
        LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
    } else {
        LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
    }

    // LED3 (bit 3)
    if (pattern & 0x08) {
        LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin);
    } else {
        LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
    }

    // LED4 (bit 4)
    if (pattern & 0x10) {
        LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin);
    } else {
        LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
    }

    // LED5 (bit 5)
    if (pattern & 0x20) {
        LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin);
    } else {
        LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
    }

    // LED6 (bit 6)
    if (pattern & 0x40) {
        LL_GPIO_SetOutputPin(LED6_GPIO_Port, LED6_Pin);
    } else {
        LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
    }

    // LED7 (bit 7)
    if (pattern & 0x80) {
        LL_GPIO_SetOutputPin(LED7_GPIO_Port, LED7_Pin);
    } else {
        LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
    }
}

// Mode 1: Back and forth single LED
void Handle_Mode_1(void)
{
    Set_LED_Pattern(1 << current_led_index);

    current_led_index += direction;

    if (current_led_index >= 7) {
        direction = -1;
    } else if (current_led_index <= 0) {
        direction = 1;
    }
}

// Mode 2: Inverse back and forth (all LEDs on except one)
void Handle_Mode_2(void)
{
    Set_LED_Pattern(~(1 << current_led_index));

    current_led_index += direction;

    if (current_led_index >= 7) {
        direction = -1;
    } else if (current_led_index <= 0) {
        direction = 1;
    }
}

// FIXED Mode 3: Sparkle mode
void Handle_Mode_3(void)
{
    if (sparkle_state == 0) {
        // Show random pattern
        if (sparkle_delay_counter == 0) {
            sparkle_pattern = Generate_Random_Pattern();
            Set_LED_Pattern(sparkle_pattern);

            // Calculate target delay in timer ticks
            uint32_t delay_ms = Generate_Random_Delay(100, 1500);
            uint32_t timer_period_ms = fast_mode ? 500 : 1000;
            sparkle_target_delay = delay_ms / timer_period_ms;
            if (sparkle_target_delay == 0) sparkle_target_delay = 1;

            // Prepare list of LEDs to turn off and count them
            sparkle_led_count = 0;
            for (int i = 0; i < 8; i++) {
                if (sparkle_pattern & (1 << i)) {
                    leds_to_turn_off[sparkle_led_count] = i;
                    sparkle_led_count++;
                }
            }
            turn_off_index = 0;
            turn_off_timer = 0;
        }

        sparkle_delay_counter++;
        if (sparkle_delay_counter >= sparkle_target_delay) {
            sparkle_state = 1;  // Move to turn-off state
            sparkle_delay_counter = 0;
            turn_off_timer = 0;
            turn_off_index = 0;

            // Generate the first turn-off delay
            if (sparkle_led_count > 0) {
                uint32_t delay_ms = Generate_Random_Delay(50, 200);
                uint32_t timer_period_ms = fast_mode ? 500 : 1000;
                turn_off_target_delay = delay_ms / timer_period_ms;
                if (turn_off_target_delay == 0) turn_off_target_delay = 1;
            }
        }
    } else {
        // Turn off LEDs one by one
        if (turn_off_index < sparkle_led_count) {
            turn_off_timer++;

            if (turn_off_timer >= turn_off_target_delay) {
                // Turn off the current LED
                sparkle_pattern &= ~(1 << leds_to_turn_off[turn_off_index]);
                Set_LED_Pattern(sparkle_pattern);
                turn_off_index++;
                turn_off_timer = 0;

                // Generate delay for next LED turn-off (if there are more LEDs)
                if (turn_off_index < sparkle_led_count) {
                    uint32_t delay_ms = Generate_Random_Delay(50, 200);
                    uint32_t timer_period_ms = fast_mode ? 500 : 1000;
                    turn_off_target_delay = delay_ms / timer_period_ms;
                    if (turn_off_target_delay == 0) turn_off_target_delay = 1;
                }
            }
        } else {
            // All LEDs turned off, ensure pattern is completely clear and reset for next cycle
            sparkle_pattern = 0x00;
            Set_LED_Pattern(sparkle_pattern);
            sparkle_state = 0;
            sparkle_delay_counter = 0;
            turn_off_timer = 0;
            turn_off_index = 0;
        }
    }
}

// Generate random pattern (0-255)
uint8_t Generate_Random_Pattern(void)
{
    return (uint8_t)(rand() % 256);
}

// Generate random delay in milliseconds
uint32_t Generate_Random_Delay(uint32_t min_ms, uint32_t max_ms)
{
    return min_ms + (rand() % (max_ms - min_ms + 1));
}

// FIXED: Debounced button press detection using LL functions
int Is_Button_Pressed_Debounced(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, int button_index)
{
    // Note: Buttons are pull-up, so pressed = LOW (0)
    if (LL_GPIO_IsInputPinSet(GPIOx, GPIO_Pin) == 0) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_button_press[button_index] > debounce_delay) {
            last_button_press[button_index] = current_time;
            return 1;
        }
    }
    return 0;
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
