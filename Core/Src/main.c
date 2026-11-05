#include "main.h"
#include <stdint.h>

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void Error_Handler(void);

/* Servo pulse limits in microseconds */
#define SERVO_MIN_PULSE  1000U
#define SERVO_MAX_PULSE  2000U

/* Convert angle (0 to 180 degrees) into PWM pulse */
static uint16_t Servo_AngleToPulse(uint8_t angle)
{
    if (angle > 180U)
        angle = 180U;

    return (uint16_t)(SERVO_MIN_PULSE +
           ((uint32_t)angle * 1000U) / 180U);
}

/* Set six independent servo positions */
static void Servo_SetPositions(uint8_t s1,
                               uint8_t s2,
                               uint8_t s3,
                               uint8_t s4,
                               uint8_t s5,
                               uint8_t s6)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
                          Servo_AngleToPulse(s1));

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
                          Servo_AngleToPulse(s2));

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
                          Servo_AngleToPulse(s3));

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,
                          Servo_AngleToPulse(s4));

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                          Servo_AngleToPulse(s5));

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
                          Servo_AngleToPulse(s6));
}

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    /* Start all six PWM channels */

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    while (1)
    {
        /* Robot position 1 */
        Servo_SetPositions(90, 45, 120, 30, 75, 150);
        HAL_Delay(2000);

        /* Robot position 2 */
        Servo_SetPositions(45, 90, 60, 120, 30, 90);
        HAL_Delay(2000);

        /* Robot position 3 */
        Servo_SetPositions(120, 30, 90, 60, 150, 45);
        HAL_Delay(2000);

        /* Return to starting position */
        Servo_SetPositions(90, 90, 90, 90, 90, 90);
        HAL_Delay(2000);
    }
}

/* System clock: Internal HSI 8 MHz */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
                            FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

/* TIM2: PA0, PA1, PA2, PA3 */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef clock = {0};
    TIM_MasterConfigTypeDef master = {0};
    TIM_OC_InitTypeDef oc = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 7;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 19999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
        Error_Handler();

    clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&htim2, &clock) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
        Error_Handler();

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim2, &master) != HAL_OK)
        Error_Handler();

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 1500;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(
            &htim2, &oc, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_ConfigChannel(
            &htim2, &oc, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_ConfigChannel(
            &htim2, &oc, TIM_CHANNEL_3) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_ConfigChannel(
            &htim2, &oc, TIM_CHANNEL_4) != HAL_OK)
        Error_Handler();

    HAL_TIM_MspPostInit(&htim2);
}

/* TIM3: PA6, PA7 */
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef clock = {0};
    TIM_MasterConfigTypeDef master = {0};
    TIM_OC_InitTypeDef oc = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 7;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 19999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
        Error_Handler();

    clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&htim3, &clock) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
        Error_Handler();

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim3, &master) != HAL_OK)
        Error_Handler();

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 1500;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(
            &htim3, &oc, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_ConfigChannel(
            &htim3, &oc, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    HAL_TIM_MspPostInit(&htim3);
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}
