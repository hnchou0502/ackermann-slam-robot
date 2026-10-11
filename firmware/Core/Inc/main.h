/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  * @version        : v66.0 (修正左輪方向腳位定義 PC8)
  * @note           :
  * - [CRITICAL FIX] 根據測試結果，將 MOTOR_B_DIR 改為 PC8 (原本錯誤定義為 PA4)。
  * - [MAPPING]
  * - MOTOR A (Right): PC6
  * - MOTOR B (Left) : PC8  <-- 修正這裡
  * - SERVO          : PC0
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
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi2;

extern volatile uint32_t g_servo_pulse_width_us;
extern const uint32_t TIM6_TICK_US;
extern const uint32_t SERVO_PERIOD_COUNTS;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void process_command(char* cmd);
void update_robot_control(void);
void convert_angle_to_pulse_width(float angle);
uint32_t convert_speed_to_pwm(float speed);

// IMU 相關
void ICM_SelectBank(uint8_t bank);
void ICM_WriteByte(uint8_t reg, uint8_t value);
uint8_t ICM_ReadByte(uint8_t reg);
void ICM_ReadBytes(uint8_t reg, uint8_t* pData, uint8_t size);
void ICM_Init(void);
void ICM_ReadRawData(void);
void ICM_CalibrateGyro(void);
void ICM_UpdateAttitude(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_CS_Pin GPIO_PIN_12
#define IMU_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// ==========================================
// 1. 馬達方向控制
// ==========================================
// 右輪方向腳 (M1) - PC6 (維持不變)
#define MOTOR_A_DIR_PORT      GPIOC
#define MOTOR_A_DIR_PIN       GPIO_PIN_6

// [CRITICAL FIX] 左輪方向腳 (M2) - 測試確認為 PC8 (原為 PA4)
#define MOTOR_B_DIR_PORT      GPIOC
#define MOTOR_B_DIR_PIN       GPIO_PIN_8

// ==========================================
// 2. 馬達速度控制 (TIM3 CH2/CH4)
// ==========================================
#define MOTOR_A_PWM_TIM       &htim3
#define MOTOR_A_PWM_CHANNEL   TIM_CHANNEL_2
#define MOTOR_B_PWM_TIM       &htim3
#define MOTOR_B_PWM_CHANNEL   TIM_CHANNEL_4

// ==========================================
// 3. 舵機 (Servo) - PC0
// ==========================================
#define SERVO_GPIO_PORT       GPIOC
#define SERVO_GPIO_PIN        GPIO_PIN_0
#define LED_GPIO_Port         GPIOC
#define LED_Pin               GPIO_PIN_13

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
