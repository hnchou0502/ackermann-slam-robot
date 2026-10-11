/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @version        : v77.0 (Odom scale calibrated 0.0004618->0.0005164)
  * @note           :
  * - [v76 修正] yaw 積分死區改為「僅車靜止時套用」。舊版一律套 0.8 deg/s 死區，
  *   把行進中的緩慢轉向整段丟棄，只留下快速抖動 -> 實測推車一趟真值 -7.9 度、
  *   積出 +2.3 度（正負號都反）。原始角速度本身很準（-7.9 度 vs 編碼器 -8.0 度）。
  *   死區不能單純調小：靜止 sigma=0.142 deg/s、bias=-0.058，死區 <5 sigma 會讓雜訊
  *   穿透且負值偏多，靜止 10 分鐘漂 -27 度。故以編碼器判定靜止，動時不套死區。
  * - [v75 安全] IWDG 硬體看門狗：Prescaler 256 / Reload 625 -> 標稱逾時 4.0s。
  *   LSI 最壞 60kHz 時仍有 2.67s，而開機序列 (陀螺儀校正 200 取樣 x 2ms) 僅
  *   約 0.6s，不會開機自我復位。餵狗點在 50Hz heartbeat 內。
  * - [v75 !! 地雷] TIM6 Period=9 與 SPI2 (POLARITY_HIGH/2EDGE/分頻32) 這三個值
  *   歷來只手改 main.c、未回寫 .ioc，任何人按一次 Generate Code 都會被打回
  *   預設值 (TIM6=99 -> 舵機週期變 200ms；SPI2 -> 18MHz mode0，IMU 直接讀不到)。
  *   2026-07-10 已補進 slam_car_fw.ioc，.ioc 現為單一真實來源。
  * - [v74 安全] CMD 逾時除了速度歸零，角速度也歸零。舊版只清 g_target_speed，
  *   會落入「速度≈0 且 |角速度|>0.05」的飽和分支，導致斷線後前輪卡在 ±30度
  *   持續堵轉。馬達本來就會停 (由下方 |v|<0.05 的 PWM 歸零守衛擋住)。
  * - [解析度提升] TIM6 改為 10us 中斷 (原本 100us)。
  * - [效果] 現在可以以 10us (約 1度) 為單位微調舵機中點，不會再亂跳了。
  * - [參數] Odom: 0.0005164f (2026-07-11 八段推車標定 ×1.1182；原 0.0004618 短 11.8%), Servo Center 1462 (2026-06-28 捲尺兩點校正)。
  * - [EKF] FBK 擴充 2 欄位: IMU yaw rate + 積分 yaw (×1000)，供 ROS
  *   robot_localization 融合。格式 $FBK,vL,vR,pwmL,pwmR,gyroZ,yaw,#。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float deadband;
    float output_alpha;
    float prev_error;
    float prev_prev_error;
    float current_output;
    float out_max;
    float out_min;
} PID_Incremental_TypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef DISTANCE_PER_COUNT
#define DISTANCE_PER_COUNT  0.0005164f   // 2026-07-11 八段推車標定 (原 0.0004618 短 11.8%)
#endif

const float MAX_SPEED = 1.0f;
const uint32_t MAX_PWM_DUTY = 19999;

const float MIN_STEERING_ANGLE = -30.0f;
const float MAX_STEERING_ANGLE = 30.0f;

const float HEADING_KP = 250.0f;
const int32_t MAX_HEADING_CORRECTION = 5000;

// [v70.0 修改] 提升解析度到 10us
const uint32_t TIM6_TICK_US = 10;        // 原本 100, 改成 10
const uint32_t SERVO_PERIOD_COUNTS = 2000; // 原本 200, 改成 2000 (20ms)

// =========================================================================
// [v70.0 校正] 舵機中點 (Fine Tuning)
// 現在解析度是 10us。
// 建議測試順序: 1400 -> 1410 -> 1420 -> 1430 ...
// 這樣就可以找到從左偏變成右偏的那個 "甜蜜點"
// [2026-06-28 捲尺兩點校正] 1430→偏左2.60° ; 1473→偏右0.85°(方向確認正確)。
//   實測靈敏度 12.4us/deg(連桿非線性，非名目16.6)，線性內插走直中位 = 1462us。
// =========================================================================
const uint32_t SERVO_CENTER_US = 1462;

const int32_t M1_MIN_STARTUP_PWM = 1800;
const int32_t M2_MIN_STARTUP_PWM = 2800;

const float TRACK_WIDTH = 0.14f;
const float WHEEL_BASE_LEN = 0.235f;

const uint32_t CMD_TIMEOUT_MS = 500;

const float DT = 0.02f;

// [v76] yaw 積分死區：**僅在車靜止時**套用。
// 靜止量測（2026-07-10, 15003 筆）：gyro bias -0.058 deg/s、sigma 0.142 deg/s。
// 死區必須 >5 sigma 才擋得住雜訊穿透（否則因 bias 偏負，靜止 10 分鐘會漂 -27 度）。
// 但行進中的緩慢轉向本身就 <0.8 deg/s，若一律套死區會被整段丟棄——實測推車一趟
// 真值 -7.9 度、套死區後積出 +2.3 度（連正負號都反）。故改為靜止才套。
const float GYRO_DEADBAND_DPS = 0.8f;
// 連續多少個 50Hz cycle 完全沒有編碼器計數，才判定為靜止（25 = 0.5 秒）。
// 極慢推車（0.01 m/s）仍約每 2~3 個 cycle 就有 1 個計數，不會誤判為靜止。
const uint8_t STATIONARY_IDLE_CYCLES = 25;

// =========================================================================
// IWDG 反向測試開關（臨時驗證看門狗真的會復位）
//   0 = 正常 v76 行為（本測試碼全部編譯為空，與正式版位元相同）。
//   1 = 啟用：開機印復位來源；收到 $WDGTEST 指令後停止餵狗，約 4s 後應被 IWDG 復位。
//   ⚠ 測完務必改回 0（或 git checkout）再燒正式版。
// =========================================================================
#define IWDG_REVERSE_TEST 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define REG_BANK_SEL    0x7F
#define REG_WHO_AM_I    0x00
#define REG_PWR_MGMT_1  0x06
#define REG_ACCEL_XOUT_H 0x2D

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile float g_target_speed = 0.0f;
volatile float g_target_angular = 0.0f;
volatile float g_lock_yaw = 0.0f;
volatile uint32_t g_last_cmd_time = 0;

#define UART_RX_BUFFER_SIZE 64
uint8_t g_uart_rx_buffer[UART_RX_BUFFER_SIZE];
volatile uint32_t g_uart_rx_index = 0;
volatile uint8_t g_uart_rx_char;

volatile uint8_t g_heartbeat_flag = 0;
volatile uint8_t g_command_ready_flag = 0;

volatile int16_t g_encoder_m1_diff = 0;
volatile int16_t g_encoder_m2_diff = 0;
volatile float g_speed_m1_mps = 0.0f;
volatile float g_speed_m2_mps = 0.0f;

volatile float g_speed_m1_filtered_mps = 0.0f;
volatile float g_speed_m2_filtered_mps = 0.0f;

float g_pos_x = 0.0f;
float g_pos_y = 0.0f;

PID_Incremental_TypeDef g_pid_m1 = {0};
PID_Incremental_TypeDef g_pid_m2 = {0};
volatile int32_t g_motor1_pwm_out = 0;
volatile int32_t g_motor2_pwm_out = 0;

int16_t g_imu_accel[3] = {0};
int16_t g_imu_gyro[3] = {0};
float g_gyro_z_offset = 0.0f;
float g_yaw_angle = 0.0f;
uint32_t g_last_imu_time = 0;
volatile float g_gyro_z_dps = 0.0f;   // [EKF] 校正後 yaw rate (deg/s)，未過死區，供 ROS robot_localization
volatile uint8_t g_stationary = 1;    // [v76] 編碼器連續 0.5s 無計數 = 車靜止；只有靜止時才對 yaw 積分套死區

volatile uint32_t g_servo_pulse_width_us = 1500;

#if IWDG_REVERSE_TEST
volatile uint8_t g_iwdg_starve = 0;   // 反向測試：置 1 後主迴圈停止餵狗
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI2_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

void process_command(char* cmd);
void update_robot_control(void);
void convert_angle_to_pulse_width(float angle);
uint32_t convert_speed_to_pwm(float speed);
void PID_Inc_Init(PID_Incremental_TypeDef* pid, float kp, float ki, float kd, float deadband, float alpha, float limit);
float PID_Inc_Compute(PID_Incremental_TypeDef* pid, float target, float measured);

// IMU
void ICM_SelectBank(uint8_t bank);
void ICM_WriteByte(uint8_t reg, uint8_t value);
uint8_t ICM_ReadByte(uint8_t reg);
void ICM_ReadBytes(uint8_t reg, uint8_t* pData, uint8_t size);
void ICM_Init(void);
void ICM_ReadRawData(void);
void ICM_CalibrateGyro(void);
void ICM_UpdateAttitude(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PID_Inc_Init(PID_Incremental_TypeDef* pid, float kp, float ki, float kd, float deadband, float alpha, float limit) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->deadband = deadband;
    pid->output_alpha = alpha;
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->current_output = 0.0f;
    pid->out_max = limit;
    pid->out_min = -limit;
}

float PID_Inc_Compute(PID_Incremental_TypeDef* pid, float target, float measured) {
    if (target == 0.0f) {
        pid->current_output = 0.0f;
        pid->prev_error = 0.0f;
        pid->prev_prev_error = 0.0f;
        return 0.0f;
    }
    float error = target - measured;
    if (fabsf(error) < pid->deadband) error = 0.0f;
    float delta_output = pid->Kp * (error - pid->prev_error) + pid->Ki * error + pid->Kd * (error - 2.0f * pid->prev_error + pid->prev_prev_error);
    float target_output = pid->current_output + delta_output;
    pid->current_output = pid->output_alpha * target_output + (1.0f - pid->output_alpha) * pid->current_output;
    if (pid->current_output > pid->out_max) pid->current_output = pid->out_max;
    if (pid->current_output < pid->out_min) pid->current_output = pid->out_min;
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
    return pid->current_output;
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_SPI2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  g_target_speed = 0.0f;
  g_target_angular = 0.0f;
  g_motor1_pwm_out = 0;
  g_motor2_pwm_out = 0;

  // 安全啟動
  HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_DIR_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_DIR_PIN, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(MOTOR_A_PWM_TIM, MOTOR_A_PWM_CHANNEL, 0);
  __HAL_TIM_SET_COMPARE(MOTOR_B_PWM_TIM, MOTOR_B_PWM_CHANNEL, 0);
  HAL_Delay(50);
  HAL_TIM_PWM_Start(MOTOR_A_PWM_TIM, MOTOR_A_PWM_CHANNEL);
  HAL_TIM_PWM_Start(MOTOR_B_PWM_TIM, MOTOR_B_PWM_CHANNEL);

  HAL_TIM_Base_Start_IT(&htim6);

  printf("\r\n=== STM32 Robot v70.0 (High Res PWM) ===\r\n");

#if IWDG_REVERSE_TEST
  // 反向測試鐵證：RCC 保留上一次的復位來源旗標。若這次開機是被 IWDG 復位的，印出來確認。
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
      printf("[BOOT] reset cause: IWDG watchdog（看門狗逾時復位＝反向測試成功）\r\n");
  } else {
      printf("[BOOT] reset cause: normal power-on / NRST（非看門狗）\r\n");
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();   // 讀完必清，否則旗標會殘留影響下次判定
#endif

  ICM_Init();
  ICM_CalibrateGyro();

  PID_Inc_Init(&g_pid_m1, 1000.0f, 1000.0f, 0.0f, 0.005f, 1.0f, (float)MAX_PWM_DUTY);
  PID_Inc_Init(&g_pid_m2, 800.0f, 200.0f, 0.0f, 0.005f, 1.0f, (float)MAX_PWM_DUTY);

  convert_angle_to_pulse_width(0.0f);
  update_robot_control();

  g_last_imu_time = HAL_GetTick();
  g_last_cmd_time = HAL_GetTick();

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim7); // Heartbeat
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&g_uart_rx_char, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static volatile uint32_t g_heartbeat_counter = 0;
  static int16_t last_encoder_m1 = 0;
  static int16_t last_encoder_m2 = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (g_heartbeat_flag == 1)
    {
        g_heartbeat_flag = 0;
        g_heartbeat_counter++;

        // [v75] 餵硬體看門狗。刻意放在 heartbeat 區塊內：TIM7 中斷或主迴圈
        // 任一環卡死 -> 餵不到狗 -> 約 4s 後 MCU 自動復位（PWM 停、舵機回中）。
#if IWDG_REVERSE_TEST
        // 反向測試：$WDGTEST 後 g_iwdg_starve=1，停止餵狗以驗證看門狗確實會復位。
        if (!g_iwdg_starve) {
            HAL_IWDG_Refresh(&hiwdg);
        }
#else
        HAL_IWDG_Refresh(&hiwdg);
#endif

        // Watchdog: 速度與角速度都要歸零。只清速度的話，下方 Ackermann 分支會
        // 命中「|v|<0.05 但 |w|>0.05 -> 舵機飽和 ±30度」，斷線後前輪卡死堵轉。
        if (HAL_GetTick() - g_last_cmd_time > CMD_TIMEOUT_MS) {
            g_target_speed = 0.0f;
            g_target_angular = 0.0f;
        }

        // Encoder
        int16_t current_encoder_m1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        int16_t current_encoder_m2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        g_encoder_m1_diff = current_encoder_m1 - last_encoder_m1;
        g_encoder_m2_diff = current_encoder_m2 - last_encoder_m2;
        last_encoder_m1 = current_encoder_m1;
        last_encoder_m2 = current_encoder_m2;

        // [v76] 靜止偵測（供 ICM_UpdateAttitude 決定要不要套 yaw 死區）
        static uint8_t idle_cycles = 0;
        if (g_encoder_m1_diff == 0 && g_encoder_m2_diff == 0) {
            if (idle_cycles < STATIONARY_IDLE_CYCLES) idle_cycles++;
        } else {
            idle_cycles = 0;
        }
        g_stationary = (idle_cycles >= STATIONARY_IDLE_CYCLES);

        g_speed_m1_mps = (float)g_encoder_m1_diff * 50.0f * DISTANCE_PER_COUNT;
        g_speed_m2_mps = (float)g_encoder_m2_diff * 50.0f * DISTANCE_PER_COUNT;

        const float alpha = 0.7f;
        g_speed_m1_filtered_mps = alpha * g_speed_m1_filtered_mps + (1.0f - alpha) * g_speed_m1_mps;
        g_speed_m2_filtered_mps = alpha * g_speed_m2_filtered_mps + (1.0f - alpha) * g_speed_m2_mps;

        // Ackermann Control
        float target_steering_angle_deg = 0.0f;
        if (fabsf(g_target_speed) > 0.05f) {
            float steering_rad = atanf((WHEEL_BASE_LEN * g_target_angular) / g_target_speed);
            target_steering_angle_deg = steering_rad * (180.0f / M_PI);
        } else if (fabsf(g_target_angular) > 0.05f) {
            target_steering_angle_deg = (g_target_angular > 0) ? MAX_STEERING_ANGLE : MIN_STEERING_ANGLE;
        } else {
            target_steering_angle_deg = 0.0f;
        }
        convert_angle_to_pulse_width(target_steering_angle_deg);

        if (fabsf(g_target_speed) < 0.05f) {
            g_motor1_pwm_out = 0;
            g_motor2_pwm_out = 0;
            g_pid_m1.prev_error = 0.0f; g_pid_m1.prev_prev_error = 0.0f; g_pid_m1.current_output = 0.0f;
            g_pid_m2.prev_error = 0.0f; g_pid_m2.prev_prev_error = 0.0f; g_pid_m2.current_output = 0.0f;
            ICM_ReadRawData(); ICM_UpdateAttitude();
        }
        else {
            float target_v_L = g_target_speed - (g_target_angular * TRACK_WIDTH / 2.0f);
            float target_v_R = g_target_speed + (g_target_angular * TRACK_WIDTH / 2.0f);

            int32_t pid_out_m1 = (int32_t)PID_Inc_Compute(&g_pid_m1, target_v_R, g_speed_m1_filtered_mps);
            int32_t pid_out_m2 = (int32_t)PID_Inc_Compute(&g_pid_m2, target_v_L, g_speed_m2_filtered_mps);

            if (target_v_R > 0.01f) pid_out_m1 += M1_MIN_STARTUP_PWM;
            else if (target_v_R < -0.01f) pid_out_m1 -= M1_MIN_STARTUP_PWM;

            if (target_v_L > 0.01f) pid_out_m2 += M2_MIN_STARTUP_PWM;
            else if (target_v_L < -0.01f) pid_out_m2 -= M2_MIN_STARTUP_PWM;

            ICM_ReadRawData(); ICM_UpdateAttitude();

            int32_t m1_final = pid_out_m1;
            int32_t m2_final = pid_out_m2;

            if (m1_final > (int32_t)MAX_PWM_DUTY) m1_final = MAX_PWM_DUTY;
            if (m1_final < -(int32_t)MAX_PWM_DUTY) m1_final = -(int32_t)MAX_PWM_DUTY;
            if (m2_final > (int32_t)MAX_PWM_DUTY) m2_final = MAX_PWM_DUTY;
            if (m2_final < -(int32_t)MAX_PWM_DUTY) m2_final = -(int32_t)MAX_PWM_DUTY;

            g_motor1_pwm_out = m1_final;
            g_motor2_pwm_out = m2_final;
        }

        float v_linear = (g_speed_m1_filtered_mps + g_speed_m2_filtered_mps) / 2.0f;
        float yaw_rad = g_yaw_angle * (M_PI / 180.0f);
        g_pos_x += v_linear * cosf(yaw_rad) * DT;
        g_pos_y += v_linear * sinf(yaw_rad) * DT;

        update_robot_control();

        // [整數協議回傳]
        if (g_heartbeat_counter % 2 == 0) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            // [EKF] 擴充 FBK：附 IMU yaw rate (deg/s) 與積分 yaw (deg)，皆 ×1000 整數
            // 格式: $FBK,vL,vR,pwmL,pwmR,gyroZ_x1000,yaw_x1000,#
            printf("$FBK,%d,%d,%d,%d,%d,%d,#\r\n",
                   (int)(g_speed_m2_filtered_mps * 1000.0f),
                   (int)(g_speed_m1_filtered_mps * 1000.0f),
                   (int)g_motor2_pwm_out,
                   (int)g_motor1_pwm_out,
                   (int)(g_gyro_z_dps * 1000.0f),
                   (int)(g_yaw_angle * 1000.0f));
        }
    }

    if (g_command_ready_flag == 1) {
        g_command_ready_flag = 0;
        process_command((char*)g_uart_rx_buffer);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 625;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 71;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7199;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 199;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// [手動解析] $CMD,vx,wz,#  (vx/wz 為 m/s、rad/s 各 x1000 的整數)
// ⚠ 必須留在 USER CODE 區塊內。此函式原本寫在 CubeMX 生成區，
//    2026-07-10 一次 Generate Code 就被整個抹除（連結期才會爆）。
void process_command(char* cmd) {
#if IWDG_REVERSE_TEST
    // 反向測試指令：收到 $WDGTEST 就停止餵狗，約 4 秒後 MCU 應被 IWDG 復位。
    if (strncmp(cmd, "$WDGTEST", 8) == 0) {
        g_iwdg_starve = 1;
        printf("[WDGTEST] 停止餵狗，預期約 4 秒後看門狗復位...\r\n");
        return;
    }
#endif
    if (strncmp(cmd, "$CMD", 4) != 0) return;

    char* p1 = strchr(cmd, ',');
    if (p1 == NULL) return;
    p1++;

    char* p2 = strchr(p1, ',');
    if (p2 == NULL) return;

    int vx_int = atoi(p1);
    p2++;
    int wz_int = atoi(p2);

    g_target_speed = (float)vx_int / 1000.0f;
    g_target_angular = (float)wz_int / 1000.0f;
    g_last_cmd_time = HAL_GetTick();
}

// Printf
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

// UART Rx Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    if (g_uart_rx_index < UART_RX_BUFFER_SIZE - 1) {
        if (g_uart_rx_char == '\r' || g_uart_rx_char == '\n') {
            if (g_uart_rx_index > 0) {
                g_uart_rx_buffer[g_uart_rx_index] = '\0';
                g_command_ready_flag = 1;
                g_uart_rx_index = 0;
            }
        } else {
            g_uart_rx_buffer[g_uart_rx_index++] = g_uart_rx_char;
        }
    } else {
        g_uart_rx_index = 0;
    }
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&g_uart_rx_char, 1);
  }
}

// IMU Functions
void ICM_SelectBank(uint8_t bank) {
    uint8_t txData[2] = {REG_BANK_SEL & 0x7F, (bank << 4) & 0x30};
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, txData, 2, 100);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

void ICM_WriteByte(uint8_t reg, uint8_t value) {
    uint8_t txData[2] = {reg & 0x7F, value};
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, txData, 2, 100);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

uint8_t ICM_ReadByte(uint8_t reg) {
    uint8_t txData = reg | 0x80;
    uint8_t rxData = 0;
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &txData, 1, 100);
    HAL_SPI_Receive(&hspi2, &rxData, 1, 100);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return rxData;
}

void ICM_ReadBytes(uint8_t reg, uint8_t* pData, uint8_t size) {
    uint8_t txData = reg | 0x80;
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &txData, 1, 100);
    HAL_SPI_Receive(&hspi2, pData, size, 100);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

void ICM_Init(void) {
    printf("Initializing ICM-20948...\r\n");
    ICM_SelectBank(0);
    HAL_Delay(10);
    uint8_t whoami = ICM_ReadByte(REG_WHO_AM_I);
    if (whoami == 0xEA) printf("[OK] Device ID: 0x%02X\r\n", whoami);
    else printf("[ERR] Device ID: 0x%02X\r\n", whoami);
    ICM_WriteByte(REG_PWR_MGMT_1, 0x01);
    HAL_Delay(50);
}

void ICM_ReadRawData(void) {
    uint8_t buffer[12];
    ICM_SelectBank(0);
    ICM_ReadBytes(REG_ACCEL_XOUT_H, buffer, 12);
    g_imu_accel[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    g_imu_accel[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    g_imu_accel[2] = (int16_t)((buffer[4] << 8) | buffer[5]);
    g_imu_gyro[0]  = (int16_t)((buffer[6] << 8) | buffer[7]);
    g_imu_gyro[1]  = (int16_t)((buffer[8] << 8) | buffer[9]);
    g_imu_gyro[2]  = (int16_t)((buffer[10] << 8) | buffer[11]);
}

void ICM_CalibrateGyro(void) {
    printf("Calibrating Gyro (Don't Move!)...\r\n");
    int32_t sum_z = 0;
    const int samples = 200;
    for (int i = 0; i < samples; i++) {
        ICM_ReadRawData();
        sum_z += g_imu_gyro[2];
        HAL_Delay(2);
    }
    g_gyro_z_offset = (float)sum_z / (float)samples;
    printf("Calibration Done! Offset Z: %.2f\r\n", g_gyro_z_offset);
}

void ICM_UpdateAttitude(void) {
    uint32_t now = HAL_GetTick();
    float dt = (now - g_last_imu_time) / 1000.0f;
    g_last_imu_time = now;
    float gyro_z_raw = (float)g_imu_gyro[2] - g_gyro_z_offset;
    float gyro_z_dps = gyro_z_raw / 131.0f;
    g_gyro_z_dps = gyro_z_dps;   // [EKF] 上傳給 ROS（未過死區，讓濾波器自行處理雜訊）

    // [v76] 死區僅在**車靜止**時套用。靜止時真實角速度必為 0，死區純粹擋雜訊；
    // 車一動，緩慢轉向（典型 <0.8 deg/s）就是真訊號，丟掉會讓 yaw 積出反向結果。
    if (g_stationary && fabsf(gyro_z_dps) < GYRO_DEADBAND_DPS) {
        gyro_z_dps = 0.0f;
    }
    g_yaw_angle += gyro_z_dps * dt;
}

// [補回] update_robot_control
void update_robot_control(void) {
    // --- Motor 1 (Right) ---
    if (g_motor1_pwm_out > 0) {
        HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_DIR_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(MOTOR_A_PWM_TIM, MOTOR_A_PWM_CHANNEL, MAX_PWM_DUTY - g_motor1_pwm_out);
    } else if (g_motor1_pwm_out < 0) {
        HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_DIR_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(MOTOR_A_PWM_TIM, MOTOR_A_PWM_CHANNEL, abs(g_motor1_pwm_out));
    } else {
        HAL_GPIO_WritePin(MOTOR_A_DIR_PORT, MOTOR_A_DIR_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(MOTOR_A_PWM_TIM, MOTOR_A_PWM_CHANNEL, MAX_PWM_DUTY);
    }

    // --- Motor 2 (Left) ---
    if (g_motor2_pwm_out > 0) {
        HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_DIR_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(MOTOR_B_PWM_TIM, MOTOR_B_PWM_CHANNEL, g_motor2_pwm_out);
    } else if (g_motor2_pwm_out < 0) {
        HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_DIR_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(MOTOR_B_PWM_TIM, MOTOR_B_PWM_CHANNEL, MAX_PWM_DUTY - abs(g_motor2_pwm_out));
    } else {
        HAL_GPIO_WritePin(MOTOR_B_DIR_PORT, MOTOR_B_DIR_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(MOTOR_B_PWM_TIM, MOTOR_B_PWM_CHANNEL, MAX_PWM_DUTY);
    }
}

// 舵機 PWM
void convert_angle_to_pulse_width(float angle) {
    if (angle > MAX_STEERING_ANGLE) angle = MAX_STEERING_ANGLE;
    if (angle < MIN_STEERING_ANGLE) angle = MIN_STEERING_ANGLE;
    // [校正] 反轉計算符號
    float pulse_us = SERVO_CENTER_US - (angle * 16.6f);
    g_servo_pulse_width_us = (uint32_t)pulse_us;
}

uint32_t convert_speed_to_pwm(float speed) {
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    if (speed < 0) speed = 0;
    return (uint32_t)((speed / MAX_SPEED) * MAX_PWM_DUTY);
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
