/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR         (0x68 << 1)
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_ACCEL_XOUT_H     0x3B
#define REG_PWR_MGMT_1       0x6B
#define REG_WHO_AM_I         0x75

/* Fattori di conversione basati sul fondo scala impostato */
#define ACCEL_SCALE          8192.0f  // Per impostazione ±4g
#define GYRO_SCALE           65.5f    // Per impostazione ±500 dps
#define RAD_TO_DEG           57.295779513f

/* Parametri di Controllo */
#define DT                   0.01f    // Tempo di campionamento: 10ms (100 Hz)
#define ALPHA                0.98f    // Coefficiente del Filtro Complementare
#define ACCEL_OFFSET_DEG     -1.81f     // Verificato in laboratorio
/* Limiti PWM (espressi in microsecondi / impulsi del timer) */
#define PWM_MIN              1000
#define PWM_MAX              2000
#define PWM_BASE             1200     // Sostentamento minimo iniziale
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t raw_data[14];        // Buffer DMA per i 14 registri dell'MPU6050
char tx_buffer[128];         // Buffer per telemetria seriale
uint8_t rx_byte;             // Singolo byte ricevuto da UART
char rx_buffer[64];          // Buffer accumulo comandi PC
uint8_t rx_index = 0;

/* Stato di armamento motori */
volatile uint8_t system_armed = 0;

/* Stato Attitudine */
volatile float angolo_stimato = 0.0f;
volatile float target_angle = 0.0f;
volatile float gyro_offset = 0.0f;

/* Guadagni PID Singolo Anello (Aggiornabili da seriale) */
volatile float Kp = 0.0f;
volatile float Ki = 0.0f;
volatile float Kd = 0.0f;

/* Memoria Integratore */
volatile float errore_integrale = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void Parse_Command(char* cmd);
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
  /* Configurazione e sveglia dell'IMU */
  MPU6050_Init();

  /* Avvio dei canali PWM per i due motori (Iniziano inviando il minimo a 1ms) */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_MIN);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_MIN);

  /* --- ROUTINE DI AUTO-CALIBRAZIONE GYRO (1 Secondo) --- */
  float sum_gyro = 0.0f;
  int num_samples = 100;

  // ATTESA ASSESTAMENTO MECCANICO (2 secondi)
  // HAL_Delay(2000);

  for (int i = 0; i < num_samples; i++) {
      uint8_t calib_data[14];
      HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, calib_data, 14, 100);

      int16_t raw_gx = (calib_data[8] << 8) | calib_data[9];
      sum_gyro += ((float)raw_gx / GYRO_SCALE);

      // Refresh del segnale di sicurezza ESC durante il delay
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_MIN);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_MIN);

      HAL_Delay(10);
  }

  gyro_offset = sum_gyro / (float)num_samples;
  angolo_stimato = 0.0f;

  /* Abilitazione ricezione seriale e Timer Interrupt per loop di controllo */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  HAL_TIM_Base_Start_IT(&htim6);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* --- DISPATCHER DEL TIMER (100 Hz / Ogni 10 ms) --- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        /* Avvia la transazione I2C via DMA per i 14 byte consecutivi del sensore.*/
        HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, raw_data, 14);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2) {
		HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
	}
}

/* --- CALCOLO PID ED ESECUZIONE (CALLBACK COMPLETAMENTO I2C) --- */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {

        /* 1. ESTRAZIONE DATI E CONVERSIONE IN UNITÀ FISICHE */
        int16_t raw_ay = (raw_data[2] << 8) | raw_data[3];
        int16_t raw_az = (raw_data[4] << 8) | raw_data[5];
        int16_t raw_gx = (raw_data[8] << 8) | raw_data[9];

        float ay = (float)raw_ay / ACCEL_SCALE;
        float az = -((float)raw_az / ACCEL_SCALE);
        float gyro_rate = -((float)raw_gx / GYRO_SCALE) - gyro_offset;

        /* 2. FILTRO COMPLEMENTARE (SENSOR FUSION) */
        // Calcolo geometrico dell'angolo di inclinazione
        float angolo_acc = (atan2f(ay, az) * RAD_TO_DEG) - ACCEL_OFFSET_DEG;

        // Integrazione e fusione
        angolo_stimato = ALPHA * (angolo_stimato + gyro_rate * DT) + (1.0f - ALPHA) * angolo_acc;

        /* 3. ALGORITMO PID SINGOLO ANELLO */
        float errore = target_angle - angolo_stimato;

        // Proporzionale
        float P_term = Kp * errore;

        // Integrale con Anti-Windup saturato a ±300 microsecondi PWM
        if (system_armed == 1){
        	errore_integrale += errore * DT;
        } else {
        	errore_integrale = 0.0f;
        }

        float max_integral = (Ki > 0.0f) ? (300.0f / Ki) : 0.0f;
        if (errore_integrale > max_integral)  errore_integrale = max_integral;
        if (errore_integrale < -max_integral) errore_integrale = -max_integral;

        float I_term = Ki * errore_integrale;

        // Derivativo basato sulla misura diretta del Giroscopio (No Kick, Zero Rumore)
        float D_term = -Kd * gyro_rate;

        // Output totale di controllo
        float output_controllo = P_term + I_term + D_term;

        /* 4. ATTUAZIONE DIFFERENZIALE MOTORI */
        int32_t pwm_motore1 = PWM_MIN;
        int32_t pwm_motore2 = PWM_MIN;

        if (system_armed == 1) {
            pwm_motore1 = (int32_t)(PWM_BASE + output_controllo);
            pwm_motore2 = (int32_t)(PWM_BASE - output_controllo);

            // Saturazione entro il range operativo degli ESC
            if (pwm_motore1 > PWM_MAX) pwm_motore1 = PWM_MAX;
            if (pwm_motore1 < PWM_MIN) pwm_motore1 = PWM_MIN;
            if (pwm_motore2 > PWM_MAX) pwm_motore2 = PWM_MAX;
            if (pwm_motore2 < PWM_MIN) pwm_motore2 = PWM_MIN;
        }

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_motore1);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_motore2);

        /* 5. TELEMETRIA SERIALE DMA */
        if (huart2.gState == HAL_UART_STATE_READY) {
            int len = sprintf(tx_buffer, "%.2f,%.2f,%d,%d\r\n",
                              target_angle,
                              angolo_stimato,
                              (int)pwm_motore1,
                              (int)pwm_motore2);
            HAL_UART_Transmit_DMA(&huart2, (uint8_t*)tx_buffer, len);
        }
    }
}

/* --- RICEZIONE COMANDI UART --- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                Parse_Command(rx_buffer);
                rx_index = 0;
            }
        } else {
            if (rx_index < sizeof(rx_buffer) - 1) {
                rx_buffer[rx_index++] = rx_byte;
            }
        }
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

/* --- PARSER COMANDI DA PYTHON --- */
void Parse_Command(char* cmd) {
    if (cmd[0] == 'T' && cmd[1] == ':') {
        float nuovo_target = (float)atof(cmd + 2);
        if (nuovo_target > 18.0f) target_angle = 18.0f;
        else if (nuovo_target < -18.0f) target_angle = -18.0f;
        else target_angle = nuovo_target;
    }
    else if (cmd[0] == 'P' && cmd[1] == ':') {
        Kp = (float)atof(cmd + 2);
    }
    else if (cmd[0] == 'I' && cmd[1] == ':') {
        Ki = (float)atof(cmd + 2);
        errore_integrale = 0.0f;
    }
    else if (cmd[0] == 'D' && cmd[1] == ':') {
        Kd = (float)atof(cmd + 2);
    }
    else if (cmd[0] == 'S' && cmd[1] == ':') {
        int stato = atoi(cmd + 2);
        if (stato == 1) {
            system_armed = 1;
        } else {
            system_armed = 0;
            errore_integrale = 0.0f;
        }
    }
}

/* --- INIZIALIZZAZIONE HARDWARE MPU6050 --- */
void MPU6050_Init(void) {
    uint8_t check = 0;
    uint8_t write_reg = 0;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &check, 1, 1000);
    if (check != 0x68) {
        while(1); // Sensore non trovato
    }

    // 1. Wake up & impostazione sorgente clock su Gyro X
    write_reg = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &write_reg, 1, 1000);

    // 2. Filtro DLPF interno a ~42Hz
    write_reg = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &write_reg, 1, 1000);

    // 3. Giroscopio a ±500 dps
    write_reg = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &write_reg, 1, 1000);

    // 4. Accelerometro a ±4g
    write_reg = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &write_reg, 1, 1000);
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
