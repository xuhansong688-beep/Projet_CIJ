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
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include<stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_SIZE 11       // AA F(octet faible) F(octet medium) F(octet fort) A1 H1 A2 H2 P(octet faible) P((octet fort) CRC
#define RESP_FRAME_SIZE 8  // BB ARR_H ARR_L FREQ_H FREQ_M FREQ_L CRC1 CRC2
#define MAX_TABLE_SIZE 256
#define FRAME_TIMEOUT_MS 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t NUMBER_OF_SAMPLE =32;
uint8_t rx_byte;
uint8_t trigger_period = 10; //modifier le period de trigger
uint8_t frame_buf[FRAME_SIZE];
uint8_t resp_buf[RESP_FRAME_SIZE]; // 响应帧缓冲区
uint8_t frame_index = 0;
uint8_t receiving = 0;
uint32_t frame_timeout =0;
uint16_t sine_table[MAX_TABLE_SIZE];
uint16_t sineamp_table[MAX_TABLE_SIZE];
uint16_t sinphase_table[MAX_TABLE_SIZE];
float amplitude_scale = 1.0f;
uint16_t phase_offset = 0;
uint16_t xx = 2048;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SendResponseFrame(uint32_t arr, uint32_t real_freq); // 新增响应帧发送函�?
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Calculer la somme de contrôle (CRC) de la trame de réponse
uint16_t CalcRespCRC(uint8_t *buf, uint8_t len)
{
    uint16_t sum = 0;
    for(uint8_t i=0; i<len-2; i++) // Les 6 premiers octets participent au calcul
    {
        sum += buf[i];
    }
    return sum;
}

// Envoyer la trame de réponse (contient l'ARR et la fréquence réelle)
void SendResponseFrame(uint32_t arr, uint32_t real_freq)
{
    // Effacer le tampon de réponse
    memset(resp_buf, 0, sizeof(resp_buf));

    // En-tête de trame : 0xBB identifie une réponse
    resp_buf[0] = 0xBB;

    // Valeur ARR (16 bits) : Octet fort + Octet faible
    resp_buf[1] = (arr >> 8) & 0xFF;
    resp_buf[2] = arr & 0xFF;

    // Fréquence réelle (32 bits, récupération des 24 bits de poids faible) :
    // Octet fort + Octet moyen + Octet faible
    resp_buf[3] = (real_freq >> 16) & 0xFF;
    resp_buf[4] = (real_freq >> 8) & 0xFF;
    resp_buf[5] = real_freq & 0xFF;

    // Calculer la somme de contrôle (CRC sur 16 bits divisé en deux octets de 8 bits)
    uint16_t crc = CalcRespCRC(resp_buf, RESP_FRAME_SIZE);
    resp_buf[6] = (crc >> 8) & 0xFF;
    resp_buf[7] = crc & 0xFF;

    // Transmettre la trame de réponse via le port série (UART)
    HAL_UART_Transmit(&huart2, resp_buf, RESP_FRAME_SIZE, 100);
}

// Initialisation : écrire les valeurs de la fonction sinus de base
void GenerateSineTable(uint8_t amp1,uint8_t harm1,uint8_t amp2,uint8_t harm2)
{
    for (int i = 0; i < NUMBER_OF_SAMPLE; i++)
    {
        //float angle = 2.0f * 3.1415926f * i / NUMBER_OF_SAMPLE;
        //sine_table[i] = (uint16_t)(xx + (xx-1) * sinf(angle));
        float angle_1 = 2.0f * 3.1415926f * i * harm1 / NUMBER_OF_SAMPLE;
        float angle_2 = 2.0f * 3.1415926f * i * harm2 / NUMBER_OF_SAMPLE;
        sine_table[i] = (uint16_t)(xx + (xx-1)*(amp1)/(amp1+amp2)* sinf(angle_1) + (xx-1)*(amp2)/(amp1+amp2)* sinf(angle_2)); // Avec A1 + A2 <= 255
    }
}

// Ajuster le tableau sinus en fonction de l'amplitude d'entrée
/*void UpdateScaledTable(void)
{
	//   Vider le tableau de mise à l'échelle de l'amplitude du sinus,
	//   initialiser tous les éléments à 0
	memset(sineamp_table, 0, sizeof(sineamp_table));

	//   Parcourir le tableau sinus original et calculer la mise
	//   à l'échelle de l'amplitude pour chaque élément
    for (int i = 0; i < NUMBER_OF_SAMPLE; i++)
    {
        float centered = sine_table[i] - xx;
        centered *= amplitude_scale;
        sineamp_table[i] = (uint16_t)(xx + centered);
    }
}
*/

// Ajuster le tableau sinus en fonction de le phase d'entrée
void Updatephase()
{
	memset(sinphase_table, 0, sizeof(sinphase_table));
	//    Parcourir le tableau de mise à l'échelle d'amplitude et
	//    remplir le tableau de phase via le décalage de phase
	for(int i = 0;i < NUMBER_OF_SAMPLE;i++)
	{
		int index = (i + phase_offset)% NUMBER_OF_SAMPLE;
		sinphase_table[index] = sine_table[i];
	}
}

/*void SetAmplitude(uint8_t amp)
{
    amplitude_scale = (float)amp / 255.0f;;
    UpdateScaledTable();
}
*/
void Setphase(uint16_t phase)
{
	uint16_t phase_midi = phase*NUMBER_OF_SAMPLE/360;
	phase_offset = phase_midi;
	Updatephase();
}

void SetFrequency(uint32_t freq,uint8_t amp1,uint8_t harm1,uint8_t amp2,uint8_t harm2,uint16_t phase)
{
    if (freq == 0) return;
    uint32_t timer_clk = 84000000;   // APB1 Timer Clock = 84MHz
    HAL_TIM_Base_Stop(&htim6);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);

    // Modifier la taille du tableau pour faciliter la sortie
    if (freq > 80000)      NUMBER_OF_SAMPLE = 16;    // 80k~100kHz 16
        else if (freq > 32000) NUMBER_OF_SAMPLE = 32;    // 32k~80kHz 32
        else if (freq > 16000) NUMBER_OF_SAMPLE = 64;    // 16k~32kHz 64
        else                        NUMBER_OF_SAMPLE = 256;   // <16kHz 256
    GenerateSineTable(amp1,harm1,amp2,harm2);
    //SetAmplitude(amp);
    Setphase(phase);

    // Calculer la PWM sur TIM2 et la sortie DMA sur TIM6 en modifiant ARR et PSC
    uint32_t psc =0;
    float total_ticks = (float)timer_clk / (float)freq;
    uint32_t arr_base = (uint32_t)roundf(total_ticks);
    total_ticks = (float)arr_base / (float)NUMBER_OF_SAMPLE;
    uint32_t arr_dac = (uint32_t)roundf(total_ticks);
    uint32_t final_arr_pwm = (arr_dac*NUMBER_OF_SAMPLE) -1;
    uint32_t final_arr_dac = arr_dac -1;
    uint32_t comp = final_arr_pwm/trigger_period;

    // Calcul de la fréquence réelle réalisable
    uint32_t real_freq = timer_clk / (NUMBER_OF_SAMPLE * (final_arr_dac + 1));

    //eable le tim
    htim2.Instance->PSC = psc;
    htim2.Instance->ARR = final_arr_pwm;
    htim6.Instance->PSC = psc;
    htim6.Instance->ARR = final_arr_dac;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, comp);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)sinphase_table, NUMBER_OF_SAMPLE, DAC_ALIGN_12B_R);
    HAL_TIM_Base_Start(&htim6);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);

    // Émission de la trame de réponse : incluant la valeur ARR du DAC et la fréquence réelle.
    SendResponseFrame(final_arr_dac, real_freq);
}

uint8_t CalcCRC(uint8_t *buf)
{
    uint8_t sum = 0;
    for(uint8_t i=1;i<10;i++){
    sum += buf[i];
    }
    return sum & 0xFF;
}

void ProcessFrame(uint8_t *buf)
{
    if (buf[0] != 0xAA)
        return;

    uint32_t freq  = buf[1]+(buf[2] << 8)+(buf[3]<<16);
    uint8_t amp1   = buf[4];
    uint8_t harm1   = buf[5];
    uint8_t amp2   = buf[6];
    uint8_t harm2   = buf[7];
    uint16_t phase = buf[8] + (buf[9] << 8);
    uint8_t crc   = buf[10];

    uint8_t calc = CalcCRC(buf);
    if (calc != crc){
        return;}

    SetFrequency(freq,amp1,harm1,amp2,harm2,phase);
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
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  GenerateSineTable(1,1,1,1);
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)sinphase_table, NUMBER_OF_SAMPLE, DAC_ALIGN_12B_R);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint8_t b = rx_byte;

        if (!receiving)
        {
            if (b == 0xAA)
            {
                receiving = 1;
                frame_index = 0;
                frame_buf[frame_index++] = b;
            }
        }
        else
        {
            frame_buf[frame_index++] = b;
            if (frame_index >= FRAME_SIZE)
            {
                receiving = 0;
                ProcessFrame(frame_buf);
            }
        }

        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
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
