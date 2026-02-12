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
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Compile_Data.h"
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


char MENU_POPUP_LED_mode = 0;
char MAIN_BELL_LED_mode = 0;
char SUB_BELL_LED_mode = 0;
char LOCAL_BELL_LED_mode = 0;
char SIREN_LED_mode = 0;
char EMERGENCY_LED_mode = 0;
char FIRE_LED_mode = 0;
char RUN_LED_mode = 0;
char ERR_LED_mode = 0;

char Pre_Key_MENU_POPUP;
char Pre_Key_SUB_BELL_STOP;
char Pre_Key_SIREN_STOP;
char Pre_Key_MAIN_BELL_STOP;
char Pre_Key_LOCAL_BELL_STOP;
char Pre_Key_EMERGENCY_STOP;

char Ext_phpne_mode;
char Local_phone_mode;
char phone_jack_mode;
char Sub_buzzer4_mode;
char Sub_buzzer3_mode;
char Sub_buzzer2_mode;
char Sub_buzzer1_mode;
char Main_buzzer_mode;

char Local_phone_mode_S[Phone_Reas_Num];
char phone_jack_mode_S[Phone_Reas_Num];

uint16_t Local_phone_mode_Sum;
uint16_t phone_jack_mode_Sum;


uint8_t inf_LED;
uint8_t Pre_inf_SWI,inf_SWI;
uint8_t Pre_inf_Other, inf_Other;

uint8_t Key_Push_Status;

uint8_t Uart_rx1_buf[rx1_buf_len] ;
uint8_t Uart_rx1_buf_tmp[1] ;
int rx1_State ;
int rx1_buf_count;
int rx1_buf_count_tmp;
int rx1_Receive_complete;

uint8_t Uart_tmp[] = "Test\r\n";

uint8_t Uart_rx2_buf[rx2_buf_len] ;
uint8_t Uart_rx2_buf_tmp[3] ;
int rx2_State;
int rx2_buf_count;
int rx2_buf_count_tmp;
int rx2_Receive_complete;

int Err_Cnt[3];

uint8_t Uart_tx_buf[tx_buf_len] ;

uint8_t inf_LED_sub[7];
uint8_t inf_SWI_sub[7];
uint8_t inf_Other_sub[8];

uint8_t Led_Toggle_mode = 0;


uint8_t Sub_TB_Wait_cnt= 50;
uint8_t Sub_DO24_Wait_cnt= 3;
uint8_t Sub_MCC_Wait_cnt= 15;
uint8_t Sub_MCC_R_Wait_cnt= 35;

uint8_t Sub_EBC_Wait_cnt= 3;



uint8_t Sub_DO24_Set_Num = 5;
#define Sub_DO24_Num     5
uint8_t Sub_DO24_R_Data[Sub_DO24_Num][Sub_DO24_R_length] ;
uint8_t Sub_DO24_state[Sub_DO24_Num];


uint8_t Sub_MCC_Set_Num  = 3;
#define Sub_MCC_Num 3
uint8_t Sub_MCC_R_Data[Sub_MCC_Num][Sub_MCC_R_length] ;
uint8_t Sub_MCC_S_Data[Sub_MCC_Num][Sub_MCC_S_length] ;
uint8_t Sub_MCC_state[Sub_MCC_Num];


uint8_t MCC_LED_Set[Sub_MCC_Num][40];  // 모든 led 정보
uint8_t LED_Data_Set[Sub_MCC_Num][5];  // MCC_LED_Set을 통신용 데이타로  변경
uint8_t PUMP_AUTO_set[Sub_MCC_Num][6];
uint8_t PUMP_STOP_set[Sub_MCC_Num][6];
uint8_t PUMP_MANUAL_set[Sub_MCC_Num][6];
uint8_t PUMP_CHECK_set[Sub_MCC_Num][6];
uint8_t PUMP_PS_set[Sub_MCC_Num][6];
uint8_t MCC_HANJUN_state, MCC_BALJUN_State;
uint8_t MCC_HANJUN_set[Sub_MCC_Num];
uint8_t MCC_BALJUN_set[Sub_MCC_Num];
uint8_t MCC_SW_LOCK_ON_set[Sub_MCC_Num];
uint8_t MCC_SW_LOCK_OFF_set[Sub_MCC_Num];
uint8_t MCC_TERMINATION_set[Sub_MCC_Num];
uint8_t Auto_Dip_set[Sub_MCC_Num];
uint8_t Auto_Dip[Sub_MCC_Num][6];

uint8_t PUMP_PS_Pre_set[Sub_MCC_Num][6];
uint8_t PUMP_PS_set_Mode[Sub_MCC_Num][6];


uint8_t Sub_Relay_Set_Num = 3;
#define Sub_Relay_Num 3
uint8_t Sub_Relay_R_Data[Sub_Relay_Num][Sub_Relay_R_length] ;
uint8_t Sub_Relay_S_Data[Sub_Relay_Num][Sub_Relay_S_length] ;
uint8_t Sub_Motor_Relay[Sub_Relay_Num][6];
uint8_t Sub_Motor_Run_Mode[Sub_Relay_Num][6]; // 현재 모터 동작 상태 저장
uint8_t Sub_Relay_state[Sub_Relay_Num];

uint8_t TB_UI_set = 0;
uint8_t Sub_TB_R_Data[Sub_TB_R_length] ;
uint8_t Sub_TB_S_Data[Sub_TB_S_length] ;
uint8_t Sub_TB_state;

uint8_t EB_UI_set = 0;
uint8_t Sub_EB_R_Data[Sub_EB_R_length] ;
uint8_t Sub_EB_S_Data[Sub_EB_S_length] ;
uint8_t Sub_EB_state;

char loop_out;


#define Version_SW		0
#define Version_MCC_1	1
#define Version_MCC_2	2
#define Version_MCC_3	3
#define Version_MCC_R_1	4
#define Version_MCC_R_2	5
#define Version_MCC_R_3	6
#define Version_TB		7
#define Version_DO_1	8
#define Version_DO_2	9
#define Version_DO_3	10
#define Version_DO_4	11
#define Version_DO_5	12
#define Version_EBC		13

uint8_t Version_Info[14][15];

uint8_t SUB_UART_TX_buf[rx2_buf_len] ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///////////////Start timer interrupt operation function///////////////////

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1){
	  HAL_IWDG_Refresh(&hiwdg);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USB_PCD_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
//  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /* huart1 RX Interrupt  Enable */

  /* Process Unlocked */
  __HAL_UNLOCK(&huart1);
  /* Enable the UART Parity Error Interrupt */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_PE);
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  /* huart2 RX Interrupt  Enable */
  /* Process Unlocked */
  __HAL_UNLOCK(&huart2);
  /* Enable the UART Parity Error Interrupt */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_PE);
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

  HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);

  HAL_TIM_OC_Start_IT(&htim16,TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim15,TIM_CHANNEL_1);

  HAL_GPIO_WritePin(BUZZER_1_GPIO_Port, BUZZER_1_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BUZZER_2_GPIO_Port, BUZZER_2_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BUZZER_3_GPIO_Port, BUZZER_3_Pin,GPIO_PIN_RESET);

  FIRE_LED(GPIO_PIN_RESET);
  MENU_POPUP_LED(GPIO_PIN_RESET);
  MAIN_BELL_LED(GPIO_PIN_RESET);
  SUB_BELL_LED(GPIO_PIN_RESET);
  LOCAL_BELL_LED(GPIO_PIN_RESET);
  SIREN_LED(GPIO_PIN_RESET);
  EMERGENCY_LED(GPIO_PIN_RESET);

  HAL_GPIO_WritePin(BUZZER_1_GPIO_Port, BUZZER_1_Pin,GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(BUZZER_1_GPIO_Port, BUZZER_1_Pin,GPIO_PIN_RESET);

  HAL_GPIO_WritePin(BUZZER_2_GPIO_Port, BUZZER_2_Pin,GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(BUZZER_2_GPIO_Port, BUZZER_2_Pin,GPIO_PIN_RESET);

  HAL_GPIO_WritePin(BUZZER_3_GPIO_Port, BUZZER_3_Pin,GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(BUZZER_3_GPIO_Port, BUZZER_3_Pin,GPIO_PIN_RESET);

  FIRE_LED(GPIO_PIN_SET);
  MENU_POPUP_LED(GPIO_PIN_SET);
  MAIN_BELL_LED(GPIO_PIN_SET);
  SUB_BELL_LED(GPIO_PIN_SET);
  LOCAL_BELL_LED(GPIO_PIN_SET);
  SIREN_LED(GPIO_PIN_SET);
  EMERGENCY_LED(GPIO_PIN_SET);

  for(int i=0;i<Sub_DO24_Set_Num;i++){
    for(int j=0;j<Sub_DO24_R_length;j++){
      Sub_DO24_R_Data[i][j] = 0;
    }
    Sub_DO24_state[i] = 0;
  }

  for(int i=0;i<Sub_MCC_Set_Num;i++){
    for(int j=0;j<Sub_MCC_R_length;j++){
      Sub_MCC_R_Data[i][j] = 0;
    }
    for(int j=0;j<Sub_MCC_S_length;j++){
      Sub_MCC_S_Data[i][j] = 0;
    }
    Sub_MCC_state[i] = 0;
  }

  Compile_Date();
  Read_Sub_Version();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	  __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
	  /* Enable the UART Data Register not empty Interrupt */
	  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

	  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	  __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
	  /* Enable the UART Data Register not empty Interrupt */
	  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

	  Read_Sub_Version();


	    Read_Ext_TB_Status();
	    HAL_Delay(1);
	    Read_EB_Status();
	    HAL_Delay(1);

	    for(int i=1;i<Sub_DO24_Set_Num +1;i++){
	      for(int k=0; k<2;k++){
	        Sub_DO24_state[i-1] = Read_DO24_Status(i);
	        HAL_Delay(1);
	        if(Sub_DO24_state[i-1]  == 1){
	          break;
	        }
	      }
	      HAL_Delay(1);
	    }
	  //  HAL_Delay(10);
	    for(int i=1;i<Sub_MCC_Set_Num +1;i++){

	      for(int k=0; k<2;k++){
	        Sub_MCC_state[i-1] = Read_MCC_Status(i);
	        HAL_Delay(1);
	        if(Sub_MCC_state[i-1]  == 1){
	          break;
	        }
	      }

	      if(Sub_MCC_state[i-1]  == 0){
	        Err_Cnt[i-1]++;
	      }

	      Set_Led_Data(i);
	      HAL_Delay(1);
	    }
	   // HAL_Delay(10);
	    for(int i=1;i<Sub_Relay_Set_Num +1;i++){
	      Sub_Relay_state[i-1] = Read_MCC_R_Status(i);
	      Set_pump(i);    //펌프 설정
	      HAL_Delay(1);
	    }


	      // HAL_Delay(10);
	    for(int i=1;i<Sub_Relay_Set_Num +1;i++){
	      Sub_Relay_state[i-1] = Read_MCC_R_Status(i);
	      Set_pump(i);    //펌프 설정
	      HAL_Delay(1);
	    }

	    //HAL_Delay(10);

	    for(int i=1;i<Sub_Relay_Set_Num +1;i++){
	      UI_Com_MCC_S_Relay_val(i);        // Relay의 값을 mcc에 적용
	      HAL_Delay(1);
	    }
	    //HAL_Delay(10);

	     for(int i=1;i<Sub_Relay_Set_Num +1;i++){
	      UI_Com_MCC_S_Relay_val(i);        // Relay의 값을 mcc에 적용
	      HAL_Delay(1);
	    }
	    //HAL_Delay(10);


	     UI_Com_SW_r();
	     loop_out = 1;
	     for(int i=0;i<250;i++){
	       if(rx1_Receive_complete == 1){
	         UI_Com();
	         loop_out = 0;
	         break;
	       }
	       HAL_Delay(1);
	     }

	     // 수신 완료 신호 기다리기

	     for(int i=0;i<400;i++){
	       if(rx1_Receive_complete == 1){
	         UI_Com();
	         if(loop_out == 1){
	           break;
	         }
	       }
	       HAL_Delay(1);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


GPIO_PinState Read_Key_MENU_POPUP(void){
  GPIO_PinState read_key;
  read_key = HAL_GPIO_ReadPin(MENU_POPUP_GPIO_Port, MENU_POPUP_Pin);
  return read_key;
}

GPIO_PinState Read_Key_SUB_BELL_STOP(void){
  GPIO_PinState read_key;
  read_key = HAL_GPIO_ReadPin(SUB_BELL_STOP_GPIO_Port, SUB_BELL_STOP_Pin);
  return read_key;
}

GPIO_PinState Read_Key_SIREN_STOP(void){
  GPIO_PinState read_key;
  read_key = HAL_GPIO_ReadPin(SIREN_STOP_GPIO_Port, SIREN_STOP_Pin);
  return read_key;
}

GPIO_PinState Read_Key_MAIN_BELL_STOP(void){
  GPIO_PinState read_key;
  read_key = HAL_GPIO_ReadPin(MAIN_BELL_STOP_GPIO_Port, MAIN_BELL_STOP_Pin);
  return read_key;
}

GPIO_PinState Read_Key_LOCAL_BELL_STOP(void){
  GPIO_PinState read_key;
  read_key = HAL_GPIO_ReadPin(LOCAL_BELL_STOP_GPIO_Port, LOCAL_BELL_STOP_Pin);

  return read_key;
}

GPIO_PinState Read_Key_EMERGENCY_STOP(void){
  GPIO_PinState read_key;
  read_key = HAL_GPIO_ReadPin(EMERGENCY_STOP_GPIO_Port, EMERGENCY_STOP_Pin);
  return read_key;
}

int Read_phone_jack(void){
  int read_key;
  if(HAL_GPIO_ReadPin(PHONE_JACK_IN_GPIO_Port, PHONE_JACK_IN_Pin) == GPIO_PIN_RESET){
    read_key = 1;
  }
  else{
    read_key = 0;
  }
  return read_key;
}


int Read_Local_phone(void){
  int read_key;
  if(HAL_GPIO_ReadPin(LOCAL_PHONE_IN_GPIO_Port, LOCAL_PHONE_IN_Pin) == GPIO_PIN_RESET){
    read_key = 1;
  }
  else{
    read_key = 0;
  }
  return read_key;
}


void MENU_POPUP_LED(GPIO_PinState LED){
  if(LED == GPIO_PIN_RESET){
    LED = GPIO_PIN_SET;
  }
  else{
    LED = GPIO_PIN_RESET;
  }
  HAL_GPIO_WritePin(MENU_POPUP_LED_GPIO_Port, MENU_POPUP_LED_Pin, LED);
}

void MAIN_BELL_LED(GPIO_PinState LED){
  if(LED == GPIO_PIN_RESET){
    LED = GPIO_PIN_SET;
  }
  else{
    LED = GPIO_PIN_RESET;
  }
  HAL_GPIO_WritePin(MAIN_BELL_LED_GPIO_Port, MAIN_BELL_LED_Pin, LED);
}

void SUB_BELL_LED(GPIO_PinState LED){
  if(LED == GPIO_PIN_RESET){
    LED = GPIO_PIN_SET;
  }
  else{
    LED = GPIO_PIN_RESET;
  }
  HAL_GPIO_WritePin(SUB_BELL_LED_GPIO_Port, SUB_BELL_LED_Pin, LED);
}

void LOCAL_BELL_LED(GPIO_PinState LED){
  if(LED == GPIO_PIN_RESET){
    LED = GPIO_PIN_SET;
  }
  else{
    LED = GPIO_PIN_RESET;
  }
  HAL_GPIO_WritePin(LOCAL_BELL_LED_GPIO_Port, LOCAL_BELL_LED_Pin, LED);
}

void SIREN_LED(GPIO_PinState LED){
  if(LED == GPIO_PIN_RESET){
    LED = GPIO_PIN_SET;
  }
  else{
    LED = GPIO_PIN_RESET;
  }
  HAL_GPIO_WritePin(SIREN_LED_GPIO_Port, SIREN_LED_Pin, LED);
}

void EMERGENCY_LED(GPIO_PinState LED){
  if(LED == GPIO_PIN_RESET){
    LED = GPIO_PIN_SET;
  }
  else{
    LED = GPIO_PIN_RESET;
  }
  HAL_GPIO_WritePin(EMERGENCY_BD_LED_GPIO_Port, EMERGENCY_BD_LED_Pin, LED);
}

void FIRE_LED(GPIO_PinState LED){

  if(LED == GPIO_PIN_RESET){
    LED = GPIO_PIN_SET;
  }
  else{
    LED = GPIO_PIN_RESET;
  }

  HAL_GPIO_WritePin(FIRE_LED_GPIO_Port, FIRE_LED_Pin, LED);
}

void RUN_LED(GPIO_PinState LED){
  if(LED == GPIO_PIN_RESET){
    LED = GPIO_PIN_SET;
  }
  else{
    LED = GPIO_PIN_RESET;
  }
  HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, LED);
}

void ERR_LED(GPIO_PinState LED){
  if(LED == GPIO_PIN_RESET){
    LED = GPIO_PIN_SET;
  }
  else{
    LED = GPIO_PIN_RESET;
  }
  HAL_GPIO_WritePin(ERR_LED_GPIO_Port, ERR_LED_Pin, LED);
}

void TX_LED(GPIO_PinState LED){
  if(LED == GPIO_PIN_RESET){
    LED = GPIO_PIN_SET;
  }
  else{
    LED = GPIO_PIN_RESET;
  }
  HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, LED);
}

void RX_LED(GPIO_PinState LED){
  if(LED == GPIO_PIN_RESET){
    LED = GPIO_PIN_SET;
  }
  else{
    LED = GPIO_PIN_RESET;
  }
  HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, LED);
}


int Uart1_crc = 0;

void UI_Com(void){

  // CRC 체크

  int i;

  // 전송할 중계기 정보를 확인한다.

  Uart1_crc = 0;

  for(i = 0; i <rx1_buf_count_tmp - 5 ; i++){
    Uart1_crc = Uart1_crc ^ Uart_rx1_buf[i+2];
  }

  if(Uart1_crc == Uart_rx1_buf[rx1_buf_count_tmp - 3]){
    //0x51 , 'Q' ,중계기 정보 요청 (UI에서 설정한 값만)

    if(Uart_rx1_buf[3] == 0x51){
      if(Uart_rx1_buf[4] == 0x41){
       UI_Com_All_Q();		//0x41 , 'A' ,All board
      }
      else if(Uart_rx1_buf[4] == 0x53){
         UI_Com_SW_Q();		//0x53 , 'S' Switch board 정보
      }
      else if(Uart_rx1_buf[4] == 0x4D){

        //UI_Com_MCC_Q(Uart_rx1_buf[5]);		//0x4C , 'M' ,MCC board 정보
        for(i=1;i<Sub_MCC_Set_Num +1;i++){
          if(Sub_MCC_state[i - 1] == 1){

          }

          UI_Com_MCC_Q(i);
        }

      }
      else if(Uart_rx1_buf[4] == 0x52){
      //  UI_Com_Relay_Q(Uart_rx1_buf[5]);	//0x52 , 'R' ,Relay board 정보

        for(i=1;i<Sub_Relay_Set_Num +1;i++){
          if(Sub_Relay_state[i - 1] == 1){

          }

          UI_Com_Relay_Q(i);
        }

      }
      else if(Uart_rx1_buf[4] == 0x44){
        //UI_Com_DO24_Q(Uart_rx1_buf[5]);		//0x44 , 'D' ,DO24 board 정보

          for(i=1;i<Sub_DO24_Set_Num +1;i++){
            if(Sub_DO24_state[i - 1] == 1){

            }

            UI_Com_DO24_Q(i);
          }
      }
      else if(Uart_rx1_buf[4] == 0x54){
        UI_Com_TB_Q();	//0x54 , 'T' ,TB board 정보
      }
      else if(Uart_rx1_buf[4] == 0x42){
        UI_Com_EB_Q();	//0x42 , 'B' ,EB board 정보
      }
    }
    else if(Uart_rx1_buf[3] == 0x53){

      if(Uart_rx1_buf[4] == 0x41){
       UI_Com_All_S();		//0x41 , 'A' ,All board
      }
      else if(Uart_rx1_buf[4] == 0x53){
         UI_Com_SW_S();		//0x53 , 'S' Switch board 정보
      }
      else if(Uart_rx1_buf[4] == 0x4D){
        UI_Com_MCC_S(Uart_rx1_buf[5]);		//0x4C , 'M' ,MCC board 정보
        UI_Com_MCC_Q(Uart_rx1_buf[5]);
        //Run_Sub_LDT();
      }
      else if(Uart_rx1_buf[4] == 0x52){
        UI_Com_Relay_S();	//0x52 , 'R' ,Relay board 정보

        //Run_Sub_LDT();
      }
      else if(Uart_rx1_buf[4] == 0x44){
        UI_Com_DO24_S();		//0x44 , 'D' ,DO24 board 정보
         UI_Com_DO24_Q(Uart_rx1_buf[5]);
        //Run_Sub_LDT();
      }
      else if(Uart_rx1_buf[4] == 0x54){
        UI_Com_TB_S();	//0x54 , 'T' ,TB board 정보
      }
      else if(Uart_rx1_buf[4] == 0x42){
        UI_Com_EB_S();	//0x42 , 'B' ,EB board 설정
      }
    }
    else if(Uart_rx1_buf[3] == 0x56){ //0x56 , V' ,버전 정보

    	UI_Com_V();
    	Read_Sub_Version();

    }
    else if(Uart_rx1_buf[3] == 0x45){
      loop_out = 1;
    }
  }

  rx1_Receive_complete = 0;

}

uint8_t Uart1_tx_Header[4];
uint8_t Uart1_tx_ck[1];
uint8_t Uart1_tx_END[2];


void UI_Com_All_Q(void){
  int i;

  UI_Com_SW_Q();
  UI_Com_TB_Q();
  UI_Com_EB_Q();

  for(i=1;i<Sub_DO24_Set_Num +1;i++){
    if(Sub_DO24_state[i - 1] == 1){

    }

    UI_Com_DO24_Q(i);
  }

  for(i=1;i<Sub_MCC_Set_Num +1;i++){
    if(Sub_MCC_state[i - 1] == 1){

    }

    UI_Com_MCC_Q(i);
  }


  for(i=1;i<Sub_Relay_Set_Num +1;i++){
    if(Sub_Relay_state[i - 1] == 1){

    }

    UI_Com_Relay_Q(i);
  }

  //UI_Com_MCC_Q();
  //UI_Com_Relay_Q();

}
void UI_Com_SW_Q(void){

  int i;
  uint8_t Uart1_tx_data[4];

  TX_LED(GPIO_PIN_RESET);
  Read_Sw_LTD_Status();

  Uart1_tx_Header[0] = 0x53;    //S
  Uart1_tx_Header[1] = 0x54;    //T
  Uart1_tx_Header[2] = 0x53;    //S
  Uart1_tx_Header[3] = 0x71;    //q


  if(HAL_UART_Transmit(&huart1, Uart1_tx_Header, sizeof(Uart1_tx_Header), 1000)!= HAL_OK)
  {
    Error_Handler();
  }


  Uart1_tx_data[0] = 0x53;
  Uart1_tx_data[1] = inf_LED;
  Uart1_tx_data[2] = inf_SWI;
  Uart1_tx_data[3] = inf_Other;

  if(HAL_UART_Transmit(&huart1, Uart1_tx_data, sizeof(Uart1_tx_data), 1000)!= HAL_OK)
  {
    Error_Handler();
  }

  Uart1_tx_ck[0] = 0;

  Uart1_tx_ck[0] = Uart1_tx_Header[2]^ Uart1_tx_Header[3];

  for(i = 0; i <4 ; i++){
    Uart1_tx_ck[0] = Uart1_tx_ck[0]  ^ Uart1_tx_data[i];
  }


  if(HAL_UART_Transmit(&huart1, Uart1_tx_ck, sizeof(Uart1_tx_ck), 1000)!= HAL_OK)
  {
    Error_Handler();
  }

  Uart1_tx_END[0] = 0x45;
  Uart1_tx_END[1] = 0x44;
  if(HAL_UART_Transmit(&huart1, Uart1_tx_END, sizeof(Uart1_tx_END), 1000)!= HAL_OK)
  {
    Error_Handler();
  }

  for(i=0;i<8;i++){
    inf_SWI_sub[i] = 0;
  }

  TX_LED(GPIO_PIN_SET);
}


void UI_Com_MCC_Q(int Address){

  #define  Uart1_length_MCC  18
  int i;
  uint8_t Uart1_tx_data[Uart1_length_MCC] ,UART_Crc;


  UART_Crc = 0;

  TX_LED(GPIO_PIN_RESET);


  for(i=0;i<Uart1_length_MCC;i++){
    Uart1_tx_data[i] = 0;
  }

  Sub_MCC_R_Data[Address -1 ][Sub_MCC_R_length - 5] = Sub_MCC_state[Address -1 ];

  Uart1_tx_data[0] = 0x53;    //S
  Uart1_tx_data[1] = 0x54;    //T
  Uart1_tx_data[2] = 0x53;    //S
  Uart1_tx_data[3] = 0x71;    //q

  Uart1_tx_data[4] = 0x4D;      //MCC board : '0x4D'
  Uart1_tx_data[5] = Address;

  for(i=6;i<Uart1_length_MCC;i++){
    Uart1_tx_data[i] = Sub_MCC_R_Data[Address -1 ][i-1];
  }


  Uart1_tx_data[Uart1_length_MCC - 4] = 0;

  for(i = 0; i <(Uart1_length_MCC - 5) ; i++){
    UART_Crc = UART_Crc ^ Uart1_tx_data[i + 2];
  }


  Uart1_tx_data[Uart1_length_MCC - 3] = UART_Crc;

  Uart1_tx_data[Uart1_length_MCC - 2] = 0x45;
  Uart1_tx_data[Uart1_length_MCC - 1] = 0x44;

  if(HAL_UART_Transmit(&huart1, Uart1_tx_data, sizeof(Uart1_tx_data), 1000)!= HAL_OK)
  {
    Error_Handler();
  }

  TX_LED(GPIO_PIN_SET);

}

void UI_Com_EB_Q(void){

  #define  Uart1_length_EB  Sub_EB_R_length
  int i;
  uint8_t Uart1_tx_data[Uart1_length_EB] , UART_Crc;

  TX_LED(GPIO_PIN_RESET);

  Uart1_tx_data[0] = 0x53;    //S
  Uart1_tx_data[1] = 0x54;    //T
  Uart1_tx_data[2] = 0x53;    //S
  Uart1_tx_data[3] = 0x71;    //q

  Uart1_tx_data[4] = 0x42;      //EB board: '0x42'


  for(i=0;i<(Uart1_length_EB - 7);i++){
    Uart1_tx_data[i+5] = Sub_EB_R_Data[i+4];
  }

  Uart1_tx_data[Uart1_length_EB - 5] = Sub_EB_state;
  Uart1_tx_data[Uart1_length_EB - 4] = Sub_EB_R_Data[Uart1_length_EB - 4];

  UART_Crc = 0;


  for(i = 0; i <(Uart1_length_EB - 5) ; i++){
    UART_Crc = UART_Crc^ Uart1_tx_data[i + 2];
  }


  Uart1_tx_data[Uart1_length_EB - 3] = UART_Crc;

  Uart1_tx_data[Uart1_length_EB - 2] = 0x45;
  Uart1_tx_data[Uart1_length_EB - 1] = 0x44;

  if(HAL_UART_Transmit(&huart1, Uart1_tx_data, sizeof(Uart1_tx_data), 1000)!= HAL_OK)
  {
    Error_Handler();
  }

  TX_LED(GPIO_PIN_SET);

}

void UI_Com_Relay_Q(int Address){

  #define  Uart1_length_Relay  16
  int i;
  uint8_t Uart1_tx_data[Uart1_length_Relay] , UART_Crc;

  TX_LED(GPIO_PIN_RESET);

  for(i=0;i<Uart1_length_Relay;i++){
    Uart1_tx_data[i] = 0;
  }

  Sub_Relay_R_Data[Address -1 ][Sub_Relay_R_length - 5] = Sub_Relay_state[Address -1 ];


  Uart1_tx_data[0] = 0x53;    //S
  Uart1_tx_data[1] = 0x54;    //T
  Uart1_tx_data[2] = 0x53;    //S
  Uart1_tx_data[3] = 0x71;    //q

  Uart1_tx_data[4] = 0x52;      //Relay board : '0x52'
  Uart1_tx_data[5] = Address;

  for(i=6;i<Uart1_length_Relay;i++){
    Uart1_tx_data[i] = Sub_Relay_R_Data[Address -1 ][i-1];
  }

  Uart1_tx_data[Uart1_length_Relay - 4] = 0;

  UART_Crc = 0;

  for(i = 0; i <(Uart1_length_Relay - 5) ; i++){
    UART_Crc = UART_Crc^ Uart1_tx_data[i + 2];
  }

  Uart1_tx_data[Uart1_length_Relay - 3] = UART_Crc;

  Uart1_tx_data[Uart1_length_Relay - 2] = 0x45;
  Uart1_tx_data[Uart1_length_Relay - 1] = 0x44;

  if(HAL_UART_Transmit(&huart1, Uart1_tx_data, sizeof(Uart1_tx_data), 1000)!= HAL_OK)
  {
    Error_Handler();
  }

  TX_LED(GPIO_PIN_SET);

}
void UI_Com_DO24_Q(int Address){

  #define  Uart1_length_DO24  15
  int i;
  uint8_t Uart1_tx_data[Uart1_length_DO24] , UART_Crc;

  TX_LED(GPIO_PIN_RESET);

  for(i=0;i<Uart1_length_DO24;i++){
    Uart1_tx_data[i] = 0;
  }

  Uart1_tx_data[0] = 0x53;    //S
  Uart1_tx_data[1] = 0x54;    //T
  Uart1_tx_data[2] = 0x53;    //S
  Uart1_tx_data[3] = 0x71;    //q
  Uart1_tx_data[4] = 0x44;      //DO24 boaed : '0x44'
  Uart1_tx_data[5] = Address;



  for(i=0;i<5;i++){
    Uart1_tx_data[i+6] = Sub_DO24_R_Data[Address -1 ][i+5];
  }

  Uart1_tx_data[10] = Sub_DO24_state[Address -1 ];

  UART_Crc = 0;


  for(i = 0; i <(Uart1_length_DO24 - 5) ; i++){
    UART_Crc = UART_Crc^ Uart1_tx_data[i + 2];
  }

  Uart1_tx_data[Uart1_length_DO24 - 3] = UART_Crc;

  Uart1_tx_data[Uart1_length_DO24 - 2] = 0x45;
  Uart1_tx_data[Uart1_length_DO24 - 1] = 0x44;

  if(HAL_UART_Transmit(&huart1, Uart1_tx_data, sizeof(Uart1_tx_data), 1000)!= HAL_OK)
  {
    Error_Handler();
  }

  TX_LED(GPIO_PIN_SET);


}
void UI_Com_TB_Q(void){

  #define  Uart1_length_TB  19
  int i;
  uint8_t Uart1_tx_data[Uart1_length_TB] , UART_Crc;

  TX_LED(GPIO_PIN_RESET);

  Uart1_tx_data[0] = 0x53;    //S
  Uart1_tx_data[1] = 0x54;    //T
  Uart1_tx_data[2] = 0x53;    //S
  Uart1_tx_data[3] = 0x71;    //q

  Uart1_tx_data[4] = 0x54;      //TB board: '0x54'


  for(i=0;i<(Uart1_length_TB - 7);i++){
    Uart1_tx_data[i+5] = Sub_TB_R_Data[i+4];
  }

  Uart1_tx_data[Uart1_length_TB - 5] = Sub_TB_state;

  UART_Crc = 0;

  //UART_Crc = Uart1_tx_Header[2]^ Uart1_tx_Header[3];

  for(i = 0; i <(Uart1_length_TB - 5) ; i++){
    UART_Crc = UART_Crc^ Uart1_tx_data[i + 2];
  }

  Uart1_tx_data[Uart1_length_TB - 4] = Sub_TB_R_Data[14];

  Uart1_tx_data[Uart1_length_TB - 3] = UART_Crc;

  Uart1_tx_data[Uart1_length_TB - 2] = 0x45;
  Uart1_tx_data[Uart1_length_TB - 1] = 0x44;

  if(HAL_UART_Transmit(&huart1, Uart1_tx_data, sizeof(Uart1_tx_data), 1000)!= HAL_OK)
  {
    Error_Handler();
  }

  TX_LED(GPIO_PIN_SET);
  /*
  if(Sub_TB_R_Data[7] == 0){
    MCC_HANJUN_set = 0;
    MCC_BALJUN_set = 0;
  }
  else if(Sub_TB_R_Data[7] == 1){
    MCC_HANJUN_set = 1;
    MCC_BALJUN_set = 0;
  }
  else if(Sub_TB_R_Data[7] == 2){
    MCC_HANJUN_set = 0;
    MCC_BALJUN_set = 1;
  }
  else{
    MCC_HANJUN_set = 0;
    MCC_BALJUN_set = 0;
  }
*/
}


void UI_Com_All_S(void){
  UI_Com_SW_S();
  //UI_Com_MCC_S();
  //UI_Com_Relay_S();
  //UI_Com_DO24_S();
  //UI_Com_TB_S();
}
void UI_Com_SW_S(void){
  int i;

  inf_LED  = Uart_rx1_buf[5];
  inf_Other = Uart_rx1_buf[7];

  for(i=0;i<7;i++){
    inf_LED_sub[i] =  (inf_LED >> i) & 0x01;
    inf_Other_sub[i] =  (inf_Other >> i) & 0x01;
  }

  if(inf_LED_sub[6] == 1){
    FIRE_LED(GPIO_PIN_RESET);
    FIRE_LED_mode = 1;
  }
  else{
    FIRE_LED(GPIO_PIN_SET);
    FIRE_LED_mode = 0;
  }

  if(inf_LED_sub[5] == 1){
    MENU_POPUP_LED(GPIO_PIN_RESET);
  }
  else{
    MENU_POPUP_LED(GPIO_PIN_SET);
  }

  if(inf_LED_sub[4] == 1){
   // MAIN_BELL_LED(GPIO_PIN_RESET);
    MAIN_BELL_LED_mode = 1;
  }
  else{
   // MAIN_BELL_LED(GPIO_PIN_SET);
    MAIN_BELL_LED_mode = 0;
  }

  if(inf_LED_sub[3] == 1){
    //SUB_BELL_LED(GPIO_PIN_RESET);
    SUB_BELL_LED_mode = 1;
  }
  else{
    //SUB_BELL_LED(GPIO_PIN_SET);
    SUB_BELL_LED_mode  = 0;
  }

  if(inf_LED_sub[2] == 1){
    //LOCAL_BELL_LED(GPIO_PIN_RESET);
    LOCAL_BELL_LED_mode  = 1;
  }
  else{
    //LOCAL_BELL_LED(GPIO_PIN_SET);
    LOCAL_BELL_LED_mode  = 0;
  }

  if(inf_LED_sub[1] == 1){
    //SIREN_LED(GPIO_PIN_RESET);
    SIREN_LED_mode = 1;
  }
  else{
    //SIREN_LED(GPIO_PIN_SET);
    SIREN_LED_mode  = 0;
  }

  if(inf_LED_sub[0] == 1){
    //EMERGENCY_LED(GPIO_PIN_RESET);
    EMERGENCY_LED_mode =1;
  }
  else{
    //EMERGENCY_LED(GPIO_PIN_SET);
    EMERGENCY_LED_mode  = 0;
  }

  /*
  if(inf_Other_sub[7] == 1){
    Ext_phpne_mode = 1;
  }
  else{
    Ext_phpne_mode = 0;
  }

  if(inf_Other_sub[6] == 1){
    Local_phone_mode = 1;
  }
  else{
    Local_phone_mode = 0;
  }

  if(inf_Other_sub[5] == 1){
    phone_jack_mode = 1;
  }
  else{
    phone_jack_mode = 0;
  }
  */

  if(inf_Other_sub[4] == 1){
    Sub_buzzer4_mode = 1;
  }
  else{
    Sub_buzzer4_mode = 0;
  }

  if(inf_Other_sub[3] == 1){
    Sub_buzzer3_mode = 1;
  }
  else{
    Sub_buzzer3_mode = 0;
  }

  if(inf_Other_sub[2] == 1){
    Sub_buzzer2_mode = 1;
  }
  else{
    Sub_buzzer2_mode = 0;
  }

  if(inf_Other_sub[1] == 1){
    Sub_buzzer1_mode = 1;
  }
  else{
    Sub_buzzer1_mode = 0;
  }

  if(inf_Other_sub[0] == 1){
    Main_buzzer_mode = 1;
  }
  else{
    Main_buzzer_mode = 0;
  }

  UI_Com_TB_S_Fire();
}


void UI_Com_MCC_S(int Address){

int Uart_crc = 0, i, Uart2_length;

  // Set MCC
  Uart_crc = 0;
  Uart2_length = Sub_MCC_S_length;

  Uart_tx_buf[0] = 0x53;    //S
  Uart_tx_buf[1] = 0x54;    //T
  Uart_tx_buf[2] = 0x53;    //S
  Uart_tx_buf[3] = 0x53;    //S
  Uart_tx_buf[4] = 0x4D;    //M
  Uart_tx_buf[5] = Uart_rx1_buf[5];    //address
  Uart_tx_buf[6] = Uart_rx1_buf[6];    //LED정보 1 ~8
  Uart_tx_buf[7] = Uart_rx1_buf[7];    //LED정보 9 ~16
  Uart_tx_buf[8] = Uart_rx1_buf[8];    //LED정보 17 ~24
  Uart_tx_buf[9] = Uart_rx1_buf[9];    //LED정보 25 ~32
  Uart_tx_buf[10] = Uart_rx1_buf[10];   //LED정보 33 ~40
  Uart_tx_buf[11] = 0x00;   //dummy1
  Uart_tx_buf[12] = 0x00;   //dummy2
  Uart_tx_buf[13] = 0x00;   //CRC
  Uart_tx_buf[14] = 0x45;   //E
  Uart_tx_buf[15] = 0x44;   //D

  if(Uart_rx1_buf[12] == 0){
    Sub_MCC_Set_Num = 1;
    Sub_Relay_Set_Num = 1;
  }
  else{
    Sub_MCC_Set_Num = Uart_rx1_buf[12];
    Sub_Relay_Set_Num = Uart_rx1_buf[12];
  }

  for(i = 0; i <Uart2_length - 5 ; i++){
    Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
  }
  Uart_tx_buf[Uart2_length - 3] = Uart_crc ;

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);
  HAL_Delay(1);
  if(HAL_UART_Transmit(&huart2, Uart_tx_buf, Uart2_length, 1000)!= HAL_OK)
  {
    Error_Handler();
  }

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);


}

void UI_Com_MCC_S_Relay_val(int Address){

  int Uart_crc = 0, i, Uart2_length;

  // Set MCC
  Uart_crc = 0;
  Uart2_length = Sub_MCC_S_length;

  Uart_tx_buf[0] = 0x53;    //S
  Uart_tx_buf[1] = 0x54;    //T
  Uart_tx_buf[2] = 0x53;    //S
  Uart_tx_buf[3] = 0x53;    //S
  Uart_tx_buf[4] = 0x4D;    //M
  Uart_tx_buf[5] = Address;    //address
  Uart_tx_buf[6] = Uart_rx1_buf[6];    //LED정보 1 ~8
  Uart_tx_buf[7] = Uart_rx1_buf[7];    //LED정보 9 ~16
  Uart_tx_buf[8] = Uart_rx1_buf[8];    //LED정보 17 ~24
  Uart_tx_buf[9] = Uart_rx1_buf[9];    //LED정보 25 ~32
  Uart_tx_buf[10] = Uart_rx1_buf[10];   //LED정보 33 ~40
  Uart_tx_buf[11] = 0x00;   //dummy1
  Uart_tx_buf[12] = 0x00;   //dummy2
  Uart_tx_buf[13] = 0x00;   //CRC
  Uart_tx_buf[14] = 0x45;   //E
  Uart_tx_buf[15] = 0x44;   //D

  MCC_LED_Set[Address-1][0]  = PUMP_AUTO_set[Address-1][0] ;
  MCC_LED_Set[Address-1][5]  = PUMP_AUTO_set[Address-1][1] ;
  MCC_LED_Set[Address-1][10]  = PUMP_AUTO_set[Address-1][2] ;
  MCC_LED_Set[Address-1][15]  = PUMP_AUTO_set[Address-1][3] ;
  MCC_LED_Set[Address-1][20]  = PUMP_AUTO_set[Address-1][4] ;
  MCC_LED_Set[Address-1][25]  = PUMP_AUTO_set[Address-1][5] ;

  MCC_LED_Set[Address-1][1]  = PUMP_STOP_set[Address-1][0] ;
  MCC_LED_Set[Address-1][6]  = PUMP_STOP_set[Address-1][1] ;
  MCC_LED_Set[Address-1][11]  = PUMP_STOP_set[Address-1][2] ;
  MCC_LED_Set[Address-1][16]  = PUMP_STOP_set[Address-1][3] ;
  MCC_LED_Set[Address-1][21]  = PUMP_STOP_set[Address-1][4] ;
  MCC_LED_Set[Address-1][26]  = PUMP_STOP_set[Address-1][5] ;

  MCC_LED_Set[Address-1][2]  = PUMP_MANUAL_set[Address-1][0] ;
  MCC_LED_Set[Address-1][7]  = PUMP_MANUAL_set[Address-1][1] ;
  MCC_LED_Set[Address-1][12]  = PUMP_MANUAL_set[Address-1][2] ;
  MCC_LED_Set[Address-1][17]  = PUMP_MANUAL_set[Address-1][3] ;
  MCC_LED_Set[Address-1][22]  = PUMP_MANUAL_set[Address-1][4] ;
  MCC_LED_Set[Address-1][27]  = PUMP_MANUAL_set[Address-1][5] ;

  MCC_LED_Set[Address-1][3]  = PUMP_CHECK_set[Address-1][0] ;
  MCC_LED_Set[Address-1][8]  = PUMP_CHECK_set[Address-1][1] ;
  MCC_LED_Set[Address-1][13]  = PUMP_CHECK_set[Address-1][2] ;
  MCC_LED_Set[Address-1][18]  = PUMP_CHECK_set[Address-1][3] ;
  MCC_LED_Set[Address-1][23]  = PUMP_CHECK_set[Address-1][4] ;
  MCC_LED_Set[Address-1][28]  = PUMP_CHECK_set[Address-1][5] ;

  MCC_LED_Set[Address-1][4]  = PUMP_PS_set[Address-1][0] ;
  MCC_LED_Set[Address-1][9]  = PUMP_PS_set[Address-1][1] ;
  MCC_LED_Set[Address-1][14]  = PUMP_PS_set[Address-1][2] ;
  MCC_LED_Set[Address-1][19]  = PUMP_PS_set[Address-1][3] ;
  MCC_LED_Set[Address-1][24]  = PUMP_PS_set[Address-1][4] ;
  MCC_LED_Set[Address-1][29]  = PUMP_PS_set[Address-1][5] ;

  MCC_LED_Set[Address-1][30] = MCC_HANJUN_set[Address-1];
  MCC_LED_Set[Address-1][31] = MCC_BALJUN_set[Address-1];
  MCC_LED_Set[Address-1][32] = MCC_SW_LOCK_ON_set[Address-1];
  MCC_LED_Set[Address-1][33] = MCC_SW_LOCK_OFF_set[Address-1];
  MCC_LED_Set[Address-1][34] = MCC_TERMINATION_set[Address-1];


  Uart_tx_buf[6] = (MCC_LED_Set[Address-1][0] << 7)|(MCC_LED_Set[Address-1][1] << 6)|(MCC_LED_Set[Address-1][2] << 5)|(MCC_LED_Set[Address-1][3] << 4)|
        (MCC_LED_Set[Address-1][4] << 3)|(MCC_LED_Set[Address-1][5] << 2)|(MCC_LED_Set[Address-1][6] << 1)|(MCC_LED_Set[Address-1][7] << 0);
  Uart_tx_buf[7] = (MCC_LED_Set[Address-1][8] << 7)|(MCC_LED_Set[Address-1][9] << 6)|(MCC_LED_Set[Address-1][10] << 5)|(MCC_LED_Set[Address-1][11] << 4)|
          (MCC_LED_Set[Address-1][12] << 3)|(MCC_LED_Set[Address-1][13] << 2)|(MCC_LED_Set[Address-1][14] << 1)|(MCC_LED_Set[Address-1][15] << 0);
  Uart_tx_buf[8] = (MCC_LED_Set[Address-1][16] << 7)|(MCC_LED_Set[Address-1][17] << 6)|(MCC_LED_Set[Address-1][18] << 5)|(MCC_LED_Set[Address-1][19] << 4)|
          (MCC_LED_Set[Address-1][20] << 3)|(MCC_LED_Set[Address-1][21] << 2)|(MCC_LED_Set[Address-1][22] << 1)|(MCC_LED_Set[Address-1][23] << 0);
  Uart_tx_buf[9] = (MCC_LED_Set[Address-1][24] << 7)|(MCC_LED_Set[Address-1][25] << 6)|(MCC_LED_Set[Address-1][26] << 5)|(MCC_LED_Set[Address-1][27] << 4)|
          (MCC_LED_Set[Address-1][28] << 3)|(MCC_LED_Set[Address-1][29] << 2)|(MCC_LED_Set[Address-1][30] << 1)|(MCC_LED_Set[Address-1][31] << 0);
  Uart_tx_buf[10] = (MCC_LED_Set[Address-1][32] << 7)|(MCC_LED_Set[Address-1][33] << 6)|(MCC_LED_Set[Address-1][34] << 5);


  Uart_crc = 0;

  for(i = 0; i <Uart2_length - 5 ; i++){
    Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
  }
  Uart_tx_buf[Uart2_length - 3] = Uart_crc ;

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);

  if(HAL_UART_Transmit(&huart2, Uart_tx_buf, Uart2_length, 1000)!= HAL_OK)
  {
    Error_Handler();
  }

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);
}


void UI_Com_Relay_S(void){

}
void UI_Com_DO24_S(void){
  int Uart_crc = 0, i, Uart2_length;
  // Set DO24
  Uart_crc = 0;
  Uart2_length = Sub_DO24_S_length;

  Uart_tx_buf[0] = 0x53;    //S
  Uart_tx_buf[1] = 0x54;    //T
  Uart_tx_buf[2] = 0x53;    //S
  Uart_tx_buf[3] = 0x53;    //S
  Uart_tx_buf[4] = 0x44;    //D
  Uart_tx_buf[5] = Uart_rx1_buf[5];    //address
  Uart_tx_buf[6] = Uart_rx1_buf[6];    //Relay 정보 1 ~8
  Uart_tx_buf[7] = Uart_rx1_buf[7];    //Relay 정보 9 ~16
  Uart_tx_buf[8] = Uart_rx1_buf[8];    //Relay 정보 17 ~24
  Uart_tx_buf[9] = 0x00;   //dummy1
  Uart_tx_buf[10] = 0x00;   //dummy2
  Uart_tx_buf[11] = 0x00;   //CRC
  Uart_tx_buf[12] = 0x45;   //E
  Uart_tx_buf[13] = 0x44;   //D

  Sub_DO24_Set_Num= Uart_rx1_buf[10];

  for(i = 0; i <Uart2_length - 5 ; i++){
    Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
  }
  Uart_tx_buf[Uart2_length - 3] = Uart_crc ;

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);

  if(HAL_UART_Transmit(&huart2, Uart_tx_buf, Uart2_length, 100)!= HAL_OK)
  {
    Error_Handler();
  }
  //HAL_Delay(10);
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);
}

void UI_Com_TB_S(void){

  int i, Uart_crc;

  int Uart2_length = Sub_TB_S_length;

  for(i=0;i<Uart2_length;i++){
      Uart_tx_buf[i] = Uart_rx1_buf[i];
  }
  Uart_tx_buf[8] = FIRE_LED_mode;    //Fire

  Uart_crc = 0;
  for(i = 0; i <Uart2_length - 5 ; i++){
    Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
  }
  Uart_tx_buf[9] = Uart_crc ;

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);
  HAL_Delay(10);
  if(HAL_UART_Transmit(&huart2, Uart_tx_buf, Uart2_length, 1000)!= HAL_OK)
  {
    Error_Handler();
  }
  //HAL_Delay(10);
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);
  HAL_Delay(10);
}

void UI_Com_TB_S_Fire(void){

  int i, Uart_crc;

  int Uart2_length = Sub_TB_S_length;

  Uart_tx_buf[0] = 0x53;    //S
  Uart_tx_buf[1] = 0x54;    //T
  Uart_tx_buf[2] = 0x55;    //U
  Uart_tx_buf[3] = 0x53;    //S
  Uart_tx_buf[4] = 0x54;    //T
  Uart_tx_buf[5] = 0x01;    //address
  Uart_tx_buf[6] = 0x00;    //Test mode
  Uart_tx_buf[7] = 0x00;    //dummy1
  Uart_tx_buf[8] = FIRE_LED_mode;    //Fire
  Uart_tx_buf[9] = 0x00;   //CRC
  Uart_tx_buf[10] = 0x45;   //E
  Uart_tx_buf[11] = 0x44;   //D

  Uart_crc = 0;
  for(i = 0; i <Uart2_length - 5 ; i++){
    Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
  }
  Uart_tx_buf[9] = Uart_crc ;

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);
//  HAL_Delay(10);
  if(HAL_UART_Transmit(&huart2, Uart_tx_buf, Uart2_length, 1000)!= HAL_OK)
  {
    Error_Handler();
  }
  //HAL_Delay(10);
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);
  HAL_Delay(10);
}

char Uart_tx_buf_EB[Sub_EB_S_length], cnt_eb;
void UI_Com_EB_S(void){

  int i;

  int Uart2_length = Sub_EB_S_length;
  cnt_eb++;

  for(i=0;i<Uart2_length;i++){
      Uart_tx_buf_EB[i] = Uart_rx1_buf[i];
  }


  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);
  HAL_Delay(10);
  if(HAL_UART_Transmit(&huart2, Uart_tx_buf_EB, Uart2_length, 1000)!= HAL_OK)
  {
    Error_Handler();
  }
  //HAL_Delay(10);
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);
  HAL_Delay(10);


}


void Read_Sw_LTD_Status(void){


  //Local_phone_mode = Read_Local_phone();
  //phone_jack_mode = Read_phone_jack();

  inf_Other_sub[7] = Ext_phpne_mode;
  inf_Other_sub[6] = Local_phone_mode;
  inf_Other_sub[5] = phone_jack_mode;
  inf_Other_sub[4] = Sub_buzzer4_mode;
  inf_Other_sub[3] = Sub_buzzer3_mode;
  inf_Other_sub[2] = Sub_buzzer2_mode;
  inf_Other_sub[1] = Sub_buzzer1_mode;
  inf_Other_sub[0] = Main_buzzer_mode;

  inf_SWI= ((inf_SWI_sub[5] & 0x01) << 5)|((inf_SWI_sub[4] & 0x01) << 4)|((inf_SWI_sub[3] & 0x01) << 3)|
          ((inf_SWI_sub[2] & 0x01) << 2)|((inf_SWI_sub[1] & 0x01) << 1)|((inf_SWI_sub[0] & 0x01) << 0);

  inf_Other= ((inf_Other_sub[7] & 0x01) << 7)|((inf_Other_sub[6] & 0x01) << 6)|
              ((inf_Other_sub[5] & 0x01) << 5)|((inf_Other_sub[4] & 0x01) << 4)|((inf_Other_sub[3] & 0x01) << 3)|
              ((inf_Other_sub[2] & 0x01) << 2)|((inf_Other_sub[1] & 0x01) << 1)|((inf_Other_sub[0] & 0x01) << 0);

}

void Read_EB_Status(void){

  int Uart_crc = 0, i, Uart2_length = Sub_LDT_Q_length;
  int j;


  Uart_tx_buf[0] = 0x53;    //S
  Uart_tx_buf[1] = 0x54;    //T
  Uart_tx_buf[2] = 0x53;    //S
  Uart_tx_buf[3] = 0x51;    //Q
  Uart_tx_buf[4] = 0x4D;    //M
  Uart_tx_buf[5] = 0x01;    //address
  Uart_tx_buf[6] = 0x00;   //CRC
  Uart_tx_buf[7] = 0x45;   //E
  Uart_tx_buf[8] = 0x44;   //D

  for(i=0;i<rx2_buf_len;i++){
    Uart_rx2_buf[i] = 0;
  }

  Uart_crc = 0;
  Uart_tx_buf[4] = 0x42;    //B
  for(i = 0; i <Uart2_length - 5 ; i++){
    Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
  }
  Uart_tx_buf[6] = Uart_crc ;

  rx2_Receive_complete = 0;

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);
  HAL_Delay(1);
  if(HAL_UART_Transmit(&huart2, Uart_tx_buf, Uart2_length, 1000)!= HAL_OK)
  {
    Error_Handler();
  }
  //HAL_Delay(1);  //응답이 빨라 바로 수신상태로 전환
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);

  for(j=0;j<Sub_EBC_Wait_cnt;j++){
    if(rx2_Receive_complete == 1){
      break;
    }
    HAL_Delay(1);
  }

  if(rx2_Receive_complete == 1){
    Sub_EB_state = 1;
    for(i=0;i<Sub_EB_R_length;i++){
      Sub_EB_R_Data[i] = Uart_rx2_buf[i];
    }
    Sub_EB_R_Data[13] = Sub_EB_state;

  }
  else{
    Sub_EB_state = 0;
    rx2_Receive_complete = 0;
  }

}


void Read_Ext_TB_Status(void){

  int Uart_crc = 0, i, Uart2_length = Sub_LDT_Q_length;
  int j;


  Uart_tx_buf[0] = 0x53;    //S
  Uart_tx_buf[1] = 0x54;    //T
  Uart_tx_buf[2] = 0x53;    //S
  Uart_tx_buf[3] = 0x51;    //Q
  Uart_tx_buf[4] = 0x4D;    //M
  Uart_tx_buf[5] = 0x01;    //address
  Uart_tx_buf[6] = 0x00;   //CRC
  Uart_tx_buf[7] = 0x45;   //E
  Uart_tx_buf[8] = 0x44;   //D


  for(i=0;i<rx2_buf_len;i++){
    Uart_rx2_buf[i] = 0;
  }

  Uart_crc = 0;
  Uart_tx_buf[4] = 0x54;    //T
  for(i = 0; i <Uart2_length - 5 ; i++){
    Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
  }
  Uart_tx_buf[6] = Uart_crc ;
  rx2_Receive_complete = 0;

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);
  HAL_Delay(1);
  if(HAL_UART_Transmit(&huart2, Uart_tx_buf, Uart2_length, 1000)!= HAL_OK)
  {
    Error_Handler();
  }
  //HAL_Delay(1);  //응답이 빨라 바로 수신상태로 전환
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);


  for(j=0;j<Sub_TB_Wait_cnt;j++){
    if(rx2_Receive_complete == 1){
      break;
    }
    HAL_Delay(1);
  }

  if(rx2_Receive_complete == 1){

	Uart_crc = 0;
	for(i = 0; i <Sub_TB_R_length - 5 ; i++){
	    Uart_crc = Uart_crc ^ Uart_rx2_buf[i+2];
	}


	if(Uart_crc == Uart_rx2_buf[Sub_TB_R_length - 3]){
		Sub_TB_state = 1;
		for(i=0;i<Sub_TB_R_length;i++){
		  Sub_TB_R_Data[i] = Uart_rx2_buf[i];
		}
		rx2_Receive_complete = 0;

		if(Sub_TB_R_Data[7] == 0){
		  MCC_HANJUN_state = 0;
		  MCC_BALJUN_State = 0;
		}
		else if(Sub_TB_R_Data[7] == 1){
		  MCC_HANJUN_state = 1;
		  MCC_BALJUN_State = 0;
		}
		else if(Sub_TB_R_Data[7] == 2){
		  MCC_HANJUN_state = 0;
		  MCC_BALJUN_State = 1;
		}
	}

  }
  else{
    Sub_TB_state = 0;
    rx2_Receive_complete = 0;
  }
}

 int Resp_Suc;

int Read_DO24_Status(int Address){

  int Uart_crc = 0, i, Uart2_length = Sub_LDT_Q_length;
  int j;
  //int Resp_Suc;

  Uart_tx_buf[0] = 0x53;    //S
  Uart_tx_buf[1] = 0x54;    //T
  Uart_tx_buf[2] = 0x53;    //S
  Uart_tx_buf[3] = 0x51;    //Q
  Uart_tx_buf[4] = 0x44;    //D
  Uart_tx_buf[5] = Address;    //address
  Uart_tx_buf[6] = 0x00;   //CRC
  Uart_tx_buf[7] = 0x45;   //E
  Uart_tx_buf[8] = 0x44;   //D


  for(i=0;i<rx2_buf_len;i++){
    Uart_rx2_buf[i] = 0;
  }

  Uart_crc = 0;

  for(i = 0; i <Uart2_length - 5 ; i++){
    Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
  }
  Uart_tx_buf[6] = Uart_crc ;

  rx2_Receive_complete = 0;

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);
  HAL_Delay(1);
  if(HAL_UART_Transmit(&huart2, Uart_tx_buf, Uart2_length, 1000)!= HAL_OK)
  {
    Error_Handler();
  }
  //HAL_Delay(1);      //응답이 빨라 바로 수신상태로 전환
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);


  for(j=0;j<Sub_DO24_Wait_cnt;j++){
    if(rx2_Receive_complete == 1){
      break;
    }
    HAL_Delay(1);
  }

  if(rx2_Receive_complete == 1){

		Uart_crc = 0;
		for(i = 0; i <rx2_buf_count_tmp - 5 ; i++){
		    Uart_crc = Uart_crc ^ Uart_rx2_buf[i+2];
		}

		if(Uart_crc == Uart_rx2_buf[rx2_buf_count_tmp - 3]){

			if(Uart_rx2_buf[4] == Address){
			  for(i=0;i<Sub_DO24_R_length;i++){
				Sub_DO24_R_Data[Address - 1][i] = Uart_rx2_buf[i];
			  }
			  Resp_Suc = 1;
			}
			rx2_Receive_complete = 0;

		}

  }
  else{
    Resp_Suc = 0;
    rx2_Receive_complete = 0;
  }

  return Resp_Suc;
}


int Read_MCC_Status(int Address){

  int Uart_crc = 0, i, Uart2_length = Sub_LDT_Q_length;
  int j;
 // int Resp_Suc;

  for(i=0;i<rx2_buf_len;i++){
    Uart_rx2_buf[i] = 0;
  }

  Uart_tx_buf[0] = 0x53;    //S
  Uart_tx_buf[1] = 0x54;    //T
  Uart_tx_buf[2] = 0x53;    //S
  Uart_tx_buf[3] = 0x51;    //Q
  Uart_tx_buf[4] = 0x4D;    //M
  Uart_tx_buf[5] = Address;    //address
  Uart_tx_buf[6] = 0x00;   //CRC
  Uart_tx_buf[7] = 0x45;   //E
  Uart_tx_buf[8] = 0x44;   //D


  Uart_tx_buf[4] = 0x4D;    //M
  for(i = 0; i <Uart2_length - 5 ; i++){
    Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
  }
  Uart_tx_buf[6] = Uart_crc ;

  rx2_Receive_complete = 0;
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);
  HAL_Delay(1);
  if(HAL_UART_Transmit(&huart2, Uart_tx_buf, Uart2_length, 1000)!= HAL_OK)
  {
    Error_Handler();
  }
  //HAL_Delay(1);      //응답이 빨라 바로 수신상태로 전환
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);


  for(j=0;j<Sub_MCC_Wait_cnt;j++){
    if(rx2_Receive_complete == 1){
      break;
    }
    HAL_Delay(1);
  }

  if(rx2_Receive_complete == 1){

		Uart_crc = 0;
		for(i = 0; i <rx2_buf_count_tmp - 5 ; i++){
		    Uart_crc = Uart_crc ^ Uart_rx2_buf[i+2];
		}

		if(Uart_crc == Uart_rx2_buf[rx2_buf_count_tmp - 3]){
			if(Uart_rx2_buf[4] == Address){
			  Resp_Suc = 1;
			  for(i=0;i<Sub_MCC_R_length;i++){
				Sub_MCC_R_Data[Address-1][i] = Uart_rx2_buf[i];
			  }
			  for(i=0;i<5;i++){
				LED_Data_Set[Address -1][i] = Sub_MCC_R_Data[Address-1][6+i];
			  }
			  Auto_Dip_set[Address -1] = Sub_MCC_R_Data[Address-1][11];
			  for(i=0;i<6;i++){
				Auto_Dip[Address -1][i] = (Auto_Dip_set[Address -1]>>(7 - i) & 0x01);
			  }
			}
		}

    rx2_Receive_complete = 0;
  }
  else{
    Resp_Suc = 0;
    rx2_Receive_complete = 0;
  }

  return Resp_Suc;

}


int Read_MCC_R_Status(int Address){

  int Uart_crc = 0, i, Uart2_length = Sub_LDT_Q_length;
  int j;
  //int Resp_Suc;



  for(i=0;i<rx2_buf_len;i++){
    Uart_rx2_buf[i] = 0;
  }

  Uart_tx_buf[0] = 0x53;    //S
  Uart_tx_buf[1] = 0x54;    //T
  Uart_tx_buf[2] = 0x53;    //S
  Uart_tx_buf[3] = 0x51;    //Q
  Uart_tx_buf[4] = 0x52;    //R
  Uart_tx_buf[5] = Address;    //address
  Uart_tx_buf[6] = 0x00;   //CRC
  Uart_tx_buf[7] = 0x45;   //E
  Uart_tx_buf[8] = 0x44;   //D


  Uart_crc = 0;
  for(i = 0; i <Uart2_length - 5 ; i++){
    Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
  }
  Uart_tx_buf[6] = Uart_crc ;
  rx2_Receive_complete = 0;

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);
  HAL_Delay(1);

  if(HAL_UART_Transmit(&huart2, Uart_tx_buf, Uart2_length, 1000)!= HAL_OK)
  {
    Error_Handler();
  }
  //HAL_Delay(1);      //응답이 빨라 바로 수신상태로 전환
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);


  for(j=0;j<Sub_MCC_R_Wait_cnt;j++){
    if(rx2_Receive_complete == 1){
      break;
    }
    HAL_Delay(1);
  }
  HAL_Delay(5);


  if(rx2_Receive_complete == 1){

		Uart_crc = 0;
		for(i = 0; i <Sub_Relay_R_length - 5 ; i++){
		    Uart_crc = Uart_crc ^ Uart_rx2_buf[i+2];
		}


		if(Uart_crc == Uart_rx2_buf[Sub_Relay_R_length - 3]){
			Uart_crc = 0;
			for(i = 0; i <rx2_buf_count_tmp - 5 ; i++){
				Uart_crc = Uart_crc ^ Uart_rx2_buf[i+2];
			}

			if(Uart_crc == Uart_rx2_buf[Sub_Relay_R_length - 3]){

				Resp_Suc = 1;
				for(i=0;i<Sub_Relay_R_length;i++){
				  Sub_Relay_R_Data[Address-1][i] = Uart_rx2_buf[i];
				}

				rx2_Receive_complete = 0;

				for(i=0;i<8;i++){
				  PUMP_PS_Pre_set[Address-1][i] = PUMP_PS_set[Address-1][i]; //이전 압력 스위치 값 저장
				  PUMP_PS_set[Address-1][i] = (Sub_Relay_R_Data[Address-1][7] >> (7 - i)) & 0x01 ;
				  PUMP_CHECK_set[Address-1][i] = (Sub_Relay_R_Data[Address-1][8] >> (7 - i)) & 0x01 ;
				}
			}

		}

  }
  else{
    Resp_Suc = 0;
    rx2_Receive_complete = 0;
  }

  for(i=0;i<rx2_buf_len;i++){
    Uart_rx2_buf[i] = 0;
  }


  return Resp_Suc;

}

void Set_Led_Data(int Address){



  PUMP_AUTO_set[Address-1][0] = (LED_Data_Set[Address-1][0] >> 7) & 0x01 ;//-> PUMP1_AUTO
  PUMP_STOP_set[Address-1][0] = (LED_Data_Set[Address-1][0] >> 6) & 0x01 ;//-> PUMP1_STOP
  PUMP_MANUAL_set[Address-1][0] = (LED_Data_Set[Address-1][0] >> 5) & 0x01 ;//-> PUMP1_MANUAL
  PUMP_CHECK_set[Address-1][0] = (LED_Data_Set[Address-1][0] >> 4) & 0x01 ;//-> PUMP1_CHECK
  PUMP_PS_set[Address-1][0] = (LED_Data_Set[Address-1][0] >> 3) & 0x01 ;//-> PUMP1_PS

  PUMP_AUTO_set[Address-1][1] = (LED_Data_Set[Address-1][0] >> 2) & 0x01 ;//-> PUMP2_AUTO
  PUMP_STOP_set[Address-1][1] = (LED_Data_Set[Address-1][0] >> 1) & 0x01 ;//-> PUMP2_STOP
  PUMP_MANUAL_set[Address-1][1] = (LED_Data_Set[Address-1][0] >> 0) & 0x01 ;//-> PUMP2_MANUAL
  PUMP_CHECK_set[Address-1][1] = (LED_Data_Set[Address-1][1] >> 7) & 0x01 ;//-> PUMP2_CHECK
  PUMP_PS_set[Address-1][1] = (LED_Data_Set[Address-1][1] >> 6) & 0x01 ;//-> PUMP2_PS

  PUMP_AUTO_set[Address-1][2] = (LED_Data_Set[Address-1][1] >> 5) & 0x01 ;//-> PUMP3_AUTO
  PUMP_STOP_set[Address-1][2] = (LED_Data_Set[Address-1][1] >> 4) & 0x01 ;//-> PUMP3_STOP
  PUMP_MANUAL_set[Address-1][2] = (LED_Data_Set[Address-1][1] >> 3) & 0x01 ;//-> PUMP3_MANUAL
  PUMP_CHECK_set[Address-1][2] = (LED_Data_Set[Address-1][1] >> 2) & 0x01 ;//-> PUMP3_CHECK
  PUMP_PS_set[Address-1][2] = (LED_Data_Set[Address-1][1] >> 1) & 0x01 ;//-> PUMP3_PS

  PUMP_AUTO_set[Address-1][3] = (LED_Data_Set[Address-1][1] >> 0) & 0x01 ;//-> PUMP4_AUTO
  PUMP_STOP_set[Address-1][3] = (LED_Data_Set[Address-1][2] >> 7) & 0x01 ;//-> PUMP4_STOP
  PUMP_MANUAL_set[Address-1][3] = (LED_Data_Set[Address-1][2] >> 6) & 0x01 ;//-> PUMP4_MANUAL
  PUMP_CHECK_set[Address-1][3] = (LED_Data_Set[Address-1][2] >> 5) & 0x01 ;//-> PUMP4_CHECK
  PUMP_PS_set[Address-1][3] = (LED_Data_Set[Address-1][2] >> 4) & 0x01 ;//-> PUMP4_PS

  PUMP_AUTO_set[Address-1][4] = (LED_Data_Set[Address-1][2] >> 3) & 0x01 ;//-> PUMP5_AUTO
  PUMP_STOP_set[Address-1][4] = (LED_Data_Set[Address-1][2] >> 2) & 0x01 ;//-> PUMP5_STOP
  PUMP_MANUAL_set[Address-1][4] = (LED_Data_Set[Address-1][2] >> 1) & 0x01 ;//-> PUMP5_MANUAL
  PUMP_CHECK_set[Address-1][4] = (LED_Data_Set[Address-1][2] >> 0) & 0x01 ;//-> PUMP5_CHECK
  PUMP_PS_set[Address-1][4] = (LED_Data_Set[Address-1][3] >> 7) & 0x01 ;//-> PUMP5_PS

  PUMP_AUTO_set[Address-1][5] = (LED_Data_Set[Address-1][3] >> 6) & 0x01 ;//-> PUMP6_AUTO
  PUMP_STOP_set[Address-1][5] = (LED_Data_Set[Address-1][3] >> 5) & 0x01 ;//-> PUMP6_STOP
  PUMP_MANUAL_set[Address-1][5] = (LED_Data_Set[Address-1][3] >> 4) & 0x01 ;//-> PUMP6_MANUAL
  PUMP_CHECK_set[Address-1][5] = (LED_Data_Set[Address-1][3] >> 3) & 0x01 ;//-> PUMP6_CHECK
  PUMP_PS_set[Address-1][5] = (LED_Data_Set[Address-1][3] >> 2) & 0x01 ;//-> PUMP6_PS

  //MCC_HANJUN_set[Address-1] = (LED_Data_Set[Address-1][3] >> 1) & 0x01 ;//-> HANJUN
  //MCC_BALJUN_set[Address-1] = (LED_Data_Set[Address-1][3] >> 0) & 0x01 ;//-> BALJUN
  MCC_HANJUN_set[Address-1] = MCC_HANJUN_state ;//-> HANJUN
  MCC_BALJUN_set[Address-1] = MCC_BALJUN_State ;//-> BALJUN
  MCC_SW_LOCK_ON_set[Address-1] = (LED_Data_Set[Address-1][4] >> 7) & 0x01 ;//-> SW_LOCK_ON
  MCC_SW_LOCK_OFF_set[Address-1] = (LED_Data_Set[Address-1][4] >> 6) & 0x01 ;//-> SW_LOCK_OFF
  //MCC_TERMINATION_set[Address] = (LED_Data_Set[Address-1][4] >> 5) & 0x01 ;//-> TERMINATION

  // 한전 발전 정보는 TB보드로 부터 읽어 MCC LED제어와 UI에 보고한다. UI로 부터 제어는 받지 않는다.


}



void UI_Com_SW_r(void){

  uint8_t Uart1_tx_data[1];

  TX_LED(GPIO_PIN_RESET);

  Uart1_tx_Header[0] = 0x53;    //S
  Uart1_tx_Header[1] = 0x54;    //T
  Uart1_tx_Header[2] = 0x53;    //S
  Uart1_tx_Header[3] = 0x72;    //r

  if(HAL_UART_Transmit(&huart1, Uart1_tx_Header, sizeof(Uart1_tx_Header), 1000)!= HAL_OK)
  {
    Error_Handler();
  }

  Uart1_tx_data[0] = 0;

  if(HAL_UART_Transmit(&huart1, Uart1_tx_data, sizeof(Uart1_tx_data), 1000)!= HAL_OK)

  Uart1_tx_ck[0] = 0;

  Uart1_tx_ck[0] = Uart1_tx_Header[2]^ Uart1_tx_Header[3]^ Uart1_tx_data[0];


  if(HAL_UART_Transmit(&huart1, Uart1_tx_ck, sizeof(Uart1_tx_ck), 1000)!= HAL_OK)
  {
    Error_Handler();
  }

  Uart1_tx_END[0] = 0x45;
  Uart1_tx_END[1] = 0x44;
  if(HAL_UART_Transmit(&huart1, Uart1_tx_END, sizeof(Uart1_tx_END), 1000)!= HAL_OK)
  {
    Error_Handler();
  }

  TX_LED(GPIO_PIN_SET);

}






void Set_pump(int Address){

  int Uart_crc = 0, i, Uart2_length;

  // Set Pump
  Uart_crc = 0;
  Uart2_length = Sub_Relay_S_length;

  Uart_tx_buf[0] = 0x53;    //S
  Uart_tx_buf[1] = 0x54;    //T
  Uart_tx_buf[2] = 0x53;    //S
  Uart_tx_buf[3] = 0x53;    //S
  Uart_tx_buf[4] = 0x52;    //R
  Uart_tx_buf[5] = Address;    //address
  Uart_tx_buf[6] = 0x00;   //Relay 동작
  Uart_tx_buf[7] = 0x00;   //dummy1
  Uart_tx_buf[8] = 0x00;   //dummy2
  Uart_tx_buf[9] = 0x00;   //CRC
  Uart_tx_buf[10] = 0x45;   //E
  Uart_tx_buf[11] = 0x44;   //D


  for(i=0;i<6;i++){

    /*
    if((PUMP_AUTO_set[Address - 1][i] == 1)&(PUMP_STOP_set[Address - 1][i] == 0)        // Auto
       &(PUMP_MANUAL_set[Address - 1][i] == 0)){


         if(Auto_Dip[Address - 1][i] == 1){             //지속 동작

           if(Sub_Motor_Run_Mode[Address-1][i] == 0){                 // 현재 모터가 동작중이 아닌때
              if(PUMP_PS_set[Address-1][i] == 1){                             //펌프 저압력 신호 입력
                    Sub_Motor_Relay[Address-1][i] = 1;
                    Sub_Motor_Run_Mode[Address-1][i] = 1;
              }
           }
           else if(Sub_Motor_Run_Mode[Address-1][i] == 1){               // 현재 모터가 동작 중일때
              if(PUMP_PS_set[Address-1][i] == 1){                             //펌프 저압력 신호 입력
                    Sub_Motor_Relay[Address-1][i] = 1;
                    Sub_Motor_Run_Mode[Address-1][i] = 1;
              }
              else if(PUMP_PS_set[Address-1][i] == 0){                         //펌프 저압력 신호 입력 해제시에도 동작
                    Sub_Motor_Relay[Address-1][i] = 1;
                    Sub_Motor_Run_Mode[Address-1][i] = 1;
              }
           }
         }
         else if(Auto_Dip[Address - 1][i] == 0){             //자동 동작
           if(PUMP_PS_set[Address-1][i] == 1){
             Sub_Motor_Relay[Address-1][i] = 1;
             Sub_Motor_Run_Mode[Address-1][i] = 1;
           }
           else if(PUMP_PS_set[Address-1][i] == 0){
             Sub_Motor_Relay[Address-1][i] = 0;
             Sub_Motor_Run_Mode[Address-1][i] = 0;
           }
         }

       }

    else if((PUMP_AUTO_set[Address - 1][i] == 0)&(PUMP_STOP_set[Address - 1][i] == 0)   //Manual 무조건 모터 동작
       &(PUMP_MANUAL_set[Address - 1][i] == 1)){

          Sub_Motor_Relay[Address-1][i] = 1;
          Sub_Motor_Run_Mode[Address-1][i] = 1;


       }
    else if((PUMP_AUTO_set[Address - 1][i] == 0)&(PUMP_STOP_set[Address - 1][i] == 1)   //Stop 무조건 모터 정지
       &(PUMP_MANUAL_set[Address - 1][i] == 0)){

         Sub_Motor_Relay[Address-1][i] = 0;
          Sub_Motor_Run_Mode[Address-1][i] = 0;

       }
    */



    if((PUMP_AUTO_set[Address - 1][i] == 1)&(PUMP_STOP_set[Address - 1][i] == 0)        // Auto
       &(PUMP_MANUAL_set[Address - 1][i] == 0)){

         if((PUMP_PS_set_Mode[Address-1][i]  == 0)&(PUMP_PS_set[Address-1][i] == 1) ){
           PUMP_PS_set_Mode[Address-1][i]  = 1;
         }


         if(Auto_Dip[Address - 1][i] == 1){             //지속 동작

           if(Sub_Motor_Run_Mode[Address-1][i] == 0){                 // 현재 모터가 동작중이 아닌때
              if(PUMP_PS_set_Mode[Address-1][i] == 1){                             //펌프 저압력 신호 입력
                    Sub_Motor_Relay[Address-1][i] = 1;
                    Sub_Motor_Run_Mode[Address-1][i] = 1;
              }
           }
           else if(Sub_Motor_Run_Mode[Address-1][i] == 1){               // 현재 모터가 동작 중일때
              if(PUMP_PS_set_Mode[Address-1][i] == 1){                             //펌프 저압력 신호 입력
                    Sub_Motor_Relay[Address-1][i] = 1;
                    Sub_Motor_Run_Mode[Address-1][i] = 1;
              }
              else if(PUMP_PS_set_Mode[Address-1][i] == 0){                        //펌프 저압력 신호 입력 으로 돌디 않을때에는 멈춤
                    Sub_Motor_Relay[Address-1][i] = 0;
                    Sub_Motor_Run_Mode[Address-1][i] = 0;
              }
           }
         }
         else if(Auto_Dip[Address - 1][i] == 0){             //자동 동작

           PUMP_PS_set_Mode[Address-1][i]  = PUMP_PS_set[Address-1][i] ;

           if(PUMP_PS_set[Address-1][i] == 1){
             Sub_Motor_Relay[Address-1][i] = 1;
             Sub_Motor_Run_Mode[Address-1][i] = 1;
           }
           else if(PUMP_PS_set[Address-1][i] == 0){
             Sub_Motor_Relay[Address-1][i] = 0;
             Sub_Motor_Run_Mode[Address-1][i] = 0;
           }
         }



       }

    else if((PUMP_AUTO_set[Address - 1][i] == 0)&(PUMP_STOP_set[Address - 1][i] == 0)   //Manual 무조건 모터 동작
       &(PUMP_MANUAL_set[Address - 1][i] == 1)){

          Sub_Motor_Relay[Address-1][i] = 1;
          Sub_Motor_Run_Mode[Address-1][i] = 1;

         PUMP_PS_set_Mode[Address-1][i]  = 0;
       }
    else if((PUMP_AUTO_set[Address - 1][i] == 0)&(PUMP_STOP_set[Address - 1][i] == 1)   //Stop 무조건 모터 정지
       &(PUMP_MANUAL_set[Address - 1][i] == 0)){

         Sub_Motor_Relay[Address-1][i] = 0;
          Sub_Motor_Run_Mode[Address-1][i] = 0;

          PUMP_PS_set_Mode[Address-1][i]  = 0;

       }

  }


  for(i=0;i<6;i++){
    Uart_tx_buf[6] = Uart_tx_buf[6] | ( ( (Sub_Motor_Relay[Address - 1][i] & 0x01) << (7 - i) )) ;
  }

  Uart_crc = 0;
  for(i = 0; i <Uart2_length - 5 ; i++){
    Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
  }
  Uart_tx_buf[Uart2_length - 3] = Uart_crc ;

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);

  if(HAL_UART_Transmit(&huart2, Uart_tx_buf, Uart2_length, 100)!= HAL_OK)
  {
    Error_Handler();
  }
  //HAL_Delay(10);
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);

}

void Read_Sub_Version(void){

	uint8_t Uart_crc = 0;

	Version_Info[0][0] = 0x53;    //S
	Version_Info[0][1] = 0x54;    //T
	Version_Info[0][2] = 0x53;    //S
	Version_Info[0][3] = 0x76;    //v
	Version_Info[0][4] = 0x53;     //SW Board
	Version_Info[0][5] = 0x01;   //ADDRESS
	Version_Info[0][6] = F_Version_Year;
	Version_Info[0][7] = F_Version_Month;
	Version_Info[0][8] = F_Version_Day;
	Version_Info[0][9] = F_Version_Hour;
	Version_Info[0][10] = F_Version_Min;
	Version_Info[0][11] = F_Version_Sec;
	Version_Info[0][12] = 0x00;   //CRC
	Version_Info[0][13] = 0x45;   //E
	Version_Info[0][14] = 0x44;   //D

	for(int i = 0; i <Sub_V_length - 5 ; i++){
		Uart_crc = Uart_crc ^ Version_Info[0][i+2];
	}
	Version_Info[0][12] = Uart_crc ;

	for(int i=1 ; i<14 ; i++){
		Send_Sub_Version_Requst(i);
	}

//	Send_Sub_Version_Requst(1);	//Version_MCC_1
//	Send_Sub_Version_Requst(4);	//Version_MCC_R_1
//	Send_Sub_Version_Requst(7);	//Version_TB
//	Send_Sub_Version_Requst(8);	//Version_DO_1
//	Send_Sub_Version_Requst(13);	//Version_EBC
}

void Send_Sub_Version_Requst(uint8_t Board_Name){

	uint8_t Uart_crc = 0;

	SUB_UART_TX_buf[0] = 0x53;    //S
	SUB_UART_TX_buf[1] = 0x54;    //T
	SUB_UART_TX_buf[2] = 0x53;    //S
	SUB_UART_TX_buf[3] = 0x56;    //V
	SUB_UART_TX_buf[4] = 0;    //Board_Name
	SUB_UART_TX_buf[5] = 0;    //address
	SUB_UART_TX_buf[6] = 0x00;   //CRC
	SUB_UART_TX_buf[7] = 0x45;   //E
	SUB_UART_TX_buf[8] = 0x44;   //D

	if(Board_Name == Version_SW){
		SUB_UART_TX_buf[4] = 0x53;    //Board_Name
		SUB_UART_TX_buf[5] = 1;    //address
	}
	else if(Board_Name == Version_MCC_1){
		SUB_UART_TX_buf[4] = 0x4D;    //Board_Name
		SUB_UART_TX_buf[5] = 1;    //address
	}
	else if(Board_Name == Version_MCC_2){
		SUB_UART_TX_buf[4] = 0x4D;    //Board_Name
		SUB_UART_TX_buf[5] = 2;    //address
	}
	else if(Board_Name == Version_MCC_3){
		SUB_UART_TX_buf[4] = 0x4D;    //Board_Name
		SUB_UART_TX_buf[5] = 3;    //address
	}
	else if(Board_Name == Version_MCC_R_1){
		SUB_UART_TX_buf[4] = 0x52;    //Board_Name
		SUB_UART_TX_buf[5] = 1;    //address
	}
	else if(Board_Name == Version_MCC_R_2){
		SUB_UART_TX_buf[4] = 0x52;    //Board_Name
		SUB_UART_TX_buf[5] = 2;    //address
	}
	else if(Board_Name == Version_MCC_R_3){
		SUB_UART_TX_buf[4] = 0x52;    //Board_Name
		SUB_UART_TX_buf[5] = 3;    //address
	}
	else if(Board_Name == Version_TB){
		SUB_UART_TX_buf[4] = 0x54;    //Board_Name
		SUB_UART_TX_buf[5] = 1;    //address
	}
	else if(Board_Name == Version_DO_1){
		SUB_UART_TX_buf[4] = 0x44;    //Board_Name
		SUB_UART_TX_buf[5] = 1;    //address
	}
	else if(Board_Name == Version_DO_2){
		SUB_UART_TX_buf[4] = 0x44;    //Board_Name
		SUB_UART_TX_buf[5] = 2;    //address
	}
	else if(Board_Name == Version_DO_3){
		SUB_UART_TX_buf[4] = 0x44;    //Board_Name
		SUB_UART_TX_buf[5] = 3;    //address
	}
	else if(Board_Name == Version_DO_4){
		SUB_UART_TX_buf[4] = 0x44;    //Board_Name
		SUB_UART_TX_buf[5] = 4;    //address
	}
	else if(Board_Name == Version_DO_5){
		SUB_UART_TX_buf[4] = 0x44;    //Board_Name
		SUB_UART_TX_buf[5] = 5;    //address
	}
	else if(Board_Name == Version_EBC){
		SUB_UART_TX_buf[4] = 0x42;    //Board_Name
		SUB_UART_TX_buf[5] = 1;    //address
	}

	for(int i = 0; i <Sub_Q_length - 5 ; i++){
		Uart_crc = Uart_crc ^ SUB_UART_TX_buf[i+2];
	}
	SUB_UART_TX_buf[6] = Uart_crc ;

	for(int i=0; i<rx2_buf_len; i++){
		Uart_rx2_buf[i] = 0;
		}


	  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
	  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);

	  if(HAL_UART_Transmit(&huart2, SUB_UART_TX_buf, Sub_Q_length, 100)!= HAL_OK)
	  {
	    Error_Handler();
	  }
	  //HAL_Delay(10);
	  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);

	for(int j=0;j<Sub_Version_Wait_cnt;j++){
		if(rx2_Receive_complete == 1){
		  break;
		}
		HAL_Delay(1);
	}

	if(rx2_Receive_complete == 1){
		for(int i=0; i<15; i++){
			Version_Info[Board_Name][i] = Uart_rx2_buf[i];
		}
	}
	else{
		Version_Info[Board_Name][0] = SUB_UART_TX_buf[0];    //S
		Version_Info[Board_Name][1] = SUB_UART_TX_buf[1];    //T
		Version_Info[Board_Name][2] = SUB_UART_TX_buf[2];    //S
		Version_Info[Board_Name][3] = 0x76;    //v
		Version_Info[Board_Name][4] = SUB_UART_TX_buf[4];    //Board_Name
		Version_Info[Board_Name][5] = SUB_UART_TX_buf[5];    //address
		Version_Info[Board_Name][6] = 0;    //F_Version_Year
		Version_Info[Board_Name][7] = 0;    //F_Version_Month
		Version_Info[Board_Name][8] = 0;    //F_Version_Day
		Version_Info[Board_Name][9] = 0;    //F_Version_Hour
		Version_Info[Board_Name][10] = 0;    //F_Version_Min
		Version_Info[Board_Name][11] = 0;    //F_Version_Sec
		Version_Info[Board_Name][12] = 0x00;   //CRC
		Version_Info[Board_Name][13] = 0x45;   //E
		Version_Info[Board_Name][14] = 0x44;   //D

		for(int i = 0; i <15 - 5 ; i++){
			Uart_crc = Uart_crc ^ Version_Info[Board_Name][i+2];
		}

		Version_Info[Board_Name][12] = Uart_crc ;
	}

	rx2_Receive_complete = 0;

}


void UI_Com_V(void){

	if(Uart_rx1_buf[4] == 0x53){ // Switch board : 'S' '0x53'
		Send_Version_UI_TX(Version_SW);
	}
	else if(Uart_rx1_buf[4] == 0x4D){ // MCC board : 'M' '0x4D'
		if(Uart_rx1_buf[5] == 0x01){
			Send_Version_UI_TX(Version_MCC_1);
		}
		else if(Uart_rx1_buf[5] == 0x02){
			Send_Version_UI_TX(Version_MCC_2);
		}
		else if(Uart_rx1_buf[5] == 0x03){
			Send_Version_UI_TX(Version_MCC_3);
		}
	}
	else if(Uart_rx1_buf[4] == 0x52){ // Relay board : 'R' '0x52'
		if(Uart_rx1_buf[5] == 0x01){
			Send_Version_UI_TX(Version_MCC_R_1);
		}
		else if(Uart_rx1_buf[5] == 0x02){
			Send_Version_UI_TX(Version_MCC_R_2);
		}
		else if(Uart_rx1_buf[5] == 0x03){
			Send_Version_UI_TX(Version_MCC_R_3);
		}
	}
	else if(Uart_rx1_buf[4] == 0x44){ // DO24 boaed : '0x44' '0x44'
		if(Uart_rx1_buf[5] == 0x01){
			Send_Version_UI_TX(Version_DO_1);
		}
		else if(Uart_rx1_buf[5] == 0x02){
			Send_Version_UI_TX(Version_DO_2);
		}
		else if(Uart_rx1_buf[5] == 0x03){
			Send_Version_UI_TX(Version_DO_3);
		}
		else if(Uart_rx1_buf[5] == 0x04){
			Send_Version_UI_TX(Version_DO_4);
		}
		else if(Uart_rx1_buf[5] == 0x05){
			Send_Version_UI_TX(Version_DO_5);
		}
	}
	else if(Uart_rx1_buf[4] == 0x54){ // TB board: '0x54' '0x54'
		Send_Version_UI_TX(Version_TB);
	}
	else if(Uart_rx1_buf[4] == 0x42){ // EBC board : 'B' '0x42'
		Send_Version_UI_TX(Version_EBC);
	}
}

void Send_Version_UI_TX(uint8_t Board_Name){

	if(HAL_UART_Transmit(&huart1, Version_Info[Board_Name], 15, 1000)!= HAL_OK)
	  {
		Error_Handler();
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
