/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

int buzzer_cnt = 0, Main_buzzer_cnt = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM15 global interrupt.
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim15);
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */


  HAL_GPIO_TogglePin(RUN_LED_GPIO_Port, RUN_LED_Pin);

  if(Led_Toggle_mode == 1){
    Led_Toggle_mode = 0;

    if(MAIN_BELL_LED_mode == 1){
      HAL_GPIO_WritePin(MAIN_BELL_LED_GPIO_Port, MAIN_BELL_LED_Pin,GPIO_PIN_SET);
    }
    else{
      HAL_GPIO_WritePin(MAIN_BELL_LED_GPIO_Port, MAIN_BELL_LED_Pin,GPIO_PIN_RESET);
    }

    if(SUB_BELL_LED_mode == 1){
      HAL_GPIO_WritePin(SUB_BELL_LED_GPIO_Port, SUB_BELL_LED_Pin,GPIO_PIN_SET);
    }
    else{
      HAL_GPIO_WritePin(SUB_BELL_LED_GPIO_Port, SUB_BELL_LED_Pin,GPIO_PIN_RESET);
    }

    if(LOCAL_BELL_LED_mode == 1){
      HAL_GPIO_WritePin(LOCAL_BELL_LED_GPIO_Port, LOCAL_BELL_LED_Pin, GPIO_PIN_SET);
    }
    else{
      HAL_GPIO_WritePin(LOCAL_BELL_LED_GPIO_Port, LOCAL_BELL_LED_Pin,GPIO_PIN_RESET);
    }

    if(SIREN_LED_mode == 1){
      HAL_GPIO_WritePin(SIREN_LED_GPIO_Port, SIREN_LED_Pin, GPIO_PIN_SET);
    }
    else{
      HAL_GPIO_WritePin(SIREN_LED_GPIO_Port, SIREN_LED_Pin, GPIO_PIN_RESET);
    }

    if(EMERGENCY_LED_mode == 1){
      HAL_GPIO_WritePin(EMERGENCY_BD_LED_GPIO_Port, EMERGENCY_BD_LED_Pin, GPIO_PIN_SET);
    }
    else{
      HAL_GPIO_WritePin(EMERGENCY_BD_LED_GPIO_Port, EMERGENCY_BD_LED_Pin, GPIO_PIN_RESET);
    }

  }
  else{
    Led_Toggle_mode = 1;

    HAL_GPIO_WritePin(MAIN_BELL_LED_GPIO_Port, MAIN_BELL_LED_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SUB_BELL_LED_GPIO_Port, SUB_BELL_LED_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LOCAL_BELL_LED_GPIO_Port, LOCAL_BELL_LED_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SIREN_LED_GPIO_Port, SIREN_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EMERGENCY_BD_LED_GPIO_Port, EMERGENCY_BD_LED_Pin, GPIO_PIN_RESET);

  }

  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

	HAL_GPIO_WritePin(BUZZER_3_GPIO_Port, BUZZER_3_Pin,GPIO_PIN_RESET);

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  if(Read_Key_MENU_POPUP() == GPIO_PIN_RESET){
    if(Pre_Key_MENU_POPUP == 0){
      //MENU_POPUP_LED_mode = ~(MENU_POPUP_LED_mode) & 0x01 ;
      inf_SWI_sub[5] = 1;        //전체화면
      HAL_GPIO_WritePin(BUZZER_3_GPIO_Port, BUZZER_3_Pin,GPIO_PIN_SET);
    }
    Pre_Key_MENU_POPUP = 1;
    Key_Push_Status = 1;
  }
  else{
    Pre_Key_MENU_POPUP = 0;
  }

  if(Read_Key_MAIN_BELL_STOP() == GPIO_PIN_RESET){
    if(Pre_Key_MAIN_BELL_STOP == 0){
      //MAIN_BELL_LED_mode = ~(MAIN_BELL_LED_mode) & 0x01 ;
      if(Main_buzzer_mode == 1){
        Main_buzzer_mode = 0;
      }
      inf_SWI_sub[4] = 1  ;      //주음향 정지
      HAL_GPIO_WritePin(BUZZER_3_GPIO_Port, BUZZER_3_Pin,GPIO_PIN_SET);
    }
    Pre_Key_MAIN_BELL_STOP = 1;
  }
  else{
    Pre_Key_MAIN_BELL_STOP = 0;
  }

  if(Read_Key_SUB_BELL_STOP() == GPIO_PIN_RESET){
    if(Pre_Key_SUB_BELL_STOP == 0){
      //SUB_BELL_LED_mode = ~(SUB_BELL_LED_mode) & 0x01 ;

      if(Sub_buzzer1_mode == 1){
        Sub_buzzer1_mode = 0;
      }
      if(Sub_buzzer2_mode == 1){
        Sub_buzzer2_mode = 0;
      }
      if(Sub_buzzer3_mode == 1){
        Sub_buzzer3_mode = 0;
      }
      if(Sub_buzzer4_mode == 1){
        Sub_buzzer4_mode = 0;
      }
      inf_SWI_sub[3] = 1;       //기타음향 정지
      HAL_GPIO_WritePin(BUZZER_3_GPIO_Port, BUZZER_3_Pin,GPIO_PIN_SET);
    }
    Pre_Key_SUB_BELL_STOP = 1;
  }
  else{
    Pre_Key_SUB_BELL_STOP = 0;
  }

  if(Read_Key_LOCAL_BELL_STOP() == GPIO_PIN_RESET){
    if(Pre_Key_LOCAL_BELL_STOP == 0){
      //LOCAL_BELL_LED_mode = ~(LOCAL_BELL_LED_mode) & 0x01 ;
      inf_SWI_sub[2] = 1;       //지구벨 정지
      HAL_GPIO_WritePin(BUZZER_3_GPIO_Port, BUZZER_3_Pin,GPIO_PIN_SET);
    }
    Pre_Key_LOCAL_BELL_STOP = 1;
  }
  else{
    Pre_Key_LOCAL_BELL_STOP = 0;
  }

  if(Read_Key_SIREN_STOP() == GPIO_PIN_RESET){
    if(Pre_Key_SIREN_STOP == 0){
      //SIREN_LED_mode = ~(SIREN_LED_mode) & 0x01 ;
      inf_SWI_sub[1] = 1;       //사이렌 정지
      HAL_GPIO_WritePin(BUZZER_3_GPIO_Port, BUZZER_3_Pin,GPIO_PIN_SET);
    }
    Pre_Key_SIREN_STOP = 1;
  }
  else{
    Pre_Key_SIREN_STOP = 0;
  }

  if(Read_Key_EMERGENCY_STOP() == GPIO_PIN_RESET){
    if(Pre_Key_EMERGENCY_STOP == 0){
      //EMERGENCY_LED_mode = ~(EMERGENCY_LED_mode) & 0x01 ;
      inf_SWI_sub[0] = 1;       //비상 방송 정지
      HAL_GPIO_WritePin(BUZZER_3_GPIO_Port, BUZZER_3_Pin,GPIO_PIN_SET);
    }
    Pre_Key_EMERGENCY_STOP = 1;
  }
  else{
    Pre_Key_EMERGENCY_STOP = 0;
  }

  for(int i=0; i<Phone_Reas_Num; i++){
	  phone_jack_mode_S[i+1] = phone_jack_mode_S[i];
	  Local_phone_mode_S[i+1] = Local_phone_mode_S[i];
  }

  if(Read_phone_jack() == GPIO_PIN_RESET){
	phone_jack_mode_S[0] = 1;
  }
  else{
	phone_jack_mode_S[0] = 0;
  }

  if(Read_Local_phone() == GPIO_PIN_RESET){
	Local_phone_mode_S[0] = 1;
  }
  else{
	Local_phone_mode_S[0] = 0;
  }

  Local_phone_mode_Sum = 0;
  phone_jack_mode_Sum = 0;

  for(int i=0; i<Phone_Reas_Num; i++){
	  phone_jack_mode_Sum += phone_jack_mode_S[i];
	  Local_phone_mode_Sum += Local_phone_mode_S[i];
  }

  if(Local_phone_mode_Sum > (Phone_Reas_Num/2)){
	  Local_phone_mode = 0;
  }
  else{
	  Local_phone_mode = 1;
  }

  if(phone_jack_mode_Sum  > (Phone_Reas_Num/2)){
	  phone_jack_mode = 0;
  }
  else{
	  phone_jack_mode = 1;
  }

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */



  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */


  if( Main_buzzer_mode == 1){
    if(Main_buzzer_cnt  == 0){
      HAL_GPIO_WritePin(BUZZER_1_GPIO_Port, BUZZER_1_Pin, GPIO_PIN_SET);
    }
    else{
      HAL_GPIO_WritePin(BUZZER_1_GPIO_Port, BUZZER_1_Pin, GPIO_PIN_RESET);
    }

    if(Main_buzzer_cnt > (2 + 2)){
      Main_buzzer_cnt = -1;
    }
  }
  else{
    HAL_GPIO_WritePin(BUZZER_1_GPIO_Port, BUZZER_1_Pin, GPIO_PIN_RESET);
  }


  if(Sub_buzzer1_mode == 1){
    if((buzzer_cnt == 0) | (buzzer_cnt == 2)){
      HAL_GPIO_WritePin(BUZZER_2_GPIO_Port, BUZZER_2_Pin, GPIO_PIN_SET);
    }
    else{
      HAL_GPIO_WritePin(BUZZER_2_GPIO_Port, BUZZER_2_Pin, GPIO_PIN_RESET);
    }

    if(buzzer_cnt > (4 + 2) ){
      buzzer_cnt = -1;
    }
  }
  else if(Sub_buzzer2_mode == 1){
    if((buzzer_cnt == 0) | (buzzer_cnt == 2) | (buzzer_cnt == 4)){
      HAL_GPIO_WritePin(BUZZER_2_GPIO_Port, BUZZER_2_Pin, GPIO_PIN_SET);
    }
    else{
      HAL_GPIO_WritePin(BUZZER_2_GPIO_Port, BUZZER_2_Pin, GPIO_PIN_RESET);
    }

    if(buzzer_cnt > (6 + 2) ){
      buzzer_cnt = -1;
    }
  }
  else if(Sub_buzzer3_mode == 1){
    HAL_GPIO_WritePin(BUZZER_2_GPIO_Port, BUZZER_2_Pin, GPIO_PIN_SET);

    if(buzzer_cnt > (10 + 2) ){
      buzzer_cnt = -1;
    }
  }
  else if(Sub_buzzer4_mode == 1){
    if((buzzer_cnt == 0) | (buzzer_cnt == 2) | (buzzer_cnt == 4) | (buzzer_cnt == 6)){
      HAL_GPIO_WritePin(BUZZER_2_GPIO_Port, BUZZER_2_Pin, GPIO_PIN_SET);
    }
    else{
      HAL_GPIO_WritePin(BUZZER_2_GPIO_Port, BUZZER_2_Pin, GPIO_PIN_RESET);
    }

    if(buzzer_cnt > (10 + 2) ){
      buzzer_cnt = -1;
    }
  }
  else{
    HAL_GPIO_WritePin(BUZZER_2_GPIO_Port, BUZZER_2_Pin, GPIO_PIN_RESET);
  }
  //HAL_GPIO_TogglePin(BUZZER_2_GPIO_Port, BUZZER_2_Pin);
  Main_buzzer_cnt++;
  buzzer_cnt++;


  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  RX_LED(GPIO_PIN_RESET);

  Uart_rx1_buf_tmp[0] = USART1->RDR;

  if((Uart_rx1_buf_tmp[0] == 0x53)&( rx1_State == 0)){
      rx1_State = 1;
      rx1_buf_count = 0;
    }

  if(rx1_State == 1){
    Uart_rx1_buf[rx1_buf_count] = Uart_rx1_buf_tmp[0];
    rx1_buf_count++;
  }

  /*헤더확인을 통한 잘못된 데이타 확인*/
  if(rx1_buf_count > 3){
    if((Uart_rx1_buf[0] == 0x53) &(Uart_rx1_buf[1] == 0x54)){
    }
    else{
      rx1_State = 0;
      rx1_buf_count = 0;
    }
  }

  if((Uart_rx1_buf[rx1_buf_count - 1] == 0x44) & (Uart_rx1_buf[rx1_buf_count - 2] == 0x45)){

    rx1_Receive_complete = 1;
    rx1_buf_count_tmp = rx1_buf_count;
    rx1_State = 0;
    rx1_buf_count = 0;
  }

  RX_LED(GPIO_PIN_SET);

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */


  Uart_rx2_buf_tmp[0] = USART2->RDR;

if((Uart_rx2_buf_tmp[0] == 0x53)&( rx2_State == 0)){
    rx2_State = 1;
    rx2_buf_count = 0;
  }

if(rx2_State == 1){
  Uart_rx2_buf[rx2_buf_count] = Uart_rx2_buf_tmp[0];
  rx2_buf_count++;
}

/*헤더확인을 통한 잘못된 데이타 확인*/
if(rx2_buf_count > 3){
  if((Uart_rx2_buf[0] == 0x53) &(Uart_rx2_buf[1] == 0x54)){
  }
  else{
    rx2_State = 0;
    rx2_buf_count = 0;
  }
}

if((Uart_rx2_buf[rx2_buf_count - 1] == 0x44) & (Uart_rx2_buf[rx2_buf_count - 2] == 0x45)){

  rx2_Receive_complete = 1;
  rx2_buf_count_tmp = rx2_buf_count;
  rx2_State = 0;
  rx2_buf_count = 0;
}

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
