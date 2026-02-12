/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RUN_LED_Pin GPIO_PIN_13
#define RUN_LED_GPIO_Port GPIOC
#define MAIN_BELL_LED_Pin GPIO_PIN_14
#define MAIN_BELL_LED_GPIO_Port GPIOC
#define MENU_POPUP_LED_Pin GPIO_PIN_15
#define MENU_POPUP_LED_GPIO_Port GPIOC
#define ENHANCED_UART4_TXD_Pin GPIO_PIN_0
#define ENHANCED_UART4_TXD_GPIO_Port GPIOA
#define ENHANCED_UART4_RXD_Pin GPIO_PIN_1
#define ENHANCED_UART4_RXD_GPIO_Port GPIOA
#define UART2_TXD_Pin GPIO_PIN_2
#define UART2_TXD_GPIO_Port GPIOA
#define UART2_RXD_Pin GPIO_PIN_3
#define UART2_RXD_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_4
#define RS485_DE_GPIO_Port GPIOA
#define RS485_RE_Pin GPIO_PIN_5
#define RS485_RE_GPIO_Port GPIOA
#define SUB_BELL_LED_Pin GPIO_PIN_6
#define SUB_BELL_LED_GPIO_Port GPIOA
#define LOCAL_BELL_LED_Pin GPIO_PIN_7
#define LOCAL_BELL_LED_GPIO_Port GPIOA
#define BUZZER_3_Pin GPIO_PIN_0
#define BUZZER_3_GPIO_Port GPIOB
#define TX_LED_Pin GPIO_PIN_1
#define TX_LED_GPIO_Port GPIOB
#define RX_LED_Pin GPIO_PIN_2
#define RX_LED_GPIO_Port GPIOB
#define MENU_POPUP_Pin GPIO_PIN_10
#define MENU_POPUP_GPIO_Port GPIOB
#define MAIN_BELL_STOP_Pin GPIO_PIN_11
#define MAIN_BELL_STOP_GPIO_Port GPIOB
#define SUB_BELL_STOP_Pin GPIO_PIN_12
#define SUB_BELL_STOP_GPIO_Port GPIOB
#define LOCAL_BELL_STOP_Pin GPIO_PIN_13
#define LOCAL_BELL_STOP_GPIO_Port GPIOB
#define SIREN_STOP_Pin GPIO_PIN_14
#define SIREN_STOP_GPIO_Port GPIOB
#define EMERGENCY_STOP_Pin GPIO_PIN_15
#define EMERGENCY_STOP_GPIO_Port GPIOB
#define SIREN_LED_Pin GPIO_PIN_8
#define SIREN_LED_GPIO_Port GPIOA
#define EMERGENCY_BD_LED_Pin GPIO_PIN_9
#define EMERGENCY_BD_LED_GPIO_Port GPIOA
#define FIRE_LED_Pin GPIO_PIN_10
#define FIRE_LED_GPIO_Port GPIOA
#define ERR_LED_Pin GPIO_PIN_15
#define ERR_LED_GPIO_Port GPIOA
#define BUZZER_1_Pin GPIO_PIN_4
#define BUZZER_1_GPIO_Port GPIOB
#define BUZZER_2_Pin GPIO_PIN_5
#define BUZZER_2_GPIO_Port GPIOB
#define UART1_TXD_Pin GPIO_PIN_6
#define UART1_TXD_GPIO_Port GPIOB
#define UART1_RXD_Pin GPIO_PIN_7
#define UART1_RXD_GPIO_Port GPIOB
#define BOOT_0_Pin GPIO_PIN_3
#define BOOT_0_GPIO_Port GPIOH
#define LOCAL_PHONE_IN_Pin GPIO_PIN_8
#define LOCAL_PHONE_IN_GPIO_Port GPIOB
#define PHONE_JACK_IN_Pin GPIO_PIN_9
#define PHONE_JACK_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void UI_Com(void);

void UI_Com_All_Q(void);
void UI_Com_SW_Q(void);
void UI_Com_MCC_Q(int Address);
void UI_Com_Relay_Q(int Address);
void UI_Com_DO24_Q(int Address);
void UI_Com_TB_Q(void);
void UI_Com_EB_Q(void);

void UI_Com_SW_r(void);


void UI_Com_All_S(void);
void UI_Com_SW_S(void);
void UI_Com_MCC_S(int Address);
void UI_Com_MCC_S_Relay_val(int Address);
void UI_Com_Relay_S(void);
void UI_Com_DO24_S(void);
void UI_Com_TB_S(void);
void UI_Com_TB_S_Fire(void);
void UI_Com_EB_S(void);

void Read_Sub_Version(void);
void Send_Sub_Version_Requst(uint8_t Board_Name);

GPIO_PinState Read_Key_MENU_POPUP(void);
GPIO_PinState Read_Key_SUB_BELL_STOP(void);
GPIO_PinState Read_Key_SIREN_STOP(void);
GPIO_PinState Read_Key_MAIN_BELL_STOP(void);
GPIO_PinState Read_Key_LOCAL_BELL_STOP(void);
GPIO_PinState Read_Key_EMERGENCY_STOP(void);

int Read_phone_jack(void);
int Read_Local_phone(void);

void MENU_POPUP_LED(GPIO_PinState LED);
void MAIN_BELL_LED(GPIO_PinState LED);
void SUB_BELL_LED(GPIO_PinState LED);
void LOCAL_BELL_LED(GPIO_PinState LED);
void SIREN_LED(GPIO_PinState LED);
void EMERGENCY_LED(GPIO_PinState LED);
void FIRE_LED(GPIO_PinState LED);

void RUN_LED(GPIO_PinState LED);
void ERR_LED(GPIO_PinState LED);
void TX_LED(GPIO_PinState LED);
void RX_LED(GPIO_PinState LED);

void Read_Sw_LTD_Status(void);
void Read_Ext_TB_Status(void);
int Read_DO24_Status(int Address);
int Read_MCC_Status(int Address);
int Read_MCC_R_Status(int Address);
void Read_EB_Status(void);

void Set_Led_Data(int Address);

void Set_pump(int Address);

void UI_Com_V(void);
void Send_Version_UI_TX(uint8_t Board_Name);

extern  char MENU_POPUP_LED_mode;
extern  char MAIN_BELL_LED_mode;
extern  char SUB_BELL_LED_mode;
extern  char LOCAL_BELL_LED_mode;
extern  char SIREN_LED_mode;
extern  char EMERGENCY_LED_mode;
extern  char FIRE_LED_mode;
extern  char RUN_LED_mode;
extern  char ERR_LED_mode;

extern char Pre_Key_MENU_POPUP;
extern char Pre_Key_SUB_BELL_STOP;
extern char Pre_Key_SIREN_STOP;
extern char Pre_Key_MAIN_BELL_STOP;
extern char Pre_Key_LOCAL_BELL_STOP;
extern char Pre_Key_EMERGENCY_STOP;

extern  char Ext_phpne_mode;
extern  char Local_phone_mode;
extern  char phone_jack_mode;
extern  char Sub_buzzer4_mode;
extern  char Sub_buzzer3_mode;
extern  char Sub_buzzer2_mode;
extern  char Sub_buzzer1_mode;
extern  char Main_buzzer_mode;

extern  char loop_out;

#define rx1_buf_len 40
extern uint8_t Uart_rx1_buf[rx1_buf_len] ;
extern uint8_t Uart_rx1_buf_tmp[1] ;
extern int rx1_State ;
extern int rx1_buf_count;
extern int rx1_buf_count_tmp;
extern int rx1_Receive_complete;

#define rx2_buf_len 40
extern uint8_t Uart_rx2_buf[rx2_buf_len] ;
extern uint8_t Uart_rx2_buf_tmp[3] ;
extern int rx2_State ;
extern int rx2_buf_count;
extern int rx2_buf_count_tmp;
extern int rx2_Receive_complete;

#define tx_buf_len 30
extern uint8_t Uart_tx_buf[tx_buf_len] ;

# define Sub_LDT_Q_length       9

# define Sub_MCC_R_length      17
# define Sub_MCC_S_length      16

# define Sub_Relay_R_length     15
# define Sub_Relay_S_length     12

# define Sub_DO24_R_length      15
# define Sub_DO24_S_length      14

# define Sub_TB_R_length        18
# define Sub_TB_S_length        12

# define Sub_EB_R_length        18
# define Sub_EB_S_length        18

# define Sub_V_length        15
# define Sub_Q_length       9
#define Sub_Version_Wait_cnt 15

extern uint8_t inf_LED;
extern uint8_t inf_SWI;
extern uint8_t inf_Other;
extern uint8_t Key_Push_Status;

extern uint8_t Led_Toggle_mode ;

#define Phone_Reas_Num  20

extern char Local_phone_mode_S[Phone_Reas_Num];
extern char phone_jack_mode_S[Phone_Reas_Num];

extern uint16_t Local_phone_mode_Sum;
extern uint16_t phone_jack_mode_Sum;
/*
0b0100 0000 : 화재
0b0010 0000 : 전체화면
0b0001 0000 : 주음향 정지
0b0000 1000 : 기타음향 정지
0b0000 0100 : 지구벨 정지
0b0000 0010 : 사이렌 정지
0b0000 0001 : 비상 방송 정지
*/
extern uint8_t inf_LED_sub[7];

/*
0b0010 0000 : 전체화면
0b0001 0000 : 주음향 정지
0b0000 1000 : 기타음향 정지
0b0000 0100 : 지구벨 정지
0b0000 0010 : 사이렌 정지
0b0000 0001 : 비상 방송 정지
*/
extern uint8_t inf_SWI_sub[7];

/*
0b0010 0000 : Ext_phone
0b0001 0000 : Local_phone
0b0000 1000 : phone_jack
0b0000 0100 : 부저 3
0b0000 0010 : 부저 2
0b0000 0001 : 부저 1
*/
extern uint8_t inf_Other_sub[8];




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
