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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include "Encoder.h"
#include "PS5.h"
#include "duty.h"
#include "Robomaster.h"

#include <sensor_msgs/msg/joy.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define DIAM 125.0
//#define DEADZONE 0.2
#define DEADZONE 0.2f
#define MAX_VELOCITY 4000  // 最大目標速度（rpm相当）



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
rcl_publisher_t publisher_float;
std_msgs__msg__Float32 pub_float_msg;

rcl_publisher_t publisher_int;
std_msgs__msg__Int32 pub_int_msg;

rcl_publisher_t publisher_en;
std_msgs__msg__Int32 pub_en_msg;

rcl_publisher_t publisher_can;
std_msgs__msg__Int32 pub_can_msg;

rcl_publisher_t publisher_ext;
std_msgs__msg__Float32 pub_ext_msg;

rcl_publisher_t publisher_can_ext;
std_msgs__msg__String pub_can_ext_msg;

rcl_publisher_t publisher;
std_msgs__msg__String msg;

// rcl_subscription_t subscriber_can;
// std_msgs__msg__Float32 sub_can_msg;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
PS5 ps5;
int number = 0;
float can_1 = 0;
float can_2 = 0;
float can_3 = 0;
float can_4 = 0;
float can_5 = 0;
float can_6 = 0;
float can_7 = 0;
float can_8 = 0;
float can_9 = 0;
float can_10 = 0;
float can_11 = 0;
float can_12 = 0;
float can_13 = 0;
float can_14 = 0;
float can_15 = 0;
float can_16 = 0;

float T = 0.02;
float goal = 50;
float pre_e = 0;
float integral = 0;
float KP = 0.01;
float KD = 0.1;
float KI = 0.1;
int a_t = 0;
int arar = 0;
int flag = 0;
int motor_active = 0;  // モーター制御フラグ（0: 停止中, 1: 動作中）
int motor_active_2 = 0;
int motor_active_3 = 0;

float integral_limit = 0.001;
//float tolerance = 2;
//float e_resol = 72000.0;
float tolerance_e = 2 /360* 72000;
float counts_per_deg = 200;
float tole_cnt = 5*200;

int encoder_count_1 = 0;
Encoder_t encoder = {GPIOC, GPIO_PIN_1, GPIOC,GPIO_PIN_0, 256};
int encoder_count_2 = 0;
Encoder_t encoder_2 = {GPIOC, GPIO_PIN_2, GPIOC,GPIO_PIN_3, 256};


int time = 0;
int time_2 = 0;
int flag_m = 0;
int flag_n = 0;
int flag_h = 0;

typedef struct {
    float kp, ki, kd;
    float integral;
    float last_error;
} PID_t;

PID_t pid[4]; // 3輪分プラスyutaの3508のPID

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

//CAN_TxHeaderTypeDef TxHeader2;
//uint8_t TxData2[8];
uint32_t TxMailbox2;

CAN_RxHeaderTypeDef RxHeader2;
uint8_t RxData2[8];

uint32_t id;
uint32_t dlc;
uint8_t can_data[8];
uint8_t can_data2[8];

uint32_t order_data[4];//0-3 can1 4-7 can2
uint32_t order_data_2[4];
uint32_t order_data_3[4];
uint32_t order_data_4[4];

uint32_t last_time = 0;
int16_t prev_order_data[4] = {0};

moto_measure_t moto1;
moto_measure_t moto2;  // 0x202用のモータデータ
moto_measure_t moto3;
moto_measure_t moto4;
moto_measure_t moto5;
moto_measure_t moto6;
moto_measure_t moto7;
moto_measure_t moto8;

moto_measure_t moto9;
moto_measure_t moto10;
moto_measure_t moto11;
moto_measure_t moto12;
moto_measure_t moto13;
moto_measure_t moto14;
moto_measure_t moto15;
moto_measure_t moto16;


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1){
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK){
			//CAN1の受信データ格納 or 処理
			switch (RxHeader.StdId) {
			case 0x201:
				moto1.msg_cnt++ <= 50 ? get_moto_offset(&moto1, RxData) : encoder_data_handler(&moto1, RxData);
				//can_1 = encoder_1(&moto1, RxData);
				break;
			case 0x202:
				moto2.msg_cnt++ <= 50 ? get_moto_offset(&moto2, RxData) : encoder_data_handler(&moto2, RxData);
				//can_1 = encoder_1(&moto2, RxData);
				break;
			case 0x203:
				moto3.msg_cnt++ <= 50 ? get_moto_offset(&moto3, RxData) : encoder_data_handler(&moto3, RxData);
				//can_3 = encoder_1(&moto3, RxData);
				break;
			case 0x204:
				moto4.msg_cnt++ <= 50 ? get_moto_offset(&moto4, RxData) : encoder_data_handler(&moto4, RxData);
				//can_4 = encoder_1(&moto4, RxData);
				break;
			case 0x205:
				moto5.msg_cnt++ <= 50 ? get_moto_offset(&moto5, RxData) : encoder_data_handler(&moto5, RxData);
				can_5 = encoder_1(&moto5, RxData);
				break;
			case 0x206:
				moto6.msg_cnt++ <= 50 ? get_moto_offset(&moto6, RxData) : encoder_data_handler(&moto6, RxData);
				can_6 = encoder_1(&moto6, RxData);
				break;
			case 0x207:
				moto7.msg_cnt++ <= 50 ? get_moto_offset(&moto7, RxData) : encoder_data_handler(&moto7, RxData);
				//can_7 = encoder_1(&moto7, RxData);
				break;
			case 0x208:
				moto8.msg_cnt++ <= 50 ? get_moto_offset(&moto8, RxData) : encoder_data_handler(&moto8, RxData);
				//can_8 = encoder_1(&moto8, RxData);
				break;
			}

		}
		else {
			Error_Handler();
		}
	}
//	else if(hcan->Instance == CAN2){
//		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader2, RxData2) == HAL_OK){
//			//CAN2の受信データ格納 or 処理
//			switch (RxHeader2.StdId) {
//			case 0x201:
//				moto9.msg_cnt++ <= 50 ? get_moto_offset(&moto9, RxData2) : encoder_data_handler(&moto9, RxData2);
//				can_9 = encoder_1(&moto9, RxData2);
//				break;
//			case 0x202:
//				moto10.msg_cnt++ <= 50 ? get_moto_offset(&moto10, RxData2) : encoder_data_handler(&moto10, RxData2);
//				can_10 = encoder_1(&moto10, RxData2);
//				break;
//			case 0x203:
//				moto11.msg_cnt++ <= 50 ? get_moto_offset(&moto11, RxData2) : encoder_data_handler(&moto11, RxData2);
//				can_11 = encoder_1(&moto11, RxData2);
//				break;
//			case 0x204:
//				moto12.msg_cnt++ <= 50 ? get_moto_offset(&moto12, RxData2) : encoder_data_handler(&moto12, RxData2);
//				can_12 = encoder_1(&moto12, RxData2);
//				break;
//			case 0x205:
//				moto13.msg_cnt++ <= 50 ? get_moto_offset(&moto13, RxData2) : encoder_data_handler(&moto13, RxData2);
//				can_13 = encoder_1(&moto13, RxData2);
//				break;
//			case 0x206:
//				moto14.msg_cnt++ <= 50 ? get_moto_offset(&moto14, RxData2) : encoder_data_handler(&moto14, RxData2);
//				break;
//			case 0x207:
//				moto15.msg_cnt++ <= 50 ? get_moto_offset(&moto15, RxData2) : encoder_data_handler(&moto15, RxData2);
//				break;
//			case 0x208:
//				moto16.msg_cnt++ <= 50 ? get_moto_offset(&moto16, RxData2) : encoder_data_handler(&moto16, RxData2);
//				break;
//			}
//
//		}
//		else {
//			Error_Handler();
//		}
//	}

	// 共通のデータ処理（必要に応じて）
}



void subscription_joy_callback(const void * msgin)
{
	/*
	Lチカ用pin
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //LED turned on
   */

  sensor_msgs__msg__Joy * msg = (sensor_msgs__msg__Joy *)msgin;

  gets_ps5(msg, &ps5);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);


}

void timer_callback()
{
//	char buffer[256];
//	// フォーマット：見やすいように整形
//	snprintf(buffer, sizeof(buffer),
//	  "can_1: %d",can_1);
//
//	// Stringメッセージのセットアップ
//	msg.data.data = buffer;
//	msg.data.size = strlen(buffer);
//	msg.data.capacity = sizeof(buffer);
//
//	// Publish
//	rcl_publish(&publisher, &msg, NULL);

	pub_int_msg.data = 1;
	rcl_publish(&publisher_int, &pub_int_msg, NULL);
}


void sendCAN1(uint32_t id, uint8_t data[]){
	//koko senngenn to ue no shoukyo

	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8];
	uint32_t TxMailbox;


	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) Error_Handler();
	if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)){
		TxHeader.StdId = id;                    // CAN ID
		TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
		TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
		TxHeader.DLC = 8;   //size of data                    // データ長(バイト)
		TxHeader.TransmitGlobalTime = DISABLE;  // ???

		for(int i=0;i<TxHeader.DLC;i++){
			TxData[i] = data[i];
		}
		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			Error_Handler();
		}

	}
}


//void sendCAN2(uint32_t id, uint8_t data[]){
//	//koko senngenn to ue no shoukyo
//
//	CAN_TxHeaderTypeDef TxHeader;
//	uint8_t TxData[8];
//	uint32_t TxMailbox;
//
//	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 3) Error_Handler();
//	if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)){
//		TxHeader.StdId = id;                    // CAN ID
//		TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
//		TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
//		TxHeader.DLC = 8;   //size of data                    // データ長(バイト)
//		TxHeader.TransmitGlobalTime = DISABLE;  // ???
//
//		for(int i=0;i<TxHeader.DLC;i++){
//			TxData[i] = data[i];
//		}
//		if(HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK){
//			Error_Handler();
//		}
//
//	}
//}

void C610_C620(uint32_t id, uint32_t OrderData[],int i)
{
	//orderDataCAN1の処理sendCAN1
	uint8_t data[8];
	//data = orderdata >> 8;
	data[0] = OrderData[0] >> 8;
	data[1] = OrderData[0];
	data[2] = OrderData[1] >> 8;
	data[3] = OrderData[1];
	data[4] = OrderData[2] >> 8;
	data[5] = OrderData[2];
	data[6] = OrderData[3] >> 8;
	data[7] = OrderData[3];
	if(i == 1)
	{
		sendCAN1(id,data);
	}
//	else if(i == 2){
//		sendCAN2(id,data);
//	}
}

void test() {
	if (ps5.circle_btn) {
		order_data[0] = 1000;
//		order_data[1] = 1000;
//		order_data[2] = 1000;
//		order_data[3] = 1000;
//		order_data_2[0] = 1000;
//		order_data_2[1] = 1000;
//		order_data_2[2] = 1000;
//		order_data_2[3] = 1000;
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else {
		order_data[0] = 0;
//		order_data[1] = 0;
//		order_data[2] = 0;
//		order_data[3] = 0;
//		order_data_2[0] = 0;
//		order_data_2[1] = 0;
//		order_data_2[2] = 0;
//		order_data_2[3] = 0;
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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /*defaulttask start*/
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  // CANスタート
  HAL_CAN_Start(&hcan1);
//  HAL_CAN_Start(&hcan2);
  // 割り込み有効

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
	  Error_Handler();
  }
//  if(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
//	  Error_Handler();
//  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterIdHigh         = 0;                        // フィルターID(上位16ビット)
  sFilterConfig.FilterIdLow          = 0;                        // フィルターID(下位16ビット)
  sFilterConfig.FilterMaskIdHigh     = 0;                        // フィルターマスク(上位16ビット)
  sFilterConfig.FilterMaskIdLow      = 0;                        // フィルターマスク(下位16ビット)
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;    // フィルタースケール
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;         // フィルターに割り当てるFIFO
  sFilterConfig.FilterBank           = 0;                        // フィルターバンクNo
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;    // フィルターモード
  sFilterConfig.SlaveStartFilterBank = 14;                       // スレーブCANの開始フィルターバンクNo
  sFilterConfig.FilterActivation     = ENABLE;                   // フィルター無効／有効



  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
      Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4799;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4799;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC10 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
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
	    printf("Error on default allocators (line %d)\n", __LINE__);
	}

	rcl_subscription_t subscriber_joy;
	sensor_msgs__msg__Joy sub_joy_msg;
	rclc_support_t support;
	rcl_allocator_t allocator;
	rcl_node_t node;

	allocator = rcl_get_default_allocator();

	// set domain_id
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	size_t domain_id = (size_t)2; // here is personal ROS_DOMAIN_ID
	rcl_init_options_set_domain_id(&init_options, domain_id);

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));



	// create node
	RCCHECK(rclc_node_init_default(&node, "f446re_node", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_best_effort(
		 &publisher_int,
		 &node,
		 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		 "/f446re_int_msg"));
//	RCCHECK(rclc_publisher_init_best_effort(
//		 &publisher_en,
//		 &node,
//		 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//		 "/f446re_int_msg_2"));
	// create subscriber
	RCCHECK(rclc_subscription_init_default(
	    &subscriber_joy,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
	    "/joy"));
//	RCCHECK(rclc_publisher_init_best_effort(
//		 &publisher_float,
//		 &node,
//		 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//		 "/can_msg"));
//
//	RCCHECK(rclc_publisher_init_best_effort(
//		 &publisher_ext,
//		 &node,
//		 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//		 "/can_msg_ext"));

//	RCCHECK(rclc_publisher_init_best_effort(
//		 &publisher,
//		 &node,
//		 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
//		 "/can_msg_ext_2"));

	// create timer
	rcl_timer_t timer_pub;
	const unsigned int timer_timeout_pub = 10;
	RCCHECK(rclc_timer_init_default(
		&timer_pub,
		&support,
		RCL_MS_TO_NS(timer_timeout_pub),
		timer_callback));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
//	RCCHECK(rclc_executor_add_timer(&executor, &timer_pub));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_joy, &sub_joy_msg, &subscription_joy_callback, ON_NEW_DATA));

	// initialize message memory

	sub_joy_msg.buttons.capacity = 100;
	sub_joy_msg.buttons.data = (char * ) malloc(100 * sizeof(char));
	sub_joy_msg.buttons.size = 0;

	sub_joy_msg.axes.capacity = 100;
	sub_joy_msg.axes.data = (float * ) malloc(100 * sizeof(char));
	sub_joy_msg.axes.size = 0;

	sub_joy_msg.header.frame_id.capacity = 100;
	sub_joy_msg.header.frame_id.data = (char * ) malloc(100 * sizeof(char));
	sub_joy_msg.header.frame_id.size = 0;

	// execute subscriber
	while (1) {
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //LED turned on
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

	}

	// cleaning Up
	RCCHECK(rcl_publisher_fini(&publisher_int, &node))
	RCCHECK(rcl_subscription_fini(&subscriber_joy, &node));
	RCCHECK(rcl_node_fini(&node));
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  test();

	  C610_C620(0x200, order_data, 1);

	  osDelay(1);
//
//	  C610_C620(0x200, order_data_3,2);
//
//	  osDelay(1);
//
//	  C610_C620(0x1FF, order_data_2,1);
//
//	  osDelay(1);
//
//	  C610_C620(0x1FF, order_data_4,2);
//
//	  osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
