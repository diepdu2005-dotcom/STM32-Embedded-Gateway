/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Modbus.h"
#include "wizchip_conf.h"
#include "socket.h"
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
/* USER CODE BEGIN Variables */
// Báo cho freertos.c biết là các biến/hàm này đã được tạo ở main.c rồi
extern uint16_t Gateway_Register_Map[128];
extern modbusHandler_t ModbusRTU_Master;
extern void W5500_Hardware_Init(void);

// 1. Định nghĩa cấu trúc gói tin chuyển từ TCP sang RTU
typedef struct {
    uint8_t  function_code;
    uint16_t address;
    uint16_t value;
} RTU_Command_t;

// 2. Khai báo Handle cho Mutex và Queue
osMutexId_t RegMapMutexHandle;
osMessageQueueId_t CmdQueueHandle;
/* USER CODE END Variables */
/* Definitions for ModbusTCP_Task */
osThreadId_t ModbusTCP_TaskHandle;
const osThreadAttr_t ModbusTCP_Task_attributes = {
  .name = "ModbusTCP_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RTU_Poll_Task */
osThreadId_t RTU_Poll_TaskHandle;
const osThreadAttr_t RTU_Poll_Task_attributes = {
  .name = "RTU_Poll_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartModbusTCP_Task(void *argument);
void StartRTU_Poll_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
	const osMutexAttr_t RegMapMutex_attr = { .name = "RegMapMutex" };
	RegMapMutexHandle = osMutexNew(&RegMapMutex_attr);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	  const osMessageQueueAttr_t CmdQueue_attr = { .name = "CmdQueue" };
	  // Tạo Queue chứa tối đa 10 lệnh, mỗi lệnh có kích thước bằng struct RTU_Command_t
	  CmdQueueHandle = osMessageQueueNew(10, sizeof(RTU_Command_t), &CmdQueue_attr);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ModbusTCP_Task */
  ModbusTCP_TaskHandle = osThreadNew(StartModbusTCP_Task, NULL, &ModbusTCP_Task_attributes);

  /* creation of RTU_Poll_Task */
  RTU_Poll_TaskHandle = osThreadNew(StartRTU_Poll_Task, NULL, &RTU_Poll_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartModbusTCP_Task */
/**
  * @brief  Function implementing the ModbusTCP_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartModbusTCP_Task */
void StartModbusTCP_Task(void *argument)
{
  /* USER CODE BEGIN StartModbusTCP_Task */
	  uint16_t len = 0;
	  uint8_t tcp_rx_buffer[256];
	  uint8_t tcp_tx_buffer[256];

	  #define TCP_SOCKET 0
	  #define MODBUS_TCP_PORT 502

	  // 1. Khởi tạo phần cứng W5500 (Gắn SPI, nạp IP)
	  W5500_Hardware_Init();

	  /* Infinite loop */
	  for(;;)
	  {
	    // 2. State Machine quản lý Socket của W5500
	    switch(getSn_SR(TCP_SOCKET)) {

	        case SOCK_ESTABLISHED:
	            if(getSn_IR(TCP_SOCKET) & Sn_IR_CON) {
	                setSn_IR(TCP_SOCKET, Sn_IR_CON);
	            }

	            if((len = getSn_RX_RSR(TCP_SOCKET)) > 0) {
	                if(len > 256) len = 256;
	                recv(TCP_SOCKET, tcp_rx_buffer, len);

	                // --- BÓC TÁCH MODBUS TCP HEADER ---
	                if(len >= 12 && tcp_rx_buffer[2] == 0x00 && tcp_rx_buffer[3] == 0x00) {
	                    uint8_t unit_id = tcp_rx_buffer[6];
	                    uint8_t function_code = tcp_rx_buffer[7];

	                    // Xử lý lệnh Đọc (Function Code 03)
	                    if(function_code == 0x03) {
	                        uint16_t start_addr = (tcp_rx_buffer[8] << 8) | tcp_rx_buffer[9];
	                        uint16_t quantity = (tcp_rx_buffer[10] << 8) | tcp_rx_buffer[11];


	                        // Kiểm tra giới hạn mảng
	                        if((start_addr + quantity) <= 128 && quantity > 0) {
	                            uint16_t tx_index = 0;
	                            uint8_t byte_count = quantity * 2;

	                            // Đóng gói Header trả về
	                            tcp_tx_buffer[tx_index++] = tcp_rx_buffer[0];
	                            tcp_tx_buffer[tx_index++] = tcp_rx_buffer[1];
	                            tcp_tx_buffer[tx_index++] = 0x00;
	                            tcp_tx_buffer[tx_index++] = 0x00;

	                            uint16_t payload_length = 3 + byte_count;
	                            tcp_tx_buffer[tx_index++] = (payload_length >> 8) & 0xFF;
	                            tcp_tx_buffer[tx_index++] = payload_length & 0xFF;

	                            tcp_tx_buffer[tx_index++] = unit_id;
	                            tcp_tx_buffer[tx_index++] = 0x03;
	                            tcp_tx_buffer[tx_index++] = byte_count;

	                            // KHÓA MUTEX: Không cho ai ghi vào mảng lúc ta đang bốc dữ liệu ra
	                            osMutexAcquire(RegMapMutexHandle, osWaitForever);

	                            // Lấy dữ liệu từ Register Map (đã được Task kia cập nhật)
	                            for(int i = 0; i < quantity; i++) {
	                                tcp_tx_buffer[tx_index++] = (Gateway_Register_Map[start_addr + i] >> 8) & 0xFF;
	                                tcp_tx_buffer[tx_index++] = Gateway_Register_Map[start_addr + i] & 0xFF;
	                            }

	                            // MỞ MUTEX: Lấy xong rồi, trả lại mảng cho hệ thống
	                            osMutexRelease(RegMapMutexHandle);
	                            // Phản hồi về PC
	                            send(TCP_SOCKET, tcp_tx_buffer, tx_index);
	                        }
	                    }

                        // XỬ LÝ LỆNH GHI (FUNCTION CODE 0x06 - WRITE SINGLE REGISTER)
	                    else if(function_code == 0x06) {
	                    	uint16_t write_addr = (tcp_rx_buffer[8] << 8) | tcp_rx_buffer[9];
	                    	uint16_t write_value = (tcp_rx_buffer[10] << 8) | tcp_rx_buffer[11];

	                        // 1. Cập nhật giá trị mới vào Register Map nội bộ
	                            if(write_addr < 128) {
	                            	//Gateway_Register_Map[write_addr] = write_value;
	                            	// KHÓA MUTEX: Cập nhật RAM an toàn
	                            	osMutexAcquire(RegMapMutexHandle, osWaitForever);
	                            	Gateway_Register_Map[write_addr] = write_value;
	                            	osMutexRelease(RegMapMutexHandle);
	                            	// ĐÓNG GÓI LỆNH VÀ ĐẨY VÀO QUEUE CHO TẦNG RS485
	                            	RTU_Command_t new_cmd;
	                            	new_cmd.function_code = 0x06;
	                            	new_cmd.address = write_addr;
	                            	new_cmd.value = write_value;
	                            	osMessageQueuePut(CmdQueueHandle, &new_cmd, 0, 0); // Đẩy vào Queue ngay lập tức
	                            }

	                            // 2. Trả lời PC (Theo chuẩn Modbus TCP, lệnh 0x06 chỉ cần vọng (echo) lại y hệt Request)
	                             uint16_t tx_index = 0;

	                             // Copy lại toàn bộ 12 byte từ PC gửi xuống để vọng về
	                             for(int i = 0; i < 12; i++) {
	                            	 tcp_tx_buffer[tx_index++] = tcp_rx_buffer[i];
	                             }

	                             // Phản hồi về PC để xác nhận đã ghi thành công
	                             send(TCP_SOCKET, tcp_tx_buffer, tx_index);
	                      }
	                }
	            }
	            break;

	        case SOCK_CLOSE_WAIT:
	            disconnect(TCP_SOCKET);
	            break;

	        case SOCK_CLOSED:
	            socket(TCP_SOCKET, Sn_MR_TCP, MODBUS_TCP_PORT, 0);
	            break;

	        case SOCK_INIT:
	            listen(TCP_SOCKET);
	            break;
	    }

	    // Nhường CPU cho các Task khác
	    osDelay(10);
	  }
  /* USER CODE END StartModbusTCP_Task */
}

/* USER CODE BEGIN Header_StartRTU_Poll_Task */
/**
* @brief Function implementing the RTU_Poll_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRTU_Poll_Task */
void StartRTU_Poll_Task(void *argument)
{
  /* USER CODE BEGIN StartRTU_Poll_Task */
  /* Infinite loop */
	RTU_Command_t received_cmd;
	modbus_t telegram;

	  // Thiết lập gói tin hỏi
	  telegram.u8id = 1;         // Slave ID
	  telegram.u8fct = 3;        // Function Code 03 (Read Holding Registers)
	  telegram.u16RegAdd = 0;    // Đọc từ thanh ghi số 0
	  telegram.u16CoilsNo = 4;   // Đọc 4 thanh ghi liên tiếp
	  telegram.u16reg = Gateway_Register_Map; // Dữ liệu trả về lưu thẳng vào mảng này

	  /* Infinite loop */
	  for(;;)
	  {
		  // Đứng chờ ở Queue xem có ai ra lệnh không. Chờ tối đa 500ms (500 ticks)
		      osStatus_t status = osMessageQueueGet(CmdQueueHandle, &received_cmd, NULL, 500);

		      if (status == osOK)
		      {
		          // TRƯỜNG HỢP 1: CÓ LỆNH TỪ PC (Nhận được từ Queue)
		          if (received_cmd.function_code == 0x06)
		          {
		              telegram.u8id = 1;            // Gửi tới Relay ID = 1
		              telegram.u8fct = 6;           // Lệnh Ghi (Write Single Register)
		              telegram.u16RegAdd = received_cmd.address;
		              telegram.u16CoilsNo = 1;
		              telegram.u16reg = &received_cmd.value;

		              // Lập tức gửi lệnh đóng/ngắt xuống RS485
		              ModbusQuery(&ModbusRTU_Master, telegram);
		          }
		      }
		      else
		      {
		          // TRƯỜNG HỢP 2: HẾT 500MS MÀ KHÔNG AI RA LỆNH -> ĐI HỎI CẢM BIẾN
		          telegram.u8id = 1;
		          telegram.u8fct = 3;        // Lệnh Đọc
		          telegram.u16RegAdd = 0;
		          telegram.u16CoilsNo = 4;
		          telegram.u16reg = Gateway_Register_Map; // Thư viện sẽ tự cập nhật vào mảng này

		          ModbusQuery(&ModbusRTU_Master, telegram);
		      }
	  }
  /* USER CODE END StartRTU_Poll_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

