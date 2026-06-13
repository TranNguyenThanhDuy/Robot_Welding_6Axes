/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Mã nguồn STM32 quét nút bấm chống dội phím tuyệt đối
  * @note           : Đã loại bỏ ngắt EXTI để sửa lỗi truyền trùng lặp liên tục
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Các biến cờ lưu trạng thái nút nhấn (0: OFF, 1: ON)
uint8_t state_OnOff = 0;
uint8_t state_Record = 0;

// Mảng lưu trạng thái trước đó của các nút để bắt cạnh nhấn xuống (Edge Detection)
uint8_t last_btn_state = 1;
uint8_t last_btn2_state = 1;
uint8_t last_btn3_state = 1;
uint8_t last_btn4_state = 1;
uint8_t last_btn5_state = 1;
/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void uartsend(const char* msg);
void Scan_Buttons(void); // Hàm quét phím tuần tự thay thế cho ngắt EXTI cũ
/* USER CODE END 0 */

/**
  * @brief  Điểm khởi chạy chính của chương trình
  * @retval int
  */
int main(void)
{
  /* Khởi tạo thư viện HAL */
  HAL_Init();

  /* Cấu hình chính xác Clock hệ thống lên chuẩn 72MHz */
  SystemClock_Config();

  /* Khởi tạo các ngoại vi phần cứng */
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* Vòng lặp vô hạn */
  while (1)
  {
      // Liên tục gọi hàm quét và xử lý nút bấm
      Scan_Buttons();

      // Delay nhỏ 10ms để giảm tải cho CPU và tạo khoảng thời gian lọc dội phím tự nhiên
      HAL_Delay(10);
  }
}

/**
  * @brief  Hàm quét trạng thái và xử lý chống dội tuyệt đối cho 5 nút bấm
  * @note   Chỉ kích hoạt truyền UART ĐÚNG 1 LẦN duy nhất khi nút được nhấn xuống
  */
void Scan_Buttons(void)
{
    // ----------------- NÚT 1: ON/OFF SERVO -----------------
    uint8_t current_btn = HAL_GPIO_ReadPin(GPIOB, button_Pin);
    // Phát hiện cạnh xuống (Trạng thái trước đó là 1 (nhả), trạng thái hiện tại là 0 (nhấn))
    if (last_btn_state == 1 && current_btn == 0)
    {
        HAL_Delay(20); // Chờ 20ms vượt qua vùng nhiễu dội cơ khí ban đầu
        if (HAL_GPIO_ReadPin(GPIOB, button_Pin) == 0) // Xác nhận nút thực sự bị nhấn
        {
            if (state_OnOff == 0) {
                state_OnOff = 1;
                uartsend("on");
            } else {
                state_OnOff = 0;
                uartsend("off");
            }
        }
    }
    last_btn_state = current_btn; // Lưu lại trạng thái để so sánh cho vòng quét sau

    // ----------------- NÚT 2: HOME ROBOT -----------------
    uint8_t current_btn2 = HAL_GPIO_ReadPin(GPIOB, button2_Pin);
    if (last_btn2_state == 1 && current_btn2 == 0)
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, button2_Pin) == 0)
        {
            uartsend("home");
        }
    }
    last_btn2_state = current_btn2;

    // ----------------- NÚT 3: RECORD / STOP -----------------
    // Lưu ý: Sửa lại đúng Port của button3 theo cấu hình CubeMX của bạn (ở đây tạm để button3_GPIO_Port)
    uint8_t current_btn3 = HAL_GPIO_ReadPin(button3_GPIO_Port, button3_Pin);
    if (last_btn3_state == 1 && current_btn3 == 0)
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(button3_GPIO_Port, button3_Pin) == 0)
        {
            if (state_Record == 0) {
                state_Record = 1;
                uartsend("record");
            } else {
                state_Record = 0;
                uartsend("stop");
            }
        }
    }
    last_btn3_state = current_btn3;

    // ----------------- NÚT 4: GO TRAJECTORY -----------------
    uint8_t current_btn4 = HAL_GPIO_ReadPin(GPIOB, button4_Pin);
    if (last_btn4_state == 1 && current_btn4 == 0)
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, button4_Pin) == 0)
        {
            uartsend("go");
        }
    }
    last_btn4_state = current_btn4;

    // ----------------- NÚT 5: MOVE POSITION -----------------
    uint8_t current_btn5 = HAL_GPIO_ReadPin(GPIOB, button5_Pin);
    if (last_btn5_state == 1 && current_btn5 == 0)
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, button5_Pin) == 0)
        {
            uartsend("movepos");
        }
    }
    last_btn5_state = current_btn5;
}

/**
  * @brief  Hàm đóng gói dữ liệu và truyền qua UART1
  */
void uartsend(const char* msg)
{
    char txBuffer[64];
    snprintf(txBuffer, sizeof(txBuffer), "%s\r\n", msg);
    HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, strlen(txBuffer), 100);
}

/**
  * @brief  Cấu hình xung Clock hệ thống (72MHz)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
  * @brief  Khởi tạo cổng USART1 (115200 Baud)
  */
static void MX_USART1_UART_Init(void)
{
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
}

/**
  * @brief  Khởi tạo các chân GPIO dạng INPUT thông thường
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // SỬA ĐỔI: Chuyển sang chế độ GPIO_MODE_INPUT thông thường thay vì Ngắt EXTI
  GPIO_InitStruct.Pin = button3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(button3_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = button_Pin|button2_Pin|button4_Pin|button5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
