#include <bsp_usart.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
uint8_t RxTemp = 0;

#define GLASSES_UART_FRAME_LEN      14U
#define GLASSES_UART_PAYLOAD_LEN    9U
#define GLASSES_YAW_PWM_PER_DEG     12.0f
#define GLASSES_YAW_PWM_LIMIT       600.0f
#define GLASSES_UART_TIMEOUT_MS     250U
#define UART1_TEST_MODE             1U
#define UART1_TEST_RX_MAX           96U
#define UART2_TEST_MODE             1U

/* CAM -> STM32: AA 55 | 09 | yaw(float LE) | pitch(float LE) |
 *               walk(0/1) | CRC16-Modbus low byte | high byte. */
static uint8_t glasses_rx_byte;
static uint8_t glasses_rx_build[GLASSES_UART_FRAME_LEN];
static uint8_t glasses_rx_frame[GLASSES_UART_FRAME_LEN];
static volatile uint8_t glasses_rx_index;
static volatile uint8_t glasses_frame_ready;
static uint8_t glasses_yaw_reference_valid;
static float glasses_yaw_reference_deg;
static uint32_t glasses_last_valid_ms;

static char uart1_test_rx_build[UART1_TEST_RX_MAX];
static char uart1_test_rx_line[UART1_TEST_RX_MAX];
static volatile uint8_t uart1_test_rx_length;
static volatile uint8_t uart1_test_line_ready;
static volatile uint8_t uart1_test_last_byte;
static volatile uint8_t uart1_test_byte_ready;
static uint32_t uart1_test_last_heartbeat_ms;
static char uart2_test_rx_build[UART1_TEST_RX_MAX];
static char uart2_test_rx_line[UART1_TEST_RX_MAX];
static volatile uint8_t uart2_test_rx_length;
static volatile uint8_t uart2_test_line_ready;
static uint32_t uart2_test_last_heartbeat_ms;

volatile float Glasses_Pitch_Deg;
volatile float Glasses_Yaw_Deg;
volatile uint8_t Glasses_Imu_Valid;

static uint16_t Glasses_Crc16Modbus(const uint8_t *data, uint8_t length)
{
  uint16_t crc = 0xFFFFU;
  uint8_t i;
  uint8_t bit;
  for (i = 0U; i < length; ++i)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; ++bit)
      crc = (crc & 1U) ? ((crc >> 1U) ^ 0xA001U) : (crc >> 1U);
  }
  return crc;
}

/* Convert the circular yaw difference into [-180, +180] degrees. */
static float Glasses_Wrap180(float angle)
{
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

static void Glasses_Imu_RxByte(uint8_t byte)
{
  /* Resynchronize after noise or a truncated frame. */
  if (glasses_rx_index == 0U)
  {
    if (byte == 0xAAU) glasses_rx_build[glasses_rx_index++] = byte;
    return;
  }
  if (glasses_rx_index == 1U)
  {
    if (byte == 0x55U) glasses_rx_build[glasses_rx_index++] = byte;
    else
    {
      glasses_rx_index = (byte == 0xAAU) ? 1U : 0U;
      if (glasses_rx_index == 1U) glasses_rx_build[0] = 0xAAU;
    }
    return;
  }
  glasses_rx_build[glasses_rx_index++] = byte;
  if (glasses_rx_index == GLASSES_UART_FRAME_LEN)
  {
    if (!glasses_frame_ready)
    {
      memcpy(glasses_rx_frame, glasses_rx_build, GLASSES_UART_FRAME_LEN);
      glasses_frame_ready = 1U;
    }
    glasses_rx_index = 0U;
  }
}

static void Uart2_Test_RxByte(uint8_t byte)
{
  if (byte == '\r') return;

  if (byte == '\n')
  {
    if (uart2_test_rx_length > 0U && !uart2_test_line_ready)
    {
      uart2_test_rx_build[uart2_test_rx_length] = '\0';
      memcpy(uart2_test_rx_line, uart2_test_rx_build, uart2_test_rx_length + 1U);
      uart2_test_line_ready = 1U;
    }
    uart2_test_rx_length = 0U;
    return;
  }

  if (uart2_test_rx_length < (UART1_TEST_RX_MAX - 1U))
  {
    uart2_test_rx_build[uart2_test_rx_length++] = (char)byte;
  }
  else
  {
    uart2_test_rx_length = 0U;
  }
}

void Glasses_Imu_Uart2_Start(void)
{
  glasses_rx_index = 0U;
  glasses_frame_ready = 0U;
  glasses_yaw_reference_valid = 0U;
  glasses_last_valid_ms = HAL_GetTick();
  Glasses_Imu_Valid = 0U;
  HAL_UART_Receive_IT(&huart2, &glasses_rx_byte, 1U);
#if UART2_TEST_MODE
  printf("[UART2 TEST] RX=PA3 TX=PA2, 115200 8N1\r\n");
#else
  printf("[CAM] UART2 RX started: 115200 8N1, PA3=RX\r\n");
#endif
}

void Glasses_Imu_Process(void)
{
  uint16_t crc_received, crc_calculated;
  float yaw, pitch, yaw_delta, turn_pwm;
  uint8_t walk, i;
  uint8_t frame[GLASSES_UART_FRAME_LEN];

  if (!glasses_frame_ready)
  {
    if (Glasses_Imu_Valid &&
        (HAL_GetTick() - glasses_last_valid_ms > GLASSES_UART_TIMEOUT_MS))
    {
      Glasses_Imu_Valid = 0U;
      glasses_yaw_reference_valid = 0U;
      Move_Z = 0.0f;
      printf("[CAM] UART2 timeout, steering cleared\r\n");
    }
    return;
  }

  /* The ISR may receive the next frame while UART1 is printing this one. */
  __disable_irq();
  memcpy(frame, glasses_rx_frame, GLASSES_UART_FRAME_LEN);
  glasses_frame_ready = 0U;
  __enable_irq();

  crc_received = (uint16_t)frame[12] | ((uint16_t)frame[13] << 8U);
  crc_calculated = Glasses_Crc16Modbus(&frame[2], 1U + GLASSES_UART_PAYLOAD_LEN);
  if (frame[2] != GLASSES_UART_PAYLOAD_LEN || crc_received != crc_calculated)
  {
    printf("[CAM] UART2 invalid: len=%u crc=%04X/%04X\r\n", frame[2], crc_received, crc_calculated);
    return;
  }
  memcpy(&yaw, &frame[3], sizeof(yaw));
  memcpy(&pitch, &frame[7], sizeof(pitch));
  walk = frame[11];
  if (!isfinite(yaw) || !isfinite(pitch) || walk > 1U)
  {
    printf("[CAM] UART2 invalid payload\r\n");
    return;
  }
  Glasses_Pitch_Deg = pitch;
  Glasses_Yaw_Deg = yaw;
  Glasses_Imu_Valid = 1U;
  glasses_last_valid_ms = HAL_GetTick();

  /* The first received yaw is the robot's straight-ahead heading.  A later
   * head rotation changes Move_Z, which is added by Turn_PD() to motor PWM. */
  if (!glasses_yaw_reference_valid)
  {
    glasses_yaw_reference_deg = yaw;
    glasses_yaw_reference_valid = 1U;
  }
  yaw_delta = Glasses_Wrap180(yaw - glasses_yaw_reference_deg);
  turn_pwm = -yaw_delta * GLASSES_YAW_PWM_PER_DEG;
  if (turn_pwm > GLASSES_YAW_PWM_LIMIT) turn_pwm = GLASSES_YAW_PWM_LIMIT;
  if (turn_pwm < -GLASSES_YAW_PWM_LIMIT) turn_pwm = -GLASSES_YAW_PWM_LIMIT;
  Move_Z = walk ? turn_pwm : 0.0f;

  printf("[CAM] yaw=%.2f pitch=%.2f walk=%u delta=%.2f turn=%.1f raw=",
         yaw, pitch, walk, yaw_delta, Move_Z);
  for (i = 0U; i < GLASSES_UART_FRAME_LEN; ++i) printf("%02X ", frame[i]);
  printf("\r\n");
}

/* UART1 terminal test: send a heartbeat once per second and echo every line
 * terminated by CR or LF.  This deliberately bypasses Bluetooth parsing. */
void Uart1_Test_Process(void)
{
  char received[UART1_TEST_RX_MAX];
  uint8_t byte;

  if (HAL_GetTick() - uart1_test_last_heartbeat_ms >= 1000U)
  {
    uart1_test_last_heartbeat_ms = HAL_GetTick();
    printf("[UART1 TEST] heartbeat\r\n");
  }

  if (uart1_test_byte_ready)
  {
    __disable_irq();
    byte = uart1_test_last_byte;
    uart1_test_byte_ready = 0U;
    __enable_irq();
    printf("[UART1 TEST] RX byte: 0x%02X\r\n", byte);
  }

  if (!uart1_test_line_ready) return;
  __disable_irq();
  memcpy(received, uart1_test_rx_line, UART1_TEST_RX_MAX);
  uart1_test_line_ready = 0U;
  __enable_irq();
  printf("[UART1 TEST] ECHO: %s\r\n", received);
}

/* UART2 terminal test. PA2 sends a heartbeat; a line containing PING receives
 * one PONG reply. This avoids amplifying a serial loopback into an echo loop. */
void Uart2_Test_Process(void)
{
  char received[UART1_TEST_RX_MAX];
  static const uint8_t heartbeat[] = "[UART2 TEST] heartbeat\r\n";
  static const uint8_t pong[] = "PONG\r\n";

  if (HAL_GetTick() - uart2_test_last_heartbeat_ms >= 1000U)
  {
    uart2_test_last_heartbeat_ms = HAL_GetTick();
    HAL_UART_Transmit(&huart2, (uint8_t *)heartbeat, sizeof(heartbeat) - 1U, 20U);
  }

  if (!uart2_test_line_ready) return;
  __disable_irq();
  memcpy(received, uart2_test_rx_line, UART1_TEST_RX_MAX);
  uart2_test_line_ready = 0U;
  __enable_irq();
  printf("[UART2 TEST] RX line: %s\r\n", received);
  if (strcmp(received, "PING") == 0)
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)pong, sizeof(pong) - 1U, 20U);
    printf("[UART2 TEST] PONG sent\r\n");
  }
}

static void Uart1_Test_RxByte(uint8_t byte)
{
  /* Immediate echo makes a PC-to-STM32 RX wiring test independent of a
   * terminal application's CR/LF setting. */
  if (!uart1_test_byte_ready)
  {
    uart1_test_last_byte = byte;
    uart1_test_byte_ready = 1U;
  }

  if (byte == '\r' || byte == '\n')
  {
    if (uart1_test_rx_length > 0U && !uart1_test_line_ready)
    {
      uart1_test_rx_build[uart1_test_rx_length] = '\0';
      memcpy(uart1_test_rx_line, uart1_test_rx_build, uart1_test_rx_length + 1U);
      uart1_test_line_ready = 1U;
    }
    uart1_test_rx_length = 0U;
    return;
  }

  if (uart1_test_rx_length < (UART1_TEST_RX_MAX - 1U))
  {
    uart1_test_rx_build[uart1_test_rx_length++] = (char)byte;
  }
  else
  {
    uart1_test_rx_length = 0U;
  }
}

/* USER CODE BEGIN 0 */
//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  //Add the following code to support the printf function without selecting use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数   Support functions required by the standard library          
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式   Define _sys_exit() to avoid using semihosting mode 
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 Redefine fputc function
int fputc(int ch, FILE *f)
{      
		while((USART1->SR&0X40)==0);//Flag_Show!=0  使用串口1   Use serial port 1
		USART1->DR = (u8) ch;      
		return ch;
}

#endif




/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
//void USART1_UART_Init(void)
//{
//	//配置不配中断,不需要此串口中断 Configuration does not match interruption, this serial port interruption is not required
////  // Start receiving interrupt 启动接收中断
////  HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
#if UART1_TEST_MODE
    Uart1_Test_RxByte(RxTemp);
#else
    /* USB-UART uses the same $...# protocol as the Bluetooth link. */
    deal_bluetooth(RxTemp);
#endif
    HAL_UART_Receive_IT(&huart1, &RxTemp, 1);
  }
  else if (huart->Instance == USART2)
  {
#if UART2_TEST_MODE
    Uart2_Test_RxByte(glasses_rx_byte);
#else
    Glasses_Imu_RxByte(glasses_rx_byte);
#endif
    HAL_UART_Receive_IT(&huart2, &glasses_rx_byte, 1U);
  }
}

void USART1_Send_Char(char *s)
{
  while (*s != '\0')
  {
    while ((USART1->SR & USART_SR_TXE) == 0) {}
    USART1->DR = (uint8_t)*s++;
  }
}
