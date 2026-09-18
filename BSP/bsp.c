#include "bsp.h"
#include "intsever.h"

void bsp_init(void)
{
    /* Pitch servo on J6-L8 (PB1): start and hold its mechanical centre. */
    Servo_Pitch_Init();
    Servo_Pitch_StartSweep90To0();
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    delay_init();
    init_led_gpio();
    init_beep();
    Motor_start();
    Encoder_Init_TIM3();
    Encoder_Init_TIM4();
    HAL_Delay(300);
    MPU6050_initialize();
    /* DMP initialization also configures the 200 Hz sensor interrupt. */
    DMP_Init();
    Battery_init();
    HAL_UART_Receive_IT(&huart1, &RxTemp, 1);
}
