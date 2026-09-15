#include "bsp.h"
#include "intsever.h"

void bsp_init(void)
{
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
}
