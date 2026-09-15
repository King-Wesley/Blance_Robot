#include "bsp_key.h"

uint16_t g_key1_long_press = 0;


// 判断按键是否被按下，按下返回KEY_PRESS，松开返回KEY_RELEASE Determine whether the key is pressed. If pressed, returns KEY_PRESS; if released, returns KEY_RELEASE
static uint8_t Key1_is_Press(void)
{
	if (!HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin))
	{
		return KEY_PRESS; // 如果按键被按下，则返回KEY_PRESS If the key is pressed, it returns KEY_PRESS
	}
	return KEY_RELEASE;   // 如果按键是松开状态，则返回KEY_RELEASE If the key is released, it returns KEY_RELEASE
}


// 读取按键K1的长按状态，累计达到长按时间返回1，未达到返回0.
// timeout为设置时间长度，单位为秒
// Read the long press status of button K1. If the long press time is reached, return 1. If it is not reached, return 0.
// timeout is the set time length in seconds
uint8_t Key1_Long_Press(uint16_t timeout)
{
	if (g_key1_long_press > 0)
	{
		if (g_key1_long_press < timeout * 100 + 2)
		{
			g_key1_long_press++;
			if (g_key1_long_press == timeout * 100 + 2)
			{
				return 1;
			}
			return 0;
		}
	}
	return 0;
}



// Read the status of button K1, press to return 1, release to return 0.
// mode: set mode, 0: press to return 1 all the time; 1: press to return 1 only once
// 读取按键K1的状态，按下返回1，松开返回0.
// mode:设置模式，0：按下一直返回1；1：按下只返回一次1
uint8_t Key1_State(uint8_t mode)
{
	static uint16_t key1_state = 0;

	if (Key1_is_Press() == KEY_PRESS)
	{
		if (key1_state < (mode + 1) * 2)
		{
			key1_state++;
		}
	}
	else
	{
		key1_state = 0;
		g_key1_long_press = 0;
	}
	if (key1_state == 2)
	{
		g_key1_long_press = 1;
		return KEY_PRESS;
	}
	return KEY_RELEASE;
}


/*********************************************END OF FILE**********************/
