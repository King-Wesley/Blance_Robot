#include "app.h"
extern volatile u8 newLineReceived;
extern volatile u8 bulettohflag;

void app_user(void)
{
    Uart1_Test_Process();
    Uart2_Test_Process();
    Glasses_Imu_Process();
    if (newLineReceived) {
        ProtocolCpyData();
        Protocol();
    }
    if (bulettohflag) {
        bulettohflag = 0;
        SendAutoUp();
    }
}
