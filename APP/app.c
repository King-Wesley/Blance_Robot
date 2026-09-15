#include "app.h"
extern volatile u8 newLineReceived;
extern volatile u8 bulettohflag;

void app_user(void)
{
    if (newLineReceived) {
        ProtocolCpyData();
        Protocol();
    }
    if (bulettohflag) {
        bulettohflag = 0;
        SendAutoUp();
    }
}
