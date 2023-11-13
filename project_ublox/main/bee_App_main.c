/**
 * @file bee_App_main.c
 * @author nguyen__viet_hoang
 * @date 21 September 2023
 * @brief app_main for ublox project
 */
#include "bee_Uart.h"
#include "bee_rs485.h"
#include "bee_Button.h"
#include "bee_Pairing_ble.h"

void button_vEventCallback(uint8_t pin)
{
    if (pin == BUTTON_ADVERTISE_BLE)
    {
        wifi_vRetrySmartConfig();
    }
}

void app_main()
{
    button_vCreateInput(BUTTON_ADVERTISE_BLE, HIGH_TO_LOW);
    button_vSetCallback(button_vEventCallback);

    uart_vCreate();
    rs485_init();
    rs485_start();
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/