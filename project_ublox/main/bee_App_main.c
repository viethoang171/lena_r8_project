/**
 * @file bee_App_main.c
 * @author nguyen__viet_hoang
 * @date 21 September 2023
 * @brief app_main for ublox project
 */

#include "bee_Uart.h"
#include "bee_Sht3x.h"
#include "bee_Lena_r8.h"
#include "bee_Led.h"
#include "bee_rs485.h"

void app_main()
{
    output_vCreate(LED_CONNECTED_BROKER);
    uart_vCreate();
    rs485_init();
    rs485_start();

    sht3x_start();
    mqtt_vLena_r8_start();
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/