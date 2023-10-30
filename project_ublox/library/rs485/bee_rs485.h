/*****************************************************************************
 *
 * @file 	bee_rs485.h
 * @author 	tuha
 * @date 	6 Sep 2023
 * @brief	module for project rs485 communication
 *
 ***************************************************************************/

#ifndef BEE_RS485
#define BEE_RS485

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define TX_PIN (17)
#define RX_PIN (16)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define RTS_PIN (4)

// CTS is not used in RS485 Half-Duplex Mode
#define CTS_PIN (UART_PIN_NO_CHANGE)

#define BUF_SIZE (4096)
#define BAUD_RATE (9600)

// Read packet timeout
#define PACKET_READ_TICS (100 / portTICK_PERIOD_MS)
#define RX_TASK_STACK_SIZE (4096)
#define RX_TASK_PRIO (2)
#define UART_PORT_2 (2)

// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define RX_READ_TOUT (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks

#define CRC8_POLYNOMIAL 0x31

#define ADDRESS_SLAVE_1 0X01
#define ADDRESS_SLAVE_2 0X02
#define ADDRESS_SLAVE_3 0X03

#define BEE_TIME_TRANSMIT_DATA_RS485 1000

typedef struct
{
    uint32_t voltage3pha;
    uint32_t voltageL1;
    uint32_t voltageL2;
    uint32_t voltageL3;

    uint32_t voltageL1L2;
    uint32_t voltageL3L2;
    uint32_t voltageL1L3;

    uint32_t current3pha;
    uint32_t currentL1;
    uint32_t currentL2;
    uint32_t currentL3;
    uint32_t currentN;

    uint32_t actpower3pha;
    uint32_t actpowerL1;
    uint32_t actpowerL2;
    uint32_t actpowerL3;

    uint32_t ractpower3pha;
    uint32_t ractpowerL1;
    uint32_t ractpowerL2;
    uint32_t ractpowerL3;

    uint32_t aprtpower3pha;
    uint32_t aprtpowerL1;
    uint32_t aprtpowerL2;
    uint32_t aprtpowerL3;

    uint16_t Frequency;
} data_3pha_t;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/**
 * @brief Initialize RS485 communication.
 *
 * This function initializes the RS485 communication by configuring the UART
 * interface, setting pins, and enabling the RS485 half-duplex mode.
 *
 */
void rs485_init();

/**
 * @brief Receive and process data from RS485 communication.
 *
 * This task function handles receiving and processing data from RS485 communication.
 * It waits for UART events, reads incoming data, performs CRC16 verification, and processes
 * the received data to extract relevant information.
 *
 * @param[in] pvParameters Unused parameter for FreeRTOS task.
 */
void RX_task(void *pvParameters);

/**
 * @brief Transmit data over the UART communication.
 *
 * This function transmits data over the UART communication on the specified port.
 *
 * @param[in] port The UART port to use for transmission.
 * @param[in] str The data to be transmitted as a character array.
 * @param[in] length The length of the data to be transmitted.
 *
 * @note If the transmission fails, it logs an error and aborts the program.
 */
void TX(const int port, const char *str, uint8_t length);

/**
 * @brief Create a request to read holding registers for a Modbus device.
 *
 * This function constructs a Modbus request for reading holding registers, which includes
 * the slave address, function code, starting register address, the number of registers to read,
 * and CRC checksum.
 *
 * @param[in] slave_addr The slave address of the Modbus device.
 *
 * @return A dynamically allocated character array containing the Modbus request.
 *         The caller is responsible for freeing the memory when done using it.
 *         Returns NULL if memory allocation fails.
 */
char *read_holding_registers(uint8_t slave_addr);

/**
 * @brief Create a JSON representation of 3-phase data and associated metadata.
 *
 * This function constructs a JSON object representing 3-phase data along with metadata
 * such as a device token, command name, and object type. The function also increments a
 * transaction code.
 *
 * @return A dynamically allocated character array containing the JSON representation of 3-phase data.
 *         The caller is responsible for freeing the memory when done using it.
 *         Returns NULL if memory allocation fails.
 */
char *pack_3pha_data(void);
void rs485_start(void);

#endif

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/