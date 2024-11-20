/*
 Author:    Jerry Black
 User:      Tyler Inkley

Inkley_MasterTester

• The evaluation board acts as a CAN bus host, sending commands and receiving data from a sensor module on the other end of the CAN bus
• It can also function as a diagnostic tool or controller in future implementations, such as a single-board computer managing other modules
• A serial interface (UART) is used to interact with the system via a user menu
• Allows the user to send commands such as reading sensor data, recording data to flash memory, and erasing flash
• Commands include operations to read sensor versions, start/stop data recordings, and manage flash memory
• The system is equipped to receive and respond to CAN messages, process responses, and handle CAN bus interrupts
• The code initializes and manages communication through UART, I2C, and CAN interfaces
 */

//*****************************************************************************
//
// Firmware Libraries
//
//*****************************************************************************

// Standard C Libraries
#include <stdbool.h>                // For boolean types
#include <stdint.h>                 // For fixed-width integer types
#include <stdlib.h>                 // For memory allocation, process control, conversions
#include <stdio.h>                  // For input/output operations

// Tiva C Series-specific hardware headers (Hardware memory mapping, interrupts, peripherals)
#include "inc/hw_memmap.h"          // Memory map definitions for the Tiva C Series
#include "inc/hw_ints.h"            // Interrupt definitions for the Tiva C Series
#include "inc/hw_can.h"             // CAN controller definitions for the Tiva C Series

// Tiva C Series Driver Library headers (Peripheral drivers and system control)
#include "driverlib/adc.h"          // ADC driver library
#include "inc/hw_i2c.h"             // I2C hardware definitions
#include "driverlib/can.h"          // CAN bus driver library
#include "driverlib/gpio.h"         // GPIO driver library
#include "driverlib/pin_map.h"      // Pin mapping definitions
#include "driverlib/interrupt.h"    // Interrupt controller driver library
#include "driverlib/sysctl.h"       // System control driver library (clock, power, etc.)
#include "driverlib/uart.h"         // UART driver library
#include "driverlib/i2c.h"          // I2C driver library
#include "driverlib/systick.h"      // SysTick timer driver library
#include "driverlib/flash.h"        // Flash memory driver library (for storing sensor data)

// Utility libraries for Tiva C Series
#include "utils/uartstdio.h"        // UART standard I/O utility functions

//*****************************************************************************
//
// System Configuration and Communication Settings
//
//*****************************************************************************

#define BuildVersion 1000           // Defines the build version of the firmware

// I2C Settings
#define NUM_I2C_DATA 8              // Number of data bytes expected for I2C communication
#define SLAVE_ADDRESS 0x3C          // I2C slave address for the connected device

// SysTick timer settings
#define SYSTICK_TIMING   1000       // SysTick timing value (1000 = 1ms, used for time-based operations)

// UART Settings
#define SerialBASE  UART0_BASE      // Base address for UART0, used for serial communication
#define SerialBAUD  115200          // Baud rate for UART communication (115200 bps)
char RcvString[1024];               // Global buffer for receiving serial data

// Flash Settings
#define FlashUserSpace  0x30000     // Starting address for flash memory user space
uint32_t FlashSampleSize = 0x10000; // Default sample size for flash memory (64 KB)

// CAN Bus Settings
#define CAN_ID             0x101    // CAN bus ID for the main module
#define CAN_SENSOR_ID      0x107    // CAN bus ID for the sensor module
#define CAN_BAUD           500000   // CAN bus baud rate set to 500Kbps

// Global CAN message status flags
#define CAN_F_EMPTY     0           // Flag indicating the CAN buffer is empty
#define CAN_F_NEW       1           // Flag indicating a new CAN message has been received
#define CAN_F_OVERRUN   2           // Flag indicating a CAN buffer overrun (data loss)

// Tracks the last detected CAN message (initially set to an invalid value)
unsigned short CANLastDetected = 0xffff;

// System clock speed in Hz (80 MHz)
uint32_t SystemClockSpeed = 80000000;

// Global Timer: Tracks system's elasped time in ms
uint32_t GlobalTimer = 0;

// Structure to hold a CAN message
typedef struct {
    char FLAGS;                     // Flags indicating the status of the CAN message
    short ID;                       // CAN message ID
    char MSG[8];                    // CAN message data (up to 8 bytes)
} CAN_MSG_T;

CAN_MSG_T CAN_RECV;                 // Global variable to store received CAN messages

// Structure to hold CAN broadcast data
typedef struct {
    unsigned short ID;              // CAN broadcast ID
    uint32_t Value;                 // Value associated with the CAN broadcast
} CANBroadcast;

// Array to hold up to 10 CAN broadcast modules
static CANBroadcast CAN_MODULES[10];

// Buffer to hold messages for printing/debugging
char PrintMsg[255];

// I2C Timeout setting
#define I2C_TimeOut 10000           // Timeout value for I2C communication (in cycles)

//*****************************************************************************
//
// Sensor Commands
//
//*****************************************************************************

// Inkley Sensor Commands
/*
#define icmdReadVersion         01  // Command to read the version of the sensor
#define icmdReadData            02  // Command to read sensor data
#define icmdFlashStart          03  // Command to start recording data into flash memory
#define icmdFlashReadPos        04  // Command to read data from a specific position in flash memory
#define icmdFlashEraseFull      05  // Command to erase the entire flash memory
#define icmdFlashSetSampleSize  06  // Command to set the sample size for flash memory
#define icmdFlashStatus         07  // Command to get the status of the flash memory read (e.g., percentage complete)
#define icmdFlashGetData        08  // Get Flash sample from sensor module and store it locally
#define icmdFlashGenCSV         09  // Generate a CSV file from the flash data stored locally
*/

enum {
    icmdReadVersion = 0x01,         // Read sensor firmware version
    icmdReadData,                   // Retrieve current sensor data
    icmdFlashStart,                 // Start recording data into flash memory
    icmdFlashReadPos,               // Read data from a specific flash memory position
    icmdFlashEraseFull,             // Erase all data in flash memory
    icmdFlashSetSampleSize,         // Set the size of samples to store in flash
    icmdFlashStatus,                // Retrieve flash memory operation status
    icmdFlashGetData,               // Fetch raw data from flash memory
    icmdFlashGenCSV                 // Generate CSV-formatted output from flash data
};

int TimeOutClock = 0;               // Global variable to track timeout events
int I2C_TimeOutClock = 0;           // Global variable to track I2C timeout events

uint32_t I2C_RcvCommand = 0;        // Stores the last received I2C command
uint32_t I2C_RcvCommandParam = 0;   // Stores the parameter associated with the received I2C command

bool I2C_RcvNewCommand = false;     // Flag indicating whether a new I2C command has been received

//*****************************************************************************
//
// Utility Functions
//
//*****************************************************************************

// Delays the execution for a specified number of milliseconds
void DelayMS(unsigned int delay)
{
    // The SysCtlDelay function introduces a delay based on the system clock speed
    // The formula ensures the delay is calibrated to milliseconds
    SysCtlDelay(( SysCtlClockGet() / 3 / 1000) * delay );
}

// Clears a specific bit in a number
uint32_t bit_clear(uint32_t number, uint32_t bit)
{
    // Uses bitwise AND and NOT to clear the bit at the specified position
    return number & ~((uint32_t)1 << bit);
}

// Toggles a specific bit in a number
uint32_t bit_toogle(uint32_t number, uint32_t bit)
{
    // Uses bitwise XOR to toggle the bit at the specified position
    return number ^ ((uint32_t)1 << bit);
}

// Sets a specific bit in a number
uint32_t bit_set(uint32_t number, uint32_t bit)
{
    // Uses bitwise OR to set the bit at the specified position
    return number | ((uint32_t)1 << bit);
}

// Checks if a specific bit in a number is set
bool bit_check(uint32_t number, uint32_t bit)
{
    // Shifts the bit to the right and checks if it is set (returns true if set)
    return (number >> bit) & (uint32_t)1;
}

//*****************************************************************************
//
// SysTick Interrupt Handler: Handles system tick interrupts that occur periodically,
// used for time-based tasks or scheduling
//
//*****************************************************************************

void SysTickIntHandler(void)
{
    // Currently empty, the interrupt service routine (ISR) can be filled
    // to handle periodic tasks triggered by the SysTick timer
}

//*****************************************************************************
//
// I2C0 Data Slave Interrupt Handler: Handles I2C slave interrupts on I2C0
// Triggered when the slave device on I2C0 is addressed or when data is
// transmitted/received
//
//*****************************************************************************

void I2C0SlaveIntHandler(void)
{
    // Clear the I2C0 interrupt flag to acknowledge and reset the interrupt
    I2CSlaveIntClear(I2C0_BASE);
}

//*****************************************************************************
//
// SysTick Initialization: Configures the SysTick timer to trigger an interrupt
// at a rate determined by SYSTICK_TIMING (1ms in this case)
//
//*****************************************************************************

void Init_Systick (void)
{
    // Set the SysTick period based on the system clock speed and the timing setting
    // In this case, it will trigger an interrupt every 1 millisecond
    SysTickPeriodSet(SysCtlClockGet()/SYSTICK_TIMING);

    // Enable the SysTick Interrupt to allow the system to handle SysTick-based tasks
    SysTickIntEnable();

    // Enable the SysTick Timer, starting the countdown for periodic interrupts
    SysTickEnable();
}

//*****************************************************************************
//
// I2C Initialization: Configures and initializes the I2C0 peripheral for both
// master and slave modes, sets up the corresponding GPIO pins, and enables
// interrupts for I2C communication
//
//*****************************************************************************

void Init_I2C(void)
{
    // Enable the I2C0 peripheral; must be done before any I2C0 operations
    // can be performed
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    // Enable GPIO port B, which is required for I2C0 on pins B2 (SCL) and B3 (SDA)
    // Consult the data sheet if different GPIO pins are used for I2C on your device
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 on pins B2 (SCL) and B3 (SDA)
    // This is necessary for devices that support pin muxing
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Configure GPIO pins B2 and B3 for I2C operation; these pins are set to
    // open-drain with weak pull-ups for I2C communication
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    // Optional: Enable loopback mode for I2C0; this is useful for debugging
    // as it connects the I2C master and slave internally, allowing testing
    // without external devices (Disabled in this case)
    // HWREG(I2C0_BASE + I2C_O_MCR) |= 0x01;

    // Enable the I2C0 interrupt in the NVIC (Nested Vectored Interrupt Controller)
    // This allows the processor to handle I2C interrupts
    IntEnable(INT_I2C0);

    // Enable the I2C0 slave interrupt; this allows the slave device to interrupt
    // the processor only when it receives data
    I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);

    // Initialize the I2C0 master module using the system clock
    // The third parameter sets the data transfer rate: false for 100kbps, true for 400kbps
    // In this case, 100kbps is chosen for communication
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    // Enable the I2C0 slave module; this prepares I2C0 to operate in slave mode
    I2CSlaveEnable(I2C0_BASE);

    // Set the slave address to SLAVE_ADDRESS (defined earlier in the code)
    // In loopback mode, the slave address is arbitrary but typically must be
    // configured correctly for actual I2C communication
    I2CSlaveInit(I2C0_BASE, SLAVE_ADDRESS);
}

//*****************************************************************************
//
// CAN Communication and Handling Functions: Functions to send and receive messages
// over the CAN bus, poll for new messages, and handle CAN interrupts; including:
// - CANSendINT: Sends 4-byte integer data over CAN
// - CANSendMSG: Sends 8-byte array data over CAN
// - CANPollCheck: Polls the CAN bus for new messages with a specific ID
// - IntCAN0Handler: Handles CAN0 interrupts and processes received messages
//
//*****************************************************************************

//*****************************************************************************
//
// CANSendINT: Sends a 4-byte integer (uint32_t) message over the CAN bus
//
// \param CANID:        The CAN message ID to send
// \param pui8MsgData:  The integer message data to send
//
// \return 0 if successful, or 0xFFFFFFFF if there is a timeout error
//
//*****************************************************************************

uint32_t CANSendINT(unsigned long CANID, uint32_t pui8MsgData)
{
    unsigned long TimeOut = 0;                          // Timeout counter
    tCANMsgObject sCANMessage;                          // CAN message object

    // Set up the CAN message ID and data
    sCANMessage.ui32MsgID = CANID;                      // Set the CAN message ID
    sCANMessage.ui32Flags = 0;                          // No special flags
    sCANMessage.ui32MsgLen = 4;                         // Message length (4 bytes for uint32_t)
    sCANMessage.pui8MsgData = (uint8_t *)&pui8MsgData;  // Cast the data pointer to uint8_t

    // Send the CAN message using message object 32
    CANMessageSet(CAN0_BASE, 32, &sCANMessage, MSG_OBJ_TYPE_TX);

    // Wait for the message to be transmitted, checking for a timeout
    while (CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST) != 0)
    {
        TimeOut++;
        SysCtlDelay(SysCtlClockGet() / 30000);          // Add delay between checks

        // If the timeout exceeds a threshold, return an error code
        if (TimeOut > 0x0001000)
        {
            return 0xffffffff;                          // Timeout error
        }
    }
    return 0;  // Success
}

//*****************************************************************************
//
// CANSendMSG: Sends an 8-byte message (array of bytes) over the CAN bus
//
// \param CANID:        The CAN message ID to send
// \param pui8MsgData:  Pointer to the 8-byte message data to send
//
// \return 0 if successful, or 0xFFFFFFFF if there is a timeout error
//
//*****************************************************************************

uint32_t CANSendMSG(unsigned long CANID, uint8_t *pui8MsgData)
{
    unsigned long TimeOut = 0;                          // Timeout counter
    tCANMsgObject sCANMessage;                          // CAN message object

    // Set up the CAN message ID and data
    sCANMessage.ui32MsgID = CANID;                      // Set the CAN message ID
    sCANMessage.ui32Flags = 0;                          // No special flags
    sCANMessage.ui32MsgLen = 8;                         // Message length (8 bytes)
    sCANMessage.pui8MsgData = pui8MsgData;              // Pointer to the data to send

    // Send the CAN message using message object 32
    CANMessageSet(CAN0_BASE, 32, &sCANMessage, MSG_OBJ_TYPE_TX);

    // Wait for the message to be transmitted, checking for a timeout
    while (CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST) != 0)
    {
        TimeOut++;
        SysCtlDelay(SysCtlClockGet() / 3000);           // Add delay between checks

        // If the timeout exceeds a threshold, return an error code
        if (TimeOut > 5000)
        {
            return 0xffffffff;                          // Timeout error
        }
    }
    return 0;  // Success
}

//*****************************************************************************
//
// CANPollCheck: Polls the CAN bus for a specific message ID and checks for
// new data; if new data is available for the specified message ID, it reads
// the CAN message into a buffer
//
// \param candata:  Pointer to the buffer where the received CAN data will be stored
// \param MsgID:    The CAN message ID to check for
// \param Response: (Not used in the function, but could be used for handling specific responses)
//
// \return The number of messages received with the specified message ID
//
//*****************************************************************************

uint32_t CANPollCheck(unsigned char *candata, int MsgID, unsigned char Response)
{
    int rValue = 0;                        // Counter for the number of received messages
    uint32_t ulNewData;                    // Holds the status of new CAN data

    tCANMsgObject sMsgObjectRx;            // CAN message object for receiving data

    // Set up the CAN message object to expect 8 bytes of data
    sMsgObjectRx.ui32MsgLen = 8;
    sMsgObjectRx.pui8MsgData = candata;    // Point the message data buffer to 'candata'

    // Get the status of new data available on the CAN bus
    ulNewData = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);

    // Loop while there is new data for the specified message ID (MsgID - 1 due to zero-indexing)
    while (ulNewData & (1 << (MsgID - 1)))
    {
        // Read the message from the specified message object (MsgID) and store it in sMsgObjectRx
        // 'true' indicates that the message should be cleared from the message object after reading
        CANMessageGet(CAN0_BASE, MsgID, &sMsgObjectRx, true);
        rValue++;                           // Increment the counter for each received message

        // Check again if there is more new data for the specified message ID
        ulNewData = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);
    }

    return rValue;                          // Return the number of messages received
}

//*****************************************************************************
//
// CAN Interrupt Handler (CAN0): Handles interrupts on the CAN0 interface
// It processes incoming CAN messages, checks for message ID matches, and
// handles overrun conditions
//
//*****************************************************************************

void IntCAN0Handler(void)
{
    uint32_t ulStatus, ulNewData;           // Variables to store interrupt status and new data status
    tCANMsgObject tempCANMsgObject;         // Temporary CAN message object
    uint8_t CANMsg[8];                      // Buffer to hold received CAN data (8 bytes)
    unsigned char CANSlot = 1;              // Slot in which the CAN message will be stored

    // Set up the temporary CAN message object to receive 8 bytes of data
    tempCANMsgObject.pui8MsgData = CANMsg;
    tempCANMsgObject.ui32MsgLen = 8;

    // Get the cause of the interrupt and clear it
    ulStatus = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
    CANIntClear(CAN0_BASE, ulStatus);

    // If the interrupt is not a controller status interrupt, handle it
    if (ulStatus != CAN_INT_INTID_STATUS)
    {
        // Get the controller status
        ulStatus = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        // Check if new data is available on the CAN bus
        ulNewData = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);
        if (ulNewData)
        {
            // Check if there is new data for the specified CAN slot
            if (ulNewData & (1 << (CANSlot - 1)))
            {
                // Get the CAN message and clear the pending flag
                CANMessageGet(CAN0_BASE, CANSlot, &tempCANMsgObject, true);

                // If the message ID matches CAN_ID, process it
                if (tempCANMsgObject.ui32MsgID == CAN_ID)
                {
                    CAN_RECV.ID = tempCANMsgObject.ui32MsgID;                       // Store the message ID
                    memcpy(CAN_RECV.MSG, CANMsg, 8);                                // Copy the message data to the CAN_RECV buffer

                    // Check if the CAN buffer already contains a message (overrun condition)
                    if (bit_check(CAN_RECV.FLAGS, CAN_F_NEW))
                    {
                        CAN_RECV.FLAGS = bit_set(CAN_RECV.FLAGS, CAN_F_OVERRUN);    // Set the overrun flag
                    }
                    CAN_RECV.FLAGS = bit_set(CAN_RECV.FLAGS, CAN_F_NEW);            // Set the flag indicating a new message
                }

                // Handle broadcast messages (ID 0x7DF)
                if (tempCANMsgObject.ui32MsgID == 0x7DF)
                {
                    // TODO: Search for the module in the list or find an open slot

                    // Store the broadcast message details in CAN_MODULES[0]
                    CAN_MODULES[0].ID = (CANMsg[1] << 8) + CANMsg[2];    // Module ID from CAN message
                    CAN_MODULES[0].Value = (CANMsg[4] << 24) + (CANMsg[5] << 16) + (CANMsg[6] << 8) + CANMsg[7];  // Module value
                }
            }
        }
    }
}

//*****************************************************************************
//
// CAN Listener Setup: Configures a CAN message object to receive messages
// with the specified message ID (MsgID); sets up the message object with
// ID filtering and extended ID settings
//
//*****************************************************************************

void CANListnerEX(int MsgID)
{
    tCANMsgObject sMsgObjectRx;              // CAN message object for receiving

    // Set up the message object to use ID filtering and extended IDs
    sMsgObjectRx.ui32MsgID = 0;
    sMsgObjectRx.ui32MsgIDMask = 0;
    sMsgObjectRx.ui32Flags = MSG_OBJ_USE_ID_FILTER | MSG_OBJ_EXTENDED_ID;

    // Set the message length to 8 bytes
    sMsgObjectRx.ui32MsgLen = 8;

    // The message data buffer is set to a dummy value (not used in this configuration)
    sMsgObjectRx.pui8MsgData = (unsigned char *)0xffffffff;

    // Configure the CAN message object to receive messages with the given MsgID
    CANMessageSet(CAN0_BASE, MsgID, &sMsgObjectRx, MSG_OBJ_TYPE_RX);
}

//*****************************************************************************
//
// UART Initialization: Configures and initializes the UART0 interface for serial
// communication; sets the UART baud rate, pin configurations, and peripheral function
// settings
//
//*****************************************************************************

void Init_UART(uint32_t Baud)
{
    // Enable the UART0 peripheral and GPIO port A; required for the UART
    // operation and must be enabled before configuring the UART or GPIO pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);    // Enable UART0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);    // Enable GPIO Port A

    // Configure the pin muxing for the UART0 function on GPIO pins A0 (RX) and A1 (TX)
    // This allows these pins to be used for UART communication instead of general GPIO
    // Consult the datasheet for pin allocation based on the specific device being used
    GPIOPinConfigure(GPIO_PA0_U0RX);                // Configure PA0 for UART0 RX
    GPIOPinConfigure(GPIO_PA1_U0TX);                // Configure PA1 for UART0 TX

    // Set the GPIO pins A0 and A1 for UART operation
    // This configures them as UART peripheral pins rather than general-purpose I/O
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART0 module for 8-N-1 operation (8 data bits, no parity, 1 stop bit)
    // with the specified baud rate; the system clock frequency is used for timing
    UARTConfigSetExpClk(SerialBASE, SysCtlClockGet(), Baud, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

//*****************************************************************************
//
// CAN Initialization: Configures and initializes the CAN0 interface for communication
// This function sets up the CAN baud rate, pin configurations, and interrupts
//
//*****************************************************************************

void Init_CAN(uint32_t Baud)
{
    // Enable GPIO Port B and configure pins B4 and B5 for CAN0 operation
    // These pins are used for CAN0 RX (receive) and TX (transmit) respectively
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);                // Enable GPIO Port B
    GPIOPinConfigure(GPIO_PB4_CAN0RX);                          // Configure PB4 for CAN0 RX
    GPIOPinConfigure(GPIO_PB5_CAN0TX);                          // Configure PB5 for CAN0 TX
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);   // Set PB4 and PB5 for CAN functionality

    // Enable CAN0 peripheral and initialize it
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);                 // Enable CAN0 peripheral
    CANInit(CAN0_BASE);                                         // Initialize CAN0 module

    // Set the CAN baud rate using the system clock and the specified baud rate
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), Baud);

    // Enable CAN interrupts for master, error, and status changes
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    // Enable the CAN0 interrupt in the NVIC (Nested Vectored Interrupt Controller)
    IntEnable(INT_CAN0);

    // Enable the CAN0 module for operation
    CANEnable(CAN0_BASE);

    // Short delay to ensure CAN setup stability
    DelayMS(10);

    // Initialize the CAN listener for receiving broadcast messages on mailbox 1
    CANListnerEX(1);                                            // Set up listener on mailbox 1 for broadcast reception

    // Additional delay to allow CAN listener to be fully initialized
    DelayMS(10);
}

//*****************************************************************************
//
// I2C_SendData: Sends a 32-bit data word over the I2C bus to a specified slave
// This function breaks the data into four 8-bit segments and sends them sequentially
// using I2C burst mode
//
//*****************************************************************************

void I2C_SendData(uint32_t SData)
{
    // Set the slave address for I2C communication
    // 'false' indicates that a write operation is to be performed
    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);

    // Send the most significant byte (MSB) of the 32-bit data
    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData >> 24));

    // Start the I2C burst transmission
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait for the I2C master to finish sending the byte or timeout
    I2C_TimeOutClock = I2C_TimeOut;
    while (I2CMasterBusy(I2C0_BASE))
    {
        if (--I2C_TimeOutClock == 0) break;
    }

    // Send the second byte of the 32-bit data
    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData >> 16));
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

    // Wait for the I2C master to finish sending the byte or timeout
    I2C_TimeOutClock = I2C_TimeOut;
    while (I2CMasterBusy(I2C0_BASE))
    {
        if (--I2C_TimeOutClock == 0) break;
    }

    // Send the third byte of the 32-bit data
    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData >> 8));
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

    // Wait for the I2C master to finish sending the byte or timeout
    I2C_TimeOutClock = I2C_TimeOut;
    while (I2CMasterBusy(I2C0_BASE))
    {
        if (--I2C_TimeOutClock == 0) break;
    }

    // Send the least significant byte (LSB) of the 32-bit data
    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData));
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

    // Wait for the I2C master to finish sending the byte or timeout
    I2C_TimeOutClock = I2C_TimeOut;
    while (I2CMasterBusy(I2C0_BASE))
    {
        if (--I2C_TimeOutClock == 0) break;
    }
}

//*****************************************************************************
//
// UART String Transmission: Sends a null-terminated string over the UART interface
//
// \param Msg - Pointer to the string to be sent
//
// \return The number of characters sent
//
//*****************************************************************************

int UARTStrPut(char *Msg)
{
    int StrPos = 0;  // Position within the string

    // Loop through each character in the string until the null terminator is encountered
    while (Msg[StrPos] != 0)
    {
        // Send the current character over the UART interface
        UARTCharPut(SerialBASE, Msg[StrPos++]);
    }

    return StrPos;  // Return the number of characters sent
}

//*****************************************************************************
//
// UARTHasData: Checks if there is any data available in the UART receive buffer
//
// \return True if data is available, otherwise false
//
//*****************************************************************************
bool UARTHasData()
{
    // Check if there are any characters available in the UART0 receive buffer
    return UARTCharsAvail(UART0_BASE);
}

//*****************************************************************************
//
// UART String Reception (Blocking): Reads a string from the UART interface,
// blocking until a newline ('\n') or carriage return ('\r') character is received
//
// \return Pointer to the received string
//
//*****************************************************************************
char *UARTStrGet()
{
    int StrPos = 0;          // Position within the receive string
    char cThisChar;          // Character currently being read

    do
    {
        // Block until a character is received from the UART interface
        cThisChar = UARTCharGet(UART0_BASE);

        // Store the received character in the global buffer (RcvString)
        RcvString[StrPos++] = cThisChar;

        // Echo the received character back to the UART (for user feedback)
        UARTCharPut(UART0_BASE, cThisChar);

    }
    // Continue until a newline ('\n') or carriage return ('\r') is received
    while ((cThisChar != '\n') && (cThisChar != '\r'));

    // Return the received string
    return RcvString;
}

//*****************************************************************************
//
// SendMenu: Displays the main menu over the UART interface; shows the current
// system status, detected CAN modules, and a list of available commands
//
//*****************************************************************************
void SendMenu(void)
{
    // Send a welcome message indicating the sensor controller is online
    UARTStrPut("\r\nInkley Sensor Controller Online.\r\n");
    UARTStrPut("\r\n");

    // Display the host clock speed in MHz
    sprintf(PrintMsg, "\r\nHost Clock: %d MHZ \r\n", SystemClockSpeed / 1000000);
    UARTStrPut(PrintMsg);

    // If a CAN module has been detected, display its ID
    if (CAN_MODULES[0].ID > 0)
    {
        sprintf(PrintMsg, "Detected Module: %04X\r\n", CAN_MODULES[0].ID);
        UARTStrPut(PrintMsg);
        CANLastDetected = CAN_MODULES[0].ID;  // Update the last detected CAN module ID
    }

    // Prompt the user to type a command and press enter
    UARTStrPut("\r\nType command # and press enter.\r\n\r\n");

    // Display the list of available commands
    UARTStrPut("\r\nCommands:\r\n");
    UARTStrPut("1 - Read Version\r\n");
    UARTStrPut("2 - Sensor Read Data\r\n");
    UARTStrPut("3 - Start recording sensor data to flash memory\r\n");
    UARTStrPut("4 - Read Flash at position\r\n");
    UARTStrPut("5 - Erase Flash\r\n");
    UARTStrPut("6 - Set flash memory sample size\r\n");
    UARTStrPut("7 - Get flash memory status\r\n");
    UARTStrPut("8 - Get flash memory sample.\r\n");
    UARTStrPut("9 - Generate a CSV file from flash memory sample.\r\n");

    // Display a prompt (>) for user input
    UARTStrPut("\r\n\r\n");
    UARTCharPut(UART0_BASE, '>');
}

//*****************************************************************************
//
// UARTClearScreen: Clears the terminal screen and resets the cursor position
// using ANSI escape codes; this function is useful for clearing previous output
//
//*****************************************************************************
void UARTClearScreen(void)
{
    char clrBuf[10];

    // Clear the screen using the ANSI escape sequence (ESC[2J)
    sprintf(clrBuf, "%c[2J", 0x1b);     // 0x1b is the ASCII code for ESC
    UARTStrPut(clrBuf);

    // Move the cursor back to the top-left corner (row 0, column 0) using the
    // ANSI escape sequence (ESC[0;0H)
    sprintf(clrBuf, "%c[0;0H", 0x1b);   // Reset cursor to the home position
    UARTStrPut(clrBuf);
}

//*****************************************************************************
//
// Main Function: Initializes system peripherals (UART, I2C, CAN, etc.), displays
// a menu to the user, and processes commands entered via UART; interacts with
// the CAN bus to send and receive sensor commands and data, and manages the response
// processing loop
//
//*****************************************************************************

int main(void)
{
    // CAN and Data Processing Variables: used for storing CAN message data, processing
    // responses, managing sensor samples, and formatting output for logging/communication
    char CAN_RECV_DATA[255];        // Buffer for formatted CAN data output
    uint8_t CAN_MSG[8];             // Array for CAN message payload
    uint32_t SampleValue = 0;       // Current sensor sample value
    uint8_t CMD_RESPID = 0;         // Response ID of the last processed command
    uint32_t SampleRecv = 0xFFFFFF; // Last received sensor sample (default value)
    uint32_t lop = 0;               // Auxiliary loop counter
    uint32_t Flash_Data = 0;        // Data for flash memory operations
    char CSV_Line[255];             // Buffer for CSV-formatted output

    // Set the system clock to 80MHz (using a 16MHz crystal and PLL)
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Get and store the system clock speed
    SystemClockSpeed = SysCtlClockGet();

    // Initialize system peripherals: SysTick, UART, I2C, and CAN
    Init_Systick();
    Init_UART(115200);      // UART initialized with 115200 baud rate
    Init_I2C();
    Init_CAN(CAN_BAUD);     // CAN initialized with 500Kbps baud rate

    // Turn on CAN listener for mailbox 1 and initialize the CAN modules
    CANListnerEX(1);
    CAN_MODULES[0].ID = 0;

    // Allow some startup time (2 seconds)
    DelayMS(2000);

    // Display the main menu over UART
    SendMenu();

    // Main command processing loop
    while (1)
    {
        // Check if there is data available from the UART
        if (UARTHasData())
        {
            // Prepare the CAN message with default values
            CAN_MSG[0] = 0;                     // Blank - no command
            CAN_MSG[1] = CAN_ID >> 8;           // #define icmdReadVersion         01  // Command to read the version of the sensor
            CAN_MSG[2] = (uint8_t)CAN_ID;       // #define icmdReadData            02  // Command to read sensor data
            CAN_MSG[3] = 0;                     // #define icmdFlashStart          03  // Command to start recording data into flash memory
            CAN_MSG[4] = 0;                     // #define icmdFlashReadPos        04  // Command to read data from a specific position in flash memory
            CAN_MSG[5] = 0;                     // #define icmdFlashEraseFull      05  // Command to erase the entire flash memory
            CAN_MSG[6] = 0;                     // #define icmdFlashSetSampleSize  06  // Command to set the sample size for flash memory
            CAN_MSG[7] = 0;                     // #define icmdFlashStatus         07  // Command to get the status of the flash memory read (e.g., percentage complete)
            //TODO: Don't we need the below CAN messages prepared? I get a warning when I uncomment 'subscript out of range'
            //CAN_MSG[8] = 0;                     // #define icmdFlashGetData        08  // Get Flash sample from sensor module and store it locally
            //CAN_MSG[9] = 0;                     // #define icmdFlashGenCSV         09  // Generate a CSV file from the flash data stored locally

            // Wait for the user to enter a command via UART and process it
            switch (strtoul(UARTStrGet(), NULL, 0))
            {
            //*****************************************************************************
            //
            // UART Command Processing: Handles user commands entered via UART. Commands
            // are processed and mapped to corresponding CAN messages sent to the sensor
            // module. Includes extended functionality for retrieving flash data and
            // generating CSV files.
            //
            // - icmdReadVersion: Requests the firmware version from the sensor module.
            // - icmdReadData: Requests the latest sensor data sample.
            // - icmdFlashStart: Initiates recording of sensor data to flash memory.
            // - icmdFlashReadPos: Retrieves the current read position in flash memory.
            // - icmdFlashEraseFull: Erases all flash memory data.
            // - icmdFlashSetSampleSize: Configures the flash memory sample size.
            // - icmdFlashStatus: Queries the status of flash memory operations.
            // - icmdFlashGetData: Retrieves stored flash memory samples.
            // - icmdFlashGenCSV: Generates CSV-formatted output from flash data.
            //
            // Each command triggers the corresponding CAN message, and the sensor module
            // responds with relevant data or an acknowledgment. Additional functionality
            // like CSV generation adds utility for data export.
            //
            //****************************************************************************

            case icmdReadVersion:           // Read Version Command
                    UARTStrPut("Requesting Version from sensor module. \r\n");
                    CAN_MSG[0] = icmdReadVersion;
                    if (CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                    {
                        UARTStrPut("Command Sent. \r\n");
                    }
                    break;

                case icmdReadData:              // Read Sensor Data Command
                    UARTStrPut("Reading Sensor Data. \r\n");
                    CAN_MSG[0] = icmdReadData;
                    if (CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                    {
                        UARTStrPut("Command Sent. \r\n");
                    }
                    break;

                case icmdFlashStart:            // Start Recording to Flash Command
                    UARTStrPut("Getting FLASH memory status. \r\n");
                    CAN_MSG[0] = icmdFlashStart;
                    if (CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                    {
                        UARTStrPut("Command Sent. \r\n");
                    }
                    break;

                case icmdFlashReadPos:          // Read Flash Memory at Position Command
                    UARTStrPut("Reading FLASH memory data. \r\n");
                    CAN_MSG[0] = icmdFlashReadPos;
                    if (CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                    {
                        UARTStrPut("Command Sent. \r\n");
                    }
                    break;

                case icmdFlashEraseFull:        // Erase Flash Memory Command
                    UARTStrPut("Erasing FLASH memory. \r\n");
                    CAN_MSG[0] = icmdFlashEraseFull;
                    if (CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                    {
                        UARTStrPut("Command Sent. \r\n");
                    }
                    break;

                case icmdFlashSetSampleSize:    // Set Flash Sample Size Command
                    UARTStrPut("Setting Sample size. Enter Value in HEX. Default is 0x10000. \r\n");
                    SampleValue = strtoul(UARTStrGet(), NULL, 0);
                    CAN_MSG[0] = icmdFlashSetSampleSize;
                    CAN_MSG[3] = SampleValue >> 24;
                    CAN_MSG[4] = SampleValue >> 16;
                    CAN_MSG[5] = SampleValue >> 8;
                    CAN_MSG[6] = SampleValue;
                    if (CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                    {
                        UARTStrPut("Command Sent. \r\n");
                    }
                    break;

                case icmdFlashStatus:           // Get Flash Memory Status Command
                    UARTStrPut("Getting FLASH memory status... \r\n");
                    CAN_MSG[0] = icmdFlashStatus;
                    if (CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                    {
                        UARTStrPut("Command Sent. \r\n");
                    }
                    break;

                case icmdFlashGetData:          // Retrieve Flash Memory Samples
                    UARTStrPut("Requesting flash memory samples from sensor module. \r\n");
                    CAN_MSG[0] = icmdFlashGetData;
                    if (CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                    {
                        UARTStrPut("Command Sent. \r\n");
                    }
                    break;

                case icmdFlashGenCSV:           // Generate CSV from Flash Data
                    GlobalTimer = 0;
                    UARTStrPut("CSV BEGIN:\r\n\r\n\r\n");

                    // Add column headers to the CSV output
                    sprintf(CSV_Line, "TimeStamp,Pressure\r\n");
                    UARTStrPut(CSV_Line);

                    // Loop through flash memory to generate rows of CSV data
                    for (lop = FlashUserSpace; lop <= FlashUserSpace + FlashSampleSize; lop += 4)
                    {
                        Flash_Data = *((uint32_t *)lop);                            // Read data from flash memory
                        sprintf(CSV_Line, "%d,%d\r\n", GlobalTimer, Flash_Data);    // Format as CSV
                        UARTStrPut(CSV_Line);                                       // Send CSV line via UART
                        GlobalTimer++;                                              // Increment timestamp
                    }

                    UARTStrPut("\r\n\r\n\r\n CSV END:\r\n");                        // Indicate end of CSV
                    break;

                default:                        // Unknown Command
                    UARTClearScreen();          // Clear the screen
                    SendMenu();                 // Re-display the menu
                    break;
            }
        }

        // Call the CAN interrupt handler to process incoming CAN messages
        IntCAN0Handler();

        // Check for any changes in the detected CAN module
        if (CANLastDetected != CAN_MODULES[0].ID)
        {
            // Optionally re-display the menu if a new CAN module is detected
            // SendMenu();
        }

        // Process new CAN messages if available.
        if (bit_check(CAN_RECV.FLAGS, CAN_F_NEW))
        {
            CMD_RESPID = CAN_RECV.MSG[3];       // Extract the command response ID

            // Combine the message data to form a sample value
            SampleValue  =  CAN_RECV.MSG[4] << 24;
            SampleValue +=  CAN_RECV.MSG[5] << 16;
            SampleValue +=  CAN_RECV.MSG[6] << 8;
            SampleValue +=  CAN_RECV.MSG[7];

            // Process the response based on the received command ID
            switch (CMD_RESPID)
            {
                case icmdReadVersion:           // Read Version
                    sprintf(CAN_RECV_DATA, "Module firmware: %d\r\n", SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                case icmdReadData:              // Read Sensor Data
                    sprintf(CAN_RECV_DATA, "RAW sensor data: %d\r\n", SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                case icmdFlashStart:            // Start recording data into flash
                    sprintf(CAN_RECV_DATA, "Flash Recording Started: %08X\r\n", SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                case icmdFlashReadPos:          // Read Flash at position
                    sprintf(CAN_RECV_DATA, "Flash Recording Position: %08X\r\n", SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                case icmdFlashEraseFull:        // Erase Flash
                    sprintf(CAN_RECV_DATA, "Flash Erase Done: %08X\r\n", SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                case icmdFlashSetSampleSize:   // Set Flash Sample Size
                    sprintf(CAN_RECV_DATA, "Flash Sample Size Set: %08X\r\n", SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                case icmdFlashStatus:           // Get flash memory status
                    sprintf(CAN_RECV_DATA, "Flash Start Position Status: %08X\r\n", SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                case icmdFlashGetData:          // Get Flash sample from sensor module and store it locally
                    if (SampleRecv == 0xFFFFFF)                 // Check if this is the first data packet being received
                    {
                        // Display the size of the sample being received
                        sprintf(CAN_RECV_DATA, "Receiving Sample Data Size: %08X\r\n", SampleValue);
                        UARTStrPut(CAN_RECV_DATA);

                        // Initialize the flash sample size and starting address
                        FlashSampleSize = SampleValue;
                        SampleRecv = FlashUserSpace;

                        // Erase the flash memory at the starting address to prepare for writing
                        FlashErase(SampleRecv);
                    }
                    else
                    {
                        if (SampleValue == 0)                   // Check if this is the end of the sample transmission
                        {
                            // Reset the sample receiving process
                            SampleRecv = 0xFFFFFF;

                            // Indicate that the sample reception has completed
                            sprintf(CAN_RECV_DATA, "Sample Received.\r\n");
                            UARTStrPut(CAN_RECV_DATA);
                        }
                        else
                        {
                            // Incrementally erase flash blocks as needed
                            if ((SampleRecv & 0x7FF) == 0x400)  // Erase flash in 0x400 blocks
                            {
                                FlashErase(SampleRecv);
                            }

                            // Program (write) the received sample value to flash memory
                            FlashProgram(&SampleValue, SampleRecv += 4, 4);
                        }
                    }
                    break;

                //TODO: Missing case for icmdFlashGenCSV?

                default:
                    sprintf(CAN_RECV_DATA, "Recv Data: %d\r\n", SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;
            }

            // Clear the new message flag after processing
            CAN_RECV.FLAGS = bit_clear(CAN_RECV.FLAGS, CAN_F_NEW);
        }

        // Handle CAN message buffer overrun conditions
        if (bit_check(CAN_RECV.FLAGS, CAN_F_OVERRUN))
        {
            // Clear the overrun flag after detecting the condition
            CAN_RECV.FLAGS = bit_clear(CAN_RECV.FLAGS, CAN_F_OVERRUN);
        }
    }
}
