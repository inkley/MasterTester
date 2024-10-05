//-----------------------------------------------
//
//
//----------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_can.h"
#include "driverlib/adc.h"
#include "inc/hw_i2c.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"

#define BuildVersion 1000

//I2C Settings
#define NUM_I2C_DATA 8
#define SLAVE_ADDRESS 0x3C

#define SYSTICK_TIMING   1000 //1000 = 1mS   1 = 1 second  10 = 100mS 100 = 10mS

#define SerialBASE  UART0_BASE
#define SerialBAUD  115200
char RcvString[1024]; //created a global variable for recieving serial data.

//CAN Settings
#define CAN_ID             0x101
#define CAN_SENSOR_ID      0x107
#define CAN_BAUD    500000 //500K

//Global CAN
#define CAN_F_EMPTY     0
#define CAN_F_NEW       1
#define CAN_F_OVERRUN   2

unsigned short CANLastDetected=0xffff;

uint32_t SystemClockSpeed=80000000;

typedef struct {
    char FLAGS;
    short ID;
    char MSG[8];
}CAN_MSG_T;

CAN_MSG_T CAN_RECV;

typedef struct {
    unsigned short ID;
    uint32_t Value;
}CANBroadcast;

static CANBroadcast CAN_MODULES[10];

char PrintMsg[255];

#define I2C_TimeOut 10000

//Inkley Sensor Commands
#define icmdReadVersion         01 // Read Version
#define icmdReadData            02 // Sensor Read Data
#define icmdFlashStart          03 // Start recording data into flash
#define icmdFlashReadPos        04 // Read Flash at position
#define icmdFlashEraseFull      05 // Erase Flash
#define icmdFlashSetSampleSize  06 // Set flash sample size.
#define icmdFlashStatus         07 // Get Status of flash reading. Read percent 100 means done.


int TimeOutClock = 0;
int I2C_TimeOutClock = 0;

uint32_t I2C_RcvCommand=0;
uint32_t I2C_RcvCommandParam=0;

bool I2C_RcvNewCommand = false;

//utility functions
void DelayMS(unsigned int delay)
{
    SysCtlDelay(( SysCtlClockGet() / 3 / 1000) * delay );
}
uint32_t bit_clear(uint32_t number,uint32_t bit)
{
    return number & ~((uint32_t)1 << bit);
}

uint32_t bit_toogle(uint32_t number,uint32_t bit)
{
    return number ^ ((uint32_t)1 << bit);
}

uint32_t bit_set(uint32_t number, uint32_t bit)
{
    return number | ((uint32_t)1 << bit);
}

bool bit_check(uint32_t number,uint32_t bit)
{
    return (number >> bit) & (uint32_t)1;
}


//*****************************************************************************
//
// The interrupt handler for the for Systick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{

}

//*****************************************************************************
//
// The interrupt handler for the for I2C0 data slave interrupt.
//
//*****************************************************************************
void
I2C0SlaveIntHandler(void)
{
    //
    // Clear the I2C0 interrupt flag.
    //
    I2CSlaveIntClear(I2C0_BASE);
}

void Init_Systick (void)
{


    SysTickPeriodSet(SysCtlClockGet()/SYSTICK_TIMING); //Set SYSTICK to interrupt every 1mS

    //
    // Enable the SysTick Interrupt.
    //
    SysTickIntEnable();

    //
    // Enable SysTick.
    //
    SysTickEnable();
}

void Init_I2C(void)
{
    //
        // The I2C0 peripheral must be enabled before use.
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

        //
        // For this example I2C0 is used with PortB[3:2].  The actual port and
        // pins used may be different on your part, consult the data sheet for
        // more information.  GPIO port B needs to be enabled so these pins can
        // be used.
        // TODO: change this to whichever GPIO port you are using.
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

        //
        // Configure the pin muxing for I2C0 functions on port B2 and B3.
        // This step is not necessary if your part does not support pin muxing.
        // TODO: change this to select the port/pin you are using.
        //
        GPIOPinConfigure(GPIO_PB2_I2C0SCL);
        GPIOPinConfigure(GPIO_PB3_I2C0SDA);

        //
        // Select the I2C function for these pins.  This function will also
        // configure the GPIO pins pins for I2C operation, setting them to
        // open-drain operation with weak pull-ups.  Consult the data sheet
        // to see which functions are allocated per pin.
        // TODO: change this to select the port/pin you are using.
        //
        GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);

        //
        // Enable loopback mode.  Loopback mode is a built in feature that helps
        // for debug the I2Cx module.  It internally connects the I2C master and
        // slave terminals, which effectively lets you send data as a master and
        // receive data as a slave.  NOTE: For external I2C operation you will need
        // to use external pull-ups that are faster than the internal pull-ups.
        // Refer to the datasheet for more information.
        //
        //HWREG(I2C0_BASE + I2C_O_MCR) |= 0x01;

        //
        // Enable the I2C0 interrupt on the processor (NVIC).
        //
        IntEnable(INT_I2C0);

        //
        // Configure and turn on the I2C0 slave interrupt.  The I2CSlaveIntEnableEx()
        // gives you the ability to only enable specific interrupts.  For this case
        // we are only interrupting when the slave device receives data.
        //
        I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);

        //
        // Enable and initialize the I2C0 master module.  Use the system clock for
        // the I2C0 module.  The last parameter sets the I2C data transfer rate.
        // If false the data rate is set to 100kbps and if true the data rate will
        // be set to 400kbps.  For this example we will use a data rate of 100kbps.
        //
        I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

        //
        // Enable the I2C0 slave module.
        //
        I2CSlaveEnable(I2C0_BASE);

        //
        // Set the slave address to SLAVE_ADDRESS.  In loopback mode, it's an
        // arbitrary 7-bit number (set in a macro above) that is sent to the
        // I2CMasterSlaveAddrSet function.
        //
        I2CSlaveInit(I2C0_BASE, SLAVE_ADDRESS);



}


uint32_t CANSendINT(unsigned long CANID, uint32_t pui8MsgData)
{
        unsigned long TimeOut=0;
        tCANMsgObject sCANMessage;
        //uint32_t ui32MsgData;

        //ui32MsgData = 0;
        sCANMessage.ui32MsgID = CANID;
        sCANMessage.ui32Flags = 0;
        sCANMessage.ui32MsgLen = 4;
        sCANMessage.pui8MsgData =(uint8_t *)&pui8MsgData;

        TimeOut=0;
        CANMessageSet(CAN0_BASE, 32, &sCANMessage, MSG_OBJ_TYPE_TX);


        while(CANStatusGet(CAN0_BASE,CAN_STS_TXREQUEST) !=0)
        {
                TimeOut++;
                SysCtlDelay(SysCtlClockGet() / 30000);
                if(TimeOut>0x0001000)
                {
                     return 0xffffffff;
                }
        }
        return 0;
}
uint32_t CANSendMSG(unsigned long CANID, uint8_t *pui8MsgData)
{
        unsigned long TimeOut=0;
        tCANMsgObject sCANMessage;
        //uint32_t ui32MsgData;

        //ui32MsgData = 0;
        sCANMessage.ui32MsgID = CANID;
        sCANMessage.ui32Flags = 0;
        sCANMessage.ui32MsgLen = 8;
        sCANMessage.pui8MsgData = pui8MsgData;

        TimeOut=0;
        CANMessageSet(CAN0_BASE, 32, &sCANMessage, MSG_OBJ_TYPE_TX);


        while(CANStatusGet(CAN0_BASE,CAN_STS_TXREQUEST) !=0)
        {
                TimeOut++;
                SysCtlDelay(SysCtlClockGet() / 3000);

                if(TimeOut>5000)
                {
                     return 0xffffffff;
                }
        }
        return 0;
}

uint32_t CANPollCheck(unsigned char *candata,int MsgID,unsigned char Response)
{

    int rValue=0;
    uint32_t ulNewData;

    tCANMsgObject sMsgObjectRx;

    sMsgObjectRx.ui32MsgLen = 8;
    sMsgObjectRx.pui8MsgData = candata;

    ulNewData = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);

    while(ulNewData & (1 << (MsgID - 1))) // (0x1 << i) & ulIDStatus
    {
            //
            // Read the message out of the message object.
            //
           CANMessageGet(CAN0_BASE, MsgID, &sMsgObjectRx, true);
           rValue++;
           ulNewData = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);
     }

    return rValue;
}

void IntCAN0Handler(void)
{
    uint32_t ulStatus,ulNewData;
    tCANMsgObject tempCANMsgObject;
    uint8_t CANMsg[8];
    unsigned char CANSlot=1;

    tempCANMsgObject.pui8MsgData = CANMsg;
    tempCANMsgObject.ui32MsgLen=8;


    ulStatus = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
    CANIntClear(CAN0_BASE, ulStatus);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ulStatus != CAN_INT_INTID_STATUS)
    {
        ulStatus = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL); //Read cont status

        ulNewData = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);
        if(ulNewData)
        {
                if(ulNewData & (1 << (CANSlot - 1)))
                {
                    CANMessageGet(CAN0_BASE, CANSlot,&tempCANMsgObject, true); //Get Data and Clear Pending flag
                    if(tempCANMsgObject.ui32MsgID == CAN_ID)
                    {
                        CAN_RECV.ID = tempCANMsgObject.ui32MsgID;
                        memcpy(CAN_RECV.MSG,CANMsg,8);

                        if(bit_check(CAN_RECV.FLAGS,CAN_F_NEW )) //Already have a message in the buffer so overrun condition.
                        {
                            CAN_RECV.FLAGS = bit_set(CAN_RECV.FLAGS,CAN_F_OVERRUN);
                        }
                        CAN_RECV.FLAGS = bit_set(CAN_RECV.FLAGS,CAN_F_NEW);
                    }
                    if(tempCANMsgObject.ui32MsgID == 0x7DF) //Broadcast Message
                       {
                          //Todo search for module already in the list or find an open slot.

                          CAN_MODULES[0].ID = (CANMsg[1] <<8 ) + CANMsg[2];
                          CAN_MODULES[0].Value = (CANMsg[4] >> 24)+(CANMsg[5] >> 16)+(CANMsg[6] >> 8) + CANMsg[7];
                       }
                }
        }
    }
}

//Setup a Can Message location for Receiving
void CANListnerEX(int MsgID)
{
   tCANMsgObject sMsgObjectRx;
   sMsgObjectRx.ui32MsgID = 0;
   sMsgObjectRx.ui32MsgIDMask = 0;
   sMsgObjectRx.ui32Flags = MSG_OBJ_USE_ID_FILTER | MSG_OBJ_EXTENDED_ID;
   sMsgObjectRx.ui32MsgLen = 8;
   sMsgObjectRx.pui8MsgData = (unsigned char *)0xffffffff;
   CANMessageSet(CAN0_BASE, MsgID, &sMsgObjectRx, MSG_OBJ_TYPE_RX);
}

void Init_UART(uint32_t Baud)
{
     //
    // Enable the peripherals used by this example.
    // The UART itself needs to be enabled, as well as the GPIO port
    // containing the pins that will be used.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the GPIO pin muxing for the UART function.
    // This is only necessary if your part supports GPIO pin function muxing.
    // Study the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Since GPIO A0 and A1 are used for the UART function, they must be
    // configured for use as a peripheral function (instead of GPIO).
    // TODO: change this to match the port/pin you are using
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    // This function uses SysCtlClockGet() to get the system clock
    // frequency.  This could be also be a variable or hard coded value
    // instead of a function call.
    //
    UARTConfigSetExpClk(SerialBASE, SysCtlClockGet(), Baud,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
}

void Init_CAN(uint32_t Baud)
{
    //Using Port B pins 4 and 5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //Using CAN0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    //Init CAN Perph.
    CANInit(CAN0_BASE);

    //Set baud rate
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), Baud);

    //Tell which interrupts to trigger
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    //Enable Interrupts
    IntEnable(INT_CAN0);

    //Enable CAN
    CANEnable(CAN0_BASE);

    DelayMS(10);
    CANListnerEX(1); //broadcast lister will receive on mail box 1.
    DelayMS(10);
}

void I2C_SendData(uint32_t SData)
{

    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);

    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData >> 24));

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    I2C_TimeOutClock = I2C_TimeOut;
    while(I2CMasterBusy(I2C0_BASE))
    {
        if(--I2C_TimeOutClock==0)break;
    };

    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData >> 16));
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    I2C_TimeOutClock = I2C_TimeOut;
    while(I2CMasterBusy(I2C0_BASE))
    {
        if(--I2C_TimeOutClock==0)break;
    };

    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData >> 8));
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    I2C_TimeOutClock = I2C_TimeOut;
    while(I2CMasterBusy(I2C0_BASE))
    {
        if(--I2C_TimeOutClock==0)break;
    };

    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData));
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    I2C_TimeOutClock = I2C_TimeOut;
    while(I2CMasterBusy(I2C0_BASE))
    {
        if(--I2C_TimeOutClock==0)break;
    };
}

int UARTStrPut(char *Msg )
{
    int StrPos=0;
    while(Msg[StrPos]!=0)
    {
        UARTCharPut(SerialBASE, Msg[StrPos++]);
    }
    return StrPos;
}

bool UARTHasData()
{
    return UARTCharsAvail(UART0_BASE);
}
//blocking - waits for an enter (\r) from serial
char *UARTStrGet()
{
    int StrPos=0;
    char cThisChar;

    do
    {
        //
        // Read a character using the blocking read function.  This function
        // will not return until a character is available.
        //
        cThisChar = UARTCharGet(UART0_BASE);
        RcvString[StrPos++] = cThisChar;
        
        //
        // Write the same character using the blocking write function.  This
        // function will not return until there was space in the FIFO and
        // the character is written.
        //
        UARTCharPut(UART0_BASE, cThisChar);

    }
    while((cThisChar != '\n') && (cThisChar != '\r'));

    return RcvString;
}

void SendMenu(void)
{

    UARTStrPut("\r\nInkley Sensor Controller Online.\r\n");
    UARTStrPut("\r\n");

    sprintf(PrintMsg,"\r\nHost Clock: %d MHZ \r\n",SystemClockSpeed/1000000);
    UARTStrPut(PrintMsg);

    if(CAN_MODULES[0].ID>0)
    {
        sprintf(PrintMsg,"Detected Module: %04X\r\n",CAN_MODULES[0].ID);
        UARTStrPut(PrintMsg);
        CANLastDetected = CAN_MODULES[0].ID;
    }
    UARTStrPut("\r\nType command # and press enter.\r\n\r\n");

    UARTStrPut("\r\nCommands:\r\n");
    UARTStrPut("1 - Read Version\r\n");
    UARTStrPut("2 - Sensor Read Data\r\n");
    UARTStrPut("3 - Start recording sensor data to flash memory\r\n");
    UARTStrPut("4 - Read Flash at position\r\n");
    UARTStrPut("5 - Erase Flash\r\n");
    UARTStrPut("6 - Set flash memory sample size\r\n");
    UARTStrPut("7 - Get flash memory status\r\n");
    UARTStrPut("\r\n\r\n");
    UARTCharPut(UART0_BASE,'>');
}
void UARTClearScreen(void)
{
    char clrBuf[10];

   // If intention is to only clear current display, and not the entire buffer:

    sprintf(clrBuf,"%c[2J", 0x1b);
    UARTStrPut(clrBuf);
   // Followed by this, if you want to move the cursor back to row:column 0:0, then:

    sprintf(clrBuf,"%c[0;0H", 0x1b);
    UARTStrPut(clrBuf);
}

int main(void)
{

    char CAN_RECV_DATA[20];
    uint8_t CAN_MSG[8];
    uint32_t SampleValue=0;
    uint8_t CMD_RESPID=0;



    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
              SYSCTL_XTAL_16MHZ);

    SystemClockSpeed = SysCtlClockGet();


    Init_Systick();
    Init_UART(115200);
    Init_I2C();
    Init_CAN(CAN_BAUD);//500Kbits

    //Turn of the CAN bus listening using message 1 of the 32 message box in the CAN Perph.
    CANListnerEX(1);
    CAN_MODULES[0].ID=0;


    //give the system some startup time
    DelayMS(2000);

    SendMenu();

/*
//Inkley Sensor Commands
#define icmdReadVersion         01 // Read Version
#define icmdReadData            02 // Sensor Read Data
#define icmdFlashStart          03 // Start recording data into flash
#define icmdFlashReadPos        04 // Read Flash at position
#define icmdFlashEraseFull      05 // Erase Flash
#define icmdFlashSetSampleSize  06 //Set flash sample size.
#define icmdFlashStatus         07// Get Status of flash reading. Read percent 100 means done.
*/
    while(1)
    {
        if(UARTHasData())
        {
            CAN_MSG[0]=0;
            CAN_MSG[1] = CAN_ID >> 8;
            CAN_MSG[2] = (uint8_t)CAN_ID;
            CAN_MSG[3]=0;
            CAN_MSG[4]=0;
            CAN_MSG[5]=0;
            CAN_MSG[6]=0;
            CAN_MSG[7]=0;

            //Wait for user to type in a command.
            switch(strtoul(UARTStrGet(),NULL,0))
            {
                case icmdReadVersion: //Read Version

                    UARTStrPut("Requesting Version from sensor module. \r\n");
                    CAN_MSG[0] = icmdReadVersion;
                    if(CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                      {
                          UARTStrPut("Command Sent. \r\n");
                      }
                    break;

                case icmdReadData: // Read Sensor Data
                    UARTStrPut("Reading Sensor Data. \r\n");
                    CAN_MSG[0] = icmdReadData;
                    if(CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                      {
                          UARTStrPut("Command Sent. \r\n");
                      }
                    break;

                case icmdFlashStart: // Start recording data into flash
                    UARTStrPut("Getting FLASH memory status. \r\n");
                    CAN_MSG[0] = icmdFlashStart;
                    if(CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                      {
                          UARTStrPut("Command Sent. \r\n");
                      }
                    break;

                case icmdFlashReadPos: // Read Flash at position
                    UARTStrPut("Reading FLASH memory data. \r\n");
                    CAN_MSG[0] = icmdFlashReadPos;
                    if(CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                      {
                          UARTStrPut("Command Sent. \r\n");
                      }
                    break;

                case icmdFlashEraseFull: // Erase Flash
                    UARTStrPut("Erasing FLASH memory. \r\n");
                    CAN_MSG[0] = icmdFlashEraseFull;
                    if(CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                    {
                        UARTStrPut("Command Sent. \r\n");
                    }
                    break;

                case icmdFlashSetSampleSize: // Set Flash Sample Size
                     UARTStrPut("Setting Sample size. Enter Value in HEX. Default is 0x10000. \r\n");
                     SampleValue = strtoul(UARTStrGet(),NULL,0);
                     CAN_MSG[0] = icmdFlashEraseFull;
                     CAN_MSG[3] = SampleValue >> 24;
                     CAN_MSG[4] = SampleValue >> 16;
                     CAN_MSG[5] = SampleValue >> 8;
                     CAN_MSG[6] = SampleValue;

                     if(CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                     {
                         UARTStrPut("CAN Network Failed! \r\n");
                     }
                     else
                     {
                           UARTStrPut("Command Sent. \r\n");
                     }
                     break;

                case icmdFlashStatus: // Get flash memory status
                    UARTStrPut("Getting FLASH memory status... \r\n");
                    CAN_MSG[0] = icmdFlashStatus;
                    if(CANSendMSG(CAN_SENSOR_ID, CAN_MSG))
                    {
                        UARTStrPut("CAN Network Failed! \r\n");
                    }
                    else
                    {
                        UARTStrPut("Command Sent. \r\n");
                    }
                    break;

                default:
                    //UARTStrPut("UNKNOWN Command! \r\n");
                    UARTClearScreen();
                    SendMenu();
                    break;
            }
        }
        IntCAN0Handler();
        if(CANLastDetected != CAN_MODULES[0].ID)
        {
            //SendMenu();
        }
        if(bit_check(CAN_RECV.FLAGS,CAN_F_NEW))
        {
            CMD_RESPID = CAN_RECV.MSG[3];

            SampleValue  =  CAN_RECV.MSG[4] << 24;
            SampleValue +=  CAN_RECV.MSG[5] << 16;
            SampleValue +=  CAN_RECV.MSG[6] << 8;
            SampleValue +=  CAN_RECV.MSG[7] ;
            switch(CMD_RESPID)
            {
                case icmdReadVersion: //Read Version
                    sprintf(CAN_RECV_DATA,"Module firmware: %d\r\n",SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                case icmdReadData: // Read Sensor Data
                    sprintf(CAN_RECV_DATA,"RAW sensor data: %d\r\n",SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                case icmdFlashStart: // Start recording data into flash
                    sprintf(CAN_RECV_DATA,"Flash Recording Started: %08X\r\n",SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                case icmdFlashReadPos: // Read Flash at position
                    sprintf(CAN_RECV_DATA,"Flash Recording Position: %08X\r\n",SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                case icmdFlashEraseFull: // Erase Flash
                    sprintf(CAN_RECV_DATA,"Flash Erase Done: %08X\r\n",SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                case icmdFlashSetSampleSize: // Set Flash Sample Size
                    sprintf(CAN_RECV_DATA,"Flash Sample Size Set: %08X\r\n",SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                case icmdFlashStatus: // Get flash memory status
                    sprintf(CAN_RECV_DATA,"Flash Start Position Status: %08X\r\n",SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

                default:
                    sprintf(CAN_RECV_DATA,"Recv Data: %d\r\n",SampleValue);
                    UARTStrPut(CAN_RECV_DATA);
                    break;

            }


            CAN_RECV.FLAGS = bit_clear(CAN_RECV.FLAGS,CAN_F_NEW);

        }
        if(bit_check(CAN_RECV.FLAGS,CAN_F_OVERRUN))
        {
            //detected an over run condition
            CAN_RECV.FLAGS = bit_clear(CAN_RECV.FLAGS,CAN_F_OVERRUN);
        }
    }
}