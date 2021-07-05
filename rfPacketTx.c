/*
 *TIRE PRESSURE MONITORING SYSTEM
 *Date   : 24.06.2021
 *Author : MOUAIAD SALAAS
 *this project copy right return to INOVAR R&D SOLUTIONS limited company
 */


//NOTES:
/*(CC1310)
 *1.OUR USED PINS ARE:
 *                        IOID_20   :   FXTH Activate PIN ,
 *
 *                        IOID_16   :   Our board led,
 *
 *                        IOID_28   :   CC1190 PIN to close before sleep,
 *                        IOID_29   :   CC1190 PIN to close before sleep,
 *                        IOID_30   :   CC1190 PIN to close before sleep,
 *
 *                        IOID_8    :   CC1310 SPI MISO PIN,
 *                        IOID_9    :   CC1310 SPI MOSI PIN,
 *                        IOID_10   :   CC1310 SPI CLCK PIN,
 *                        IOID_11   :   CC1310 SPI CS   PIN,
 *
 */


/*(FXTH)
 *1.OUR USED PINS ARE:
 *                        PTA0   :   FXTH SPI MOSI PIN,
 *                        PTA1   :   FXTH SPI CLCK PIN,
 *                        PTA2   :   FXTH SPI MISO PIN,
 *                        PTB1   :   FXTH SPI CS   PIN,
 */

/************************************************************ Includes ***********************************************************************/
/* Standard C Libraries */
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

/* POSIX Header files */
#include <pthread.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/Board.h>
#include <ti/drivers/Power.h>

/* RTOS header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */
#include "Board.h"
#include "smartrf_settings/smartrf_settings.h"

/************************************************************ defines & variables ***********************************************************************/
#define delayMs(a)                Task_sleep(a*1000 / Clock_tickPeriod);

/* Stack size in bytes */
#define THREADSTACKSIZE    1024

Task_Params taskParams;

Char task0Stack[THREADSTACKSIZE],
     task1Stack[THREADSTACKSIZE];

/* Sytstem Tasks and clocks Structs */
Task_Struct task0Struct,
            task1Struct;

/*SPI*/
SPI_Handle      slaveSpi;
SPI_Params      spiParams;
SPI_Transaction transaction;

/*SPI variables*/
uint8_t Buf[16];
uint16_t TPMS_ID = 0;

uint16_t TPMS_Pressure = 0;
uint16_t Pressure_OUT = 0;

uint16_t TPMS_Accel_Z = 0;
float Accel_Z_OUT = 0;

uint16_t TPMS_Accel_X = 0;
float Accel_X_OUT = 0;

float TPMS_Volt = 0;
uint16_t TPMS_Temp = 0;

/*clock*/
Clock_Params clkParams;
Clock_Struct clk1Struct;
Clock_Handle clk1Handle;

/*Clock variables*/
uint16_t counterGln = 0;
uint16_t counterRf  = 0;
/*Semaphore*/
Semaphore_Params semParams;
Semaphore_Struct semStruct,
                 sem2Struct;

/*Semaphore variables*/
bool flag = false;



/***** Defines *****/

/* Do power measurement */
//#define POWER_MEASUREMENT

/* Packet TX Configuration */
#define PAYLOAD_LENGTH      30
#ifdef POWER_MEASUREMENT
#define PACKET_INTERVAL     5  /* For power measurement set packet interval to 5s */
#else
#define PACKET_INTERVAL     500000  /* Set packet interval to 500000us or 500ms */
#endif


/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

static uint8_t packet[PAYLOAD_LENGTH];
static uint16_t seqNumber;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
//    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    IOID_20        | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    IOID_16        | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,

    IOID_28 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* IOID_28 initially off */
    IOID_29 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* IOID_29 initially off */
    IOID_30 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* IOID_30 initially off */
#ifdef POWER_MEASUREMENT
#if defined(Board_CC1350_LAUNCHXL)
    Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
#endif
    PIN_TERMINATE
};

uint16_t TPMS_Pressure_Wanted = 40;
bool Send_flag = true;

uint16_t TPMS_Pressure_Maximum_Confirm = 0;
bool     Tire_Explosion_flag  = false;
/*******************************************************************Prototypes*********************************************************/

/*1.SPI Callback function */
static void transferCallback(SPI_Handle handle, SPI_Transaction *transaction)
{
    // Start another transfer
    SPI_transfer(handle, transaction);
//    if(Send_flag == false){

        TPMS_Temp = Buf[13] - 55;

        TPMS_Volt = (Buf[12] + 122)/100;

        TPMS_Accel_X = Buf[10];
        TPMS_Accel_X <<= 8;
        TPMS_Accel_X |= Buf[11];
        Accel_X_OUT = 0.039*TPMS_Accel_X+(-10-0.039);

        TPMS_Accel_Z = Buf[8];
        TPMS_Accel_Z <<= 8;
        TPMS_Accel_Z |= Buf[9];
        Accel_Z_OUT = 0.118*TPMS_Accel_Z+(-30-0.118);

        TPMS_Pressure = Buf[6];
        TPMS_Pressure <<= 8;
        TPMS_Pressure |= Buf[7];

//        TPMS_Pressure  = TPMS_Pressure - 10 ;

        Pressure_OUT = 2.750*TPMS_Pressure+(100-2.750);

        TPMS_ID = Buf[5];
        TPMS_ID <<= 8;
        TPMS_ID |= Buf[4];
        TPMS_ID <<= 8;
        TPMS_ID |= Buf[3];
        TPMS_ID <<= 8;
        TPMS_ID |= Buf[2];
        TPMS_ID <<= 8;
        TPMS_ID |= Buf[1];
        TPMS_ID <<= 8;
        TPMS_ID |= Buf[0];


        //Note : we said here TPMS_Pressure_Wanted -TPMS_Pressure)== 10 because the difference betweeen first measure and second my be so little (1 or so small value)
        // and that may effect our algorithm
        if(((TPMS_Pressure_Wanted -TPMS_Pressure)>= 10)&& (Tire_Explosion_flag == false)){   //if pressure now less than excpected pressure by ten
            TPMS_Pressure_Maximum_Confirm++;        //we use a counter to count how many time did it occur
            //Set sleep duration to be 1 minute     //then we set Mcu sleep time to be 1 minute instead 20 minute so we check it fast
            TPMS_Pressure_Wanted = TPMS_Pressure;   //now we need to renew our pressure reference to be last pressure we measured

            if(TPMS_Pressure_Maximum_Confirm ==3){  //in case our counter became three so we need to send warning
                Tire_Explosion_flag = true;         //put the tire explosion flag to be true
            }

        }else if(((TPMS_Pressure_Maximum_Confirm > 0) && (TPMS_Pressure == TPMS_Pressure_Wanted))){
                // this means that the tire not exploded yet but there is stable pressure decrease
        }else{
            TPMS_Pressure_Maximum_Confirm = 0;
            Tire_Explosion_flag = false;
            TPMS_Pressure_Wanted= 40;
            //Set sleep duration to be 20 minute as before
        }

        if(Tire_Explosion_flag){    //if Tire_Explosion_flag has been set so we need to send the data through RF
            Send_flag = true;
            TPMS_Pressure_Wanted= 40;
//            TPMS_Pressure = 40;
            TPMS_Pressure_Maximum_Confirm = 0;
            //Set sleep duration to be 20 minute as before
        }
//    }

}

/**************************************************************************************************************************************/
/*2.spiInit function */
void spiInit()
{
    SPI_Params_init(&spiParams);
    spiParams.frameFormat           = SPI_POL0_PHA1;
    spiParams.mode                  = SPI_SLAVE;
    spiParams.transferMode          = SPI_MODE_CALLBACK;
    spiParams.transferCallbackFxn   = transferCallback;
    spiParams.dataSize              = 8; //8bit
    //spiParams.bitRate               = 38460; //38460

    // Configure the transaction
    transaction.count = 14;
    transaction.txBuf = NULL;
    transaction.rxBuf = Buf;

    slaveSpi = SPI_open(Board_SPI0, &spiParams);

    if (slaveSpi == NULL) {
       // printf("Error initializing slave SPI\n");
        while (1);
    }
    else {
        //printf("Slave SPI initialized\n");
    }

    SPI_transfer(slaveSpi, &transaction);
}

/**************************************************************************************************************************************/
/*3.clk2Fxn function */
void clk2Fxn ( UArg arg0 ){

    counterGln++;
    counterRf++;
}

/**************************************************************************************************************************************/
/*4.MainThread */
void MainThread(void *arg0)
{

    RF_Params rfParams;

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL)
    {
        while(1);
    }

    uint32_t    standbyDuration = 1;
    /* CC1190 PINS*/
    PIN_setOutputValue(ledPinHandle, IOID_28, 0);
    PIN_setOutputValue(ledPinHandle, IOID_29, 0);
    PIN_setOutputValue(ledPinHandle, IOID_30, 0);

    //we need to close rf and we need to make led turned on with one not zero
    while(1){
        PIN_setOutputValue(ledPinHandle, IOID_28, 0);
        PIN_setOutputValue(ledPinHandle, IOID_29, 0);
        PIN_setOutputValue(ledPinHandle, IOID_30, 0);
        PIN_setOutputValue(ledPinHandle, IOID_20, 0);
        sleep(standbyDuration);

        delayMs(10);
        // Read sensor over SPI
        PIN_setOutputValue(ledPinHandle, IOID_20, 1);

        counterGln = 0;

        Clock_start(clk1Handle);

        spiInit();

        // Read sensor over SPI
        while(counterGln <= 1000){
        }
        //SPI_transfer(slaveSpi, &transaction);

        Clock_stop(clk1Handle);
        SPI_transferCancel(slaveSpi);
        SPI_close(slaveSpi);

        PIN_setOutputValue(ledPinHandle, IOID_20, 0);




/********************************************************************************************************/
//        if(Send_flag == true){
//        RF_Params_init(&rfParams);

//
//        #ifdef POWER_MEASUREMENT
//        #if defined(Board_CC1350_LAUNCHXL)
//            /* Route out PA active pin to Board_DIO30_SWPWR */
//            PINCC26XX_setMux(ledPinHandle, Board_DIO30_SWPWR, PINCC26XX_MUX_RFC_GPO1);
//        #endif
//        #endif
//
//            RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
//            RF_cmdPropTx.pPkt = packet;
//            RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
//
//            /* Request access to the radio */
//        #if defined(DeviceFamily_CC26X0R2)
//            rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
//        #else
//            rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
//        #endif// DeviceFamily_CC26X0R2
//
//            /* Set the frequency */
//            RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
//
//            /* Create packet with incrementing sequence number and random payload */
//            packet[0] = (uint8_t)(seqNumber >> 8);
//            packet[1] = (uint8_t)(seqNumber++);
//            packet[2] = (uint8_t)('B');
//            packet[3] = (uint8_t)('a');
//            packet[4] = (uint8_t)('s');
//            packet[5] = (uint8_t)('i');
//            packet[6] = (uint8_t)('n');
//            packet[7] = (uint8_t)('c');
//            packet[8]  = TPMS_Pressure;
//            uint8_t i;
//
//
//            /* Send packet */
//            RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx,
//                                                       RF_PriorityNormal, NULL, 0);
//            switch(terminationReason)
//            {
//                case RF_EventLastCmdDone:
//                    // A stand-alone radio operation command or the last radio
//                    // operation command in a chain finished.
//                    break;
//                case RF_EventCmdCancelled:
//                    // Command cancelled before it was started; it can be caused
//                // by RF_cancelCmd() or RF_flushCmd().
//                    break;
//                case RF_EventCmdAborted:
//                    // Abrupt command termination caused by RF_cancelCmd() or
//                    // RF_flushCmd().
//                    break;
//                case RF_EventCmdStopped:
//                    // Graceful command termination caused by RF_cancelCmd() or
//                    // RF_flushCmd().
//                    break;
//                default:
//                    // Uncaught error event
//                    while(1);
//            }
//
//            uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;
//            switch(cmdStatus)
//            {
//                case PROP_DONE_OK:
//                    // Packet transmitted successfully
//
//                    RF_close(rfHandle);
//                    Send_flag = true;
//                    break;
//                case PROP_DONE_STOPPED:
//                    // received CMD_STOP while transmitting packet and finished
//                    // transmitting packet
//                    break;
//                case PROP_DONE_ABORT:
//                    // Received CMD_ABORT while transmitting packet
//                    break;
//                case PROP_ERROR_PAR:
//                    // Observed illegal parameter
//                    break;
//                case PROP_ERROR_NO_SETUP:
//                    // Command sent without setting up the radio in a supported
//                    // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
//                    break;
//                case PROP_ERROR_NO_FS:
//                    // Command sent without the synthesizer being programmed
//                    break;
//                case PROP_ERROR_TXUNF:
//                    // TX underflow observed during operation
//                    break;
//                default:
//                    // Uncaught error event - these could come from the
//                    // pool of states defined in rf_mailbox.h
//                    while(1);
//            }
//
//    #ifndef POWER_MEASUREMENT
//            PIN_setOutputValue(ledPinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
//    #endif
//            /* Power down the radio */
//            RF_yield(rfHandle);
//
//    #ifdef POWER_MEASUREMENT
//            /* Sleep for PACKET_INTERVAL s */
//            sleep(PACKET_INTERVAL);
//    #else
//            /* Sleep for PACKET_INTERVAL us */
//            usleep(PACKET_INTERVAL);
//    #endif
//
//        }

    }
}

/**************************************************************************************************************************************/
/*Main*/
int main(void){

    /*A.SYSTEM PREPARING : Start*/

    Board_init();   //Board initialization
    SPI_init();

    /*1.MainThread task initialization*/
    Task_Params_init(&taskParams);
    taskParams.stackSize = THREADSTACKSIZE;
    taskParams.stack = &task0Stack;
    taskParams.instance->name = "MainThread";
    Task_construct(&task0Struct, (Task_FuncPtr)MainThread, &taskParams, NULL);

    /*3.Clock initialization*/
    Clock_Params_init(&clkParams);
    clkParams.period = 100;
    clkParams.startFlag = TRUE;

    Clock_construct(&clk1Struct, ( Clock_FuncPtr )clk2Fxn, 50, &clkParams);
    clk1Handle = Clock_handle(&clk1Struct);

    Clock_stop(clk1Handle);

    /*3.Semaphore initialization*/
    Semaphore_Params_init(&semParams);
    Semaphore_construct(&semStruct, 0, &semParams);

    BIOS_start();

}
