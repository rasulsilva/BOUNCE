///Rasul Silva
//LAB AO2     TA: Sharmila Kulkarni
//ID: 913737619
//EEC 172
//LAB 2

// Standard includes
#include <string.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "gpio.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"

// Common interface includes
#include "uart_if.h"
#include "pin_mux_config.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "i2c_if.h"


#define APPLICATION_VERSION     "1.1.1"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);

//timer_if variables
volatile unsigned long timearray[18];//17
volatile unsigned long diffarray[18];
char lastfour[4]="    ";
unsigned long arrayindex;
volatile unsigned long count;


volatile unsigned int time = 0;
volatile unsigned int timeouttime = 0;
volatile unsigned int prevtimeouttime = 0;
volatile unsigned int timeoutlength = 0;
volatile unsigned int normtime = 0;
int i;
int charx = 50;
int chary = 30;
int pressflag = 0;
int timerflag = 0;



static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;
//

volatile unsigned long SW2_intcount;
volatile unsigned long SW3_intcount;
volatile unsigned long displaycount;
volatile unsigned long remotepresscount;
volatile unsigned char SW2_intflag;
volatile unsigned char SW3_intflag;
volatile unsigned char remotepressflag;
volatile unsigned long timeout = 0;
volatile unsigned char sendchar = " ";


int w;
int e = 0;
int q;
int a;
int sendx;
int pulse = 0;
int wave[17];
int bitvalue;
int currnum;
int prevnum;
int indextup = 11;
unsigned char goodchar = ' ';
unsigned char prevchar = ' ';
unsigned char word[10] = "          ";
int wordindex = 0;
int extract_flag = 0;


//additions for final
int gameclock = 0;
int endtime = 0;
int state = 0;
int statezerocount = 0;
int stateonecount = 0;
int statetwocount = 0;
char gameclockbuffer[5];

int ballx = 40;
int bally =  52;
int prevxpos;
int prevypos;
int xspeed = 5;
int yspeed = 0;

int death = 0;

unsigned char Acceladdr = 0x18;//addresses to make code cleaner later
unsigned char xreg = 0x3;
unsigned char yreg = 0x5;
unsigned char zreg = 0x7;
unsigned char xbuff[256];//buffers used for write and read
unsigned char ybuff[256];
unsigned char high[2]="1 ";

int xdata;
int ydata;

int paddlevelocity;
int paddleleftside = 5;

char str[1] = "0";
int gravity = 4;



//additions end

static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;

volatile char Tx[10];
volatile char Rx[10];

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif




#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************



//*****************************************************************************
//
//! SPI Slave Interrupt handler
//!
//! This function is invoked when SPI slave has its receive register full or
//! transmit register empty.
//!
//! \return None.
//
//*****************************************************************************

// an example of how you can use structs to organize your pin settings for easier maintenance
typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

static void SlaveIntHandler()
{
    unsigned long ulRecvData;
    unsigned long ulStatus;

    ulStatus = MAP_SPIIntStatus(GSPI_BASE,true);

    MAP_SPIIntClear(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    if(ulStatus & SPI_INT_TX_EMPTY)
    {
        MAP_SPIDataPut(GSPI_BASE,g_ucTxBuff[ucTxBuffNdx%TR_BUFF_SIZE]);
        ucTxBuffNdx++;
    }

    if(ulStatus & SPI_INT_RX_FULL)
    {
        MAP_SPIDataGetNonBlocking(GSPI_BASE,&ulRecvData);
        g_ucTxBuff[ucRxBuffNdx%TR_BUFF_SIZE] = ulRecvData;
        Report("%c",ulRecvData);
        ucRxBuffNdx++;
    }
}

//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI modelue as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void MasterMain()
{

    unsigned long ulUserData;
    unsigned long ulDummy;




    //
    // Initialize the message
    //
    memcpy(g_ucTxBuff,MASTER_MSG,sizeof(MASTER_MSG));

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    Message("Enabled SPI Interface in Master Mode\n\r");

    //
    // User input
    //
    Report("Press any key to transmit data....");

    //
    // Read a character from UART terminal
    //
    ulUserData = MAP_UARTCharGet(UARTA0_BASE);


    //
    // Send the string to slave. Chip Select(CS) needs to be
    // asserted at start of transfer and deasserted at the end.
    //
    MAP_SPITransfer(GSPI_BASE,g_ucTxBuff,g_ucRxBuff,50,
            SPI_CS_ENABLE|SPI_CS_DISABLE);

    //
    // Report to the user
    //
    Report("\n\rSend      %s",g_ucTxBuff);
    Report("Received  %s",g_ucRxBuff);

    //
    // Print a message
    //
    Report("\n\rType here (Press enter to exit) :");

    //
    // Initialize variable
    //
    ulUserData = 0;

    //
    // Enable Chip select
    //
    MAP_SPICSEnable(GSPI_BASE);

    //
    // Loop until user "Enter Key" is
    // pressed
    //
    while(ulUserData != '\r')
    {
        //
        // Read a character from UART terminal
        //
        ulUserData = MAP_UARTCharGet(UARTA0_BASE);

        //
        // Echo it back
        //
        MAP_UARTCharPut(UARTA0_BASE,ulUserData);

        //
        // Push the character over SPI
        //
        MAP_SPIDataPut(GSPI_BASE,ulUserData);

        //
        // Clean up the receive register into a dummy
        // variable
        //
        MAP_SPIDataGet(GSPI_BASE,&ulDummy);
    }

    //
    // Disable chip select
    //
    MAP_SPICSDisable(GSPI_BASE);
}

//*****************************************************************************
//
//! SPI Slave mode main loop
//!
//! This function configures SPI modelue as slave and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void SlaveMain()
{
    //
    // Initialize the message
    //
    memcpy(g_ucTxBuff,SLAVE_MSG,sizeof(SLAVE_MSG));

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_SLAVE,SPI_SUB_MODE_0,
                     (SPI_HW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Register Interrupt Handler
    //
    MAP_SPIIntRegister(GSPI_BASE,SlaveIntHandler);

    //
    // Enable Interrupts
    //
    MAP_SPIIntEnable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    Message("Enabled SPI Interface in Slave Mode\n\rReceived : ");
}



//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}



void paintball(int xposition, int yposition, int colornum) {//paints a ball with the color specified by third parameter
    if (colornum == 1) {
    fillCircle(xposition,yposition,6,RED);
        }
    else if (colornum == 2) {
    fillCircle(xposition,yposition,6,MAGENTA);
        }
    else if (colornum == 3) {
        fillCircle(xposition,yposition,6,BLUE);
        }
    else if (colornum == 4) {
        fillCircle(xposition,yposition,6,CYAN);
        }
    else if (colornum == 5) {
        fillCircle(xposition,yposition,6,GREEN);
        }
    else  {
        fillCircle(xposition,yposition,6,YELLOW);
        }

    }


void eraseball(int xposition, int yposition) {//erase ball at parameters x and y positions
    fillCircle(xposition,yposition,6,0);
    }

void getgoodchar(int previousnum, int currentnum, int prevchar) {

    if (previousnum != currentnum) {

        if (currentnum == 1){goodchar = ' ';}
        else if (currentnum == 2){goodchar = 'A';}
        else if (currentnum == 3){goodchar = 'D';}
        else if (currentnum == 4){goodchar = 'G';}
        else if (currentnum == 5){goodchar = 'J';}
        else if (currentnum == 6){goodchar = 'M';}
        else if (currentnum == 7){goodchar = 'P';}
        else if (currentnum == 8){goodchar = 'T';}
        else if (currentnum == 9){goodchar = 'W';}
        else if (currentnum == 0){goodchar = ' ';}
        }
    else
        if (previousnum == currentnum) {//bang
        if (previousnum == 2) {
            if (prevchar == 'A') {
                goodchar = 'B';

            }
            else if(prevchar == 'B'){
                goodchar = 'C';

            }
            else if(prevchar == 'C'){
                goodchar = 'A';

            }
        }
        else if (previousnum == 3) {
                    if (prevchar == 'D') {
                        goodchar = 'E';
                    }
                    else if(prevchar == 'E'){
                        goodchar = 'F';
                    }
                    else if(prevchar == 'F'){
                        goodchar = 'D';
                    }
                }
        else if (previousnum == 4) {
                    if (prevchar == 'G') {
                        goodchar = 'H';
                    }
                    else if(prevchar == 'H'){
                        goodchar = 'I';
                    }
                    else if(prevchar == 'I'){
                        goodchar = 'G';
                    }
                }
        else if (previousnum == 5) {
                    if (prevchar == 'J') {
                        goodchar = 'K';
                    }
                    else if(prevchar == 'K'){
                        goodchar = 'L';
                    }
                    else if(prevchar == 'L'){
                        goodchar = 'J';
                    }
                }
        else if (previousnum == 6) {
                    if (prevchar == 'M') {
                        goodchar = 'N';
                    }
                    else if(prevchar == 'N'){
                        goodchar = 'O';
                    }
                    else if(prevchar == 'O'){
                        goodchar = 'M';
                    }
                }
        else if (previousnum == 7) {
                    if (prevchar == 'P') {
                        goodchar = 'Q';
                    }
                    else if(prevchar == 'Q'){
                        goodchar = 'R';
                    }
                    else if(prevchar == 'R'){
                        goodchar = 'S';
                    }
                    else if(prevchar == 'S'){
                        goodchar = 'P';
                    }
                }
         else if (previousnum == 8) {
                    if (prevchar == 'T') {
                        goodchar = 'U';
                    }
                    else if(prevchar == 'U'){
                        goodchar = 'V';
                    }
                    else if(prevchar == 'V'){
                        goodchar = 'T';
                    }
         }
         else if (previousnum == 9) {
                           if (prevchar == 'W') {
                               goodchar = 'X';
                           }
                           else if(prevchar == 'X'){
                               goodchar = 'Y';
                           }
                           else if(prevchar == 'Y'){
                               goodchar = 'Z';
                           }
                           else if(prevchar == 'Z'){
                               goodchar = 'W';
                           }
                }







    }//bang

}


static void GPIOA2IntHandler(void) { // remote press handler
    unsigned long ulStatus;
    ulStatus = MAP_GPIOIntStatus (GPIOA2_BASE, true);
    MAP_GPIOIntClear(GPIOA2_BASE, ulStatus);        // clear interrupts on GPIOA2
    remotepresscount++;
    remotepressflag=1;

    time = Timer_IF_GetCount(g_ulBase, 0x000000ff);
    normtime = time/1000;

    if (remotepresscount > 18) {
      remotepresscount = 1;
    };
    if (remotepresscount > 0) {
      wave[remotepresscount-1] = bitvalue;
      timearray[remotepresscount-1] = normtime;
    };

    MAP_GPIOIntDisable(GPIOA2_BASE, 0x2);
}



void TimerBaseIntHandler(void){
    Timer_IF_InterruptClear(g_ulBase);// Clear the timer interrupt.
    g_ulTimerInts ++;
    //GPIO_IF_LedToggle(MCU_GREEN_LED_GPIO);
}

void extractmessage() {
    unsigned long ulStatus;

    ulStatus = UARTIntStatus(UARTA1_BASE, true);
    UARTIntClear(UARTA1_BASE, ulStatus );
    //UARTIntDisable(UARTA1_BASE, UART_INT_RX);
    w = 0;
    while (UARTCharsAvail(UARTA1_BASE)){
        Rx[w] = 1;
        w=w+1;

    }
    extract_flag = 1;
    //UARTIntEnable(UARTA1_BASE, UART_INT_RX);


}
void UART1InterruptInit() {
//    UARTConfigSetExpClk(UARTA0_BASE, MAP_PRCMPeripheralClockGet(PRCM_UARTA0),
//                    UART_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
//                    UART_CONFIG_PAR_NONE));

    UARTConfigSetExpClk(UARTA1_BASE, MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                        UART_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));
        UARTEnable(UARTA1_BASE);
    //UARTFIFODisable(UARTA1_BASE);
    UARTIntRegister(UARTA1_BASE,extractmessage);
    UARTIntClear(UARTA1_BASE, UART_INT_RX);
    UARTIntEnable(UARTA1_BASE, UART_INT_RX);

}






void main()

{
    unsigned long ulStatus;

    BoardInit();

    // Muxing UART and SPI lines.
    PinMuxConfig();

    UART1InterruptInit();

    GPIOPinWrite(GPIOA1_BASE, 0x1,0x1);// set high for OC

    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);// Enable the SPI module clock


    MAP_PRCMPeripheralReset(PRCM_GSPI);// Reset the peripheral
    MAP_SPIReset(GSPI_BASE);

    // Configure SPI interface
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                        SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                        (SPI_HW_CTRL_CS |
                        SPI_4PIN_MODE |
                        SPI_TURBO_OFF |
                        SPI_CS_ACTIVEHIGH |
                        SPI_WL_8));

    MAP_SPIEnable(GSPI_BASE);

    Adafruit_Init();//initialize OLED
    I2C_IF_Open(I2C_MASTER_MODE_FST);//initialize I2C

    g_ulBase = TIMERA0_BASE;
        Timer_IF_Init(PRCM_TIMERA0, g_ulBase, 0x00000022, 0x000000ff, 0);//PERIODIC TIMER
        Timer_IF_IntSetup(g_ulBase, 0x000000ff, TimerBaseIntHandler);
        Timer_IF_Start(g_ulBase, 0x000000ff, 100000);


        // Register the interrupt handlers
        //MAP_GPIOIntRegister(GPIOA1_BASE, GPIOA1IntHandler);
        MAP_GPIOIntRegister(GPIOA2_BASE, GPIOA2IntHandler);


        // Configure rising edge interrupts on remote press and SW3
        //MAP_GPIOIntTypeSet(GPIOA1_BASE, 0x20, GPIO_RISING_EDGE);  // SW3
        MAP_GPIOIntTypeSet(GPIOA2_BASE, 0x2, GPIO_RISING_EDGE); // remote press

        //ulStatus = MAP_GPIOIntStatus (GPIOA1_BASE, false);
        //MAP_GPIOIntClear(GPIOA1_BASE, ulStatus);          // clear interrupts on GPIOA1
        ulStatus = MAP_GPIOIntStatus (GPIOA2_BASE, false);
        MAP_GPIOIntClear(GPIOA2_BASE, ulStatus);            // clear interrupts on GPIOA2   (remote press)

        // clear global variables
        SW2_intcount=0;
        SW3_intcount=0;
        SW2_intflag=0;
        SW3_intflag=0;
        displaycount=0;
        remotepresscount=0;
        remotepressflag=0;
        arrayindex = 0;


        // Enable SW2 and SW3 interrupts
        //MAP_GPIOIntEnable(GPIOA1_BASE, 0x20);
        MAP_GPIOIntEnable(GPIOA2_BASE, 0x2);


//        Message("\t\t****************************************************\n\r");
//        Message("\t\t\tPress any button on the remote\n\r");
//        Message("\t\t\tbuttons outside of the set will not be accomodated\n\r");
//        Message("\t\t ****************************************************\n\r");
//        Message("\n\n\n\r");
        //Report("remote falling edges = %d\r\n",remotepresscount);
       // printf("remote falling edges = %d\r\n",remotepresscount);


        int i = 0;



    fillScreen(0);
    while(1){

        timeouttime = Timer_IF_GetCount(g_ulBase, 0x000000ff);
        timeoutlength = (timeouttime - prevtimeouttime)/10000000;
        //printf("timeoutlength: %d\n",timeoutlength);
        //Report("timeouttime:  %d\n", timeoutlength);
        if (remotepressflag){
            prevtimeouttime = timeouttime;
        };
        if (timeoutlength > 13) {
            gameclock = gameclock + 1;
            prevtimeouttime = timeouttime;
            word[wordindex] = goodchar;
            //printf("word: %s", word);
            wordindex++;
            if (wordindex > 9) {
                wordindex = 9;
            }
            charx = charx + 7;
            timerflag = 1;
        };
//        Report("charx:  %d\n", charx);

        MAP_GPIOIntEnable(GPIOA2_BASE, 0x2);

                   if (remotepressflag) {
                       remotepressflag = 0;

                           if (remotepresscount == 18 ){

                               for (i=0; i<18; i++) { diffarray[i] = timearray[i+1] - timearray[i];}

                               if (diffarray[1] > 355 && diffarray[1] < 375) {indextup = 11;}
                               else if (diffarray[2] > 355 && diffarray[2] < 375) {indextup = 12;}
                               else if (diffarray[3] > 355 && diffarray[3] < 375) {indextup = 13;}
                               else if (diffarray[4] > 355 && diffarray[4] < 375) {indextup = 14;}
                               else if (diffarray[0] > 355 && diffarray[0] < 375) {indextup = 10;}

                               if (diffarray[indextup] < 90///0
                                                         && diffarray[indextup+1] < 90
                                                         && diffarray[indextup+2] < 90
                                                         && diffarray[indextup+3] < 90
                                                         && diffarray[indextup+4] < 90
                                                         )//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                         {// Report("0 ");
                                                         printf("0\n");
                                                         prevnum = currnum;
                                                         currnum = 0;
                                                         prevchar = goodchar;
                                                         getgoodchar(prevnum, currnum, prevchar );
//                                                         Report("%c ",goodchar);
                                                         //drawChar(charx, chary, ' ', 0xFFFF, 0x0000, 1);
                                                         charx = charx + 7;
//                                                         if (timerflag == 1) {
//                                                           charx = charx + 7;
//                                                         };
                                                         }
                               else if (diffarray[indextup] > 90///1
                                                         && diffarray[indextup+1] < 90
                                                         && diffarray[indextup+2] < 90
                                                         && diffarray[indextup+3] < 90
                                                         && diffarray[indextup+4] < 90
                                                         )
                                                         {// Report("1");
                                                         printf("1\n");
                                                         prevnum = currnum;
                                                         currnum = 1;
                                                         prevchar = goodchar;
                                                         getgoodchar(prevnum, currnum, prevchar );
//                                                         Report("%c ", goodchar);
                                                         //drawChar(charx, chary, goodchar, 0xFFFF, 0x0000, 1);
                                                         //Report("goodchar: %cs\n\r", goodchar);


                                                         }
                               else if (diffarray[indextup] < 90////2
                                                         && diffarray[indextup+1] > 90
                                                         && diffarray[indextup+2] < 90
                                                         && diffarray[indextup+3] < 90
                                                         && diffarray[indextup+4] < 90
                                                         )
                                                         {// Report("2 ");
                                                         printf("2\n");
                                                         prevnum = currnum;
                                                         currnum = 2;
                                                         prevchar = goodchar;
                                                         getgoodchar(prevnum, currnum, prevchar);
                                                         //drawChar(charx, chary, goodchar, 0xFFFF, 0x0000, 1);
//                                                         Report("%c ", goodchar);


                                                         }
                               else if (diffarray[indextup] > 90/////3
                                                         && diffarray[indextup+1] > 90
                                                         && diffarray[indextup+2] < 90
                                                         && diffarray[indextup+3] < 90
                                                         && diffarray[indextup+4] < 90
                                                         )
                                                         {// Report("3 ");
                                                         printf("3\n");
                                                         prevnum = currnum;
                                                         currnum = 3;
                                                         prevchar = goodchar;
                                                         getgoodchar(prevnum, currnum, prevchar);
                                                         //drawChar(charx, chary, goodchar, 0xFFFF, 0x0000, 1);
//                                                         Report("%c ", goodchar);

                                                         }
                               else if (diffarray[indextup] < 90/////4
                                                         && diffarray[indextup+1] < 90
                                                         && diffarray[indextup+2] > 90
                                                         && diffarray[indextup+3] < 90
                                                         && diffarray[indextup+4] < 90
                                                         )
                                                         { //Report("4 ");
                                                         printf("4\n");
                                                         prevnum = currnum;
                                                         currnum = 4;
                                                         prevchar = goodchar;
                                                         getgoodchar(prevnum, currnum, prevchar);
                                                         //drawChar(charx, chary, goodchar, 0xFFFF, 0x0000, 1);
//                                                         Report("%c ", goodchar);

                                                         }
                               else if (diffarray[indextup] > 90/////5
                                                         && diffarray[indextup+1] < 90
                                                         && diffarray[indextup+2] > 90
                                                         && diffarray[indextup+3] < 90
                                                         && diffarray[indextup+4] < 90
                                                         )
                                                         { //Report("5 ");
                                                         printf("5\n");
                                                         prevnum = currnum;
                                                         currnum = 5;
                                                         prevchar = goodchar;
                                                         getgoodchar(prevnum, currnum, prevchar);
                                                         //drawChar(charx, chary, goodchar, 0xFFFF, 0x0000, 1);
//                                                         Report("%c ", goodchar);

                                                         }
                               else if (diffarray[indextup] < 90//////6
                                                         && diffarray[indextup+1] > 90
                                                         && diffarray[indextup+2] > 90
                                                         && diffarray[indextup+3] < 90
                                                         && diffarray[indextup+4] < 90
                                                         )
                                                         { //Report("6 ");
                                                         printf("6\n");
                                                         prevnum = currnum;
                                                         currnum = 6;
                                                         prevchar = goodchar;
                                                         getgoodchar(prevnum, currnum, prevchar);
                                                         //drawChar(charx, chary, goodchar, 0xFFFF, 0x0000, 1);
//                                                         Report("%c ", goodchar);


                                                         }
                               else if (diffarray[indextup] > 90//////7
                                                         && diffarray[indextup+1] > 90
                                                         && diffarray[indextup+2] > 90
                                                         && diffarray[indextup+3] < 90
                                                         && diffarray[indextup+4] < 90
                                                         )
                                                         { //Report("7 ");
                                                         printf("7\n");
                                                         prevnum = currnum;
                                                         currnum = 7;
                                                         prevchar = goodchar;
                                                         getgoodchar(prevnum, currnum, prevchar);
                                                         //drawChar(charx, chary, goodchar, 0xFFFF, 0x0000, 1);
//                                                         Report("%c ", goodchar);

                                                         }
                               else if (diffarray[indextup] < 90//////8
                                                         && diffarray[indextup+1] < 90
                                                         && diffarray[indextup+2] < 90
                                                         && diffarray[indextup+3] > 90
                                                         && diffarray[indextup+4] < 90
                                                         )
                                                         { //Report("8 ");
                                                         printf("8\n");
                                                         prevnum = currnum;
                                                         currnum = 8;
                                                         prevchar = goodchar;
                                                         getgoodchar(prevnum, currnum, prevchar);
                                                         //drawChar(charx, chary, goodchar, 0xFFFF, 0x0000, 1);
//                                                         Report("%c ", goodchar);

                                                         }
                               else if (diffarray[indextup] > 90///////9
                                                         && diffarray[indextup+1] < 90
                                                         && diffarray[indextup+2] < 90
                                                         && diffarray[indextup+3] > 90
                                                         && diffarray[indextup+4] < 90
                                                         )
                                                         { //Report("9 ");
                                                         printf("9\n");
                                                         prevnum = currnum;
                                                         currnum = 9;
                                                         prevchar = goodchar;
                                                         getgoodchar(prevnum, currnum, prevchar);
                                                         //drawChar(charx, chary, goodchar, 0xFFFF, 0x0000, 1);
//                                                         Report("%c ", goodchar);

                                                         }
                               else if (diffarray[indextup] < 90///////mute
                                                         && diffarray[indextup+1] > 90
                                                         && diffarray[indextup+2] < 90
                                                         && diffarray[indextup+3] < 90
                                                         && diffarray[indextup+4] > 90
                                                         )
                                                         { //Report("MUTE ");
                                                         //printf("MUTE\n");
                                                         //Outstr("Mute");


                                                         }
                               else if (diffarray[indextup] < 90///////last
                                                         && diffarray[indextup+1] < 90
                                                         && diffarray[indextup+2] < 90
                                                         && diffarray[indextup+3] > 90
                                                         && diffarray[indextup+4] > 90
                                                        )
                                                        {
//                                                        Report("channel down ");
                                                        //drawChar(charx, chary, '\b', 0xFFFF, 0x0000, 1);
                                                        //drawChar(charx, chary, ' ', 0xFFFF, 0x0000, 1);
                                                        charx = charx - 7;
                                                        word[wordindex] = " ";
                                                        wordindex--;
                                                        if (wordindex < 0) {
                                                            wordindex = 0;
                                                        }
                                                        //printf("channel down\n");
                                                        //Outstr("channel down");


                                                        }
                               else if (diffarray[indextup] > 90///////last
                                                        && diffarray[indextup+1] > 90
                                                        && diffarray[indextup+2] > 90
                                                        && diffarray[indextup+3] < 90
                                                        && diffarray[indextup+4] > 90
                                                        )
                                                        {
//                                                           Report("channel up ");
//                                                        drawChar(36, chary, ' ', 0xFFFF, 0x0000, 1);
//                                                        drawChar(43, chary, ' ', 0xFFFF, 0x0000, 1);
//                                                        drawChar(50, chary, ' ', 0xFFFF, 0x0000, 1);
//                                                        drawChar(57, chary, ' ', 0xFFFF, 0x0000, 1);
//                                                        drawChar(64, chary, ' ', 0xFFFF, 0x0000, 1);
//                                                        drawChar(71, chary, ' ', 0xFFFF, 0x0000, 1);
//                                                        drawChar(78, chary, ' ', 0xFFFF, 0x0000, 1);
//                                                        drawChar(85, chary, ' ', 0xFFFF, 0x0000, 1);
//                                                        drawChar(92, chary, ' ', 0xFFFF, 0x0000, 1);
//                                                        drawChar(99, chary, ' ', 0xFFFF, 0x0000, 1);
//                                                        drawChar(106, chary, ' ', 0xFFFF, 0x0000, 1);
//                                                        drawChar(113, chary, ' ', 0xFFFF, 0x0000, 1);
                                                        for (a = 0; a<10; a++){
                                                          Tx[a] = word[a];
                                                        }
                                                        e = 0;
                                                        while (e<10) {
                                                            UARTCharPut(UARTA1_BASE, Tx[e]);

                                                            printf("send %d",Tx[e]);
                                                            e++;
                                                        }
                                                        }


                                if (extract_flag == 1){
                                    printf("extract entered\n");
                                    q = 0;
                                    sendx = 38;
                                    printf("Rx:(");
                                    while (q<10){
                                        sendx = sendx + 7;
                                        //drawChar(sendx, 60, Rx[q], WHITE, BLACK, 0x01);
                                        printf("Rx:(");
                                        printf(" %c, ", Rx[q]);
                                        printf(")\n");
                                        q++;
                                    }
                                    printf(")");
                                    extract_flag = 0;
                                }




//                               printf("indextup: %d\n", indextup);
//                               printf("diff : (");
//                                                 for (i = 0; i<18;i++) {
//                                                      if (diffarray[i] != ""){
//                                                      printf("%d,", diffarray[i]);
//                                                         }
//                                                      }
//                                                 printf(")\n");
//                             printf("word: %s", word);
                             printf("currnum: %d   gameclock: %d\n", currnum, gameclock);
                    }
              }


                   //state machine begin
                            //state 0
                            if (state == 0) {
                                if (statezerocount == 0){



//                                    fillCircle(60,70,20,RED);
//                                    fillRoundRect(46,70,27,27,2,CYAN);
//                                    fillTriangle(55,60,60,50,65,60,0);
                                    //fillRoundRect(0,120,40,10,1,RED);
                                    //fillRoundRect(80,120,40,10,1,RED);
                                    Outstr("BOUNCE", 10, 20, GREEN, 0x0, 3);

                                    drawCircle(20, 75, 10, CYAN);
                                    drawChar(20, 75, '1', YELLOW, 0x0000, 1);
                                    Outstr("PLAY", 40, 70, MAGENTA, 0x0, 2);

                                    drawCircle(20, 105, 10, CYAN);
                                    drawChar(20, 105, '2', YELLOW, 0x0000, 1);
                                    Outstr("Top Score:", 40, 100, MAGENTA, 0x0, 1);

                                    statezerocount = 1;
                                }

                                if (currnum == 1){
                                  state = 1;
                                }
                                else if (currnum == 2) {
                                    fillCircle(20,105,2,RED);
                                                                fillCircle(20,105,4,RED);
                                                                fillCircle(20,105,6,RED);
                                                                fillCircle(20,105,8,RED);
                                                                fillCircle(20,105,10,RED);

                                                                fillCircle(20,105,2,0);
                                                                fillCircle(20,105,4,0);
                                                                fillCircle(20,105,6,0);
                                                                fillCircle(20,105,8,0);
                                                                fillCircle(20,105,10,0);
                                                                currnum = 0;

                                    Outstr("HIGH: ", 10, 50, GREEN, 0x0, 1);
                                    Outstr(str, 50, 50, CYAN, 0x0, 1);

                                }


                            }
                            //state  0 end
                            //state 1 begin
                            else if (state == 1) {
                                gameclock = 0;

                                if (stateonecount == 0){
                                fillCircle(20,75,2,RED);
                                fillCircle(20,75,4,RED);
                                fillCircle(20,75,6,RED);
                                fillCircle(20,75,8,RED);
                                fillCircle(20,75,10,RED);

                                fillCircle(20,75,2,0);
                                fillCircle(20,75,4,0);
                                fillCircle(20,75,6,0);
                                fillCircle(20,75,8,0);
                                fillCircle(20,75,10,0);
                                fillScreen(0);
                                Outstr("BOUNCE!", 10, 10, CYAN, 0x0, 1);
//                                while (e<3) {
//                                                                         UARTCharPut(UARTA1_BASE, str[e]);
//                                                                         printf("send %d",str[e]);
//                                                                         e++;
//                                                                         }
                                while (death == 0){
                                    gameclock++;
                                    //I2C_IF_Write(Acceladdr,&xreg,1,0);//tell x reg to get ready
                                    //I2C_IF_Read(Acceladdr, &xbuff[0], 1);//read data from x reg
                                    I2C_IF_Write(Acceladdr,&yreg,1,0);//tell y reg to get ready
                                    I2C_IF_Read(Acceladdr, &ybuff[0], 1);//read data from y reg

                                    //xdata = xbuff[0];//take 0th index for x accelerometer data
                                    ydata = ybuff[0];//take 0th index for y acceleremoter data

                                    //printf("( ydata ): ( %d )\n", ydata);

                                    //here I take the y accelerometer data and map it to corresponding speed
                                    if (ydata>=0 && ydata<=5){paddlevelocity = 0;}
                                    else if (ydata>=6 && ydata<15) {paddlevelocity = 2;}
                                    else if (ydata>=16 && ydata<25) {paddlevelocity = 4;}
                                    else if (ydata>=26 && ydata<35) {paddlevelocity = 6;}
                                    else if (ydata>=36 && ydata<45) {paddlevelocity = 8;}
                                    else if (ydata>=46 && ydata<55) {paddlevelocity = 10;}
                                    else if (ydata>=56 && ydata<75) {paddlevelocity = 12;}
                                    else if (ydata>250 && ydata<=265) {paddlevelocity = 0;}
                                    else if (ydata>=240 && ydata<250) {paddlevelocity = -2;}
                                    else if (ydata>=230 && ydata<240) {paddlevelocity = -4;}
                                    else if (ydata>=220 && ydata<230) {paddlevelocity = -6;}
                                    else if (ydata>=200 && ydata<220) {paddlevelocity = -8;}
                                    else if (ydata>=170 && ydata<200) {paddlevelocity = -10;}

                                    //paddleleftside limits:  0 and 80
                                    paddleleftside += paddlevelocity;
                                    if (paddleleftside < 5){paddleleftside = 5;}
                                    else if (paddleleftside > 80){paddleleftside = 80;}

                                    fillRoundRect(paddleleftside,120,40,10,1,RED);
                                    fillRoundRect(paddleleftside-10,120,10,10,1,0);
                                    fillRoundRect(paddleleftside + 40, 120, 10,10,1,0);

                                    if (gameclock % 4 == 0){
                                        sprintf(str,"%d", (gameclock/4));
                                        printf("gameclock: %s", str);
                                        Outstr(str, 105, 10, MAGENTA, 0x0, 2);
                                    }
                                    eraseball(prevxpos,prevypos);
                                    paintball(ballx,bally,5);
//                                    ballx = ballx + xspeed;

                                    if (ballx > 110) {
                                        xspeed = -10;
                                    }
                                    else if(ballx < 20) {
                                        xspeed = 10;
                                    }
//
//                                    bally = bally + yspeed;
//                                    yspeed = yspeed + gravity;
                                    if (bally == 112){
                                        if (ballx>=paddleleftside && ballx<(paddleleftside+40)){
                                        yspeed = -20;
                                        }
                                        else {

                                            death = 1;
                                            printf("death\n");
                                        }
                                    }

                                    prevxpos = ballx;
                                    prevypos = bally;
                                    bally = bally + yspeed;
                                    yspeed = yspeed + gravity;
                                    ballx = ballx + xspeed;
                                    printf("(ballx , bally): (%d , %d)\n", ballx, bally);
                                    printf("(xsp , ysp    ): (%d , %d)\n", xspeed, yspeed);


                                }


                                endtime = gameclock;
                                state = 2;
                                stateonecount = 1;
                                }
                            //paintball(50,50,MAGENTA);

                            }
                            //state 1 end
                            else if (state == 2) {
                                eraseball(prevxpos,prevypos);
                                printf("u done died!\n");

                                fillCircle(ballx,120,5,RED);
                                fillCircle(ballx,120,7,YELLOW);
                                fillCircle(ballx,120,10,RED);
                                fillCircle(ballx,120,12,YELLOW);
                                fillCircle(ballx,120,15,RED);
                                fillCircle(ballx,120,17,YELLOW);
                                fillCircle(ballx,120,20,RED);
                                fillCircle(ballx,120,22,YELLOW);
                                fillCircle(ballx,120,25,RED);
                                fillCircle(ballx,120,27,YELLOW);
                                fillCircle(ballx,120,30,RED);
                                fillCircle(ballx,120,30,0);

                                fillScreen(0);

                                Outstr("Game Over!", 8, 8, GREEN, 0x0, 2);
                                Outstr("Game Over!", 10, 10, CYAN, 0x0, 2);
                                Outstr("Game Over!", 12, 12, MAGENTA, 0x0, 2);

                                Outstr("Your", 35, 40, GREEN, 0x0, 2);
                                Outstr("Score:", 30, 55, GREEN, 0x0, 2);
                                Outstr(str, 55, 80, YELLOW, 0x0, 1);
                                Outstr(str, 55, 80, YELLOW, 0x0, 2);
                                Outstr(str, 55, 80, YELLOW, 0x0, 3);
                                Outstr(str, 55, 80, YELLOW, 0x0, 4);
                                Outstr(str, 55, 80, YELLOW, 0x0, 5);
                                printf("before uart send\n");
                                printf("string:  %c%c%c\n",str[0], str[1], str[2]);
                                while (e<1) {
                                          UARTCharPut(UARTA1_BASE, str[e]);
                                          printf("send %c",str[e]);
                                          e++;
                                          }
                                e=0;
                                printf("after uart send\n");
                                state = 0;
                                statezerocount = 0;
                                stateonecount = 0;
                                gameclock = 0;
                                str[0] = ' ';
                                str[1] = ' ';
                                str[2] = ' ';
                                ballx = 40;
                                bally =  52;
                                prevxpos = 0;
                                prevypos = 0;
                                xspeed = 5;
                                yspeed = 0;
                                currnum = 9;
                                death = 0;

                                fillScreen(0);
                         }
                   //state machine end





       }
}

