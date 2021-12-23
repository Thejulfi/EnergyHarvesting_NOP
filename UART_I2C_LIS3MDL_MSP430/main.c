//******************************************************************************
//  UART_hello_world_MSP430 = 1MHz
//
//
//                MSP430FR5969
//             -----------------
//       RST -|     P2.0/UCA0TXD|----> PC (echo)
//            |                 |
//            |                 |
//            |     P2.1/UCA0RXD|
//            |                 |
//
//******************************************************************************

#include <msp430.h>

// The Slave ADdress (SAD) associated to the LIS3MDL is 00111x0b, whereas the x bit is modified by the SDO/SA1 pin in order to modify the device address.
#define LIS3MDL_SA1_LOW_ADDRESS  0x28
#define LIS3MDL_SA1_HIGH_ADDRESS  0x30

unsigned int Data_Cnt = 0;
char Packet[] = {0x20, 0x70, 0x00, 0x00, 0x0C}; // Default configuaration for LIS3MDL

enum regAddr
{
  WHO_AM_I    = 0x0F,

  CTRL_REG1   = 0x20,
  CTRL_REG2   = 0x21,
  CTRL_REG3   = 0x22,
  CTRL_REG4   = 0x23,
  CTRL_REG5   = 0x24,

  STATUS_REG  = 0x27,
  OUT_X_L     = 0x28,
  OUT_X_H     = 0x29,
  OUT_Y_L     = 0x2A,
  OUT_Y_H     = 0x2B,
  OUT_Z_L     = 0x2C,
  OUT_Z_H     = 0x2D,
  TEMP_OUT_L  = 0x2E,
  TEMP_OUT_H  = 0x2F,
  INT_CFG     = 0x30,
  INT_SRC     = 0x31,
  INT_THS_L   = 0x32,
  INT_THS_H   = 0x33,
};

int state;


//******************************************************************************
// configure P2.0 and P2.1 FOR UART
//******************************************************************************
void initGPIO()
{
    //******************************************************************************
    // configure SDA P2.0 and SCL P2.1 FOR UART
    //******************************************************************************

    P2SEL1 |= BIT0 | BIT1;
    P2SEL0 &= ~(BIT0 | BIT1);


    //******************************************************************************
    // configure SDA P3.6 and SCL P3.5 FOR I2C
    //******************************************************************************
    P1SEL1 &= ~BIT6;                           // We want P3.5 SCL
    P1SEL0 |= BIT6;

    P1SEL1 &= ~BIT7;                           // We wan p3.6 SDA
    P1SEL0 |= BIT7;


    //******************************************************************************
    // Switch LED1 interruption GPIO
    //******************************************************************************
    P4DIR |= BIT6;                              //  setting up LED1 P4.6
    P4OUT &= ~BIT6;                             // Clearing value for LED1

    P1DIR &= ~BIT1;                             // Seting up SW1, P1.1
    P1REN |= BIT1;                              // Enable pullup resistor for SW1
    P1OUT |= BIT1;                              // Set as output value for our system
    P1IES |= BIT1;                              // Enable interruption on rising or falling edge
    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
    __delay_cycles(100);

}

//******************************************************************************
//  Startup clock system with ~1MHz
//******************************************************************************
void initClockTo16MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
    CSCTL1 = DCORSEL | DCOFSEL_4;             // Set DCO to 16MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers

    CSCTL0_H = 0;                             // Lock CS registerss
}
void initClockTo1MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    CSCTL0_H = CSKEY >> 8;                    // Unlock clock registers
    CSCTL1 = DCOFSEL_0 | DCORSEL;             // Set DCO to 1MHz
        //DCOFSEL_0 = 1Mhz
        //DCOFSEL_1 = 5.33Mhz
        //DCOFSEL_2 = 7Mhz
        //DCOFSEL_3 = 8Mhz
        //DCOFSEL_4 = 16Mhz
        //DCOFSEL_5 = 21Mhz
        //DCOFSEL_6 = 24Mhz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers
    CSCTL0_H = 0;                             // Lock CS registers                       // Lock CS registerss
}

void I2Cwrite(){
    //******************************************************************************
    //  Begin Transmission
    //******************************************************************************
    UCB0IE |= UCTXIE0;



    __enable_interrupt();


    while(1){
        UCB0CTLW0 |= UCTXSTT;
        __delay_cycle(100);
    }

}
//******************************************************************************
//  Initialisation of an I2C communication
//******************************************************************************
void initI2C()
{
    UCB0CTLW0 = UCSWRST;                      // Enable SW reset
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC; // I2C master mode, SMCLK
    UCB0BRW = 160;                            // fSCL = SMCLK/160 = ~100kHz
    UCB0I2CSA = LIS3MDL_SA1_LOW_ADDRESS;                   // Slave Address
    UCB0CTLW1 |= UCASTP_2;                     // Auto STOP when UCB0TBCNT reached
    UCB0TBCNT = sizeof(Packet);

    UCB0CTLW0 &= ~UCSWRST;                    // Clear SW reset, resume operation
    UCB0IE |= UCTXIE;
    __enable_interrupt();
}


//******************************************************************************
//  Setup UCA0 in UART mode
//******************************************************************************
void init_UART(void){
    UCA0CTLW0 &= UCSWRST;                     //Take eUSC0 out into SW reset with UCA0CTLW0 = 1
    UCA0CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK
    // Baud Rate calculation
    // 10000000/(115200) = 8.68
    // Fractional portion = 0.68
    // MSP430FRxx Family guide Table 30-4: UCBRSx = 0xD600
    UCA0BR0 = 8;                              // Prescaler : 8000000/115200
    UCA0MCTLW |= 0xD600;                      // Modulation
    UCA0CTLW0 &= ~UCSWRST;                    //Take eUSC0 out of SW reset with UCA0CTLW0 = 0

    __delay_cycles(100);
}

void UARTprint(char mes[]){
    int position;

    for(position = 0; position<sizeof(mes);position++){
        UCA0TXBUF = mes[position];
        __delay_cycles(100);
    }
}

int main(void)
{
   WDTCTL = WDTPW | WDTHOLD;                 // Stop Watchdog
  initClockTo16MHz();
  //initClockTo1MHz();
  initGPIO();
  //init_UART();
  initI2C();

  //******************************************************************************
  // Blink under interruption with SW1 and LED1
  //******************************************************************************

  /*P1IE |= BIT1;               // Enable interruption for P1.1 SW1
  __enable_interrupt();
  P1IFG &= ~BIT1;               // Clear interruption flag

  while(1){

  }*/

  //Magnetometer default configuration
  //LISenableDefault();
  //******************************************************************************
  // Main for UART communication
  //******************************************************************************
  /*while(1){
      char test[] = "12";

      UARTprint(test);
      __delay_cycles(3000000);


  }*/

  //******************************************************************************
  // Sending a start, waiting for interruption then.
  //******************************************************************************

  while(1){
      UCB0CTLW0 |= UCTXSTT;
      __delay_cycles(100);
  }

  __no_operation(); // For debugger
}

//--------------------------------------------------------------------------------------------
//-----Interruption routine for PORT1_VECTOR when SW1 is pressed
#pragma vector = PORT1_VECTOR
__interrupt void ISR_Port1_S1(void){
    P4OUT ^= BIT6;
    P1IFG &= ~BIT1;
}

//--------------------------------------------------------------------------------------------
//-----Interruption routine when buffer for Tx communication in I2C
#pragma vector = USCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void){

    if(Data_Cnt == (sizeof(Packet)-1)){
        UCB0TXBUF = Packet[Data_Cnt];
        Data_Cnt = 0;
    }else{
        UCB0TXBUF = Packet[Data_Cnt];
        Data_Cnt++;
    }

}
