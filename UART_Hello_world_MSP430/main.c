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

//******************************************************************************
// configure P2.0 and P2.1 FOR UART
//******************************************************************************
void initGPIO()
{
    P2SEL1 |= BIT0 | BIT1;
    P2SEL0 &= ~(BIT0 | BIT1);
    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}

//******************************************************************************
//  Startup clock system with ~1MHz
//******************************************************************************
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

//******************************************************************************
//  Setup UCA0 in UART mode
//******************************************************************************
void init_UART(void){
    WDTCTL = WDTPW | WDTHOLD;                 // Stop Watchdog
    UCA0CTLW0 &= UCSWRST;                     //Take eUSC0 out into SW reset with UCA0CTLW0 = 1
    UCA0CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK
    // Baud Rate calculation
    // 10000000/(115200) = 8.68
    // Fractional portion = 0.68
    // MSP430FRxx Family guide Table 30-4: UCBRSx = 0xD600
    UCA0BR0 = 8;                              // Prescaler : 8000000/115200
    UCA0MCTLW |= 0xD600;                      // Modulation
    UCA0CTLW0 &= ~UCSWRST;                    //Take eUSC0 out of SW reset with UCA0CTLW0 = 0
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
  //Various initialisations
  initClockTo1MHz();
  initGPIO();
  init_UART();

  //******************************************************************************
  // Main
  //******************************************************************************
  //char message[] = ;          //Message buffer

  while(1){
      char test[] = "12";

      UARTprint(test);
      __delay_cycles(3000000);


  }
}

