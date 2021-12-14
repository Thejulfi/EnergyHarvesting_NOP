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

void init_UART(void){
    WDTCTL = WDTPW | WDTHOLD;                 // Stop Watchdog

    //******************************************************************************
    // configure P2.0 and P2.1 FOR UART
    //******************************************************************************
    P2SEL1 |= BIT0 | BIT1;
    P2SEL0 &= ~(BIT0 | BIT1);
    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    //******************************************************************************
    //  Startup clock system with ~1MHz
    //******************************************************************************
    CSCTL0_H = CSKEY >> 8;                    // Unlock clock registers
    CSCTL1 = DCOFSEL_0 | DCORSEL;             // Set DCO to 1MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers
    CSCTL0_H = 0;                             // Lock CS registers

    //******************************************************************************
    //  Setup UCA0 in UART mode
    //******************************************************************************
    UCA0CTLW0 &= UCSWRST;                     //Take eUSC0 out into SW reset with UCA0CTLW0 = 1
    UCA0CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK
    // Baud Rate calculation
    // 10000000/(115200) = 8.68
    // Fractional portion = 0.68
    // User's Guide Table 21-4: UCBRSx = 0xD600
    UCA0BR0 = 8;                              // Prescaler : 8000000/115200
    UCA0MCTLW |= 0xD600;                      // Modulation
    UCA0CTLW0 &= ~UCSWRST;                    //Take eUSC0 out of SW reset with UCA0CTLW0 = 0
}

int main(void)
{

  init_UART();
  //******************************************************************************
  // Main
  //******************************************************************************

  char message[] = "hello world ";          //Message buffer
  int position;                             //message index


  while(1){
      for(position = 0; position<sizeof(message);position++){
          UCA0TXBUF = message[position];
          __delay_cycles(100);
      }
      __delay_cycles(3000000);


  }

  __no_operation(); // For debugger
}
