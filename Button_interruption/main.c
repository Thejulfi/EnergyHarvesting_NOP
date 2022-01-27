#include <msp430.h> 

void initButton(){
    // Button P1.1 init
    P1DIR &= ~BIT1;
    P1REN |= BIT1;
    P1OUT |= BIT1;
    P1IES |= BIT1;


    P1IFG &= ~BIT1;
    P1IE |= BIT1;
}

// LED initialization
void init_led(void){
    P1OUT &= ~BIT0;                           // Clear P1.0 output latch for a defined power-on state
    P1DIR |= BIT0;                            // Set P1.0 to output direction
}

// toggling LED1.0 on/off
void toggle_led(void){
    P1OUT ^= BIT0; // toggle led

}



/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	PM5CTL0 &= ~LOCKLPM5;
	
	initButton(); // button init
	init_led(); // LED init

    // Endless loop waiting for interruption
    while(1){
        __bis_SR_register(LPM0_bits | GIE);         // Enter LPM0, enable interrupts
        __no_operation();                           // For debugger
    }


	return 0;
}


//******************************************************************************
// Enable/disable measurements *************************************************
//******************************************************************************
#pragma vector = PORT1_VECTOR
__interrupt void ISR_PORT1_S1(void)
{
  P1IFG &= ~BIT1; // disable interruption flag on button 1.1

  toggle_led();

}
