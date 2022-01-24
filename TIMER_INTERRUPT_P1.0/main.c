#include <msp430.h> 

void toggle_led(void){
    P1OUT ^= BIT0;                              // toggle led0
    TB0CTL &= ~TBIFG;                           // disable interrupt
}

void init_timer_interrupt(void){
    TB0CTL |= TBCLR;                            // Clock divider logic clear, always read as zero.
    TB0CTL |= TBSSEL__ACLK;                    // Timer clock source select : SMCLK.
    TB0CTL |= MC__CONTINOUS;                    // Mode control to continous mode : Timer counts up to the value set by CNTL.
    TB0CTL |= ID__1;                            // No clock divider.


    TB0CTL |= TBIE;                             // Enable interruption
    TB0CTL &= ~TBIFG;                           // Switch to interrupt pending - interrupt flag
}

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	                // stop watchdog timer
	

	// Setting up LED1
	P1DIR |= BIT0;
	P1OUT &= BIT0;


	PM5CTL0 &= ~LOCKLPM5;                       // Stop watchdog timer

	init_timer_interrupt();                     // Initialization of timer interrupt

	__bis_SR_register(LPM3_bits | GIE);         // Enter LPM3, enable interrupts
	__no_operation();                           // For debugger

	return 0;
}


//******************************************************************************
// Timer interrupt *************************************************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_B1_VECTOR))) TIMER0_B1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(TB0IV,TB0IV_TBIFG))
  {
    case TB0IV_NONE:   break;               // No interrupt
    case TB0IV_TBCCR1: break;               // CCR1 not used
    case TB0IV_TBCCR2: break;               // CCR2 not used
    case TB0IV_TBCCR3: break;               // CCR3 not used
    case TB0IV_TBCCR4: break;               // CCR4 not used
    case TB0IV_TBCCR5: break;               // CCR5 not used
    case TB0IV_TBCCR6: break;               // CCR6 not used
    case TB0IV_TBIFG:                       // overflow
        toggle_led();
      break;
    default: break;
  }
}
