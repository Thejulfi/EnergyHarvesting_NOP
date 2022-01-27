#include <msp430.h> 

#define WRITE_SIZE 1000 //size allocated to the FRAM memory

int ADC_value;
float Current;
float Voltage;
unsigned int k = 0; //Variable for the position in the FRAM
int i = 0; //Count the number of measure for ADC

void initADC (); //Initialize the ADC
void read_adc ();   //Read the value on the ADC and convert into current
void storeFRAM (); //Store current and voltage into FRAM
void init_timer_interrupt();
void initClockTo16MHz();
void initBoard ();


//Command for write into the FRAM with a array of size WRITE_SIZE (1000)
#if defined(__TI_COMPILER_VERSION__)
#pragma PERSISTENT(FRAM_write)
unsigned long FRAM_write[WRITE_SIZE] = {0};
#elif defined(__IAR_SYSTEMS_ICC__)
__persistent unsigned long FRAM_write[WRITE_SIZE] = {0};
#elif defined(__GNUC__)
unsigned long __attribute__((persistent)) FRAM_write[WRITE_SIZE] = {0};
#else
#error Compiler not supported!
#endif

  // ADC Vector function adapted from http://www.ti.com/lit/zip/slac536
  // using MSP430FR59xx_adc12_01.c
  // Written by: T. Witt / P. Thanigai, Texas Instruments Inc., November 2013

  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector = ADC12_VECTOR
  __interrupt void ADC12_ISR(void)
  #elif defined(__GNUC__)
  void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
  #else
  #error Compiler not supported!
  #endif
  {
       switch (__even_in_range(ADC12IV, ADC12IV_ADC12RDYIFG))
       {
       case ADC12IV_ADC12IFG0: // ADC12MEM0 Interrupt

           read_adc (); //Read value on ADC

           //storeFRAM(); //Store the data into the FRAM

       __bic_SR_register_on_exit(LPM0_bits | GIE); // Exit CPU, clear interrupts
       break;
       default: break;
       }
       //Realise 100 measure in the ADC
       if (i <100){
       i = i+1;
       ADC12CTL0 |= ADC12ENC | ADC12SC; //Trigger the interrupt for ADC
       }
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
          i = 0; //Reset the number of measure by the ADC
          ADC12CTL0 |= ADC12ENC | ADC12SC; //Trigger the interrupt for ADC
        break;
      default: break;
    }
  }

/*************************************MAIN*******************************************/
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    initBoard (); //Init I/O of the system
    initADC (); //Init the ADC

    initClockTo16MHz();
    init_timer_interrupt();                     // Initialization of timer interrupt

    ADC12CTL0 |= ADC12ENC | ADC12SC; //Trigger the first interrupt for ADC
    while (1)
    {
        __bis_SR_register(LPM3_bits | GIE);         // Enter LPM3, enable interrupts
        __no_operation();                           // For debugger
    }
}


/**************FONCTIONS***************************/
void read_adc (){
    ADC_value = ADC12MEM0; // Save MEM0

       //Overflow of Voltage when the real voltage is higher than 0.5V
       Voltage = ((ADC_value)*100)/1340; //Voltage * 1000 (result in mV)
       Current = (Voltage/25)*1000; //Current *0 100 (in µA)


}

void storeFRAM(){

       if(k<=1000){
      //Write the value of the current to the adress k
      FRAM_write[k] = Current;
      //k is incremented to go to the next adress of the FRAM
      k = k+1;

      //Write the value of the voltage to the adress k
      FRAM_write[k] = Voltage;
      //k is incremented to go to the next adress of the FRAM
      k = k+1;
      }
}

void initBoard ( void )
{
    // Port Configuration
    // stored in FRAM
    P1OUT = 0x00 ;
    P1DIR = 0xFF ;
    P2OUT = 0x00 ;
    P2DIR = 0xFF ;
    P3OUT = 0x00 ;
    P3DIR = 0xFF ;
    P4OUT = 0x00 ;
    P4DIR = 0xFF ;
    PJOUT = 0x00 ;
    PJSEL0 |= BIT4 | BIT5 ;
    PJDIR = 0xFFFF ;

    PM5CTL0 &= ~LOCKLPM5;
}

void initADC ()
{
    P1SEL0 |= BIT2 ; // configure P1 .2/ A2 for ADC function
    P1SEL1 |= BIT2 ;


         // Configure ADC12

    // Configure ADC12
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON;                               //  16 cycles
    ADC12CTL1 = ADC12SSEL_3 | ADC12SHP;                              //  ADC12CLK -> SMCLK
    ADC12CTL2 = ADC12RES__12BIT;                                     //  12 bit resolution
    ADC12MCTL0 = ADC12INCH_2 | REFVSEL_0;                             // Channel2 ADC input select; Vref=1.2V
    ADC12IER0 = ADC12IE0;                                            // Enable ADC conv complete interrupt

}


void initClockTo16MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    CSCTL0_H = CSKEY >> 8;                      // Unlock CS registers
    CSCTL1 = DCORSEL | DCOFSEL_4;               // Set DCO to 16MHz
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK; // Sources selection LFXTCLK for ACLK, DCOCLK for SMCLK and DCOCLK for MCLK.
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;       // Set all dividers

    CSCTL0_H = 0;                               // Lock CS registerss
}

void init_timer_interrupt(void){
    TB0CTL |= TBCLR;                            // Clock divider logic clear, always read as zero.
    TB0CTL |= TBSSEL__ACLK;                     // Timer clock source select : SMCLK.
    TB0CTL |= MC__CONTINOUS;                    // Mode control to continous mode : Timer counts up to the value set by CNTL.
    TB0CTL |= ID__8;                            // No clock divider.
    TB0CTL |= CNTL_1;                           // Counter size (12 bits)


    TB0CTL |= TBIE;                             // Enable interruption
    TB0CTL &= ~TBIFG;                           // Switch to interrupt pending - interrupt flag
}
