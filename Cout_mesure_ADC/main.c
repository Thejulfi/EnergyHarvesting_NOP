#include <msp430.h> 

#define WRITE_SIZE 1000 //size allocated to the FRAM memory

int ADC_value;
float Current;
float Voltage;
unsigned int k = 0;

void initADC (); //Initialize the ADC
void read_adc ();   //Read the value on the ADC and convert into current



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

       __bic_SR_register_on_exit(LPM0_bits | GIE); // Exit CPU, clear interrupts
       break;
       default: break;
       }
      }


#pragma vector = PORT1_VECTOR
  __interrupt void ISR_PORT1_S1(void)
  {
      P1IFG &= ~BIT1;

      ADC12CTL0 |= ADC12ENC | ADC12SC; // Start sampling/conversion
      //__bis_SR_register(LPM0_bits | GIE); // LPM0, ADC12_ISR will force exit
  }

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    initADC ();

    P1DIR &= ~BIT1;
    P1REN |= BIT1;
    P1OUT |= BIT1;
    P1IES |= BIT1;

    PM5CTL0 &= ~LOCKLPM5;

    P1IFG &= ~BIT1;
    P1IE |= BIT1;
    __enable_interrupt();

    while (1)
    {
        //ADC12CTL0 |= ADC12ENC | ADC12SC; // Start sampling/conversion
        __bis_SR_register(LPM3_bits | GIE);         // Enter LPM0, enable interrupts
        __no_operation();                           // For debugger
    }
}

void read_adc (){
    ADC_value = ADC12MEM0; // Save MEM0

       //Overflow of Voltage when the real voltage is higher than 0.5V
       Voltage = ((ADC_value)*1000)/1340; //Voltage * 1000 (result in mV)
       Current = (Voltage/25)*1000; //Current *0 100 (in µA)


        if(k<=1000){
       //Write the value of the current to the adress k
       FRAM_write[k] = Current;
       //k is incremented to go to the next adress of the FRAM
       k = k+1;

       //Write the value of the voltage to the adress k
       FRAM_write[k] = Voltage;
       //k is incremented to go to the next adress of the FRAM
       k = k+3;
       }
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


