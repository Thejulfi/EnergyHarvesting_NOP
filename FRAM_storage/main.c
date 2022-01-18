/*Ce programme permet de stocker dans la FRAM la valeur data. Cette valeur s'incrémente à chaque écriture.
 La nouvelle data est ensuite écrite à l'emplacement suivant dans la FRAM. Ainsi la valeur data à un écart de 1 entre chaque emplacement mémoire.
 Dans le cas ou on dépasse la valeur 0xFFFF, la valeur data est remise à 0. On alloue une mémoire de 1000h dans la FRAM*/

#include <msp430.h>

//Taille de la mémoire que l'on souhaite allouer (1000h)
#define WRITE_SIZE 1000

void FRAMWrite(void);

unsigned char count = 0;
unsigned short data;
unsigned int i=0;

//Commande spécifique pour activer l'écriture dans la FRAM
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

int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;                 // Stop WDT

  // Configure GPIO
  P1OUT &= ~BIT0;                           // Clear P1.0 output latch for a defined power-on state
  P1DIR |= BIT0;                            // Set P1.0 to output direction

  // Disable the GPIO power-on default high-impedance mode to activate
  // previously configured port settings
  PM5CTL0 &= ~LOCKLPM5;

  // Initialize dummy data
  data = 0x0000;

  while(1)
  {

    //Ecriture dans la FRAM
    FRAMWrite();

    //Incrémentation d'un compteur (utilisé surtout pour faire des tests)
    count++;

    //Gestion du dépassement
    if(data >=0xFFFF){
        data = 0x0000;}

    //Si count = ... on allume la LED et on la remet à 0 avce une incrémentation de la valeur data
    if (count == 1)
    {
      P1OUT ^= 0x01;                        // Toggle LED to show 512K bytes
      count = 0;                            // ..have been written
      data += 0x0001;

    }
  }
}

void FRAMWrite(void)
{
    //Ecriture à l'adresse i de la valeur data
    FRAM_write[i] = data;
    //Incrémentation de i pour ecrire la donnée suivante à un autre emplacement
    i = i+1;
}
