//******************************************************************************
//   MSP430FR59xx Demo - eUSCI_B0, I2C Master multiple byte TX/RX
//
//   Description: I2C master communicates to I2C slave sending and receiving
//   3 different messages of different length. I2C master will enter LPM0 mode
//   while waiting for the messages to be sent/receiving using I2C interrupt.
//   ACLK = NA, MCLK = SMCLK = DCO 16MHz.
//
//                                     /|\ /|\
//                   MSP430FR5969      4.7k |
//                 -----------------    |  4.7k
//            /|\ |             P1.7|---+---|-- I2C Clock (UCB0SCL)
//             |  |                 |       |
//             ---|RST          P1.6|-------+-- I2C Data (UCB0SDA)
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//
//   Nima Eskandari
//   Texas Instruments Inc.
//   April 2017
//   Built with CCS V7.0
//******************************************************************************
//******************************************************************************
//  LIS3MDL magnetometers I2C driver and FRAM storage.
//
//  Description : I2C master send 5 configurations bytes to two LIS3MDL slave then
//  read Z axis of these.
//
//                   MSP430FR5969
//                 -----------------
//                |             P1.7|---------- I2C Clock (UCB0SCL)
//                |                 |  |
//                |             P1.6|---------- I2C Data (UCB0SDA)
//                |                 |  |    |
//                |                 |  |    |
//                |                 |  |    |
//                |                 |  |    |
//                |                 |  |    |
//         Gnd    |                 |  |    |
//         /|\          LIS3MDL        |    |
//          |      -----------------   |    |
//          ------| P0           P4 |-------- SDA
//       Vcc -----| Blue            |  |
//       Gnd -----| Black        P5 |--- SCL
//                |                 |
//                |                 |
//
//
//  Julien Fichet, Tristan Picot
//  Polytech Nantes
//  January 2022
//******************************************************************************

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>


// The Slave ADdress (SAD) associated to the LIS3MDL is 00111x0b, whereas the x bit is modified by the SDO/SA1 pin in order to modify the device address.
#define SLAVE_ADDR_1  0x1C // Slave address where x is at 0
#define SLAVE_ADDR_2  0x1E // Slave address where x is at 1


/* ST_REG_X_MASTER: numerical address of the first register to write in the slave.
 *
 * REG_RD_X_SLAVE: numerical address of the register to read
 *
 * TYPE_X_LENGTH: packets' length (in byte) to read or to write
 * */

#define REG_RD_1_SLAVE      44 // OUT_X_L register address to read (0x28)
#define REG_RD_2_SLAVE      45 // OUT_X_LH register address to read (0x29)


#define ST_REG_1_MASTER      160 // CTRL_REG1 register to write first (0x20) with msb at "1" to enable address incrementation (for next CTRL_REGs)

#define DefaultConfiguration_LENGTH   5 // Write into CTRL_REG1..CTRL_REG5
#define Slave_RD_LENGTH   1 // reading one byte in OUT_X_L & OUT_X_H

#define MAX_BUFFER_SIZE     20 // Maximum buffer size in reception and transmission


#define WRITE_SIZE 1000 // FRAM size of 1000


/* DefaultConfiguration: configuration for CTRL_REG1..CTRL_REG5 (these are numerical values, not hexadecimal values)
 * CTRL_REG1 = 0x00 - temperature sensor disable, X and Y operative mode selected, Fast_ODR disabled, self-test disabled.
 * CTRL_REG2 = 0x00 - Full scale configuration set to 00 for +- 4gauss, reboot memory  content to normal mode, configuration registers and user register reset function to default value
 * CTRL_REG3 = 0x01 - data rate set in CTRL_REG1, 0:4-wire interface, Operating mode set to single-conversion mode
 * CTRL_REG4 = 0x0C - Z operation set to ultra-high performance mode, data LSb at lower address
 * CTRL_REG5 = 0x40 - FAST_READ enable, block data update for magnetic data set to default
 *
 * Slave_out_z_x are variables which will contain the values read in the registers OUT_X_L and OUT_X_LH
*/

uint8_t DefaultConfiguration [DefaultConfiguration_LENGTH] = {0,0,1,12,64}; // Default configuration to start receiving datat properly from LIS3MDL

uint8_t Slave_out_z_l;
uint8_t Slave_out_z_h;



//******************************************************************************
// FRAM variables***************************************************************
//******************************************************************************

unsigned char count = 0;
unsigned short data;
unsigned int i=0;


//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************

typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;


/* Used to track the state of the software state machine*/
I2C_Mode MasterMode = IDLE_MODE;


/* The Register Address/Command to use*/
uint8_t TransmitRegAddr = 0;

/* ReceiveBuffer: Buffer used to receive data in the ISR
 * RXByteCtr: Number of bytes left to receive
 * ReceiveIndex: The index of the next byte to be received in ReceiveBuffer
 * TransmitBuffer: Buffer used to transmit data in the ISR
 * TXByteCtr: Number of bytes left to transfer
 * TransmitIndex: The index of the next byte to be transmitted in TransmitBuffer
 * Combine_h_l: The combination of high and low bytes.
 * */
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;

int16_t value_mag2 = 0;
int16_t value_mag1 = 0;

float mes;


//******************************************************************************
// FRAM storage function's prototype *******************************************
//******************************************************************************

void FRAMWrite(int16_t data);



//******************************************************************************
// I2C Write and Read Functions ************************************************
//******************************************************************************

/* For slave device with dev_addr, writes the data specified in *reg_data
 *
 * dev_addr: The slave device address.
 *           Example: SLAVE_ADDR
 * reg_addr: The register or command to send to the slave.
 *           Example: CMD_TYPE_0_MASTER
 * *reg_data: The buffer to write
 *           Example: MasterType0
 * count: The length of *reg_data
 *           Example: TYPE_0_LENGTH
 *  */
I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

/* For slave device with dev_addr, read the data specified in slaves reg_addr.
 * The received data is available in ReceiveBuffer
 *
 * dev_addr: The slave device address.
 *           Example: SLAVE_ADDR
 * reg_addr: The register or command to send to the slave.
 *           Example: CMD_TYPE_0_SLAVE
 * count: The length of data to read
 *           Example: TYPE_0_LENGTH
 *  */
I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count);
//void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);

//******************************************************************************
// I2C functions****************************************************************
//******************************************************************************


I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB0I2CSA = dev_addr;
    UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB0IE |= UCTXIE;                        // Enable TX interrupt

    UCB0CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts



    return MasterMode;

}


I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;

    //Copy register data to TransmitBuffer
    //CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB0I2CSA = dev_addr;
    UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB0IE |= UCTXIE;                        // Enable TX interrupt

    UCB0CTLW0 |= UCTR;             // I2C TX, start condition
    UCB0CTLW0 |= UCTXSTT;

    __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;
}

void config_mag(uint32_t addr)
{
    I2C_Master_WriteReg(addr, ST_REG_1_MASTER, DefaultConfiguration, DefaultConfiguration_LENGTH);
}


int16_t read_mag(uint32_t addr)
{
    I2C_Master_ReadReg(addr, REG_RD_1_SLAVE, Slave_RD_LENGTH);
    Slave_out_z_l = ReceiveBuffer[0];

    I2C_Master_ReadReg(addr, REG_RD_2_SLAVE, Slave_RD_LENGTH);
    Slave_out_z_h = ReceiveBuffer[0];

    return (int16_t)(Slave_out_z_h << 8 | Slave_out_z_l);

}

//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************

void initGPIO_FRAM()
{
    // Configure GPIO
    P1OUT &= ~BIT0;                           // Clear P1.0 output latch for a defined power-on state
    P1DIR |= BIT0;                            // Set P1.0 to output direction
}

void initGPIO_I2C()
{
    // Configure GPIO
    P1OUT &= ~BIT0;                           // Clear P1.0 output latch
    P1DIR |= BIT0;                            // For LED
    P1SEL1 |= BIT6 | BIT7;                    // I2C pins
    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
}

/*void initButton(){

    P1DIR &= ~BIT1;
    P1REN |= BIT1;
    P1OUT |= BIT1;
    P1IES |= BIT1;


    P1IFG &= ~BIT1;
    P1IE |= BIT1;
}*/

void initClockTo16MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
    CSCTL1 = DCORSEL | DCOFSEL_4;             // Set DCO to 16MHz
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK; // Sources selection LFXTCLK for ACLK, DCOCLK for SMCLK and DCOCLK for MCLK.
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers

    CSCTL0_H = 0;                             // Lock CS registerss
}

void initI2C(uint8_t dev_addr)
{
    UCB0CTLW0 = UCSWRST;                      // Enable SW reset
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC; // I2C master mode, SMCLK
    UCB0BRW = 160;                            // fSCL = SMCLK/160 = ~100kHz
    UCB0I2CSA = dev_addr;                   // Slave Address
    UCB0CTLW0 &= ~UCSWRST;                    // Clear SW reset, resume operation
    UCB0IE |= UCNACKIE;
}


void init_timer_interrupt(void){
    TB0CTL |= TBCLR;
    TB0CTL |= TBSSEL__ACLK;
    TB0CTL |= MC__CONTINOUS;
    TB0CTL |= ID__8;
    TB0CTL |= CNTL_1;


    TB0CTL |= TBIE;
    TB0CTL &= ~TBIFG;
}

//******************************************************************************
// Various other functions *****************************************************
//******************************************************************************


// toggleing LED1.0 on/off
void toggle_led(void){
    P1OUT ^= BIT0; // toggle led
    TB0CTL &= ~TBIFG; //disable interrupt
}


// LIS3MFDL reading, calculation to obtain mA and FRAM storage.
void measurement(void){
    value_mag2 = read_mag(SLAVE_ADDR_2);
    value_mag1 = read_mag(SLAVE_ADDR_1);



    mes = fabs(((0+value_mag1)-value_mag2)*0.5*0.036539);

    FRAMWrite(mes*1000);

    count ++;

    toggle_led(); // Toggle LED1.0 to show that a data has been written in the FRAM
}


//******************************************************************************
// Main ************************************************************************
// Send and receive three messages containing the example commands *************
//******************************************************************************

int main(void) {

    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

    // Init intern SMLCK clock to 16MHz
    initClockTo16MHz();

    // GPIO for I2C and FRAM (LED to indicate write)
    initGPIO_I2C();
    initGPIO_FRAM();

    // I2C intialization
    initI2C(SLAVE_ADDR_2);
    initI2C(SLAVE_ADDR_1);

    // LIS3MDL magnetometers configuration
    config_mag(SLAVE_ADDR_2);
    config_mag(SLAVE_ADDR_1);

    // Enabling interruption every second
    init_timer_interrupt();


//    initButton();

    // Endless loop waiting for interruption
    while(1){
        __bis_SR_register(LPM0_bits | GIE);         // Enter LPM0, enable interrupts
        __no_operation();                           // For debugger
    }


    return 0;
}


//******************************************************************************
// I2C Interrupt ***************************************************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  //Must read from UCB0RXBUF
  uint8_t rx_val = 0;
  switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
  {
    case USCI_NONE:          break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:   break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
      break;
    case USCI_I2C_UCSTTIFG:  break;         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:  break;         // Vector 8: STPIFG
    case USCI_I2C_UCRXIFG3:  break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:  break;         // Vector 12: TXIFG3
    case USCI_I2C_UCRXIFG2:  break;         // Vector 14: RXIFG2
    case USCI_I2C_UCTXIFG2:  break;         // Vector 16: TXIFG2
    case USCI_I2C_UCRXIFG1:  break;         // Vector 18: RXIFG1
    case USCI_I2C_UCTXIFG1:  break;         // Vector 20: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
        rx_val = UCB0RXBUF;
        if (RXByteCtr)
        {
          ReceiveBuffer[ReceiveIndex++] = rx_val;
          RXByteCtr--;
        }

        if (RXByteCtr == 1)
        {
          UCB0CTLW0 |= UCTXSTP;
        }
        else if (RXByteCtr == 0)
        {
          UCB0IE &= ~UCRXIE;
          MasterMode = IDLE_MODE;
          __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }
        break;
    case USCI_I2C_UCTXIFG0:                 // Vector 24: TXIFG0
        switch (MasterMode)
        {
          case TX_REG_ADDRESS_MODE:
              UCB0TXBUF = TransmitRegAddr;
              if (RXByteCtr)
                  MasterMode = SWITCH_TO_RX_MODE;   // Need to start receiving now
              else
                  MasterMode = TX_DATA_MODE;        // Continue to transmision with the data in Transmit Buffer
              break;

          case SWITCH_TO_RX_MODE:
              UCB0IE |= UCRXIE;              // Enable RX interrupt
              UCB0IE &= ~UCTXIE;             // Disable TX interrupt
              UCB0CTLW0 &= ~UCTR;            // Switch to receiver
              MasterMode = RX_DATA_MODE;    // State state is to receive data
              UCB0CTLW0 |= UCTXSTT;          // Send repeated start
              if (RXByteCtr == 1)
              {
                  //Must send stop since this is the N-1 byte
                  while((UCB0CTLW0 & UCTXSTT));
                  UCB0CTLW0 |= UCTXSTP;      // Send stop condition
              }
              break;

          case TX_DATA_MODE:
              if (TXByteCtr)
              {
                  UCB0TXBUF = TransmitBuffer[TransmitIndex++];
                  TXByteCtr--;
              }
              else
              {
                  //Done with transmission
                  UCB0CTLW0 |= UCTXSTP;     // Send stop condition
                  MasterMode = IDLE_MODE;
                  UCB0IE &= ~UCTXIE;                       // disable TX interrupt
                  __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
              }
              break;

          default:
              __no_operation();
              break;
        }
        break;
    default: break;
  }
}


//******************************************************************************
// FRAM write compiler routine *************************************************
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__)
#pragma PERSISTENT(FRAM_write)
int16_t FRAM_write[WRITE_SIZE] = {0};
#elif defined(__IAR_SYSTEMS_ICC__)
__persistent unsigned long FRAM_write[WRITE_SIZE] = {0};
#elif defined(__GNUC__)
int16_t __attribute__((persistent)) FRAM_write[WRITE_SIZE] = {0};
#else
#error Compiler not supported!
#endif

// FRAM write functino
void FRAMWrite(int16_t data)
{
    //Write data in FRAM
    FRAM_write[i] = data;
    //i incrementation to write in the FRAM
    i = i+1;
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
        measurement();
      break;
    default: break;
  }
}

//comment for tristan
