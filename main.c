#include <stdint.h>
#include "msp430x22x4.h"           // chip-specific macros & defs
#include "wireless.h"              // Wireless setup & function defs

/*
  Pin usage list
  P4.4 LED out for Dalek head
  P4.5 LED out for Dalek head
  P4.3 Fan controller (enable TIP102 chip transitor, A12)
 */

/*
  Enumeration to control what command the dalek is at
	These can be passed to different methods to yield
	different results, should be passed with the global
	enum dalek_opeation
 */
enum DALEK_PRIMARY_OBJECTIVE {
	STANDBY,		// Waiting for message from sensors
	WARNING,		// We have found an intruder
	SEEKING,		// Seek out the intruder
	EXTERMINATE,	// EXTERMINATE
	RETURN			// Return back to base?
};

// Global values
volatile enum DALEK_PRIMARY_OBJECTIVE currentObjective;
volatile uint8_t motorACmd = 0x60;
volatile uint8_t motorBCmd = 0xE0;

// Wifi
static uint8_t len = 2;          // Packet Len = 3 bytes
volatile uint8_t rxPkt[2];                // Buffer to store received pkt payload

/*
  Determine if the head lights should continue blinking
	based on the value of the timer, should be in
	increments of 25ms, at 2s, stop supplying power
	to the fan.

  returns 0 if two seconds has elapsed (should keep going)
 */
uint16_t IntruderOperation( uint16_t elapsedTime ) {
	if ( elapsedTime < 32 ) {        // Should be a 4s timer (64 is 8s @ 0.25 steps)
		return 1;
	} else {
		return 0;                       // End extermination sequence
	}
}
/*
  Determine what to do with the glitter gun,
	based on the value of the timer, should be in
	increments of 25ms, at 2s, stop supplying power
	to the fan.

  returns 0 if two seconds has elapsed (should keep going)
 */
uint16_t GlitterGunOperation( uint16_t elapsedTime ) {
	if ( elapsedTime < 64 ) {        // Should be a 4s timer (64 is 8s @ 0.25 steps)
		P2OUT |= (0x04);               // Turn on 2.2 (glitter gun)
		return 1;
	} else {
		P2OUT &= ~(0x04);               // Turn off 2.2 (glitter gun)
		return 0;                       // End extermination sequence
	}
}

/*
  Returns a uint16 value to set the Timer to, this timer
  will either fire commands every 0.5, 0.375, or 0.25s
  based on the current dalek command
 */
uint16_t TimerValue() {
	uint16_t timerIncrementor = 0;
	switch ( currentObjective ) {
		case (EXTERMINATE):   // Exterminate! Blink every 25ms!
			timerIncrementor = 5715;
			break;
	case (WARNING):      			// If we're just sensing them
		timerIncrementor = 8573;	// but not moving, set to 37.5ms
		break;
	case (SEEKING):      	// Move toward them! (set to 500us)
		timerIncrementor = 571;
		break;
	case (STANDBY):			// Do not blink, stay in hiding
	case (RETURN):
	timerIncrementor = 0;
	break;
	}
	return timerIncrementor;
}

/**
  Timer interrupt for the Dalek
	  This has to handle events for:
		  intruder: Blink the headlights agressively
		  moving: sending commands to the sabetooth?
		  exterminate: Blink the headlights rapidly and run the fan

 */
#pragma vector=TIMERA1_VECTOR
__interrupt void IsrTimerTACC1(void)
{
	static volatile uint16_t ledCounter = 0;      // Count if should blink leds
	static volatile uint16_t timerInterval = 0;    // Count how long action should take
	volatile uint16_t keepState = 0;           // Stay in the current objective? (0 move on)
	enum DALEK_PRIMARY_OBJECTIVE nextObjective = STANDBY;
	switch ( __even_in_range( TAIV, 10 ) ) {
	case TAIV_TAIFG:
		if ( currentObjective == EXTERMINATE ) {  // Turn on the Glitter gun
			keepState = GlitterGunOperation(timerInterval);
			nextObjective = STANDBY;
		}
		if ( currentObjective == WARNING ) {  // Intruder found routine
			keepState = IntruderOperation(timerInterval);
			TACCR0 = TimerValue();        	// Blink the LEDs based on cmd
			nextObjective = SEEKING;			// Tell the MSP to go to seeking mode
		}
		if ( currentObjective == SEEKING ) {
			while ( !(IFG2 & UCA0TXIFG)) {};      	// Confirm that Tx Buff is empty
				UCA0TXBUF = motorACmd;				// command M1
			while ( !(IFG2 & UCA0TXIFG)) {};     	// Confirm that Tx Buff is empty
				UCA0TXBUF = motorBCmd;				// command M2
			keepState = 1;		// Handled elsewhere
		}
		if ( keepState == 0 ) {        // Done exterminating/Warning
			// STOP ALL LIGHTS, DISABLE TIMER
			if ( currentObjective == EXTERMINATE ) {
				TACTL &= ~(TAIE);                      // Disable Timer
			}
			if ( currentObjective == WARNING ) {
				/* Start the UART */
				UCA0CTL1 &= ~UCSWRST;             // Enable USCI state mach
				UCA0TXBUF = 0;                    // Init robot to stopped state
				ADC10CTL0 |= ADC10ON | REFON | ADC10IE;	// Start ADC10 Interrupts

				/* Start the ADC10 */
				ADC10CTL0 |= ENC + ADC10SC;        // Enab ADC10 & start new sample
			}
			currentObjective =  nextObjective;
			TACCR0 = TimerValue();        			// Set the timer for the next value
			P4OUT &= ~(0x30);                     	// Turn off the Head LEDs
			timerInterval = 0;
			break;
		}
		// Blink the LEDs
		if ( (currentObjective == WARNING) || (currentObjective == EXTERMINATE) ) {
			if ( ++ledCounter == 25 ) {
				ledCounter = 0;
				P4OUT ^= 0x30;                          // Blink the head of Daleks
				timerInterval += 1;                      // Step fan on counter
			}
		}
		if ( currentObjective == SEEKING ) {
			if ( ++ledCounter == 150) {					// Run the ADC every 0.5s (as per spec)
				ledCounter = 0;
				P4OUT ^= 0x30;
				ADC10CTL0 |= ENC + ADC10SC;        // Enab ADC10 & start new sample
			}
		}
		break;
	default:
		break;
	}
	TACTL &= ~(0x0001);                    // Clear TAIFG
}

#pragma vector=ADC10_VECTOR
__interrupt void IsrAdc10FoundValue(void)
{
	while ( (ADC10CTL1 & ADC10BUSY) == 0x0001 );		// Wait for a stable read
	P1OUT = 0x00;
		//	less than 1.54V, Move slowly
	if ( ADC10MEM < 0x0276 ) {							// Lost 'sight' of person (634), 1.40V
		motorACmd = 0x60;
		motorBCmd = 0xE0;
		__bic_SR_register_on_exit(CPUOFF);   // Clr prev. CPUOFF bit on stack
	}
		// EXTERMINATE! (1.584V)
	if ( ADC10MEM > 0x028D ) {						// Found our target
		while ( !(IFG2 & UCA0TXIFG)) {};      	// Confirm that Tx Buff is empty
			UCA0TXBUF = 0x00;				// command M1
		while ( !(IFG2 & UCA0TXIFG)) {};     	// Confirm that Tx Buff is empty
			UCA0TXBUF = 0x00;				// command M2

		// Disable the interrupts for this state (ADC10 and UART)
		UCA0CTL1 &= 0xFF;                 // Disable USCI state mach
		ADC10CTL0 &= ~(ADC10ON | ADC10IE);

		currentObjective = EXTERMINATE;			// Tell the MSP to go to EXTERMINATE
		TACCR0 = TimerValue();        	// Blink the LEDs based on cmd
		__bic_SR_register_on_exit(CPUOFF);   // Clr prev. CPUOFF bit on stack
	}

	// to keep CPU awake upon return.

} // end ISR

#pragma vector=PORT2_VECTOR
__interrupt void PktRxedISR(void)
//----------------------------------------------------------------------------
// Func:  Packet Received ISR:  triggered by falling edge of GDO0.
//        Parses pkt & sends 2 data bytes through UART to the host PC.
// Args:  None
// Retn:  None
//----------------------------------------------------------------------------
{
  // Buffer Len for rxPkt = only address plus data bytes;
  // pkt size byte not incl b/c it is stripped away within RX function.

  uint8_t status[2];               // Buffer to store pkt status bytes
  static uint8_t crcOk;            // Flag pkt was received w/ good CRC

  if(TI_CC_GDO0_PxIFG & TI_CC_GDO0_PIN)         // chk GDO0 bit of P2 IFG Reg
    crcOk = RFReceivePacket(rxPkt,&len,status); // Fetch packet from cc2500

  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;          // Reset GDO0 IRQ flag

  if(crcOk)                     // If RXed pkt valid, send to screen via UART
  {
    P1OUT ^= 0x03;                       // Pkt RXed =>Toggle LEDs
    if ( rxPkt[1] == 0xAA ) {
    	// Turn on the next phase of our robot if it is on standby
    	if ( currentObjective == STANDBY ) {
    		currentObjective = WARNING;
    		TACCR0 = TimerValue();
    		TACTL |= TAIE;
    	}
    }
    crcOk = 0;                           // Clear Pkt Received flag
  }
  TI_CC_SPIStrobe(TI_CCxxx0_SIDLE);      // Set cc2500 to idle mode.
  TI_CC_SPIStrobe(TI_CCxxx0_SRX);        // Set cc2500 to RX mode.
                                         // AutoCal @ IDLE to RX Transition
}


//----------------------------------------------------------------------------
// FUNC:  Setup MSP430 Ports & Clocks and reset & config cc2500
// ARGS:  none
// RETN:  none
//----------------------------------------------------------------------------
void SetupAll(void)
{
  volatile uint16_t delay;

  for(delay=0; delay<650; delay++);     // Empirical: cc2500 Pwr up settle

  // set up clock system
  BCSCTL1 = CALBC1_8MHZ;                // set DCO freq. from cal. data
  BCSCTL2 |= DIVS_3;                    // SMCLK = MCLK/8 = 1MHz
  DCOCTL  = CALDCO_8MHZ;                // set MCLK to 8MHz

  // Port config
  P1DIR |=  0x03;                       // Set LED pins to output
  P1OUT &= ~0x03;                       // Clear LEDs

  // Wireless Initialization
  TI_CC_SPISetup();                     // Initialize SPI port
  P2SEL = 0;                            // P2.6 & P2.7 = GDO0 & GDO2
  TI_CC_PowerupResetCCxxxx();           // Reset cc2500
  writeRFSettings();                    // Write RF settings to config reg

  TI_CC_GDO0_PxIES |=  TI_CC_GDO0_PIN;  // Int on GDO0 fall. edge (end of pkt)
  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;  // Clear  GDO0 IRQ flag
  TI_CC_GDO0_PxIE  |=  TI_CC_GDO0_PIN;  // Enable GDO0 IRQ
  TI_CC_SPIStrobe(TI_CCxxx0_SRX);       // Initialize cc2500 in RX mode.

  TI_CC_SPIWriteReg(TI_CCxxx0_CHANNR,   10);  // Set Your Own Channel Number
                                             // AFTER writeRFSettings (???)

  for(delay=0; delay<650; delay++);     // Empirical: Let cc2500 finish setup

  P1OUT = 0x02;                         // Setup done => Turn on green LED
}

int main( void )
{
	// Stop watchdog timer to prevent time out reset
 	WDTCTL = WDTPW + WDTHOLD;
 	P1DIR |= 0x01;                    // P1.0 = output (LED)
 	P1OUT = 0x01;

	currentObjective = STANDBY;       // Debug only

	/* Set up Pin directions for blinking headlights */
	P4DIR = 0x30;        	// Set P2.{2, 1, 0} as output

	/* Set up robot motion */
	/* Set up pin direction for UART */
	P3SEL = 0x30;                         // P3.4,5 = USCI_A0 TXD/RXD

	/* Config. UART Clock & Baud Rate */
	BCSCTL1 = CALBC1_1MHZ;                // DCO = 1 MHz
	DCOCTL  = CALDCO_1MHZ;                // DCO = 1 MHz
	UCA0CTL1 |= UCSSEL_2;                 // UART use SMCLK

	UCA0MCTL = UCBRS0;                    // Map 1MHz -> 9600 (Tbl 15-4)
	UCA0BR0  = 104;                       // Map 1MHz -> 9600 (Tbl 15-4)
	UCA0BR1  = 0;                         // Map 1MHz -> 9600 (Tbl 15-4)

	/**
	 * Set the timer:
	 *   Measured frequency: 1.143MHz, Divided by 1 = 800KHz
	 *   Up mode
	 **/
	// SMCLK | Div by 1 | Up Mode | Interrupts
	TACTL   = TASSEL_2 | ID_0 | MC_1;

	/** DEBUG CODE FOR THE IR SENSOR */
	/*
	 * Set ADC Control Register (Refs Vref+, Vss);
	 * 	Sample hold time 16 cycles;
	 * 	Turn on, enable interrupts
	 *
	 * 	Use Internal 2.5V, turn it on
	 */
	ADC10AE1 = 0x01;		// Enable ADC10 in to A12 (P4.3)
	ADC10CTL1 |= INCH_12;	// Set the Analog input to be A12
	ADC10CTL0 = SREF_1 | ADC10SHT_2 | REF2_5V;

	/* UART For the robot */
	UCA0TXBUF = 0;                     // Init robot to stopped state
	UCA0CTL1 &= ~UCSWRST;             // Enable USCI state mach

	SetupAll();

	_BIS_SR(LPM1_bits + GIE);                  // Enter LPM w/ IRQs enabled

	return 0;
}
