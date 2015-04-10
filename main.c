#include <stdint.h>
#include "msp430x22x4.h"

/*
  Pin usage list
  P2.0 LED out for Dalek head
  P2.1 LED out for Dalek head
  P2.2 Fan controller (enable TIP102 chip transitor)
  P2.3 ADC Analog Input A3 (for the IR Sensor)

  DEBUG ONLY
  P2.4 IR SESNOR
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
volatile uint8_t motorACmd = 127;
volatile uint8_t motorBCmd = 255;

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

#pragma vector=PORT2_VECTOR
__interrupt void IsrPort2(void)
{
	// Found an interrupt on PORT2.3
	if ( (P2IFG & 0x10) == 0x10 ) {
		/*
		 * Turn off interrupt for P2.3
		 * 	Move to the second stage, INTRUDER
		 * 	Dalek should flash headlights somewhat quickly
		 * 	Disable this timer, and enable the Timer IRQ
		 */
		if ( (P2IN & 0x10) == 0x10 ) {		// P2.4 is high
			P2IE = 0x00;					// Turn off interrupts
			currentObjective = WARNING;		// Change base command to WARNING
			TACTL |= TAIE;					// Turn on the timer interrupt
			TACCR0 = TimerValue();        	// Blink the LEDs based on cmd
		}
	}
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
			nextObjective = RETURN;
		}
		if ( currentObjective == WARNING ) {  // Intruder found routine
			keepState = IntruderOperation(timerInterval);
			TACCR0 = TimerValue();        	// Blink the LEDs based on cmd
			nextObjective = SEEKING;			// Tell the MSP to go to seeking mode

			/* Turn on the ADC10 converter */
			ADC10AE0 = 0x08;		// Set ADC10 in to A3 (P2.3)
			/* Set ADC Control Register (Refs Vcc, Vss); Sample hold time 16 cycles; Turn on; Interrupt enabled */
			ADC10CTL0 = SREF_0 | ADC10SHT_2 | ADC10ON | ADC10IE | ENC | ADC10SC;
		}
		if ( currentObjective == SEEKING ) {
			while ( !(IFG2 & UCA0TXIFG)) {};      	// Confirm that Tx Buff is empty
				UCA0TXBUF = motorACmd;				// command M1
			while ( !(IFG2 & UCA0TXIFG)) {};     	// Confirm that Tx Buff is empty
				UCA0TXBUF = motorBCmd;				// command M2
			keepState = 1;
		}
		if ( keepState == 0 ) {        // Done exterminating/Warning
			// STOP ALL LIGHTS, DISABLE TIMER
			if ( currentObjective == EXTERMINATE ) {
				TACTL &= ~(TAIE);                      // Disable Timer
				P2IE = 0x10;			// Set Interrupt enabled for P2.3
			}
			if ( currentObjective == WARNING ) {
				/* Start the UART */
				UCA0CTL1 &= ~UCSWRST;             // Enable USCI state mach
				UCA0TXBUF = 0;                    // Init robot to stopped state

				/* Start the ADC10 */
				ADC10CTL0 |= ENC + ADC10SC;        // Enab ADC10 & start new sample
			}
			if ( currentObjective == SEEKING ) {
				UCA0CTL1 &= 0xFF;                 // Disable USCI state mach
				UCA0TXBUF = 0;                    // Init robot to stopped state
			}
			currentObjective =  nextObjective;
			TACCR0 = TimerValue();        			// Set the timer for the next value
			P2OUT &= ~(0x03);                     	// Turn off the Head LEDs
			timerInterval = 0;
			break;
		}
		// Blink the LEDs
		if ( (currentObjective == WARNING) || (currentObjective == EXTERMINATE) ) {
			if ( ++ledCounter == 25 ) {
				ledCounter = 0;
				P2OUT ^= 0x03;                          // Blink the head of Daleks
				timerInterval += 1;                      // Step fan on counter
			}
		}
		if ( currentObjective == SEEKING ) {
			if ( ++ledCounter == 1200 ) {
				ledCounter = 0;
				P2OUT ^= 0x02;
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
	if ( ADC10MEM > 0x01BB) {		// Did not find a person
//		motorACmd = 0x00;
//		motorBCmd = 0x00;
		P2OUT |= 0x01;
	} else {						// Found our target
//		motorACmd = 0x7F;
//		motorBCmd = 0xFF;
		P2OUT &= ~(0x01);
	}

	__bic_SR_register_on_exit(CPUOFF);   // Clr prev. CPUOFF bit on stack
	// to keep CPU awake upon return.

} // end ISR

int main( void )
{
	// Stop watchdog timer to prevent time out reset
 	WDTCTL = WDTPW + WDTHOLD;
	currentObjective = SEEKING;       // Debug only

	/* Set up Pin directions for blinking headlights */
	P2DIR = 0x07;        	// Set P2.{2, 1, 0} as output

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

	/** DEBUGGING ONLY **/
	//P2IE = 0x10;			// Set Interrupt enabled for P2.3
	P2IES = 0x00;			// Rising Edge
	P2IFG = 0x00;
	P2OUT = 0x00;
	/** END OF DEBUG CODE **/

	/**
	 * Set the timer:
	 *   Measured frequency: 1.143MHz, Divided by 1 = 800KHz
	 *   Up mode
	 **/
	// SMCLK | Div by 1 | Up Mode | Interrupts
	TACTL   = TASSEL_2 | ID_0 | MC_1;

	/** DEBUG CODE FOR THE IR SENSOR */
	/* Set ADC Control Register (Refs Vref, Vss); Sample hold time 16 cycles; Turn on */
	ADC10AE1 = 0x10;		// Set ADC10 in to A3 (P4.3)
	ADC10CTL1 |= INCH_12;
	ADC10CTL0 = SREF_7 | ADC10SHT_2 | ADC10ON | ADC10IE;

	/* DEBUG ONLY FOR UART */
	UCA0TXBUF = 0;                     // Init robot to stopped state
	UCA0CTL1 &= ~UCSWRST;             // Enable USCI state mach

	TACCR0 = TimerValue();

	TACTL |= TAIE;					// Turn on the timer interrupt
//	while (1) {
		_BIS_SR(LPM1_bits + GIE);                  // Enter LPM w/ IRQs enabled
//	}

	return 0;
}
