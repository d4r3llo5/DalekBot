#include <stdint.h>
#include "msp430x22x4.h"           // chip-specific macros & defs
#include "wireless.h"              // Wireless setup & function defs

/*
  Pin usage list
  P2.0 Audio clip asking for Tea
  P2.1 Audio clip Exterminate
  P2.2 LED out for Dalek head
  P2.3 Fan controller
  P4.3 Temp sensor (enable TIP102 chip transitor, A12)
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

// Motor
volatile uint8_t lostTarget = 0;		// Boolean whether or not the dalek has lost the human
volatile uint8_t ramping = 0;			// If we lost the target and need to find it
volatile uint8_t motorACmd = 0x60;
volatile uint8_t motorBCmd = 0xE0;

// Wifi
static uint8_t len = 2;       	   // Packet Len = 3 bytes
uint8_t rxPkt[2] = {0, 0};      // Buffer to store received pkt payload

/* IR sensors */
volatile uint8_t activeSensors = 0;				// Keeps track of the IR Sensor states

/* ADC converter */
volatile uint16_t lastReadVoltage = 0;				// Keep track of the last read in voltage
const uint16_t maxVoltage = 0x0199;//0x0147;		// Max voltage used to exterminate (1.00V)
uint16_t minScanTemp = 0;							// Min voltage used to scan for target
uint16_t maxScanTemp = 0;								// Max voltage used to scan for target
uint16_t minMoveTemp = 0;								// Min voltage used to move to target
uint16_t maxMoveTemp = 0;								// Max voltage used to move to target

// Objectives
volatile uint8_t currObj = 0;		// Counter for parsing through the command array
volatile uint8_t objectives[] = {
		STANDBY,		// 0
		WARNING,		// 1
		STANDBY,		// 2
		WARNING,		// 3
		STANDBY,		// 4
		WARNING,		// 5,3
		SEEKING,		// 6,4
		EXTERMINATE,	// 7,5
		RETURN			// 8,6
};


/*
  Determine if the head lights should continue blinking
	based on the value of the timer, should be in
	increments of 25ms, at 2s, stop supplying power
	to the fan.

  returns 0 if two seconds has elapsed (should keep going)
 */
uint16_t IntruderOperation( uint16_t elapsedTime ) {
	if ( elapsedTime < 8 ) {        // Should be a 1s timer (64 is 8s @ 0.25 steps)
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
		P2OUT |= (0x04);               // Turn on 4.6 (glitter gun)
		return 1;
	} else {
		P2OUT &= ~(0x04);               // Turn off 4.6 (glitter gun)
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
	switch ( objectives[currObj] ) {
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
	uint16_t delay;

	switch ( __even_in_range( TAIV, 10 ) ) {
	case TAIV_TAIFG:
		if ( objectives[currObj] == EXTERMINATE ) {		// Turn on the Glitter gun
			P2OUT = 0x01;								// YELL EXTERMINATE
			keepState = GlitterGunOperation(timerInterval);
		}
		if ( objectives[currObj] == WARNING ) {  // Intruder found routine
			keepState = IntruderOperation(timerInterval);
			TACCR0 = TimerValue();        	// Blink the LEDs based on cmd
		}
		if ( objectives[currObj] == SEEKING ) {
			if ( lostTarget ) {
				ramping = 1;
				motorACmd = 0x40;
				motorBCmd = 0xC0;
				lostTarget = 0;
			}
			if ( motorACmd == 0x40 ) {
				motorACmd = 0x41;
			}
			if ( motorBCmd == 0xC0 ) {
				motorBCmd = 0xC1;
			}
			while ( !(IFG2 & UCA0TXIFG)) {};      	// Confirm that Tx Buff is empty
			UCA0TXBUF = motorACmd;				// command M1
			while ( !(IFG2 & UCA0TXIFG)) {};     	// Confirm that Tx Buff is empty
			UCA0TXBUF = motorBCmd;				// command M2
			keepState = 1;		// Handled elsewhere
		}
		if ( keepState == 0 ) {        // Done exterminating/Warning
			// STOP ALL LIGHTS, DISABLE TIMER
			if ( objectives[currObj] == EXTERMINATE ) {
				TACTL &= ~(TAIE);                     	// Disable Timer
				currObj = 0xFF;							// Roll over the objective counter
				for(delay=0; delay<650; delay++);     	// Empirical: Let cc2500 finish setup
				TI_CC_GDO0_PxIE  |=  TI_CC_GDO0_PIN;  	// Enable GDO0 IRQ
				for(delay=0; delay<650; delay++);     	// Empirical: Let cc2500 finish setup
				P2OUT &= 0x03;						// Stop yelling
			}
			if ( objectives[currObj] == WARNING ) {
				TI_CC_GDO0_PxIE  |=  TI_CC_GDO0_PIN; 	// Enable GDO0 IRQ
				if ( currObj == 5 ) {					// Use 5 for 3 sensors
					/* Start the UART */
					UCA0CTL1 &= ~UCSWRST;             // Enable USCI state mach
					while ( !(IFG2 & UCA0TXIFG)) {};      	// Confirm that Tx Buff is empty
					UCA0TXBUF = 0;						// Init robot to stopped state
					lostTarget = 1;

					//					ADC10CTL0 |= ADC10ON | REFON;			// Start ADC10
					//					ADC10CTL0 |= ENC | ADC10SC | ADC10IE;	// Enab ADC10, Interrupts, start new sample
					//
					//					TACTL &= ~(TAIE);					// Stop moving and look for the person
				}
			}
			currObj += 1;							// Incrementing Command counter
			TACCR0 = TimerValue();        			// Set the timer for the next value
			P2OUT &= ~(0x08);                     	// Turn off the Head LEDs
			timerInterval = 0;
			break;
		}
		// Blink the LEDs
		if ( (objectives[currObj] == WARNING) || (objectives[currObj] == EXTERMINATE) ) {
			if ( ++ledCounter == 25 ) {
				ledCounter = 0;
				timerInterval += 1;                      // Step fan on counter
			}
		}
		if ( objectives[currObj] == SEEKING ) {
			if ( (ledCounter % 72) == 0 ) {
				if ( timerInterval < 12 ) {
					motorACmd += 4;
					motorBCmd -= 4;
				} else if ( (timerInterval < 54) && (timerInterval > 29) ) {
					motorACmd -= 4;
					motorBCmd += 4;
				} else if ( (timerInterval < 84) && (timerInterval > 71) ) {
					motorACmd += 4;
					motorBCmd -= 4;
				} else if ( timerInterval > 83 ) {
					motorACmd += 4;
					motorBCmd += 4;
				} else {
					P2OUT ^= 0x08;
				}
				if ( timerInterval == 108 ) {
					P2OUT ^= 0x08;
					motorACmd = 0x40;
					motorBCmd = 0xC0;
					timerInterval = 0;
				} else {
					timerInterval++;
				}
				//				TACTL &= ~(TAIE);					// Stop moving and look for the person
				//				ADC10CTL0 |= ENC + ADC10SC;        // Enab ADC10 & start new sample
			}
			if ( ++ledCounter == 288) {					// Run the ADC about every 0.5s (as per spec)
				ledCounter = 0;
			}
		}
		break;
	default:
		break;
	}
	TACTL &= ~(0x0001);                    // Clear TAIFG
	__bic_SR_register_on_exit(CPUOFF);   // Clr prev. CPUOFF bit on stack
}

#pragma vector=ADC10_VECTOR
__interrupt void IsrAdc10FoundValue(void)
{
	while ( (ADC10CTL1 & ADC10BUSY) == 0x0001 );		// Wait for a stable read
	P2OUT ^= 0x08;                          // Blink the head of Daleks
	//	Less than 0.25V, Rotate to look for it
	if ( ADC10MEM < maxScanTemp	) {							// Lost 'sight' of person (634), 1.40V
		if ( !ramping ) {
			lostTarget = 1;
		}
	}
	// Between 0.30V and  0.96V
	if ( (ADC10MEM > minMoveTemp) && (ADC10MEM < maxMoveTemp) ) {
		motorACmd = 0x60;
		motorBCmd = 0xD0;
		lostTarget = 0;
		ramping = 0;
	}
	//
	//	// EXTERMINATE! (0.80V)
	if ( ADC10MEM > maxVoltage ) {						// Found our target
		motorACmd = 0x00;
		motorBCmd = 0x00;
		lostTarget = 0;
		ramping = 0;
	}
	//		while ( !(IFG2 & UCA0TXIFG)) {};      	// Confirm that Tx Buff is empty
	//		UCA0TXBUF = 0x00;				// command M1
	//		while ( !(IFG2 & UCA0TXIFG)) {};     	// Confirm that Tx Buff is empty
	//		UCA0TXBUF = 0x00;				// command M2
	//		//		 Disable the interrupts for this state (ADC10 and UART)
	//		UCA0CTL1 &= 0xFF;                 // Disable USCI state mach
	//		ADC10CTL0 &= ~(ADC10ON | ADC10IE);
	//		P2OUT = 0x03;					// Turn off asking for tea
	//
	//		currObj += 1;					// Tell the MSP to go to next phase
	//		TACCR0 = TimerValue();        	// Blink the LEDs based on cmd
	//		__bic_SR_register_on_exit(CPUOFF);   // Clr prev. CPUOFF bit on stack
	//		return
	//	}

	// to keep CPU awake upon return.
	TACTL |= TAIE;
	__bic_SR_register_on_exit(CPUOFF);   // Clr prev. CPUOFF bit on stack
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
	uint16_t delay;
	static uint8_t crcOk;            // Flag pkt was received w/ good CRC

	if(TI_CC_GDO0_PxIFG & TI_CC_GDO0_PIN)         // chk GDO0 bit of P2 IFG Reg
		crcOk = RFReceivePacket(rxPkt,&len,status); // Fetch packet from cc2500

	TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;          // Reset GDO0 IRQ flag

	if(crcOk)                     // If RXed pkt valid, send to screen via UART
	{
		P1OUT ^= 0x03;                       // Pkt RXed =>Toggle LEDs
		if ( (rxPkt[1] & 0x0F) == 0x05 ) {		// Good byte of data (0x1100 is a check to use)
			// If it is at one of the sensors
			if ( currObj == 0 ) {				// Depending on where we are in the code
				// The very first sensor has be\n triggered
				if ( (rxPkt[1] & 0xF0) == 0x50 ) {		// first sensor hit first time
					activeSensors |= 0x01;
					TI_CC_GDO0_PxIE  &=  ~(TI_CC_GDO0_PIN);  // Disable GDO0 IRQ
					currObj += 1;
					TACCR0 = TimerValue();
					TACTL |= TAIE;
				}
			} else if ( currObj == 2 ) {				// Second sensor hit first time
				if ( (rxPkt[1] & 0xF0) == 0xA0 ) {
					activeSensors |= 0x04;
					currObj += 1;
					TACCR0 = TimerValue();
					TACTL |= TAIE;
				}
			}
			else if ( currObj == 4 ) {				// Second sensor hit first time
				if ( (rxPkt[1] & 0xF0) == 0x90 ) {
					activeSensors |= 0x10;
					currObj += 1;
					TACCR0 = TimerValue();
					TACTL |= TAIE;
				}
			} else {
				if ( (rxPkt[1] & 0xF0) == 0x50 ) {		// first sensor
					activeSensors |= 0x01;				// Turn on
				}
				if ( (rxPkt[1] & 0xF0) == 0xA0 ) {		// second sensor
					activeSensors |= 0x04;				// Turn on
				}
				if ( (rxPkt[1] & 0xF0) == 0x90 ) {		// second sensor
					activeSensors |= 0x06;				// Turn on
				}
			}
		}
		if ( (rxPkt[1] & 0x0F) == 0x0A ) {			// Signal came in, the IR is off
			if ( (rxPkt[1] & 0xF0 ) == 0x50 ){
				activeSensors &= ~(0x01);			// First sensor is off
			} if ( (rxPkt[1] & 0xF0) == 0xA0 ) {
				activeSensors &= ~(0x04);			// Second
			} if ( (rxPkt[1] & 0xF0) == 0x90 ) {
				activeSensors &= ~(0x10);			// Second
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

	TI_CC_SPIWriteReg(TI_CCxxx0_CHANNR,   8);  // Set Your Own Channel Number
	// AFTER writeRFSettings (???)

	for(delay=0; delay<650; delay++);     // Empirical: Let cc2500 finish setup

	P1OUT = 0x02;                         // Setup done => Turn on green LED
}

void setAmbientTemp(void)
{
	uint8_t i = 0;
	uint32_t aveTemp = 0;			// Used to get percentages

	ADC10CTL0 |= ADC10ON | REFON | ENC | ADC10SC;        // Enab ADC10 & start new sample

	while ( i < 20 ) {
		while ( (ADC10CTL1 & ADC10BUSY) == 0x0001 );		// Wait for a stable read
		minScanTemp += ADC10MEM;
		i ++;
	}

	UCA0CTL1 &= 0xFF;                 // Disable USCI state mach
	ADC10CTL0 |= ~(ADC10ON | REFON | ENC | ADC10SC);

	minScanTemp = (minScanTemp / 20);
	aveTemp = maxVoltage - minScanTemp;
	maxScanTemp = ((aveTemp * 10) / 100) + minScanTemp;		// Max scan voltage
	minMoveTemp = ((aveTemp * 20) / 100) + minScanTemp;		// Min move voltage
	maxMoveTemp = ((aveTemp * 95) / 100) + minScanTemp;		// Max move voltage
}

int main( void )
{
	// Stop watchdog timer to prevent time out reset

	WDTCTL = WDTPW + WDTHOLD;
	P1DIR |= 0x01;                    // P1.0 = output (LED)
	P1OUT = 0x01;

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

	/* UART For the robot, set to defaults */
	while ( !(IFG2 & UCA0TXIFG)) {};      	// Confirm that Tx Buff is empty
	UCA0TXBUF = 0;                     // Init robot to stopped state
	UCA0CTL1 &= ~UCSWRST;             // Enable USCI state mach

	/**
	 * Set the timer:
	 *   Measured frequency: 1.143MHz, Divided by 1 = 800KHz
	 *   Up mode
	 **/
	// SMCLK | Div by 1 | Up Mode
	TACTL   = TASSEL_2 | ID_0 | MC_1 | TAIE;

	// Fan, LED, and Audio Circuit
	P2OUT = 0x0B;
	P2SEL |= ~(0x0F);
	P2DIR |= 0x0B;

	// Start the robot
	currObj = 5;
	TACCR0 = TimerValue();

	/*
	 * Set ADC Control Register (Refs Vref+, Vss);
	 * 	Sample hold time 16 cycles;
	 * 	Turn on, enable interrupts
	 *
	 * 	Use Internal 2.5V, turn it on
	 */
	ADC10AE1 = 0x01;		// Enable ADC10 in to A12 (P4.3)
	ADC10CTL1 |= INCH_14;	// Set the Analog input to be A12
	ADC10CTL0 = SREF_1 | ADC10SHT_2 | REF2_5V;
	SetupAll();

	//setAmbientTemp();
	motorACmd = 0x40;
	motorBCmd = 0xC0;
	_BIS_SR(LPM1_bits + GIE);                  // Enter LPM w/ IRQs enabled

	return 0;
}
