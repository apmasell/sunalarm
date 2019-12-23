#define F_CPU 16000000UL

#include<avr/interrupt.h>
#include<avr/io.h>
#include<avr/sleep.h>
#include<util/delay.h>
#include<stdbool.h>
#include<stdlib.h>

#define WAKEUP 6
#define DAWN_MIN 8
#define STEPS 8
#define DAWN_INTERVAL ((DAWN_MIN + 1) * STEPS - 1)

// Whether we have completed calibration or not
volatile bool initialised = false;
// The number of timer ticks we sould delay for each minute in our simulated dawn. This is the amount of AC leading wave that we are chopping off. OR the total number of ticks per AC cycle we have seen (during calibration).
int dawnTicksPerHalfWave[DAWN_INTERVAL];
// The current number of AC cycles we have seen this minute (normally); or the total number of AC cycles we have seen (during calibraiton).
volatile int cycleCounter = 0;
// The number of minutes per day; initialised to be when you plug the thing in
volatile int minuteCounter = (22 - WAKEUP) * 60 + DAWN_MIN;	// 10PM

int cmpfunc(const void *a, const void *b)
{
	return (*(int *)a - *(int *)b);
}

ISR(INT0_vect)
{
	if (!initialised) {
		// Determining the correct time count is hard, so we're going to do it
		// experimentally (as a side benefit, it will autocalibrate between 50/60Hz
		// power sources); we measure the delay between AC zero crossings many times.

		// Stop the timer
		TCCR1A = 0;
		TCCR1B = 0;
		const int currentCount = TCNT1;
		// If we got a time value, store it in our array
		if (currentCount > 0) {
			dawnTicksPerHalfWave[cycleCounter++] = currentCount;
		}
		// If we've seen enough cycles, do our calibration
		if (cycleCounter == DAWN_INTERVAL) {
			// Reset for the normal operation
			cycleCounter = 0;
			// Sort our samples and take the median
			qsort(dawnTicksPerHalfWave, DAWN_INTERVAL,
			      sizeof(dawnTicksPerHalfWave[0]), &cmpfunc);
			const int ticksPerHalfWave =
			    dawnTicksPerHalfWave[DAWN_INTERVAL / 2];
			// Compute the delay for each of our dawn minutes
			for (int i = 0; i < DAWN_INTERVAL; i++) {
				const int tickDelay = (DAWN_INTERVAL -
						       i) *
				    ticksPerHalfWave / DAWN_INTERVAL *
				    (DAWN_INTERVAL - i) / DAWN_INTERVAL *
				    (DAWN_INTERVAL - i) / DAWN_INTERVAL *
				    (DAWN_INTERVAL - i) / DAWN_INTERVAL;
				dawnTicksPerHalfWave[i] =
				    tickDelay < 1 ? 1 : tickDelay;
			}
			initialised = true;
			// Turn on the built-in LED
			PORTD |= (1 << PD6);
		} else {
			// If we haven't seen enough AC cycles, reset the timer and capture another
			TCNT1 = 0;
			TCCR1A = 0;
			TCCR1B = (1 << CS12);
		}
		return;
	}
	cycleCounter = (cycleCounter + 1) % 7200;	// AC half-cycles per minute
	if (cycleCounter == 0) {
		minuteCounter = (minuteCounter + 1) % 1440;	// minutes per day
	}
	const bool requestedOn = !(PINB & (1 << PB1));
	if (requestedOn) {	// Requested on
		PORTD |= (1 << PD2);
	} else if (minuteCounter < DAWN_MIN) {	// Dawn
		// Make sure the AC power is off
		PORTD &= ~(1 << PD2);
		// Rest the timer
		TCCR1A = 0;
		TCCR1B = 0;
		TCNT1 = 0;
		// The timer is going to fire two interrupts A to turn on the AC, and B to turn off the signal to the AC (since the TRIAC will stay on)
		TIFR1 |= (1 << OCF1A) | (1 << OCF1B);
		// Working in 15 second chunks
		const int ticks =
		    dawnTicksPerHalfWave[minuteCounter * STEPS +
					 cycleCounter / (7200 / STEPS)];
		OCR1A = ticks;
		OCR1B = ticks + 10;
		// Start the timer
		TCCR1A = 0;
		TCCR1B = (1 << CS12);
	} else if (minuteCounter < 100 && (minuteCounter > DAWN_MIN || (cycleCounter / 120) % 2 == 0 || cycleCounter > 1200)) {	// Morning period always on (blinking at start)
		PORTD |= (1 << PD2);
	} else {		// Always off
		PORTD &= ~(1 << PD2);
	}
}

/*
 * When we have delayed long enough, turn on the AC (i.e., trigger the TRIAC).
 */
ISR(TIMER1_COMPA_vect)
{
	if (initialised) {
		PORTD |= (1 << PD2);
	}
}

/*
 * The TRIAC will remain on until the zero cross, but we're going to turn the pulse off, so we don't accidentally get to the next cycle and introduce flicker
 */
ISR(TIMER1_COMPB_vect)
{
	if (initialised) {
		TCCR1A = 0;
		TCCR1B = 0;
		PORTD &= ~(1 << PD2);
	}
}

int main()
{
	// Set pin 1 (AC driver) and 6 (built-in LED) as output
	DDRD = (1 << PD2) | (1 << PD6);
	// Disable timer 0
	TCCR0B = 0;
	TCCR0A = 0;
	// Each AC half-cycle is about 8.333ms
	// Run clock at 256th speed; there are 520 ticks per AC half-cycle
	// Enable interrupt
	TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B);

	// Rising edge triggered
	EICRA = (1 << ISC01) | (1 << ISC00);
	// Set PD0 as interrupt
	EIMSK = (1 << INT0);
	TCNT1 = 0;

	sei();
	while (!initialised) ;
	while (true) {
		const bool requestedOn = !(PINB & (1 << PB1));
		if (requestedOn) {
			if (cycleCounter % 60 == 0) {
				if ((cycleCounter / 60) % 2 == 0) {
					PORTD &= ~(1 << PD6);
				} else {
					PORTD &= ~(1 << PD6);
				}
			}
		} else {
			if (cycleCounter % 120 == 80) {
				PORTD |= (1 << PD6);
			} else if (cycleCounter % 120 == 0
				   && cycleCounter / 120 <=
				   (minuteCounter / 60 + WAKEUP) % 12) {
				PORTD &= ~(1 << PD6);
			}
		}
	}
}
