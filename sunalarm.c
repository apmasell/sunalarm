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

enum current_programme {
	INIT,
	NORMAL,
	SLEEP_IN,
	MANUAL_ON
};

// The number of timer ticks we sould delay for each minute in our simulated dawn. This is the amount of AC leading wave that we are chopping off. OR the total number of ticks per AC cycle we have seen (during calibration).
int dawnTicksPerHalfWave[DAWN_INTERVAL];
volatile enum current_programme currentProgramme = INIT;
bool lastButtonState = false;
int buttonPressDuration = 0;
int manualMinutesLeft = 0;
// The current number of AC cycles we have seen this minute (normally); or the total number of AC cycles we have seen (during calibraiton).
int cycleCounter = 0;
// The number of minutes per day; initialised to be when you plug the thing in
int minuteCounter = (22 - WAKEUP) * 60 + DAWN_MIN;	// 10PM

int cmpfunc(const void *a, const void *b)
{
	return (*(int *)a - *(int *)b);
}

int compute_light_intensity(int minuteOfDay)
{
	return minuteOfDay < 100 ? minuteOfDay : 0;
}

ISR(INT0_vect)
{
	if (currentProgramme == INIT) {
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
			currentProgramme = NORMAL;
		} else {
			// If we haven't seen enough AC cycles, reset the timer and capture another
			TCNT1 = 0;
			TCCR1A = 0;
			TCCR1B = (1 << CS12);
			return;
		}
	}
	cycleCounter = (cycleCounter + 1) % 7200;	// AC half-cycles per minute
	if (cycleCounter == 0) {
		minuteCounter = (minuteCounter + 1) % 1440;	// minutes per day
	}
	const bool currentButtonState = !(PINB & (1 << PB1));
	if (lastButtonState != currentButtonState) {
		lastButtonState = currentButtonState;
		if (currentButtonState) {
			buttonPressDuration = 0;
		} else {
			switch (currentProgramme) {
			case NORMAL:
				if (buttonPressDuration < 120) {
					currentProgramme = SLEEP_IN;
				} else {
					currentProgramme = MANUAL_ON;
					manualMinutesLeft = buttonPressDuration / 4;	// 1 second of holding == 30 min of being on
				}

				break;

			case MANUAL_ON:
			case SLEEP_IN:
				currentProgramme = NORMAL;
				break;

			}
		}
	} else if (currentButtonState && buttonPressDuration < 65535) {
		buttonPressDuration += 1;
	}

	int lightIntensity = 0;
	switch (currentProgramme) {
	case NORMAL:
		lightIntensity = compute_light_intensity(minuteCounter);
		break;
	case SLEEP_IN:
		lightIntensity =
		    minuteCounter >
		    60 ? compute_light_intensity(minuteCounter - 60) : 0;
		if (minuteCounter == 300) {
			currentProgramme = NORMAL;
		}
		break;
	case MANUAL_ON:
		lightIntensity = manualMinutesLeft;
		if (cycleCounter == 0) {
			if (manualMinutesLeft == 0) {
				currentProgramme = NORMAL;
			} else {
				manualMinutesLeft -= 1;
			}

		}
		break;
	}
	if (lightIntensity == 0) {
		PORTD &= ~(1 << PD2);
	} else if (lightIntensity > 0 && lightIntensity <= STEPS) {
		PORTD &= ~(1 << PD2);
		// Rest the timer
		TCCR1A = 0;
		TCCR1B = 0;
		TCNT1 = 0;
		// The timer is going to fire two interrupts A to turn on the AC, and B to turn off the signal to the AC (since the TRIAC will stay on)
		TIFR1 |= (1 << OCF1A) | (1 << OCF1B);
		// Working in 15 second chunks
		const int ticks = dawnTicksPerHalfWave[lightIntensity - 1];
		OCR1A = ticks;
		OCR1B = ticks + 5;
		// Start the timer
		TCCR1A = 0;
		TCCR1B = (1 << CS12);
	} else {
		PORTD |= (1 << PD2);

	}
	if ((currentProgramme == SLEEP_IN) && (cycleCounter < 5760)) {
		PORTD |= (1 << PD6);
	} else {
		PORTD &= ~(1 << PD6);
	}
}

/*
 * When we have delayed long enough, turn on the AC (i.e., trigger the TRIAC).
 */
ISR(TIMER1_COMPA_vect)
{
	if (currentProgramme != INIT) {
		PORTD |= (1 << PD2);
	}
}

/*
 * The TRIAC will remain on until the zero cross, but we're going to turn the pulse off, so we don't accidentally get to the next cycle and introduce flicker
 */
ISR(TIMER1_COMPB_vect)
{
	if (currentProgramme != INIT) {
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

	//Turn on built-in LED until initialization is over
	PORTD |= (1 << PD6);

	sei();
	while (true) ;
}
