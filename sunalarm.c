#include<avr/interrupt.h>
#include<avr/io.h>
#include<avr/sleep.h>
#include<stdbool.h>

int ticksPerHalfWave = 0;
int cycleCounter = 0;
int minuteCounter = 21 * 60;	// 9PM

ISR(INT0_vect)
{
	if (ticksPerHalfWave == 0) {
		if (TCNT1 > 0) {
			ticksPerHalfWave = TCNT1;
		} else {
			TCCR1B = (1 << CS12) | (1 << CS10);
			return;
		}
	}
	cycleCounter = (cycleCounter + 1) % 7200;	// AC half-cycles per minute
	if (cycleCounter % 120 == 0) {
		PORTD ^= (1 << PD6);
	}
	if (cycleCounter == 0) {
		minuteCounter = (minuteCounter + 1) % 1440;	// minutes per day
	}

	if (PIND & (1 << PB1)) {	// Requested on
		PORTD |= (1 << PD2);

	} else if (minuteCounter < 100) {	// Dawn
		PORTD &= ~(1 << PD2);
		TCCR1B = 0;
		OCR1A = (1 - minuteCounter / 100.0) * ticksPerHalfWave + 1;
		TCNT1 = 0;
		TCCR1B = (1 << CS12) | (1 << CS10);
	} else if (minuteCounter < 220) {	// Morning period always on
		PORTD |= (1 << PD2);
	} else {		// Always off
		PORTD &= ~(1 << PD2);
	}
}

ISR(TIMER1_COMPA_vect)
{
	PORTD |= (1 << PD2);
	TCCR1B = 0;
}

int main()
{
	// Set pin 1 (AC driver) and 6 (built-in LED) as output
	DDRD = (1 << PD2) | (1 << PD6);
	// Each AC half-cycle is about 8.333ms
	// Run clock at 256th speed; there are 520 ticks per AC half-cycle
	// Enable interrupt
	TIMSK1 = (1 << OCIE1A);

	// Rising edge triggered
	EICRA = (1 << ISC01) | (1 << ISC00);
	// Set PD0 as interrupt
	EIMSK = (1 << INT0);
	TCNT1 = 0;

	sei();
	while (true) ;
}
