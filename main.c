/**
 * BrewTroller PID Display
 * 
 * For ATmega168 running internal 8MHz oscillator
 *
 * Board Summary: 
 *   Board carries 2 quad 7 segment LED displays in red and green, PNP BJTs
 *   for handling common anode power and all segment outputs are multiplexed
 *   into an ULN2803.
 *
 * Pin Config:
 *   B0		14		CA1		Common anode output to 3-to-8 pin 1
 *   B1		15		CA2		Common anode output to 3-to-8 pin 2
 *   B2		16		CA3		Common anode output to 3-to-8 pin 3
 *   B3		17		MOSI	ICSP in
 *   B4		18		MISO	ICSP out
 *   B5		19		SCK		ICSP clock
 *   AVCC	20		NC		NC
 *   B6		9		CA4		Common anode 8 drain (colon 1)
 *   B7		10		CA5		Common anode 9 drain (colon 2)
 *   C0		23		CA6		Segment A output to ULN
 *   C1		24		CA7		Segment B output to ULN
 *   C2		25		CA8		Segment C output to ULN
 *	 C3		26		ADDR	Input for address set (PCINT11)
 *	 C4		27		SDA		I2C/TWI data
 *   C5		28      SCL		I2C/TWI clock
 *   C6		1		RESET	Reset
 *   D0		2   	SEGB	RS-485 RO
 *   D1		3     	SEGC	RS-485 DI
 *   D2		4     	SEGA	RS-485 DE
 *   D3		5     	SEGD	Segment D output to ULN
 *   D4		6     	SEGE	Segment E output to ULN
 *   D5		11     	SEGF	Segment F output to ULN
 *   D6		12	   	SEGG	Segment G output to ULN
 *   D7		13     	SEGH	Segment H output to ULN
 *   VCC	7		VCC		+5v Power
 *   GND	8		GND		Ground
 *	 GND	22		GND		Ground
 */
 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include "twi.h"

/* Protocol constants */
#define BT_MODE                 0x00
#define INT_MODE                0x01
#define LAST_ASCII_CHAR         0x7F
#define UPPER_4_DIGITS          0xFD
#define LOWER_4_DIGITS          0xFE
#define ALL_8_DIGITS            0xFF

#define TWI_ADDRESS_DEFAULT		0x20
#define EEPROM_TWI_ADDRESS_LOC	0x00

#define LED_SEG_A				1
#define LED_SEG_B				2
#define LED_SEG_C				4
#define LED_SEG_D				8
#define LED_SEG_E				16
#define LED_SEG_F				32
#define LED_SEG_G				64
#define LED_SEG_H				128

#define CONFIG_OFF				0
#define CONFIG_DEBOUNCE			1
#define CONFIG_PRESENT			2
#define CONFIG_INCREMENT		3
#define CONFIG_SAVE				4

static const uint8_t led_font[] PROGMEM = {
  /* 0 - 9 */
  LED_SEG_A | LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_E | LED_SEG_F,
  LED_SEG_B | LED_SEG_C,
  LED_SEG_A | LED_SEG_B | LED_SEG_D | LED_SEG_E | LED_SEG_G,
  LED_SEG_A | LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_G,
  LED_SEG_B | LED_SEG_C | LED_SEG_F | LED_SEG_G,
  LED_SEG_A | LED_SEG_C | LED_SEG_D | LED_SEG_F | LED_SEG_G,
  LED_SEG_A | LED_SEG_C | LED_SEG_D | LED_SEG_E | LED_SEG_F | LED_SEG_G,
  LED_SEG_A | LED_SEG_B | LED_SEG_C,
  LED_SEG_A | LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_E | LED_SEG_F | LED_SEG_G,
  LED_SEG_A | LED_SEG_B | LED_SEG_C | LED_SEG_F | LED_SEG_G,
  
  /* a - z */
  LED_SEG_A | LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_E | LED_SEG_G,
  LED_SEG_C | LED_SEG_D | LED_SEG_E | LED_SEG_F | LED_SEG_G,
  LED_SEG_D | LED_SEG_E | LED_SEG_G,
  LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_E | LED_SEG_G,
  LED_SEG_A | LED_SEG_B | LED_SEG_D | LED_SEG_E | LED_SEG_F | LED_SEG_G,
  LED_SEG_A | LED_SEG_E | LED_SEG_F | LED_SEG_G,
  LED_SEG_A | LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_F | LED_SEG_G,
  LED_SEG_C | LED_SEG_E | LED_SEG_F | LED_SEG_G,
  LED_SEG_B | LED_SEG_C,
  LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_E,
  LED_SEG_A | LED_SEG_C | LED_SEG_E | LED_SEG_F | LED_SEG_G,
  LED_SEG_D | LED_SEG_E | LED_SEG_F,
  LED_SEG_A | LED_SEG_C | LED_SEG_E | LED_SEG_G,
  LED_SEG_C | LED_SEG_E | LED_SEG_G,
  LED_SEG_C | LED_SEG_D | LED_SEG_E | LED_SEG_G,
  LED_SEG_A | LED_SEG_B | LED_SEG_E | LED_SEG_F | LED_SEG_G,
  LED_SEG_A | LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_G | LED_SEG_H,
  LED_SEG_E | LED_SEG_G,
  LED_SEG_A | LED_SEG_C | LED_SEG_D | LED_SEG_F | LED_SEG_G,
  LED_SEG_D | LED_SEG_E | LED_SEG_F | LED_SEG_G,
  LED_SEG_C | LED_SEG_D | LED_SEG_E,
  LED_SEG_C | LED_SEG_D | LED_SEG_E,
  LED_SEG_A | LED_SEG_C | LED_SEG_D | LED_SEG_E,
  LED_SEG_B | LED_SEG_C | LED_SEG_E | LED_SEG_F | LED_SEG_G,
  LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_F | LED_SEG_G,
  LED_SEG_A | LED_SEG_B | LED_SEG_D | LED_SEG_E | LED_SEG_G,
};

// version number = hardware.software_major.software_minor
const char str_all_segs[] PROGMEM = "8.8.:8.8.8.8.:8.8.";
const char str_test_mode[] PROGMEM = "TESTMODE";
const char str_welcome[] PROGMEM = "pid  v6.6.2";
const char str_addr_02x[] PROGMEM = "addr  %02x";
const char str_float_format[] PROGMEM = "%5.1f";
const char str_dashes[] PROGMEM = "----";
const char str_save_02x[] PROGMEM = "save  %02x";
const char str_addr_blank[] PROGMEM = "addr    ";

char tmp_s[20];

// two line buffers used for the selective line display modes
char line_buf_1[6];
char line_buf_2[6];

volatile uint8_t config_state = CONFIG_OFF;
volatile uint16_t config_state_counter = 0;
volatile uint16_t config_state_counter2 = 0;
volatile uint8_t config_twi_address;

volatile uint8_t twi_address;

volatile uint8_t led_digit;

volatile uint8_t led_digit_data[10];

volatile uint16_t debug_led_counter = 0;

/**
 * The array below is used to control how long the timer dwells
 * on a particular digit. This is used to increase or decrease
 * the brightness of certain digits. Due to manufacturing
 * differences some colors or digits are naturally brighter
 * than others and this can be used to even up the display.
 * For a red top, green bottom display use:
 * 0, 0, 0, 0, 1, 1, 1, 1
 */
uint8_t led_digit_delays[] = {0, 0, 0, 0, 1, 1, 1, 1, 0, 1};

uint8_t led_digit_delay;

uint8_t led_map_char(char c);
void led_set_string(char *s);
void twi_data_recieved(uint8_t* buf, int length);
void twi_status(uint8_t status);
uint8_t fix_port_d(uint8_t b);
void test_mode(void);

int main(void) {
	// Read the twi_address stored in the EEPROM
	// If no address is currently stored, initialize the EEPROM with the
	// default address
	twi_address = eeprom_read_byte(EEPROM_TWI_ADDRESS_LOC);
	if (twi_address == 0xFF) {
		twi_address = TWI_ADDRESS_DEFAULT;
		eeprom_write_byte(EEPROM_TWI_ADDRESS_LOC, twi_address);
	}
	
	// Timer 0 is used to refresh the LED display
	// Clock Select, No Prescaler
	TCCR0B = _BV(CS01);
	// Interrupt Enable
	TIMSK0 = _BV(TOIE0);
	
	// Timer 1 is set up as a millisecond counter and is used
	// to fire off various functions
	// Clock Select, No Prescaler
	TCCR1B = _BV(CS10);
	// Interrupt Enable
	TIMSK1 = _BV(TOIE1);
	
	// Turn off all the segments before we start things up
	led_set_string("");
	
	// Turn on output for debug LED
	DDRB |= _BV(5);
	
	// Turn on outputs for the 3-to-8 anode driver
	DDRB |= _BV(0) | _BV(1) | _BV(2);
	
	// Turn on outputs for segments A-C to ULN
	DDRC |= _BV(0) | _BV(1) | _BV(2);
	
	// Turn on outputs for segments D-H to ULN
	DDRD |= _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7);
	
	// Turn on pull up resistor for the address set input
	// We need to do this early for the test mode check below
	PORTC |= _BV(3);

	// Turn on interrupts, starting the timers
	sei();
	
	if ((PINC & _BV(3)) == 0) {
		_delay_ms(10);
		if ((PINC & _BV(3)) == 0) {
			test_mode();
		}
	}
	
	// Print the welcome messages
	sprintf_P(tmp_s, str_all_segs);
	led_set_string(tmp_s);
	_delay_ms(2000);
	sprintf_P(tmp_s, str_welcome);
	led_set_string(tmp_s);
	_delay_ms(2000);
	sprintf_P(tmp_s, str_addr_02x, twi_address);
	led_set_string(tmp_s);
	_delay_ms(2000);
	led_set_string("");
	
	// Enable the pin change detect on PCINT11
	PCMSK1 |= _BV(PCINT11);
	// Enable the pin change interrupt for PCINT11
	PCICR |= _BV(PCIE1);

	// Initialize the two line buffers to all dashes
	sprintf_P(line_buf_1, str_dashes);
	sprintf_P(line_buf_2, str_dashes);
	
	// Initialize the twi interface
	twi_setAddress(twi_address);
	twi_attachSlaveRxEvent(twi_data_recieved);
	twi_init();
	
	for (;;) {
	}
}

void test_mode(void) {
	for (;;) {
		sprintf_P(tmp_s, str_test_mode);
		led_set_string(tmp_s);
		_delay_ms(3000);
		sprintf_P(tmp_s, str_all_segs);
		led_set_string(tmp_s);
		_delay_ms(3000);
		sprintf_P(tmp_s, str_welcome);
		led_set_string(tmp_s);
		_delay_ms(3000);
		sprintf_P(tmp_s, str_addr_02x, twi_address);
		led_set_string(tmp_s);
		_delay_ms(3000);
	}
}

void twi_data_recieved(uint8_t* buf, int length) {
	if (config_state == CONFIG_OFF) {
		if (buf[0] > LAST_ASCII_CHAR) {
			if (buf[1] == BT_MODE) {
				float *raw;
				// BrewTroller, takes 2 32 bit floats in big endian, 
				// 8 bytes total
				if ((buf[0] == ALL_8_DIGITS) || (buf[0] == UPPER_4_DIGITS)){
					raw = (float*) &buf[2];
					if (*raw == -1) {
						sprintf_P(line_buf_1, str_dashes);
					}
					else {
						sprintf_P(line_buf_1, str_float_format, (double) (*raw));
					}
				}
				if (buf[0] == LOWER_4_DIGITS){
					raw = (float*) &buf[2];
					if (*raw == -1) {
						sprintf_P(line_buf_2, str_dashes);
					}
					else {
						sprintf_P(line_buf_2, str_float_format, (double) (*raw));
					}
				}
				if (buf[0] == ALL_8_DIGITS){
					raw = (float*) &buf[6];
					if (*raw == -1) {
						sprintf_P(line_buf_2, str_dashes);
					}
					else {
						sprintf_P(line_buf_2, str_float_format, (double) (*raw));
					}
				}
 				sprintf(tmp_s, "%s%s", line_buf_1, line_buf_2);
				led_set_string(tmp_s);
			}
			else if (buf[1] == INT_MODE) {
				int16_t *raw;
				// BCS-460, takes 2 16 bit ints in big endian which are value * 10
				// 4 bytes total
				if ((buf[0] == ALL_8_DIGITS) || (buf[0] == UPPER_4_DIGITS)) {
					raw = (int16_t*) &buf[2];
					sprintf_P(line_buf_1, str_float_format, (double) (*raw / 10.0));
				}
				if (buf[0] == LOWER_4_DIGITS) {
					raw = (int16_t*) &buf[2];
					sprintf_P(line_buf_2, str_float_format, (double) (*raw / 10.0));
				}
				if (buf[0] == ALL_8_DIGITS) {
					raw = (int16_t*) &buf[4];
					sprintf_P(line_buf_2, str_float_format, (double) (*raw / 10.0));
				}
 				sprintf(tmp_s, "%s%s", line_buf_1, line_buf_2);
				led_set_string(tmp_s);
			}
		}
		else {
			// the data is a null terminated string
			led_set_string((char *) buf);
		}
	}
	// set the debug LED to on for 5 seconds
	debug_led_counter = 5000;
	PORTB |= _BV(5);
}

/**
 * Interrupt handler called when the address set button state changes.
 */
ISR(PCINT1_vect) {
	if ((PINC & _BV(3)) == 0) {
		if (config_state == CONFIG_OFF) {
			config_state = CONFIG_DEBOUNCE;
			config_state_counter = 0;
		}
	}
	else {
		if (config_state == CONFIG_INCREMENT) {
			config_state = CONFIG_SAVE;
			config_state_counter = 0;
			config_state_counter2 = 0;
		}
		else if (config_state != CONFIG_SAVE) {
			config_state = CONFIG_OFF;
			led_set_string("");
		}
	}
}

ISR(TIMER1_OVF_vect) {
	if (config_state == CONFIG_OFF) {
		// short circuits all the comparisons below since this is the
		// most common case
	}
	else if (config_state == CONFIG_DEBOUNCE) {
		if ((PINC & _BV(3)) == 0) {
			if (config_state_counter++ == 500) {
				config_state = CONFIG_PRESENT;
				config_state_counter = 0;
				config_state_counter2 = 0;
				config_twi_address = twi_address;
			}
		}
	}
	else if (config_state == CONFIG_PRESENT) {
		if (config_state_counter == 0) {
			sprintf_P(tmp_s, str_addr_02x, (int) config_twi_address);
			led_set_string(tmp_s);
		}
		else if (config_state_counter == 350) {
			sprintf_P(tmp_s, str_addr_blank);
			led_set_string(tmp_s);
		}
		
		if (++config_state_counter == 500) {
			config_state_counter = 0;
			if (++config_state_counter2 == 6) {
				config_state = CONFIG_INCREMENT;
				config_state_counter = 0;
			}
		}
	}
	else if (config_state == CONFIG_INCREMENT) {
		if (config_state_counter == 0) {
			sprintf_P(tmp_s, str_addr_02x, (int) config_twi_address);
			led_set_string(tmp_s);
		}
		
		if (++config_state_counter == 1000) {
			config_state_counter = 0;
			config_twi_address++;
			if (config_twi_address > 127) {
				config_twi_address = 0;
			}
		}
	}
	else if (config_state == CONFIG_SAVE) {
		if (config_state_counter == 0) {
			sprintf_P(tmp_s, str_save_02x, (int) config_twi_address);
			led_set_string(tmp_s);
		}
		else if (config_state_counter == 400) {
			sprintf_P(tmp_s, str_addr_02x, (int) config_twi_address);
			led_set_string(tmp_s);
		}
		
		if (++config_state_counter == 800) {
			config_state_counter = 0;
			if (++config_state_counter2 == 4) {
				config_state = CONFIG_OFF;
				twi_address = config_twi_address;
				twi_setAddress(twi_address);
				eeprom_write_byte(EEPROM_TWI_ADDRESS_LOC, twi_address);	
				led_set_string("");
			}
		}
	}
	
	// When the counter for the debug LED gets to 0 we toggle the
	// LED. The counter normally gets set to 250 but other functions
	// may change it.
	if (debug_led_counter == 0) {
		if (PORTB & _BV(5)) {
			PORTB &= ~(_BV(5));
		}
		else {
			PORTB |= _BV(5);
		}
		debug_led_counter = 250;
	}
	else {
		debug_led_counter--;
	}
	
	// Fire again in 1 ms
	TCNT1 = 0xffff - (F_CPU / 1000);
}

ISR(TIMER0_OVF_vect) {
	if (led_digit_delay) {
		led_digit_delay--;
		return;
	}
	
	// Anode drivers are on B0-B2
	// Segments are on C0-C2, D3-D7
	
	// Increment and loop the current digit.
	led_digit++;
	if (led_digit > 9) {
		led_digit = 0;
	}
	
	// The order of operations here is very important. If we don't use
	// this exact order of turning off the segments, turning off the anode
	// bits, setting the new anode bits and then turning on the segments
	// we will get digit bleed.

	// Turn off all the segments
	PORTC &= ~(0x07);
	PORTD &= ~(0xF8);

	// Turn off the anode driver bits. Because of the 3-to-8 this actually
	// turns on anode 0.
	PORTB &= ~(0x07);
	
	// Turn off the colon anode driver bits.
	DDRB &= ~(_BV(6) | _BV(7));

	if (led_digit < 8) {
		// Turn on the anode drivers
		PORTB |= (led_digit & 0x07);
	}
	else {
		// Turn on the colon anode drivers
		if (led_digit == 8) {
			DDRB |= _BV(6);
		}
		else {
			DDRB |= _BV(7);
		}
	}
	led_digit_delay = led_digit_delays[led_digit];

	// Set the segments on PORTC
	PORTC |= (led_digit_data[led_digit] & 0x07);
	
	// Set the remaining segments on PORTD
	PORTD |= (led_digit_data[led_digit] & 0xF8);
	
	// Reset the clock for 1ms and go!
	TCNT0 = 128;
}

/*
  Map the given char to the correct LED segment byte
  using the font table and some internals.
   a
 f   b  
   g  
 e   c   
   d
       h   
*/
uint8_t led_map_char(char c) {
	if (c >= 'a' && c <= 'z') {
		return pgm_read_byte(&led_font[c - 'a' + 10]); 
	}
	else if (c >= 'A' && c <= 'Z') {
		return pgm_read_byte(&led_font[c - 'A' + 10]);
	}
	else if (c >= '0' && c <= '9') {
		return pgm_read_byte(&led_font[c - '0']); 
	}
	else if (c == ' ') {
		return 0; 
	}
	else if (c == '-') {
		return LED_SEG_G;
	}
	else if (c == '.') {
		return LED_SEG_H;
	}
	else if (c == '!') {
		return LED_SEG_B | LED_SEG_C | LED_SEG_H;
	}
	else if (c == '?') {
		return LED_SEG_A | LED_SEG_B | LED_SEG_E | LED_SEG_G | LED_SEG_H;
	}
	else if (c == ',') {
		return LED_SEG_C;
	}
	else if (c == -1) {
		return LED_SEG_G | LED_SEG_H;
	}
	else {
		return 0; 
	}
}

/**
 * Print the given string to the LEDs.
 */
void led_set_string(char *s) {
	int slen = strlen(s);
	
	int8_t spos = slen - 1;
  
  	led_digit_data[8] = 0;
  	led_digit_data[9] = 0;
  
 	for (int i = 7; i >= 0; i--) {
		if (spos < 0) {
			led_digit_data[i] = 0;
    	}
	    else {
	    	uint8_t ch = s[spos];
	    	uint8_t ch_flags = 0;
	    	while (ch == '.' || ch == ':') {
	    		if (ch == '.') {
					ch_flags |= LED_SEG_H;
				}
				else if (ch == ':') {
					led_digit_data[i < 4 ? 8 : 9] |= LED_SEG_E | LED_SEG_F;
				}
	    		ch = s[--spos];
	    	}
			led_digit_data[i] = led_map_char(s[spos--]) | ch_flags;
	    }	
	}
}

/**
 * Print the given string to the LEDs.
 */
/*
void led_set_string(char *s) {
	uint8_t slen = strlen(s);
	
	uint8_t spos = 0;
	uint8_t lpos = 0;
	
	for (uint8_t i = 0; i < 10; i++) {
		led_digit_data[i] = 0;
	}
	
	while (spos < slen) {
		if (s[spos] == '.') {
			led_digit_data[lpos - 1] |= LED_SEG_H;
		}
		else if (s[spos] == ':') {
			led_digit_data[(lpos - 1) < 4 ? 8 : 9] |= LED_SEG_E | LED_SEG_F;
		}
		else if (lpos < 8) {
			led_digit_data[lpos] = led_map_char(s[spos]);
			lpos++;
		}
		spos++;
	}
}
*/
