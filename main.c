
/**
 * BrewTroller PID Display
 * 
 * For ATmega168 running internal 8MHz oscillator
 *
 * Board Summary: 
 *   Board carries 4 dual 7 segment LED displays in red and green, PNP BJTs
 *   for handling common anode power and all segment outputs are multiplexed
 *   into an ULN2803 driven by AVR PORTD.
 *
 * Pin Config:
 *   B0		14		CA1		Common anode output for digit 1
 *   B1		15		CA2		Common anode output for digit 2
 *   B2		16		CA3		Common anode output for digit 3
 *   B3		17		MOSI	ICSP in
 *   B4		18		MISO	ICSP out
 *   B5		19		SCK		ICSP clock
 *   AVCC	20		NC		NC
 *   B6		9		CA4		Common anode output for digit 4
 *   B7		10		CA5		Common anode output for digit 5
 *   C0		23		CA6		Common anode output for digit 6
 *   C1		24		CA7		Common anode output for digit 7
 *   C2		25		CA8		Common anode output for digit 8
 *	 C3		26		ADDR	Input for address set (PCINT11)
 *	 C4		27		SDA		I2C/TWI data
 *   C5		28      SCL		I2C/TWI clock
 *   C6		1		RESET	Reset
 *   D0		2   	SEGB	Segment A drain
 *   D1		3     	SEGC	Segment B drain
 *   D2		4     	SEGA	Segment C drain
 *   D3		5     	SEGD	Segment D drain
 *   D4		6     	SEGE	Segment E drain
 *   D5		11     	SEGF	Segment F drain
 *   D6		12	   	SEGG	Segment G drain
 *   D7		13     	SEGH	Segment H drain
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
const char str_welcome[] PROGMEM = "btpdv3.3.2";
const char str_addr_02x[] PROGMEM = "addr  %02x";
const char str_float_format[] PROGMEM = "%5.1f";
const char str_dashes[] PROGMEM = "----";
const char str_save_02x[] PROGMEM = "save  %02x";
const char str_addr_blank[] PROGMEM = "addr    ";

volatile uint8_t twi_address;

volatile uint8_t led_digit_data[8];

char tmp_s[17];

// two line buffers used for the selective line display modes
char line_buf_1[6];
char line_buf_2[6];

volatile uint8_t config_state = CONFIG_OFF;
volatile uint16_t config_state_counter = 0;
volatile uint16_t config_state_counter2 = 0;
volatile uint8_t config_twi_address;

/**
 * The array below is used to control how long the timer dwells
 * on a particular digit. This is used to increase or decrease
 * the brightness of certain digits. Due to manufacturing
 * differences some colors or digits are naturally brighter
 * than others and this can be used to even up the display.
 * For a red top, green bottom display use:
 * 0, 0, 0, 0, 8, 8, 8, 8
 * For an all green display use:
 * 4, 4, 4, 4, 4, 4, 4, 4
 */
//uint8_t digit_delays[] = {4, 4, 4, 4, 4, 4, 4, 4};
uint8_t digit_delays[] = {0, 0, 0, 0, 8, 8, 8, 8};

uint8_t digit_delay;

uint8_t led_map_char(char c);
void led_set_string(char *s);
void twi_data_recieved(uint8_t* buf, int length);
void twi_status(uint8_t status);
uint8_t fix_port_d(uint8_t b);

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
	
	for (int i = 0; i < 8; i++) {
		led_digit_data[i] = 0xff;
	}
	
	DDRD = 0xFF;
	
	sei();
	
	_delay_ms(2000);
	sprintf_P(tmp_s, str_welcome);
	led_set_string(tmp_s);
	_delay_ms(2000);
	sprintf_P(tmp_s, str_addr_02x, twi_address);
	led_set_string(tmp_s);
	_delay_ms(2000);
	led_set_string("");
	
	// Initialize the two line buffers to all dashes
	sprintf_P(line_buf_1, str_dashes);
	sprintf_P(line_buf_2, str_dashes);
	
	// Initialize the twi interface
	twi_setAddress(twi_address);
	twi_attachSlaveRxEvent(twi_data_recieved);
	twi_init();
	
	// Turn on pull up resistor for the address set input
	PORTC |= _BV(3);
	// Enable the pin change detect on PCINT11
	PCMSK1 |= _BV(PCINT11);
	// Enable the pin change interrupt for PCINT11
	PCICR |= _BV(PCIE1);

	for (;;) {
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
				if ((buf[0] == ALL_8_DIGITS) || (buf[0] == UPPER_4_DIGITS)){
					raw = (int16_t*) &buf[2];
					sprintf_P(line_buf_1, str_float_format, (double) (*raw / 10.0));
				}
				if (buf[0] == LOWER_4_DIGITS){
					raw = (int16_t*) &buf[2];
					sprintf_P(line_buf_2, str_float_format, (double) (*raw / 10.0));
				}
				if (buf[0] == ALL_8_DIGITS){
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
}

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
	
	// Fire again in 1 ms
	TCNT1 = 0xffff - (F_CPU / 1000);
}

ISR(TIMER0_OVF_vect) {
	if (digit_delay) {
		digit_delay--;
		return;
	}

	// B0
	if (DDRC & _BV(2)) {
		DDRC &= ~(_BV(2));
		PORTD = fix_port_d(led_digit_data[0]);
		DDRB |= _BV(0);
		digit_delay = digit_delays[0];
	}
	// B1
	else if (DDRB & _BV(0)) {
		DDRB &= ~(_BV(0));
		PORTD = fix_port_d(led_digit_data[1]);
		DDRB |= _BV(1);
		digit_delay = digit_delays[1];
	}
	// B2
	else if (DDRB & _BV(1)) {
		DDRB &= ~(_BV(1));
		PORTD = fix_port_d(led_digit_data[2]);
		DDRB |= _BV(2);
		digit_delay = digit_delays[2];
	}
	// B6
	else if (DDRB & _BV(2)) {
		DDRB &= ~(_BV(2));
		PORTD = fix_port_d(led_digit_data[3]);
		DDRB |= _BV(6);
		digit_delay = digit_delays[3];
	}
	
	// B7
	else if (DDRB & _BV(6)) {
		DDRB &= ~(_BV(6));
		PORTD = fix_port_d(led_digit_data[4]);
		DDRB |= _BV(7);
		digit_delay = digit_delays[4];
	}
	// C0
	else if (DDRB & _BV(7)) {
		DDRB &= ~(_BV(7));
		PORTD = fix_port_d(led_digit_data[5]);
		DDRC |= _BV(0);
		digit_delay = digit_delays[5];
	}
	// C1
	else if (DDRC & _BV(0)) {
		DDRC &= ~(_BV(0));
		PORTD = fix_port_d(led_digit_data[6]);
		DDRC |= _BV(1);
		digit_delay = digit_delays[6];
	}
	// C2
	else {
		DDRC &= ~(_BV(1));
		PORTD = fix_port_d(led_digit_data[7]);
		DDRC |= _BV(2);
		digit_delay = digit_delays[7];
	}
	
	TCNT0 = 128;
}

uint8_t fix_port_d(uint8_t b) {
	b = (b << 1 | ((b >> 7) & 0x01));
	return b;
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
  
 	for (int i = 7; i >= 0; i--) {
		if (spos < 0) {
			led_digit_data[i] = 0;
    	}
	    else {
	    	if (s[spos] == '.') {
	    		spos--;
				led_digit_data[i] = led_map_char(s[spos--]);
				led_digit_data[i] |= LED_SEG_H;
			}
			else {
				led_digit_data[i] = led_map_char(s[spos--]);
			}
	    }	
	}
}

