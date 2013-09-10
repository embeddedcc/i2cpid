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
 * Bootloader:
 *   The program is the TWI bootloader for the board. It implements a super
 *   simple bootloader over the TWI interface. The bootloader has only three
 *   supported commands. 
 *   SYNC requests the bootloader start, or restart, and sets the address pointer to 0.
 *   WRITE_PAGE writes a page of data to the current address pointer, verifies the write,
 *     increments the pointer and returns the verification status.
 *   BOOT exits the boot loader and starts the user code.
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
#include <avr/boot.h>
#include <inttypes.h>

#include <compat/twi.h>

#define DEBUG

#define BL_WAIT_PERIOD 5000

#define BL_REQ_SYNC '?'
#define BL_RES_SYNC '!' 

#define BL_REQ_WRITE_BUF 'f'
#define BL_RES_WRITE_BUF 'F'

#define BL_REQ_WRITE_PAGE 'w'
#define BL_RES_WRITE_PAGE 'W'

#define BL_REQ_BOOT 'b'

#define LED_SEG_A				2
#define LED_SEG_B				4
#define LED_SEG_C				8
#define LED_SEG_D				16
#define LED_SEG_E				32
#define LED_SEG_F				64
#define LED_SEG_G				128
#define LED_SEG_H				1

#define LED_0 LED_SEG_A | LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_E | LED_SEG_F
#define LED_1 LED_SEG_B | LED_SEG_C
#define LED_2 LED_SEG_A | LED_SEG_B | LED_SEG_D | LED_SEG_E | LED_SEG_G
#define LED_3 LED_SEG_A | LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_G
#define LED_4 LED_SEG_B | LED_SEG_C | LED_SEG_F | LED_SEG_G
#define LED_5 LED_SEG_A | LED_SEG_C | LED_SEG_D | LED_SEG_F | LED_SEG_G
#define LED_6 LED_SEG_A | LED_SEG_C | LED_SEG_D | LED_SEG_E | LED_SEG_F | LED_SEG_G
#define LED_7 LED_SEG_A | LED_SEG_B | LED_SEG_C
#define LED_8 LED_SEG_A | LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_E | LED_SEG_F | LED_SEG_G
#define LED_9 LED_SEG_A | LED_SEG_B | LED_SEG_C | LED_SEG_F | LED_SEG_G

#define LED_A LED_SEG_A | LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_E | LED_SEG_G
#define LED_B LED_SEG_C | LED_SEG_D | LED_SEG_E | LED_SEG_F | LED_SEG_G
#define LED_C LED_SEG_D | LED_SEG_E | LED_SEG_G
#define LED_D LED_SEG_B | LED_SEG_C | LED_SEG_D | LED_SEG_E | LED_SEG_G
#define LED_E LED_SEG_A | LED_SEG_B | LED_SEG_D | LED_SEG_E | LED_SEG_F | LED_SEG_G
#define LED_F LED_SEG_A | LED_SEG_E | LED_SEG_F | LED_SEG_G

#define TWI_ADDRESS 0x1F

#define TWI_READY 0
#define TWI_MRX   1
#define TWI_MTX   2
#define TWI_SRX   3
#define TWI_STX   4

uint8_t twi_recv_buf[1 + 2 + 1 + 32];
uint8_t twi_recv_buf_pos = 0;

uint8_t twi_send_buf[32];
uint8_t twi_send_buf_pos = 0;
uint8_t twi_send_buf_len = 0;

uint8_t twi_state;
uint8_t twi_error;

uint8_t bl_synced;
uint16_t bl_countdown = BL_WAIT_PERIOD;

#ifdef DEBUG
uint8_t led_digit_data[8];
uint8_t led_digit;
uint8_t led_digit_delay;
uint8_t led_hex_font[] = { 
	LED_0, LED_1, LED_2, LED_3, 
	LED_4, LED_5, LED_6, LED_7, 
	LED_8, LED_9, LED_A, LED_B, 
	LED_C, LED_D, LED_E, LED_F
	};
#endif

static void (*app_start)(void) = 0x0000;
static void twi_recv_complete(void);
static void twi_poll(void);
static inline void twi_reply(uint8_t ack);
static void twi_stop(void);

static void led_advance(void);
static void led_write_byte(uint8_t position, uint8_t value);

static void bl_boot(void);

int main(void) {
	// Remap interrupt table to bootloader section
	MCUCR = (1<<IVCE);
	MCUCR = (1<<IVSEL);

	// Enable pullups on TWI lines
	PORTC |= (_BV(4) | _BV(5));
	
	// initialize twi prescaler and bit rate
	TWSR &= ~(_BV(TWPS0) | _BV(TWPS1));
	
	// set the slave address
	TWAR = (TWI_ADDRESS << 1);

    // enable twi module and acks
    TWCR = _BV(TWEN) | _BV(TWEA) | _BV(TWIE);

	// Set all of PORTD to outputs to drive the segments
	DDRD = 0xFF;
	
	// Timer 1 is set up as a millisecond counter and is used
	// to fire off various functions
	// Clock Select, No Prescaler
	TCCR1B = _BV(CS10);
	// Interrupt Enable
	TIMSK1 = _BV(TOIE1);
	
	sei();

	uint16_t loop_counter = 0;
	for (;;) {
		twi_poll();
		led_advance();
		
		loop_counter++;
		
		if ((loop_counter % 5000) < 2500) {
			led_digit_data[7] |= LED_SEG_H;
		}
		else {
			led_digit_data[7] &= ~LED_SEG_H;
		}
	}
}

ISR(TIMER1_OVF_vect) {
	// Fire again in 1 ms
	TCNT1 = 0xffff - (F_CPU / 1000);
}

static void bl_boot(void) {
	PORTD = 0;
	for (;;) {
	}
}

static void led_write_byte(uint8_t position, uint8_t value) {
#ifdef DEBUG
	led_digit_data[position] = led_hex_font[(value >> 4) & 0x0f];
	led_digit_data[position + 1] = led_hex_font[value & 0x0f];
#endif
}

static void led_advance(void) {
	//Anode drivers: B0,B1,B2,B6,B7,C0,C1,C2
#ifdef DEBUG	
	if (led_digit_delay) {
		led_digit_delay--;
		return;
	}
	
	DDRB &= ~(_BV(0) | _BV(1) | _BV(2) | _BV(6) | _BV(7));
	DDRC &= ~(_BV(0) | _BV(1) | _BV(2));

	if (led_digit > 3) {
		led_digit_delay = 10;
	}

	PORTD = led_digit_data[led_digit];
		
	if (led_digit < 5) {
		if (led_digit < 3) {
			DDRB |= (_BV(led_digit));
		}
		else {
			DDRB |= (_BV(led_digit) << 3);
		}
	}
	else {
		DDRC |= (_BV(led_digit) >> 5);
	}
	
	led_digit++;
	if (led_digit > 7) {
		led_digit = 0;
	}
#endif
}

static void twi_recv_complete(void) {
	uint8_t req = twi_recv_buf[0];
	twi_send_buf_pos = 0;
	if (req == BL_REQ_SYNC) {
		bl_synced = 1;
	
		twi_send_buf[0] = BL_RES_SYNC;
		twi_send_buf_len = 1;
	}
	else if (req == BL_REQ_WRITE_BUF) {
		uint16_t address = ((twi_recv_buf[1] << 8) | twi_recv_buf[2]);
		led_write_byte(0, (address >> 8) & 0xff);
		led_write_byte(2, (address) & 0xff);
		uint8_t length = twi_recv_buf[3];
		for (uint8_t i = 0; i < length; i += 2) {
			uint16_t word = (twi_recv_buf[4 + i] | (twi_recv_buf[4 + i + 1] << 8));
			boot_page_fill_safe(address + i, word);
		}
		twi_send_buf[0] = BL_RES_WRITE_BUF;
		twi_send_buf[1] = length;
		twi_send_buf_pos = 2;
	}
	else if (req == BL_REQ_WRITE_PAGE) {
		uint16_t address = ((twi_recv_buf[1] << 8) | twi_recv_buf[2]);
		led_write_byte(4, (address >> 8) & 0xff);
		led_write_byte(6, (address) & 0xff);
		boot_page_erase_safe(address);
		boot_page_write_safe(address);
		twi_send_buf[0] = BL_RES_WRITE_PAGE;
		twi_send_buf[1] = 1;
		twi_send_buf_pos = 2;
	}
	else if (req == BL_REQ_BOOT) {
		app_start();
	}
}

static void twi_poll() {
	switch(TW_STATUS){
    	// Slave Receiver
	    case TW_SR_SLA_ACK:   // addressed, returned ack
	    case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
		    // enter slave receiver mode
			twi_state = TWI_SRX;
			// reset the receive buffer and get ready for incoming data
			twi_recv_buf_pos = 0;
			// ack
      		twi_reply(1);
      		break;
    	case TW_SR_DATA_ACK:       // data received, returned ack
      		// read the byte from TWDR
      		twi_recv_buf[twi_recv_buf_pos++] = TWDR;
      		// ack
      		twi_reply(1);
      		break;
    	case TW_SR_STOP: // stop or repeated start condition received
    		twi_recv_complete();
			// ack future responses
			twi_reply(1);
			// leave slave receiver state
			twi_state = TWI_READY;
			break;
	    case TW_SR_DATA_NACK:       // data received, returned nack
			// nack back at master
			twi_reply(0);
			break;

		// Slave Transmitter
		case TW_ST_SLA_ACK:          // addressed, returned ack
		case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
			// enter slave transmitter mode
			twi_state = TWI_STX;
			// 
			// the send buffer is set up by the processing of any
			// commands and will be ready by time this is called
			//
			// transmit first byte from buffer, fall through
		case TW_ST_DATA_ACK: // byte sent, ack returned
			// copy data to output register
			TWDR = twi_send_buf[twi_send_buf_pos++];
			// if there is more to send, ack, otherwise nack
			if(twi_send_buf_pos < twi_send_buf_len){
				twi_reply(1);
			}
			else {
				twi_reply(0);
			}
			break;
	    case TW_ST_DATA_NACK: // received nack, we are done 
    	case TW_ST_LAST_DATA: // received ack, but we are done already!
			// ack future responses
			twi_reply(1);
			// leave slave receiver state
			twi_state = TWI_READY;
			break;

	    // All
    	case TW_NO_INFO:   // no state information
			break;
	    case TW_BUS_ERROR: // bus error, illegal stop/start
    		twi_error = TW_BUS_ERROR;
			twi_stop();
			break;
	}
}

static inline void twi_reply(uint8_t ack) {
  	// transmit master read ready signal, with or without ack
  	if (ack) {
    	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  	}
  	else {
	  	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  	}
}

static void twi_stop(void) {
	// send stop condition
  	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

  	// wait for stop condition to be exectued on bus
  	// TWINT is not set after a stop condition!
  	while(TWCR & _BV(TWSTO)){
    	continue;
  	}

  	// update twi state
  	twi_state = TWI_READY;
}

