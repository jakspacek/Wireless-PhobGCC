#include "joybus.hpp"
//for serial debugging 
#include <time.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#include <cstring>

#define CSN_PIN 5
#define CE_PIN 28

const int _pinTX  = 27;
const int _pinLED = 25;

GCReport __no_inline_not_in_flash_func(buttonsToGCReport)() {
	GCReport report = {
		/*.a       = _btn.A,
		.b       = _btn.B,
		.x       = _btn.X,
		.y       = _btn.Y,
		.start   = _btn.S,
		.pad0    = 0,
		.dLeft   = _btn.Dl,
		.dRight  = _btn.Dr,
		.dDown   = _btn.Dd,
		.dUp     = _btn.Du,
		.z       = _btn.Z,
		.r       = _btn.R,
		.l       = _btn.L,
		.pad1    = 1,
		.xStick  = _btn.Ax,
		.yStick  = _btn.Ay,
		.cxStick = _btn.Cx,
		.cyStick = _btn.Cy,
		.analogL = _btn.La,
		.analogR = _btn.Ra
		*/

		.a       = 1,
		.b       = 0,
		.x       = 0,
		.y       = 0,
		.start   = 0,
		.pad0    = 0,
		.dLeft   = 0,
		.dRight  = 0,
		.dDown   = 0,
		.dUp     = 0,
		.z       = 0,
		.r       = 0,
		.l       = 0,
		.pad1    = 1,
		.xStick  = 27,
		.yStick  = 27,
		.cxStick = 0,
		.cyStick = 0,
		.analogL = 11,
		.analogR = 11
	};
	return report;
}

//RF24 radio(28, 5);
//SPI spi;

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(CSN_PIN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}
static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}
static void flush_tx() {
	// FLUSH_TX
	// dont forget to change this to RX in the copy!
	uint8_t flushcommand[] = {0b11100001};
	cs_select();
    spi_write_blocking(spi0, flushcommand, 1);
    cs_deselect();
}
static void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg | 0x20; // bitwise OR mask ; right nibble will be unaffected, left nibble will get prefix 001X
    buf[1] = data;
    cs_select();
    spi_write_blocking(spi0, buf, 2);
    cs_deselect();
}
static void write_register_5bytes(uint8_t reg, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5) {
	// Used for the address register (TX_ADDR, 0x10)
    uint8_t buf[6];
    buf[0] = reg | 0x20; // bitwise OR mask ; right nibble will be unaffected, left nibble will get prefix 001X
    buf[1] = b1;
    buf[2] = b2;
    buf[3] = b3;
    buf[4] = b4;
    buf[5] = b5;
    cs_select();
    spi_write_blocking(spi0, buf, 6);
    cs_deselect();
}

static void write_tx_payload(uint8_t *payload) {
    uint8_t writebuf[9];
    writebuf[0] = 0b10100000;
    //writebuf[0] = 0b10110000; // trying the force-no-ack, in case thats the issue?
    memcpy(&writebuf[1],&payload[0],8);

    cs_select();
    spi_write_blocking(spi0, writebuf, 9);
    cs_deselect();
    printf("\nDEBUG: here's what the tx writebuf looks like... %b,\n%b,%b,%b,%b,\n%b,%b,%b,%b", writebuf[0],writebuf[1],writebuf[2],writebuf[3],writebuf[4],writebuf[5],writebuf[6],writebuf[7],writebuf[8]);
}	

static void read_register(uint8_t reg, uint8_t *buf) {
    cs_select();
    spi_write_blocking(spi0, &reg, 1);
	asm volatile("nop \n nop \n nop");
    spi_read_blocking(spi0, 0, buf, 1);
    cs_deselect();
}

void memory_dump_to_screen() {
	for (int i = 0; i < 30; i++) {
		uint8_t register_map_id = i; // 8 bits long now.
		uint8_t reg_contents[1];
		read_register(register_map_id, reg_contents);

		printf("\n --R %02X : %08b", register_map_id, reg_contents[0]);
	}
}

void setup_registers() {
	// 00 is CONFIG. only important Set is POWER DOWN and TX
	// actually for debug lets only allow tx_ds to go thru.
	write_register(0x00, 0b01011000);
	// 01 : disable auto-ack (waste of time)
	write_register(0x01, 0b00000000);
	// 02 : rx addresses. disable all
	write_register(0x02, 0b00000000);
	// 03 is Address Widths. not super relevant
	write_register(0x03, 0b00000001);
	// 04 is autoretransmit. disable
	write_register(0x04, 0b00000000);
	// 05 is Channel. do more work here later
	write_register(0x05, 0b00011111);
	// 06 is "Rf setup register" 
	// ** sets data rate. trying 1mbps first (middle)
	// ** sets tx power. trying lowest first
	write_register(0x06, 0b00000000);
	// 07 is STATUS. just "clear" the interrupt flags (?) **
	write_register(0x07, 0b01110000);
	// 08 read only
	// 09 read only
	// 0A..0F are RX_ADDR for pipes. ** may need to set up pipe0 for retransmit... even though it is turned off.
	// 10 : TX_ADDR
	write_register_5bytes(0x10, 0x4A, 0x41, 0x4B, 0x4B, 0x4B);
	// 11..16 are for RX payload tracking
	// 17 is read-only.
	// (reserved block for payloads...)
	// 1C : dynamic payloads... NOTE last option here is enable-force-no-ack. should be irrelevant but?
	write_register(0x1C, 0b00000000);
	// 1D: features. 
	// trying "enable w-tx-payload-noack".
	write_register(0x1D, 0b00000001);
	flush_tx();

}

int main() {
	// setup for usb serial comms via printf()
    stdio_init_all();

	gpio_init(_pinLED);
	gpio_set_dir(_pinLED, GPIO_OUT);

	spi_init(spi0, 4'000 * 1000); // try the full 10MHz? datasheet says itll be compliant
	gpio_set_function(2, GPIO_FUNC_SPI);
    gpio_set_function(3, GPIO_FUNC_SPI);
    gpio_set_function(4, GPIO_FUNC_SPI);
    // chip-select
    // recall that it is active-low, so set to 1 during init 
    gpio_init(CSN_PIN);
    gpio_set_dir(CSN_PIN, GPIO_OUT);
    gpio_init(CE_PIN);
    gpio_set_dir(CE_PIN, GPIO_OUT);

    sleep_ms(1500);
    gpio_put(CSN_PIN, 1);
	memory_dump_to_screen();
	gpio_put(_pinLED, 1);
	sleep_ms(5000);
	
   	setup_registers();
   	memory_dump_to_screen();
   	gpio_put(_pinLED, 0);
   	sleep_ms(5000);

   	gpio_put(CSN_PIN, 0);

   	GCReport dummyreport = buttonsToGCReport();
   	uint8_t reportcopy[8];
   	memcpy(reportcopy , &dummyreport , 8); // confirmed this works!!!

   	// power up. ------------------
   	write_register(0x00, 0b01011010);

   	uint8_t pcount = 0;
   	// first draft at main tx loop.
   	while (true)
   	{
   		uint64_t t1 = time_us_64();
   		write_tx_payload(reportcopy);
   		gpio_put(CE_PIN, 1);
		sleep_us(15); // datasheet says 10 microseconds is minimum
		gpio_put(CE_PIN, 0);
   		int failures = 0;
   		sleep_us(250); // always fails the first check cuz transmission time! and spin-up time...
   		while (true)
   		{
   			uint8_t status1;
   			uint8_t NOP = 0xFF; 
   			cs_select();
   			spi_write_read_blocking(spi0, &NOP, &status1, 1);
			cs_deselect();
   			if ((status1 & 0x20) == 0x20) 
   			{
   				//printf("\n---SENT ONE---");
   				write_register(0x07, 0b01110000);
   				break;
   			} 
   			else 
   			{
			failures++;
   			printf("\nfailed? %d'th attempt", failures);
   			uint8_t status_reg;
   			read_register(0x07, &status_reg);
   			printf("\n--status reg says %08b", status_reg);
   			uint8_t txfifo_reg;
   			read_register(0x17, &txfifo_reg);
   			printf("\n--txfifo reg says %08b", txfifo_reg);
   			sleep_us(15);
   			printf("\nRETRANSMITTING...");
   			gpio_put(CE_PIN, 1);
			sleep_us(15); // datasheet says 10 microseconds is minimum
			gpio_put(CE_PIN, 0);
			}

   		}

   		uint64_t elapsed = time_us_64() - t1;
   		printf("\t\tflag set (transmission succeeded) after %llu microseconds.", elapsed);

   		// just delay any extra time. aiming for 1khz packet rate
   		uint64_t t_delay = (t1 + (uint64_t) 1000) - time_us_64();
   		printf("\n%llu ... ", t_delay);
   		sleep_us(t_delay);
   		// dummy transmission test pattern.
   		pcount++;
   		dummyreport.a = pcount < 128;
   		dummyreport.analogL = pcount;

	   	memcpy(reportcopy , &dummyreport , 8);

   	}

	//enterMode(_pinTX, buttonsToGCReport);
}