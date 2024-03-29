#include "joybus.hpp"
//for serial debugging 
#include <time.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include <cstring>

#include "structsAndEnums.h"
// ^ from PhobGCC-SW/PhobGCC/common/

#define CSN_PIN 5
#define CE_PIN 28

const int _pinTX  = 27;
const int _pinLED = 25;

Buttons _btn;

GCReport __no_inline_not_in_flash_func(buttonsToGCReport)() {
	GCReport report = {
		.a       = _btn.A,
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
		
	};
	return report;
}

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
static void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg | 0x20; // bitwise OR mask ; right nibble will be unaffected, left nibble will get prefix 001X
    buf[1] = data;
    cs_select();

    spi_write_blocking(spi0, buf, 2);
    cs_deselect();
    //sleep_ms(10); NO SLEEP ALLOWED!
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

static void read_register(uint8_t reg, uint8_t *buf) {
    cs_select();
    spi_write_blocking(spi0, &reg, 1);
	asm volatile("nop \n nop \n nop");
    spi_read_blocking(spi0, 0, buf, 1);
    cs_deselect();
}

static void read_rx_payload( uint8_t *read_buffer ) {
    
    uint8_t READRX = 0b01100001;
	cs_select();
	spi_write_blocking(spi0, &READRX, 1);
    spi_read_blocking(spi0, 0, read_buffer, 8);
    cs_deselect();

}

void memory_dump_to_screen() {
	for (int i = 0; i < 32; i++) {
		uint8_t register_map_id = i; // 8 bits long now.
		uint8_t reg_contents[1];
		read_register(register_map_id, reg_contents);

		printf("\n --R %02X : %08b", register_map_id, reg_contents[0]);
	}	
}

void setup_registers() {
	// 00 is CONFIG. only important Set is POWER DOWN and RX
	// actually lets also mask certain interrupts for debugging purposes.
	write_register(0x00, 0b00111001);
	// 01 : disable auto-ack (waste of time)
	write_register(0x01, 0b00000000);
	// 02 : rx addresses. pipe 0 only (?)
	write_register(0x02, 0b00000001);
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
	// 0A..0F are RX_ADDR for pipes. we set up 0A for data pipe zero.
	write_register_5bytes(0x0A, 0x4A, 0x41, 0x4B, 0x4B, 0x4B);
	// 11..16 are for RX payload tracking
	// ACTUALLY 11 sets the RX-pipe0 payload width!!!! =8 bytes, i hope
	write_register(0x11, 0b00001000);

	// 17 is read-only.
	// (reserved block for payloads...)
	// 1C : dynamic payloads... NOTE last option here is enable-force-no-ack. should be irrelevant but?
	write_register(0x1C, 0b00000000);
	// 1D: features. 
	// trying "enable w-tx-payload-noack".
	write_register(0x1D, 0b00000001);
	//flush_tx();
}

void second_core() {
	// this is normally where calibration, readSticks(), and processButtons() goes.
	while (true)
	{
		sleep_us(17); //idk, relaxation time.
		// repeatedly read for new packets
		//gpio_put(6, _btn.S);
		uint8_t status1;
		uint8_t NOP = 0xFF;
		cs_select();
		spi_write_read_blocking(spi0, &NOP, &status1, 1);
		cs_deselect();
		
		if ((status1 & 0x40) == 0x40) 
		{
			//printf("\n received one!");
			//sleep_ms(400);
			uint8_t read_buffer[8];
			read_rx_payload(read_buffer);
			write_register(0x07, 0b01110000); // clear interrupts
			
			GCReport destinationreport;
   			memcpy(&destinationreport, read_buffer , 8); 

   			_btn.A      =destinationreport.a;
			_btn.B      =destinationreport.b;
			_btn.X      =destinationreport.x;
			_btn.Y      =destinationreport.y;
			_btn.S      =destinationreport.start;
			//_btn.orig   =0;
			//_btn.errL   =0;
			//_btn.errS   =0;
			_btn.Dl     =destinationreport.dLeft;
			_btn.Dr     =destinationreport.dRight;
			_btn.Dd     =destinationreport.dDown;
			_btn.Du     =destinationreport.dUp;
			_btn.Z      =destinationreport.z;
			_btn.R      =destinationreport.r;
			_btn.L      =destinationreport.l;
			//_btn.high   =1;
			_btn.Ax     =destinationreport.xStick+1;
			_btn.Ay     =destinationreport.yStick+1;
			_btn.Cx     =destinationreport.cxStick+1;
			_btn.Cy     =destinationreport.cyStick+1;
			_btn.La     =destinationreport.analogL;
			_btn.Ra     =destinationreport.analogR;

		}
		else
		{
			//debug code used to go here


		}
		// packet has not arrived yet.
		// right now, just .. repeatedly check immediately?


	}
}

int main() {
	// setup for usb serial comms via printf()
    stdio_init_all();

	gpio_init(_pinLED);
	gpio_set_dir(_pinLED, GPIO_OUT);
	gpio_init(CE_PIN);
	gpio_set_dir(CE_PIN, GPIO_OUT);
	gpio_init(CSN_PIN);
    gpio_set_dir(CSN_PIN, GPIO_OUT);
    //gpio_init(6);
    //gpio_set_dir(6, GPIO_OUT); //latency measurement.

	spi_init(spi0, 10'000 * 1000); // try the full 10MHz? datasheet says itll be compliant
	gpio_set_function(2, GPIO_FUNC_SPI);
    gpio_set_function(3, GPIO_FUNC_SPI);
    gpio_set_function(4, GPIO_FUNC_SPI);

    gpio_put(CSN_PIN, 1);
    // do setup tasks...
    setup_registers();
    // power on receiver
    write_register(0x00, 0b00111011);
    // turn on chip-enable
    gpio_put(CE_PIN, 1);
    sleep_ms(300); // warmup? stabilize? idk
    // this "core" will only handle speaking to the console.
    // "second_core" will be for wireless comms (equivalent to stick-reading) 

    multicore_launch_core1(second_core);
    enterMode(_pinTX, buttonsToGCReport);

}