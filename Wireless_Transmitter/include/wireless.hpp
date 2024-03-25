#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "comms/gcReport.hpp"
#include <cstring>

#define CSN_PIN__TX 14
#define CE_PIN__TX 13

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(CSN_PIN__TX, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}
static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(CSN_PIN__TX, 1);
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
    buf[0] = reg | 0x20; // bitwise OR mask ; right nibble will be unaffected, left nibble will get prefix XX1X
    buf[1] = data;
    cs_select();
    spi_write_blocking(spi0, buf, 2);
    cs_deselect();
}
static void write_register_5bytes(uint8_t reg, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5) {
	// Used for the address register (TX_ADDR, 0x10)
    uint8_t buf[6];
    buf[0] = reg | 0x20; // bitwise OR mask ; right nibble will be unaffected, left nibble will get prefix XX1X
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
    //printf("\nDEBUG: here's what the tx writebuf looks like... %b,\n%b,%b,%b,%b,\n%b,%b,%b,%b", writebuf[0],writebuf[1],writebuf[2],writebuf[3],writebuf[4],writebuf[5],writebuf[6],writebuf[7],writebuf[8]);
}	

static void read_register(uint8_t reg, uint8_t *buf) {
    cs_select();
    spi_write_blocking(spi0, &reg, 1);
	asm volatile("nop \n nop \n nop");
    spi_read_blocking(spi0, 0, buf, 1);
    cs_deselect();
}


static void setup_registers() {
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
	write_register(0x06, 0b00000110);
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

void memory_dump_to_screen() {
	for (int i = 0; i < 30; i++) {
		uint8_t register_map_id = i; // 8 bits long now.
		uint8_t reg_contents[1];
		read_register(register_map_id, reg_contents);

		printf("\n --R %02X : %08b", register_map_id, reg_contents[0]);
	}
}

void setupWireless()
{
	setup_registers();
	sleep_ms(50); // no good reason
	// power up. ------------------
   	write_register(0x00, 0b01011010);
}

void synchronousSubmitForWireless(Buttons btn)
{
	GCReport report = {
		.a       = btn.A,
		.b       = btn.B,
		.x       = btn.X,
		.y       = btn.Y,
		.start   = btn.S,
		.pad0    = 0,
		.dLeft   = btn.Dl,
		.dRight  = btn.Dr,
		.dDown   = btn.Dd,
		.dUp     = btn.Du,
		.z       = btn.Z,
		.r       = btn.R,
		.l       = btn.L,
		.pad1    = 1,
		.xStick  = btn.Ax,
		.yStick  = btn.Ay,
		.cxStick = btn.Cx,
		.cyStick = btn.Cy,
		.analogL = btn.La,
		.analogR = btn.Ra
	};

   	uint8_t reportcopy[8];
   	memcpy(reportcopy , &report , 8); 
	// oops, we actually want to send a "gcreport" not a button-struct

   	write_tx_payload(reportcopy);
	gpio_put(CE_PIN__TX, 1);	
	sleep_us(15); // datasheet says 10 microseconds is minimum
	gpio_put(CE_PIN__TX, 0);

}

void monitorTransmission(uint32_t counter)
{
	if ((counter * 41) < 250)
	{
		// don't bother checking because time-on-air for the packet is not done yet. [250us is an empirical value]
		// **LATER: debug to get a slightly more accurate number? this is from trial and error on the old rig right now.
		return;
	}

	uint8_t status1;
	uint8_t NOP = 0xFF; 
	cs_select();
	spi_write_read_blocking(spi0, &NOP, &status1, 1);
	cs_deselect();
	if ((status1 & 0x20) == 0x20) 
	{
		//printf("\n---SENT ONE---");
		write_register(0x07, 0b01110000);
		// I *think* doing this (clearing the tx-successful bit) is necessary for proper operation. 
	} 
	/*
	// retransmit code from debug testing. I don't think we ever want to retransmit here 
	// (but I also don't .. know why failures to trasmit payload occur in the first place.)

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
	*/
}