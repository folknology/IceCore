#pragma once

#include <stdint.h>
#include "mystorm.h"
#include "BUI.h"
#include "FPGA.h"
#include "spi.h"

class Flash : public Spi {
	uint32_t NBYTES;
	uint32_t addr;
	uint32_t block;
	uint8_t state;
	uint8_t pagebuf[256];
	uint32_t hd;
	uint32_t nbytes;
	BIUI& Bui;
	Fpga& Ice40;
	union SIG sig =  {0x7E, 0xAA, 0x99, 0x7E} ;

	SPI_HandleTypeDef *spi;
	static const uint8_t WPGE = 0x02;
	static const uint8_t RPGE = 0x03;
	static const uint8_t STATUS = 0x05;
	static const uint8_t WEN = 0x06;
	static const uint8_t WAKEUP = 0xAB;
	static const uint8_t ERASE32 = 0x52;
	static const uint8_t ERASE64 = 0xD8;
	static const uint8_t ERASEBLK = 0xC7;

	public:
	Flash(SPI_HandleTypeDef *hspi, uint32_t img_size, BIUI& bui, Fpga& ice40);
	uint8_t erase(void);
	uint8_t erase(uint16_t size);
	uint8_t status(uint8_t timeout);
	uint8_t write(uint8_t *p, uint32_t len); // make an SPI class for this
	uint8_t read(uint8_t *p, uint32_t len); // make an SPI class for this
	// uint8_t write(uint8_t *addr, uint8_t *data, uint32_t len); // make an SPI class for this
	uint8_t write_page(uint8_t *data);
	uint8_t write_byte(uint8_t *data);
	uint8_t write_read(uint8_t *tx, uint8_t *rx, uint32_t len);
	uint8_t erase_write(uint16_t esize);
	uint8_t stream(uint8_t *data, uint32_t len);
};
