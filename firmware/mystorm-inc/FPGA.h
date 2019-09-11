#pragma once

#include <stdint.h>
#include "mystorm.h"
#include "BUI.h"
#include "spi.h"

class Fpga : public Spi {
	uint32_t NBYTES;
	uint8_t state;
	uint32_t nbytes;
	BIUI& Bui;
	union SIG sig =  {0x7E, 0xAA, 0x99, 0x7E} ;
	
	public:	
		Fpga(uint32_t img_size, BIUI& bui);
		uint8_t reset(uint8_t bit_src);
		uint8_t config(void);
		uint8_t write(uint8_t *p, uint32_t len);
		uint8_t stream(uint8_t *data, uint32_t len);
};
