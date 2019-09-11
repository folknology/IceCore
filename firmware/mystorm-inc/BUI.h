#pragma once

#include <stdint.h>
#include "mystorm.h"

class BIUI {
	uint8_t err = 0;
	uint8_t errors = 0;
	uint8_t nstatus;
	uint8_t nmode;
	uint8_t recipricol;
	uint8_t count;

	public:
		BIUI(uint8_t status, uint8_t mode, uint8_t dim);
		int error(int error);
		uint8_t error_report(char *buf, int len);
		uint8_t mode_toggle(void);
		void modulate_status(void);
		uint8_t set_status(uint8_t state);
		uint8_t set_mode(uint8_t state);
};

