#include "Flash.h"

Flash::Flash(SPI_HandleTypeDef *hspi, uint32_t img_size, BIUI& bui, Fpga& ice40) : Bui(bui), Ice40(ice40) {
	NBYTES = img_size;
	spi = hspi;
	state = DETECT;
}

/**
 * @brief Write bytes
 * 
 * @param p Pointer to bytes
 * @param len Number of bytes
 * @return uint8_t Hal_OK|BUSY|ERROR
 */
uint8_t Flash::write(uint8_t *p, uint32_t len){
	return HAL_SPI_Transmit(spi, p, len, HAL_UART_TIMEOUT_VALUE);
}

/**
 * @brief 
 * 
 * @param p Pointer to bytes
 * @param len bytes read
 * @return uint8_t Hal_OK|BUSY|ERROR
 */
uint8_t Flash::read(uint8_t *p, uint32_t len){
	return HAL_SPI_Receive(spi, p, len, HAL_UART_TIMEOUT_VALUE);
}

// uint8_t Flash::write(uint8_t *addr, uint8_t *data, uint32_t len){
// 	return HAL_SPI_Transmit(spi, data, len, HAL_UART_TIMEOUT_VALUE);
// }

/**
 * @brief Write page to flash
 * 
 * @param data bytes to write
 * @return uint8_t Hal_OK|BUSY|ERROR
 */
uint8_t Flash::write_page(uint8_t *data){
	uint8_t pre[4] = {WPGE, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)(addr)};
	HAL_SPI_Transmit(spi, pre, 4, HAL_UART_TIMEOUT_VALUE);
	return HAL_SPI_Transmit(spi, data, 256, HAL_UART_TIMEOUT_VALUE);
}

/**
 * @brief write a single byte to current address
 * 
 * @param data Byte to write
 * @return uint8_t Hal_OK|BUSY|ERROR
 */
uint8_t Flash::write_byte(uint8_t *data){
	uint8_t pre[5] = {WPGE, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)(addr), *data};
	return HAL_SPI_Transmit(spi, pre, 5, HAL_UART_TIMEOUT_VALUE);
}

/**
 * @brief Read and write
 * 
 * @param tx 
 * @param rx 
 * @param len 
 * @return uint8_t Hal_OK|BUSY|ERROR
 */
uint8_t Flash::write_read(uint8_t *tx, uint8_t *rx, uint32_t len){
	return HAL_SPI_TransmitReceive(spi, tx, rx, len, HAL_UART_TIMEOUT_VALUE);
}

/**
 * @brief Erase block
 * 
 * @return uint8_t Hal_OK|BUSY|ERROR
 */
uint8_t Flash::erase(void){
	uint8_t e = ERASEBLK;
	return HAL_SPI_Transmit(spi, &e, 1, HAL_UART_TIMEOUT_VALUE);
}

/**
 * @brief Erase 64/32 bytes
 * 
 * @param esize ERASE32, ERASE64
 * @return uint8_t Hal_OK|BUSY|ERROR
 */
uint8_t Flash::erase(uint16_t esize){
	uint8_t pre[4] = {(esize == 32) ? ERASE32 : ERASE64, (uint8_t)(block >> 16), (uint8_t)(block >> 8), (uint8_t)(block)};
	return HAL_SPI_Transmit(spi, pre, 4, HAL_UART_TIMEOUT_VALUE);
}

/**
 * @brief Erase and write to flash
 * 
 * @param esize ERASE32, ERASE64
 * @return uint8_t 0
 */
uint8_t Flash::erase_write(uint16_t esize){
	uint32_t end = addr + hd;
	uint8_t *page;
	uint16_t wsize;
	uint8_t wen = WEN;
	uint8_t rs, sts = STATUS;

	page = pagebuf;

	while(addr < end){
		wsize = (end - addr) > 255 ? 256 : end - addr;

		if((addr + wsize) >= block) { 

			gpio_low(ICE40_SPI_CS);
			write(&wen,1);
			gpio_high(ICE40_SPI_CS);

			gpio_low(ICE40_SPI_CS);
			erase(ERASE64);
			gpio_high(ICE40_SPI_CS);

			HAL_Delay(500); // 2200
			
			gpio_low(ICE40_SPI_CS);
			write(&sts,1);
			do {
				read(&rs,1);
			} while (rs & 0x01);//WEL bit?
			gpio_high(ICE40_SPI_CS);

			block += 0x10000;
		}


		if(wsize == 256){ // Page write

			gpio_low(ICE40_SPI_CS);
			write(&wen,1);
			gpio_high(ICE40_SPI_CS);

			gpio_low(ICE40_SPI_CS);
			write_page(page);
			gpio_high(ICE40_SPI_CS);

			HAL_Delay(2);

			gpio_low(ICE40_SPI_CS);
			write(&sts,1);
			do {
				read(&rs,1);
			} while (rs & 0x01);
			gpio_high(ICE40_SPI_CS);

			addr += wsize;
			page += wsize;

		} else { // remainder less than page size, byte writes
			for(uint8_t i = 0; i < wsize; i++){

				gpio_low(ICE40_SPI_CS);
				write(&wen,1);
				gpio_high(ICE40_SPI_CS);

				gpio_low(ICE40_SPI_CS);
				write_byte(page++);
				gpio_high(ICE40_SPI_CS);

				gpio_low(ICE40_SPI_CS);
				write(&sts,1);
				do {
					read(&rs,1);
				} while (rs & 0x01);
				gpio_high(ICE40_SPI_CS);

				addr++;
			}
		}
	
	}
	return 0;
}

/**
 * @brief Status of flash
 * 
 * @param timeout 
 * @return uint8_t state reg
 */
uint8_t Flash::status(uint8_t timeout){
	uint8_t rs,s= STATUS;
	if(!HAL_SPI_Transmit(spi, &s, 1, HAL_UART_TIMEOUT_VALUE))
		do {
			rs = HAL_SPI_Receive(spi, &rs, 1, HAL_UART_TIMEOUT_VALUE);

		} while(timeout-- && rs & 0x01);
	return rs;
}

/**
 * @brief Write a bitfile to flash from serial
 * 
 * @param data Pointer to bytes
 * @param len number of bytes
 * @return uint8_t 0.1
 */
uint8_t Flash::stream(uint8_t *data, uint32_t len){
	uint32_t *word;
	uint8_t *img;
	uint8_t cmd = WAKEUP;
	uint32_t end = nbytes + len;

	switch(state) {
		case DETECT: // Lets look for the Ice40 image or just pass bytes to Uart
			img = data + 4;
			word = (uint32_t *) img;
			if(*word == sig.word){ // We are inside the 1st 4 bytes Ice40 image
				nbytes = 0;
				Bui.set_status(1);
				 // Write bytes (assumes *len < NBYTES)
				nbytes += len- 4;
				addr = 0;
				block = 0;

				release_flash();
				free_flash();
				
				gpio_low(ICE40_SPI_CS);
				write(&cmd, 1);
				gpio_high(ICE40_SPI_CS);

				// Buffer prog data packet, assumes packet < 256
				for(hd = 0; hd < nbytes; hd++)
					pagebuf[hd] = *img++;

				state = PROG; // could return bytes writter here addr - 0 to indicate from comman caller how well we did, it could then get us to try again maybe?
			} else
				return 0;
			break;
		case PROG: // We are now in the Ice40 image
			img = data;
			
			while (nbytes < end ){
				nbytes++;
				pagebuf[hd++] = *img++;
				if( (nbytes >= NBYTES) || (hd == 256) ) { // Lets write flash page/remainder
					if(erase_write(ERASE64)) 
						Bui.error(1);
					hd = 0;
					if(nbytes >= NBYTES) 
						break;
				} 
			}
			break;
	}



	if(nbytes >= NBYTES) { // we cannot rely on NBYTES for general flash prog...
		Boot_Enable();
		int e = Ice40.reset(FLASH1);
		if(e) {
			Bui.error(e);
			Bui.set_status(1);
		} else 
			Bui.set_status(0);

		HAL_Delay(1000);

		if(!gpio_ishigh(ICE40_CDONE)){
			Bui.error(ICE_ERROR);
			Bui.set_status(1);
			//cdc_puts("Flash Boot Error\n");
		} else 
			Bui.set_status(0);

		Enable();
		state = DETECT;
		Bui.set_mode(0);
	}

	return 1;
}

