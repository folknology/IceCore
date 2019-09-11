#include "FPGA.h"

Fpga::Fpga(uint32_t img_size, BIUI& bui) : Bui(bui) {
	NBYTES = img_size;
	state = DETECT;
}

/**
 * @brief Reset FPGA ready for programing or loading flash
 * 
 * @param bit_src Cold boot options : MCNTRL, FLASH0 or FLASH1
 * @return uint8_t 
 */
uint8_t Fpga::reset(uint8_t bit_src){
	int timeout = 100;

	gpio_low(ICE40_CRST);

	// Determine FPGA image source, config Flash
	switch(bit_src){
		case MCNTRL : // STM32 master prog, disable flash
			gpio_low(ICE40_SPI_CS);
			protect_flash();
			hold_flash();
			break;
		case FLASH0 : // CBSDEL=00 Flash
			gpio_high(ICE40_SPI_CS);
			hold_flash();
			protect_flash();
			break;
		case FLASH1 : // CBSDEL=01 Flash
			gpio_high(ICE40_SPI_CS);
			release_flash();
			protect_flash();
			break;
	}

	while(timeout--)
		if(gpio_ishigh(ICE40_CRST))
			return TIMEOUT;

	gpio_high(ICE40_CRST);

	// Neede for STM src
	timeout = 100;
	while (gpio_ishigh(ICE40_CDONE)) {
		if (--timeout == 0)
			return TIMEOUT;
	}
	HAL_Delay(2);

	free_flash();
	release_flash();
	return OK;
}

/**
 * @brief Config after programing bitfile
 * 
 * @return uint8_t : OK, ICE_ERROR
 */
uint8_t Fpga::config(void){
	uint8_t b = 0;

	for (int timeout = 100; !gpio_ishigh(ICE40_CDONE); timeout--) {
		if (timeout == 0) {
			//cdc_puts("CDONE not set\n");
			return ICE_ERROR;
		}
		write(&b, 1);
	}

	for (int i = 0; i < 7; i++)
		write(&b, 1);

	gpio_high(ICE40_SPI_CS);

	return OK;
}

/**
 * @brief Write len bytes
 * 
 * @param p Pointer to byte buf
 * @param len Number of bytes
 * @return uint8_t : HAL_OK
 */
uint8_t Fpga::write(uint8_t *p, uint32_t len){
	uint32_t i;
	uint8_t ret,b,d;

	ret = HAL_OK;
	for(i = 0; i < len; i++)
	{
		d = *p++;
		for(b = 0; b < 8; b++){
			if(d & 0x80) {
				HAL_GPIO_WritePin(GPIOB,SPI3_SCK_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,SPI3_MISO_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,SPI3_SCK_Pin,GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(GPIOB,SPI3_MISO_Pin | SPI3_SCK_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,SPI3_SCK_Pin,GPIO_PIN_SET);
			}
			d <<= 1;
		}
		gpio_high(SPI3_SCK);
	}
	return ret;
}

/**
 * @brief Stream bitfile from serial to FPGA
 * 
 * @param data Pointer to bytes
 * @param len number of bytes
 * @return uint8_t  1,0
 */
uint8_t Fpga::stream(uint8_t *data, uint32_t len){
	uint32_t *word;
	uint8_t *img;


	switch(state) {
		case DETECT: // Lets look for the Ice40 image or just pass bytes to Uart
			img = data + 4;
			word = (uint32_t *) img;
			if(*word == sig.word){ // We are inside the 1st 4 bytes Ice40 image
				nbytes = 0;
				Bui.set_status(1);
				Disable();
				int e = reset(MCNTRL);
				if (e) {
					Bui.error(e);
					Enable(); 
				} else { // Write bytes (assumes *len < NBYTES)
					nbytes += len - 4;
					write(img, len - 4);
					state = PROG;
				}
			} else
				return 0;
			break;
		case PROG: // We are now in the Ice40 image
			nbytes += len;
			write(data, len);
			break;
	}

	if(nbytes >= NBYTES) {
		int e = config();
		if(e) {
			Bui.error(e);
			Bui.set_status(1);
		} else
			Bui.set_status(0);
		Enable();
		state = DETECT;
	}

	return 1;
}