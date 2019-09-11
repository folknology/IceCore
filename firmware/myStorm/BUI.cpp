#include "BUI.h"

BIUI::BIUI(uint8_t status, uint8_t mode, uint8_t dim){
	nstatus = status;
	nmode = mode;
	recipricol = dim;
	count = recipricol;
}

/**
 * @brief Log error
 * 
 * @param error error number
 * @return int current total of
 */
int BIUI::error(int error){
	err = error;
	errors += err ? 1 : 0;
	return errors;
}

/**
 * @brief writes errors to buf
 * 
 * @param buf buffer
 * @param len buffer length
 * @return uint8_t chars written
 */
uint8_t BIUI::error_report(char *buf, int len){
	buf[0] = '0';
	buf[1] = 'x';
	buf[2] = TO_HEX(((errors & 0xF0) >> 4));
	buf[3] = TO_HEX((errors & 0x0F));
	buf[4] = '-';
	buf[5] = 'E';
	buf[6] = 'R';
	buf[7] = 'R';
	buf[8] = 'S';
	buf[9] = '\n';
	buf[10] = '\0';

	return 6;
}

/**
 * @brief Mode toggle
 * 
 * @return uint8_t current mode
 */
uint8_t BIUI::mode_toggle(void){
	nmode = nmode ? 0 :1;
	HAL_GPIO_WritePin(GPIOB, MODE_LED_Pin, (GPIO_PinState)nmode);
	return nmode;
}

/**
 * @brief Modulate staatus indicator
 * 
 */
void BIUI::modulate_status(void){
	if(!count--) {
		if(!nstatus) 
			HAL_GPIO_WritePin(GPIOB, STATUS_LED_Pin, (GPIO_PinState)0);
		count = recipricol;
	} else
		HAL_GPIO_WritePin(GPIOB, STATUS_LED_Pin, (GPIO_PinState)1);
}

/**
 * @brief Set status
 * 
 * @param state status state
 * @return uint8_t current state
 */
uint8_t BIUI::set_status(uint8_t state){
	nstatus = state;
	HAL_GPIO_WritePin(GPIOB, STATUS_LED_Pin, (GPIO_PinState)nstatus);
	return nstatus;
}

/**
 * @brief set mode
 * 
 * @param state mmode state
 * @return uint8_t current mode
 */
uint8_t BIUI::set_mode(uint8_t state){
	nmode = state;
	HAL_GPIO_WritePin(GPIOB, MODE_LED_Pin, (GPIO_PinState)nmode);
	return nmode;
}