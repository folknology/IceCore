/*
 * Configure the ICE40 with new bitstreams:
 *	- repeatedly from usbcdc
*/
/*
Copyright (c) 2019, Alan Wood All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "main.h"
#include "usbd_cdc_if.h"
#include "stm32f7xx_hal.h"
#include "errno.h"
#include "mystorm.h"

#ifdef __cplusplus
 extern "C" {
#endif

extern SPI_HandleTypeDef hspi3;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;

// #define DMA_BYTES 2048
// #define DMAH (DMA_BYTES / 2)
// uint8_t rxdmabuf[DMA_BYTES];
// uint8_t urxdmabuf[64];
// uint8_t urxlen = 0;
// uint8_t dmai = 0;
// int dmao = DMAH - 1;

static int mode = 0;
static int err = 0;
uint8_t errors = 0;

uint32_t rxi,rxo;
uint8_t  rxb[RX];

/* functions */
static void cdc_puts(char *s);
void flash_SPI_Enable(void);
void flash_SPI_Disable(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
uint8_t flash_id(char *buf, int len);
uint8_t error_report(char *buf, int len);

/* Interrupts */
static int8_t usbcdc_rxcallback(uint8_t *data, uint32_t *len);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* Classses */
class Fpga {
	uint32_t NBYTES;
	uint8_t state;
	uint32_t nbytes;
	union SIG sig =  {0x7E, 0xAA, 0x99, 0x7E} ;
	
	public:	
		Fpga(uint32_t img_size);
		uint8_t reset(uint8_t bit_src);
		uint8_t config(void);
		uint8_t write(uint8_t *p, uint32_t len);
		uint8_t stream(uint8_t *data, uint32_t len);

};

class Flash {
	uint32_t NBYTES;
	uint32_t addr;
	uint32_t block;
	uint8_t state;
	uint32_t nbytes;
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
	Flash(SPI_HandleTypeDef *hspi, uint32_t img_size);
	uint8_t erase(void);
	uint8_t erase(uint16_t size);
	uint8_t status(uint8_t timeout);
	uint8_t write(uint8_t *p, uint32_t len); // make an SPI class for this
	uint8_t read(uint8_t *p, uint32_t len); // make an SPI class for this
	// uint8_t write(uint8_t *addr, uint8_t *data, uint32_t len); // make an SPI class for this
	uint8_t write_page(uint8_t *data);
	uint8_t write_byte(uint8_t *data);
	uint8_t write_read(uint8_t *tx, uint8_t *rx, uint32_t len);
	uint8_t erase_write(uint8_t *data, uint8_t len, uint16_t esize);
	uint8_t stream(uint8_t *data, uint32_t len);
};


/* global objects */
Fpga Ice40(IMGSIZE);
Flash flash(&hspi3, IMGSIZE);

/*
 * Setup function (called once at powerup)
 */
void
setup(void)
{

	mode_led_high();

	// Initiate Ice40 boot from flash
	flash_SPI_Enable();
	Ice40.reset(FLASH1);
	HAL_Delay(1000);
	if(!gpio_ishigh(ICE40_CDONE)){
		err = ICE_ERROR;
		cdc_puts("Flash Boot Error\n");
	} else {
		status_led_low();
	}
	//flash_SPI_Disable();

	cdc_puts(VER);
	cdc_puts("\n");
	cdc_puts("Setup done\n");

	USBD_Interface_fops_FS.Receive = &usbcdc_rxcallback;
	HAL_TIM_Base_Start_IT(&htim6);

	// if(err = HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxdmabuf, DMA_BYTES))
	// 	mode_led_low();

	if(err = HAL_UART_Receive_IT(&huart1, (uint8_t *)(rxb + rxi), 1));
		//mode_led_low();


	err = USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	errors += err ? 1 : 0;
}


/*
 * Loop function (called repeatedly)
 *	- wait for the start of a bitstream to be received on uart1 or usbcdc
 *	- receive the bitstream and pass it to the ice40
 */
void
loop(void)
{
	char buffer[16];
	// uint8_t b = 0;

	// if (err) {
	// 	status_led_toggle();
	// 	HAL_Delay(100);
	// 	if(gpio_ishigh(MODE_BOOT)) {
	// 		err = 0;
	// 		status_led_low();
	// 	}
	// }
	// if(err) {
	// 	error_report(buffer, 16);
	// 	cdc_puts(buffer);
	// 	err = 0;
	// }

	//cdc_puts("Waiting for USB serial\n");

	if(gpio_ishigh(MODE_BOOT)) {
		mode_led_toggle();
		mode = mode ? 0 : 1;
		
		// if(err) {
		// 	error_report(buffer, 16);
		// 	cdc_puts(buffer);
		// 	err = 0;
		// } else {
			// Eventually flash writing will go here, for now just report flash id
			// if(flash_id(buffer, 16))
			// 	cdc_puts(buffer);
		// }

		HAL_Delay(1000);
	}
}
/*
 * Write a string to usbcdc, and to uart1 if not detached,
 * adding an extra CR if it ends with LF
 */
static void cdc_puts(char *s){
	char *p;

	for (p = s; *p; p++);
	err = CDC_Transmit_FS((uint8_t *)s, p - s);
	errors += err ? 1 : 0;
	if (p > s && p[-1] == '\n')
		cdc_puts("\r");
}


void flash_SPI_Enable(void){

	HAL_GPIO_DeInit(GPIOB, SPI3_SCK_Pin|SPI3_MISO_Pin|SPI3_MOSI_Pin);

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	__HAL_RCC_SPI3_CLK_ENABLE();

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = SPI3_SCK_Pin|SPI3_MISO_Pin|SPI3_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void flash_SPI_Disable(void){

	__HAL_RCC_SPI3_CLK_DISABLE();

	HAL_GPIO_DeInit(GPIOB, SPI3_SCK_Pin|SPI3_MISO_Pin|SPI3_MOSI_Pin);

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = SPI3_MISO_Pin | SPI3_SCK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SPI3_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* Get flash id example flash coms */
uint8_t flash_id(char *buf, int len){
	int r, i;
	int l1 = len - 1;
	Flash flash(&hspi3, IMGSIZE);
	uint8_t uCommand = 0xAB;
	uint8_t response[3] = {0,0,0};

	release_flash();
	free_flash();
	//flash_SPI_Enable();

	gpio_low(ICE40_SPI_CS);
	flash.write(&uCommand,1);
	flash.read(response,3);
	gpio_high(ICE40_SPI_CS);

	uCommand = 0x9F;
	gpio_low(ICE40_SPI_CS);
	flash.write(&uCommand,1);
	flash.read(response,3);
	gpio_high(ICE40_SPI_CS);

	//create a Hex like string from response bytes
	for(r = 0, i = 0; r < l1; r+=5, i++){
		buf[r] = '0';
		buf[r+1] = 'x';
		buf[r+2] = TO_HEX(((response[i] & 0xF0) >> 4));
		buf[r+3] = TO_HEX((response[i] & 0x0F));
		buf[r+4] = ',';
	}
	buf[l1 -1] = '\n';
	buf[l1] = '\0';
	//flash_SPI_Disable();

	return len;
}

/* errors to char */
uint8_t error_report(char *buf, int len){
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
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  //HAL_GPIO_TogglePin(MODE_LED_GPIO_Port, MODE_LED_Pin);
}

/*
 * Interrupt callback when a packet has been read from usbcdc
 */
static int8_t usbcdc_rxcallback(uint8_t *data, uint32_t *len){

	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &data[0]);

	if(*len)
		if(mode){
			if(!flash.stream(data,*len)) {
				err = HAL_UART_Transmit_DMA(&huart1, data, *len);
				return USBD_OK;
			}
			
		} else {
			if(!Ice40.stream(data, *len)){
				// HAL_UART_Transmit(&huart1, data, *len, HAL_UART_TIMEOUT_VALUE);
				// mode_led_toggle();

				err = HAL_UART_Transmit_DMA(&huart1, data, *len);
				return USBD_OK;
				//if(temp) mode_led_low();
			}
		}

	err = USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	errors += err ? 1 : 0;

	return USBD_OK;
}

/**
  * @brief Tx Transfer completed callbacks
  * @param huart uart handle
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  //mode_led_toggle();
  err = USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  errors += err ? 1 : 0;
}


/**
  * @brief UART error callbacks
  * @param huart uart handle
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	err = huart->ErrorCode;
	errors += err ? 1 : 0;
	// cdc_puts("Uart error ");
	// cdc_puts('0' + huart->ErrorCode);
	// cdc_puts("\n");
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {	
//   /* Prevent unused argument(s) compilation warning */
//   UNUSED(huart);
//   CDC_Transmit_FS((unsigned char *)rxdmabuf, DMA_BYTES);
//   if(err = HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxdmabuf, DMA_BYTES))
// 		mode_led_low();
// }

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {	
//   /* Prevent unused argument(s) compilation warning */
//   UNUSED(huart);
//   int dmas = HAL_UART_Receive_DMA(&huart1, (uint8_t *)&rxdmabuf[dmai], DMAH);
//   CDC_Transmit_FS((unsigned char *)&rxdmabuf[dmao], DMAH);
//   if(dmas == HAL_OK) {
// 	  dmao = dmai;
// 	  dmai = dmai ? 0 : DMAH -1;
// }


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){	
  /* Prevent unused argument(s) compilation warning */
	UNUSED(huart);
  	rxi = (++rxi == RX) ? 0 : rxi;
	err = HAL_UART_Receive_IT(huart, (uint8_t *)(rxb + rxi), 1);
	errors += err ? 1 : 0;
}

/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	uint32_t bs, bp;

	//mode_led_toggle();

	if(rxo != rxi){
		bs = (rxo > rxi) ? RX - rxo : rxi - rxo;
		bp = rxo;
		if(CDC_Transmit_FS(&rxb[bp], bs) == 0U)
			rxo = (rxo == RX) ? 0 : rxo + bs;
	}
}


Fpga::Fpga(uint32_t img_size){
	NBYTES = img_size;
	state = DETECT;
}

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
	// certainly need this delay for STM as src, not sure about flash boot
	// timeout = 12800;
	// while(timeout--)
	// 	if(gpio_ishigh(ICE40_SPI_CS))
	// 		return TIMEOUT;
	HAL_Delay(2);

	free_flash();
	release_flash();
	return OK;
}

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

uint8_t Fpga::stream(uint8_t *data, uint32_t len){
	uint32_t *word;
	uint8_t *img;


	switch(state) {
		case DETECT: // Lets look for the Ice40 image or just pass bytes to Uart
			img = data + 4;
			word = (uint32_t *) img;
			if(*word == sig.word){ // We are inside the 1st 4 bytes Ice40 image
				nbytes = 0;
				status_led_high();
				flash_SPI_Disable();
				if (err = reset(MCNTRL)) 
					flash_SPI_Enable(); 
				else { // Write bytes (assumes *len < NBYTES)
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
		if(err = config())
			status_led_high();
		else
			status_led_low();
		flash_SPI_Enable();
		state = DETECT;
	}

	errors += err ? 1 : 0;

	return 1;
}


Flash::Flash(SPI_HandleTypeDef *hspi, uint32_t img_size){
	NBYTES = img_size;
	spi = hspi;
	state = DETECT;
}

uint8_t Flash::write(uint8_t *p, uint32_t len){
	return HAL_SPI_Transmit(spi, p, len, HAL_UART_TIMEOUT_VALUE);
}

uint8_t Flash::read(uint8_t *p, uint32_t len){
	return HAL_SPI_Receive(spi, p, len, HAL_UART_TIMEOUT_VALUE);
}

// uint8_t Flash::write(uint8_t *addr, uint8_t *data, uint32_t len){
// 	return HAL_SPI_Transmit(spi, data, len, HAL_UART_TIMEOUT_VALUE);
// }

uint8_t Flash::write_page(uint8_t *data){
	uint8_t pre[4] = {WPGE, addr >> 16, addr >> 8, addr};
	HAL_SPI_Transmit(spi, pre, 4, HAL_UART_TIMEOUT_VALUE);
	return HAL_SPI_Transmit(spi, data, 256, HAL_UART_TIMEOUT_VALUE);
}

uint8_t Flash::write_byte(uint8_t *data){
	uint8_t pre[5] = {WPGE, addr >> 16, addr >> 8, addr, *data};
	return HAL_SPI_Transmit(spi, pre, 5, HAL_UART_TIMEOUT_VALUE);
}

uint8_t Flash::write_read(uint8_t *tx, uint8_t *rx, uint32_t len){
	return HAL_SPI_TransmitReceive(spi, tx, rx, len, HAL_UART_TIMEOUT_VALUE);
}

uint8_t Flash::erase(void){
	uint8_t e = ERASEBLK;
	return HAL_SPI_Transmit(spi, &e, 1, HAL_UART_TIMEOUT_VALUE);
}

uint8_t Flash::erase(uint16_t esize){
	uint8_t pre[4] = {(esize == 32) ? ERASE32 : ERASE64, block >> 16, block >> 8, block};
	return HAL_SPI_Transmit(spi, pre, 4, HAL_UART_TIMEOUT_VALUE);
}

uint8_t Flash::erase_write(uint8_t *data, uint8_t len, uint16_t esize){
	uint32_t tail = addr + len;
	uint8_t *page;
	uint16_t wsize;
	uint8_t wen = WEN;
	uint8_t rs, sts = STATUS;
	// uint8_t pre[4] = {(esize == 32) ? ERASE32 : ERASE64, addr >> 16, addr >> 8, addr};

	page = data;

	while(addr < tail){
		wsize = (tail - addr) > 255 ? 256 : tail - addr;

		if((addr + wsize) >= block) { // too many times!

			gpio_low(ICE40_SPI_CS);
			write(&wen,1);
			gpio_high(ICE40_SPI_CS);

			gpio_low(ICE40_SPI_CS);
			erase(ERASE64);
			gpio_high(ICE40_SPI_CS);

			HAL_Delay(2200);
			
			// gpio_low(ICE40_SPI_CS);
			// write(&sts,1);
			// do {
			// 	read(&rs,1);
			// } while (rs & 0x01);//WEL bit?
			// gpio_high(ICE40_SPI_CS);

			mode_led_toggle();
			rs = 0;
			block += 0x10000;
			
			
		}
		gpio_low(ICE40_SPI_CS);
		write(&wen,1);
		gpio_high(ICE40_SPI_CS);

		if(wsize == 256){
			gpio_low(ICE40_SPI_CS);
			write_page(page);
			gpio_high(ICE40_SPI_CS);
			addr += wsize;
			page += wsize;
		} else { // remainder less than page size
			for(uint8_t i = 0; i < wsize; i++){
				gpio_low(ICE40_SPI_CS);
				write_byte(page++);
				gpio_high(ICE40_SPI_CS);
				addr++;
			}
		}
		
		
		

		HAL_Delay(2);

		// gpio_low(ICE40_SPI_CS);
		// write(&sts,1);
		// do {
		// 	read(&rs,1);
		// } while (rs & 0x01);
		// rs = 0;
		// gpio_high(ICE40_SPI_CS);	
	}
	return 0;
}

uint8_t Flash::status(uint8_t timeout){
	uint8_t rs,s= STATUS;
	if(!HAL_SPI_Transmit(spi, &s, 1, HAL_UART_TIMEOUT_VALUE))
		do {
			rs = HAL_SPI_Receive(spi, &rs, 1, HAL_UART_TIMEOUT_VALUE);

		} while(timeout-- && rs & 0x01);
	return rs;
}

uint8_t Flash::stream(uint8_t *data, uint32_t len){
	uint32_t *word;
	uint8_t *img;

	uint8_t cmd = WAKEUP;

	switch(state) {
		case DETECT: // Lets look for the Ice40 image or just pass bytes to Uart
			img = data + 4;
			word = (uint32_t *) img;
			if(*word == sig.word){ // We are inside the 1st 4 bytes Ice40 image
				nbytes = 0;
				status_led_high();
				//flash_SPI_Disable();
				if (err = Ice40.reset(MCNTRL)) ;
					//flash_SPI_Enable(); 
				else { // Write bytes (assumes *len < NBYTES)
					nbytes += len - 4;
					addr = 0;
					block = 0;

					release_flash();
					free_flash();
					
					gpio_low(ICE40_SPI_CS);
					write(&cmd, 1);
					gpio_high(ICE40_SPI_CS);

					if(erase_write(img, nbytes, ERASE64))
						err = 1;
				}
				state = PROG; // could return bytes writter here addr - 0 to indicate from comman caller how well we did, it could then get us to try again maybe?
			} else
				return 0;
			break;
		case PROG: // We are now in the Ice40 image
			nbytes += len;
			if(erase_write(data, len, ERASE64))
						err = 1;
			break;
	}



	if(nbytes >= NBYTES) { // we cannot rely on NBYTES for general flash prog...
		// if(err = Ice40.reset(FLASH1))
		// 	status_led_high();
		// else 
			status_led_low();
		//flash_SPI_Enable();
		state = DETECT;
		mode_led_low();
	}

	errors += err ? 1 : 0;

	return 1;
}

#ifdef __cplusplus
}
#endif

