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

#ifdef __cplusplus
 extern "C" {
#endif

#include "mystorm.h"
#include "Flash.h"
#include "usbd_cdc_if.h"
#include "stm32f7xx_hal.h"
#include "errno.h"

extern SPI_HandleTypeDef hspi3;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;

static int mode = 0;

uint32_t rxi,rxo;
uint8_t  rxb[RX];

/* functions */
static void cdc_puts(const char *s);

/* Interrupts */
static int8_t usbcdc_rxcallback(uint8_t *data, uint32_t *len);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* Classses */
BIUI Bui(1, 1, 4);
Fpga Ice40(IMGSIZE, Bui);
Flash flash(&hspi3, IMGSIZE, Bui, Ice40);

/*
 * Setup function (called once at powerup)
 */
void setup(void)
{

	Bui.set_mode(1);

	// Initiate Ice40 boot from flash
	flash.Boot_Enable();
	Ice40.reset(FLASH1);
	HAL_Delay(1000);
	if(!gpio_ishigh(ICE40_CDONE)){
		Bui.error(ICE_ERROR);
		cdc_puts("Flash Boot Error\n");
	} else {
		Bui.set_status(0);
	}
	flash.Enable();

	cdc_puts(VER);
	cdc_puts("\n");
	cdc_puts("Setup done\n");

	USBD_Interface_fops_FS.Receive = &usbcdc_rxcallback;
	HAL_TIM_Base_Start_IT(&htim6);

	Bui.error(HAL_UART_Receive_IT(&huart1, (uint8_t *)(rxb + rxi), 1));


	Bui.error(USBD_CDC_ReceivePacket(&hUsbDeviceFS));
	// errors += err ? 1 : 0;
}


/*
 * Loop function (called repeatedly)
 *	- wait for the start of a bitstream to be received on uart1 or usbcdc
 *	- receive the bitstream and pass it to the ice40
 */
void
loop(void)
{
	//char buffer[16];
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

	if(gpio_ishigh(MODE_BOOT)) {
		mode = !Bui.mode_toggle();
		HAL_Delay(1000);
	}
}
/*
 * Write a string to usbcdc, and to uart1 if not detached,
 * adding an extra CR if it ends with LF
 */
static void cdc_puts(const char *s){
	char *p;

	for (p = (char*)s; *p; p++);
	Bui.error(CDC_Transmit_FS((uint8_t *)s, p - s));
	// errors += err ? 1 : 0;
	if (p > s && p[-1] == '\n')
		cdc_puts("\r");
}



/* Get flash id example flash coms */
uint8_t flash_id(char *buf, int len){
	int r, i;
	int l1 = len - 1;
	// Flash flash(&hspi3, IMGSIZE);
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

/** TODO remove this
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
}

/*
 * Interrupt callback when a packet has been read from usbcdc
 */
static int8_t usbcdc_rxcallback(uint8_t *data, uint32_t *len){

	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &data[0]);

	if(*len) {
		if(mode){
			if(!flash.stream(data,*len)) {
				Bui.error(HAL_UART_Transmit_DMA(&huart1, data, *len));
				return USBD_OK;
			}
			
		} else {
			if(!Ice40.stream(data, *len)){
				Bui.error(HAL_UART_Transmit_DMA(&huart1, data, *len));
				return USBD_OK;
			}
		}
	}

	Bui.error(USBD_CDC_ReceivePacket(&hUsbDeviceFS));
	// errors += err ? 1 : 0;

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
  Bui.error(USBD_CDC_ReceivePacket(&hUsbDeviceFS));
//   errors += err ? 1 : 0;
}


/**
  * @brief UART error callbacks
  * @param huart uart handle
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	Bui.error(huart->ErrorCode);
	// errors += err ? 1 : 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){	
  /* Prevent unused argument(s) compilation warning */
	UNUSED(huart);
  	rxi = (++rxi == RX) ? 0 : rxi;
	Bui.error(HAL_UART_Receive_IT(huart, (uint8_t *)(rxb + rxi), 1));
	// errors += err ? 1 : 0;
}

/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	uint32_t bs, bp;

	Bui.modulate_status();

	if(rxo != rxi){
		bs = (rxo > rxi) ? RX - rxo : rxi - rxo;
		bp = rxo;
		if(CDC_Transmit_FS(&rxb[bp], bs) == 0U)
			rxo = (rxo == RX) ? 0 : rxo + bs;
	}
}

#ifdef __cplusplus
}
#endif

