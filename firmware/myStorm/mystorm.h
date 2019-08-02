#define SPI3_MOSI_Pin GPIO_PIN_5
#define SPI3_MOSI_GPIO_Port GPIOB
#define SPI3_MISO_Pin GPIO_PIN_4
#define SPI3_MISO_GPIO_Port GPIOB
#define SPI3_SCK_Pin GPIO_PIN_3
#define SPI3_SCK_GPIO_Port GPIOB

#define IMGSIZE (135100 - 4) // sig = 4
#define RX 64
#define VER "<myStorm 0.601> "

/* GPIO function Macros */
#define gpio_low(pin)	HAL_GPIO_WritePin(pin##_GPIO_Port, pin##_Pin, GPIO_PIN_RESET)
#define gpio_high(pin)	HAL_GPIO_WritePin(pin##_GPIO_Port, pin##_Pin, GPIO_PIN_SET)
#define gpio_islow(pin)	(HAL_GPIO_ReadPin(pin##_GPIO_Port, pin##_Pin) == GPIO_PIN_RESET)
#define gpio_ishigh(pin)	(HAL_GPIO_ReadPin(pin##_GPIO_Port, pin##_Pin) == GPIO_PIN_SET)
#define gpio_toggle(pin)	HAL_GPIO_TogglePin(pin##_GPIO_Port, pin##_Pin)
#define status_led(STATUS) HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, STATUS)
#define status_led_high() HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET)
#define status_led_low() HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET)
#define status_led_toggle() HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin)
#define mode_led_high() HAL_GPIO_WritePin(MODE_LED_GPIO_Port, MODE_LED_Pin, GPIO_PIN_SET)
#define mode_led_low() HAL_GPIO_WritePin(MODE_LED_GPIO_Port, MODE_LED_Pin, GPIO_PIN_RESET)
#define mode_led_toggle() HAL_GPIO_TogglePin(MODE_LED_GPIO_Port, MODE_LED_Pin)
#define hold_flash() HAL_GPIO_WritePin(ICE40_HOLD_GPIO_Port, ICE40_HOLD_Pin, GPIO_PIN_RESET)
#define release_flash() HAL_GPIO_WritePin(ICE40_HOLD_GPIO_Port, ICE40_HOLD_Pin, GPIO_PIN_SET)
#define protect_flash() HAL_GPIO_WritePin(ICE40_WP_GPIO_Port, ICE40_WP_Pin, GPIO_PIN_RESET)
#define free_flash() HAL_GPIO_WritePin(ICE40_WP_GPIO_Port, ICE40_WP_Pin, GPIO_PIN_SET)

#define TO_HEX(i) (i <= 9 ? '0' + i : 'A' - 10 + i)

enum { OK, TIMEOUT, ICE_ERROR };

enum { 
	MCNTRL = 0, 
	FLASH0, 
	FLASH1
} SOURCE_IMAGE;

enum { DETECT, PROG} STATES;
// static uint8_t state = DETECT;
// static uint32_t nbytes;

union SIG {
	uint8_t b[4];
	uint32_t word;
};
