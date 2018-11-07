#include "uart.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_ll_usart.h"


void UART_Init(uint32_t baudrate) {
	GPIO_InitTypeDef GPIO;

	// U(S)ART init
#if _UART_PORT == 1
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
#elif _UART_PORT == 2
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
#elif _UART_PORT == 3
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART3_CLK_ENABLE();
#elif _UART_PORT == 4
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_UART4_CLK_ENABLE();
#elif _UART_PORT == 5
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_UART5_CLK_ENABLE();
#endif

	GPIO.Pin = UART_TX_PIN;
	GPIO.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO.Mode = GPIO_MODE_AF_PP; // TX as AF with Push-Pull
	HAL_GPIO_Init(UART_GPIO_PORT_TX, &GPIO);

	GPIO.Pin = UART_RX_PIN;
	GPIO.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO.Mode = GPIO_MODE_INPUT; // RX as in without pull-up
	HAL_GPIO_Init(UART_GPIO_PORT_RX, &GPIO);

	UART_HandleTypeDef UART;
	UART.Instance = UART_PORT;
	UART.Init.BaudRate = baudrate;
	UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UART.Init.Mode = UART_MODE_TX_RX;
	UART.Init.WordLength = UART_WORDLENGTH_8B;
	UART.Init.Parity = UART_PARITY_NONE;
	UART.Init.StopBits = UART_STOPBITS_1;
	HAL_UART_Init(&UART);
}

void UART_SendChar(char ch) {
	while (!LL_USART_IsActiveFlag_TC(UART_PORT)); // wait for "Transmission Complete" flag cleared
	LL_USART_TransmitData8(UART_PORT, ch);
	/* **************************************************** *
	 * The second while loops serves as guarantee that		*
	 * the data is transmitted through the channel.			*
	 * This is to prevent any data corruption.				*
	 * **************************************************** */
	while (!LL_USART_IsActiveFlag_TC(UART_PORT));
}

void UART_SendInt(int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;
	if (num < 0) {
		UART_SendChar('-');
		num *= -1;
	}
	do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
	for (i--; i >= 0; i--) UART_SendChar(str[i]);
}

void UART_SendInt0(int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;
	if (num < 0) {
		UART_SendChar('-');
		num *= -1;
	}
	if ((num < 10) && (num >= 0)) UART_SendChar('0');
	do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
	for (i--; i >= 0; i--) UART_SendChar(str[i]);
}

void UART_SendHex8(uint16_t num) {
	UART_SendChar(HEX_CHARS[(num >> 4)   % 0x10]);
	UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendHex16(uint16_t num) {
	uint8_t i;
	for (i = 12; i > 0; i -= 4) UART_SendChar(HEX_CHARS[(num >> i) % 0x10]);
	UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendHex32(uint32_t num) {
	uint8_t i;
	for (i = 28; i > 0; i -= 4)	UART_SendChar(HEX_CHARS[(num >> i) % 0x10]);
	UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendStr(char *str) {
	while (*str) UART_SendChar(*str++);
}

void UART_SendBuf(char *buf, uint16_t bufsize) {
	uint16_t i;
	for (i = 0; i < bufsize; i++) UART_SendChar(*buf++);
}

void UART_SendBufPrintable(char *buf, uint16_t bufsize, char subst) {
	uint16_t i;
	char ch;
	for (i = 0; i < bufsize; i++) {
		ch = *buf++;
		UART_SendChar(ch > 32 ? ch : subst);
	}
}

void UART_SendBufHex(char *buf, uint16_t bufsize) {
	uint16_t i;
	char ch;
	for (i = 0; i < bufsize; i++) {
		ch = *buf++;
		UART_SendChar(HEX_CHARS[(ch >> 4)   % 0x10]);
		UART_SendChar(HEX_CHARS[(ch & 0x0f) % 0x10]);
	}
}

void UART_SendBufHexFancy(char *buf, uint16_t bufsize, uint8_t column_width, char subst) {
	uint16_t i = 0,len,pos;
	char buffer[column_width];

	while (i < bufsize) {
		// Line number
		UART_SendHex16(i);
		UART_SendChar(':'); UART_SendChar(' '); // Faster and less code than UART_SendStr(": ");

		// Copy one line
		if (i+column_width >= bufsize) len = bufsize - i; else len = column_width;
		memcpy(buffer,&buf[i],len);

		// Hex data
		pos = 0;
		while (pos < len) UART_SendHex8(buffer[pos++]);
		UART_SendChar(' ');

		// Raw data
		pos = 0;
		do UART_SendChar(buffer[pos] > 32 ? buffer[pos] : subst); while (++pos < len);
		UART_SendChar('\n');

		i += len;
	}
}
