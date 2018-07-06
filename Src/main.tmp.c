#include "uart.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_rcc.h"

int main() {


	UART_Init(115200);
//
//
	while (1) {
		LL_USART_TransmitData8(UART_PORT, '5');
		UART_SendChar('a');
		UART_SendChar('\r');
		UART_SendChar('\n');
		UART_SendStr("Hello to the world of tomorrow\r\n");
	}
}
