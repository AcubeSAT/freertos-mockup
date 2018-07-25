#include "stm32f1xx.h"
#include "string.h"


#define _UART_PORT 1

#if _UART_PORT == 1
	#define UART_PORT         USART1
	#define UART_TX_PIN       GPIO_PIN_9    // PA9  (USART1_TX)
	#define UART_RX_PIN       GPIO_PIN_10   // PA10 (USART1_RX)
	#define UART_GPIO_PORT_TX GPIOA
	#define UART_GPIO_PORT_RX UART_GPIO_PORT_TX
	#define UART_DMA_CHAN_TX  DMA1_Channel4
#elif _UART_PORT == 2
	#define UART_PORT         USART2
	#define UART_TX_PIN       GPIO_PIN_2    // PA2 (USART2_TX)
	#define UART_RX_PIN       GPIO_PIN_3    // PA3 (USART2_RX)
	#define UART_GPIO_PORT_TX GPIOA
	#define UART_GPIO_PORT_RX UART_GPIO_PORT_TX
	#define UART_DMA_CHAN_TX  DMA1_Channel7
#elif _UART_PORT == 3
	#define UART_PORT         USART3
	#define UART_TX_PIN       GPIO_PIN_10    // PB10 (USART3_TX)
	#define UART_RX_PIN       GPIO_PIN_11    // PB11 (USART3_RX)
	#define UART_GPIO_PORT_TX GPIOB
	#define UART_GPIO_PORT_RX UART_GPIO_PORT_TX
	#define UART_DMA_CHAN_TX  DMA1_Channel2
#elif _UART_PORT == 4
	#define UART_PORT         UART4
	#define UART_TX_PIN       GPIO_PIN_10    // PC10 (UART4_TX)
	#define UART_RX_PIN       GPIO_PIN_11    // PC11 (UART4_RX)
	#define UART_GPIO_PORT_TX GPIOC
	#define UART_GPIO_PORT_RX UART_GPIO_PORT_TX
#elif _UART_PORT == 5
	#define UART_PORT         UART5
	#define UART_TX_PIN       GPIO_PIN_12    // PC12 (UART5_TX)
	#define UART_RX_PIN       GPIO_PIN_2     // PD2  (UART5_RX)
	#define UART_GPIO_PORT_TX GPIOC
	#define UART_GPIO_PORT_RX GPIOD
#endif


#define HEX_CHARS      "0123456789ABCDEF"


void UART_Init(uint32_t baudrate);

void UART_SendChar(char ch);

void UART_SendInt(int32_t num);
void UART_SendInt0(int32_t num);
void UART_SendHex8(uint16_t num);
void UART_SendHex16(uint16_t num);
void UART_SendHex32(uint32_t num);

void UART_SendStr(char *str);

void UART_SendBuf(char *buf, uint16_t bufsize);
void UART_SendBufPrintable(char *buf, uint16_t bufsize, char subst);
void UART_SendBufHex(char *buf, uint16_t bufsize);
void UART_SendBufHexFancy(char *buf, uint16_t bufsize, uint8_t column_width, char subst);
