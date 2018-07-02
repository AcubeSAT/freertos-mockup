#ifndef TWI_H_
#define TWI_H_

#include <stm32f10x.h>

extern void TWIInit(void); //Initialize the I2C interface
extern void TWIStart(void); //Send a start signal
extern void TWIStop(void); //Send a stop signal
extern void TWIWrite(uint8_t u8data); //Send 8 bits of data
extern uint8_t TWIReadACK(void); //Check if you received the acknowledgment from the other device
extern uint8_t TWIReadNACK(void); //Expect no acknowledgment
extern void TWISendAddr(uint8_t addr, uint8_t tr_dir);

extern uint8_t TWIReadByte(uint8_t dev_addr, uint8_t registe); //Read a single byte from the slave
extern void TWIReadBytes(uint8_t dev_addr, uint8_t registe, uint8_t *byte_read, uint8_t byte_count); //Read many bytes at once
extern void TWIWriteByte(uint8_t dev_addr, uint8_t command_register, uint8_t command); //Write a byte from the slave
//A function to write many bytes will be implemented soon

#endif