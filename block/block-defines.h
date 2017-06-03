
#ifndef BLOCK_DEFINES_H
#define BLOCK_DEFINES_H

#define ACTIVATE_PHYSICAL_BLOCK0_TX UCSR0B |= _BV(UDRIE0)
#define ACTIVATE_PHYSICAL_BLOCK1_TX UCSR1B |= _BV(UDRIE1)


#define BLOCK_RING_BUFFER_SZ  175

#include "../../libs/crc16/crc16.c"
#include "../../libs/block-ringbuffer/block-ringbuffer.h"
#include "../../libs/block/block-com.h"


// Common macros for handling 16 bit variables
#ifndef UINT16_HIGH_BYTE
#define UINT16_HIGH_BYTE(a)  ((a)>>8)
#endif

#ifndef UINT16_LOW_BYTE
#define UINT16_LOW_BYTE(a)  ((a) & 0xFF)
#endif

#define min(a,b)  ((a)<(b)?(a):(b))

#define UBRRn_BAUD_DIVISOR(baud) ((F_CPU+(baud)*8l)/((baud)*16l)-1)

#define UART_BLOCK_BAUD 28800


#endif