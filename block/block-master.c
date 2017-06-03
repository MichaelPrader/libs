
#ifndef BLOCK_DEFINES_H
#include "block-defines.h"
#endif

BlockRingBuffer BlockRXBuffer[N_BLOCKINTERFACES];
BlockRingBuffer BlockTXBuffer[N_BLOCKINTERFACES];


ISR(USART0_UDRE_vect)
{
    uint8_t ref;
    
    ref = ObjPhysicalBlockInterface[0].Nx_ReferencedLogicalBlock;
    
	if (BlockRingBufferDepth(&(BlockTXBuffer[ref])))
		UDR0 = BlockRingBufferPop(&(BlockTXBuffer[ref]));
	else
		UCSR0B &= ~_BV(UDRIE0);
		
}

ISR(USART0_RX_vect)
{
	uint8_t status = UCSR0A, data = UDR0;
	uint8_t ref;
    
    ref = ObjPhysicalBlockInterface[0].Nx_ReferencedLogicalBlock;
	
	// Framing errors and other crap.  Throw it out
	if (status & (_BV(FE0) | _BV(DOR0) | _BV(UPE0) ))
	{
		data = END;
	}
	
//	if (data == END) BlockRXBuffer[ref].bufferState += 1;        // TODO in other files ??

	BlockRingBufferPush(&(BlockRXBuffer[ref]), data);
	
}

#if (N_PHYSICAL_BLOCK_INTERFACES >= 2)
ISR(USART1_UDRE_vect)
{
    uint8_t ref;
    
    ref = ObjPhysicalBlockInterface[1].Nx_ReferencedLogicalBlock;
	
	if (BlockRingBufferDepth(&(BlockTXBuffer[ref])))
		UDR1 = BlockRingBufferPop(&(BlockTXBuffer[ref]));
	else
		UCSR1B &= ~_BV(UDRIE1);
}


ISR(USART1_RX_vect)
{
	uint8_t status = UCSR1A, data = UDR1;
	uint8_t ref;
    
    ref = ObjPhysicalBlockInterface[1].Nx_ReferencedLogicalBlock;

	// Framing errors and other crap.  Throw it out
	if (status & (_BV(FE1) | _BV(DOR1) | _BV(UPE1) ))
    {
    	data = END;
    }
    
	//if (data == END) BlockRXBuffer[ref].bufferState += 1;

	BlockRingBufferPush(&(BlockRXBuffer[ref]), data);
}
#endif

void BlockUartInitialize(void)
{
    // Setting is 8-N-1

	//UCSR0A
	UBRR0 = UBRRn_BAUD_DIVISOR(UART_BLOCK_BAUD);
//	UCSR0A = _BV(U2X0);
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
	UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
	
	
#if (N_PHYSICAL_BLOCK_INTERFACES >= 2)
		//UCSR1A
	UBRR1 = UBRRn_BAUD_DIVISOR(UART_BLOCK_BAUD);
//	UCSR1A = _BV(U2X1);
	UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);
	UCSR1B = _BV(RXEN1) | _BV(TXEN1) | _BV(RXCIE1);
	
#endif

}

