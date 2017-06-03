#ifndef _BLOCK_RINGBUFFER_H_
#define _BLOCK_RINGBUFFER_H_

#ifndef END
#define END             0300    /* indicates end of packet 0xC0 */
#endif

/*#ifndef BLOCK_RING_BUFFER_SZ
#define BLOCK_RING_BUFFER_SZ  200
#endif
*/

#ifndef ROLLOVER
#define ROLLOVER( x, max )	x = ++x >= max ? 0 : x
					// count up and wrap around
#endif



#define BUFFER_NEARLY_FULL   2  // two bytes empty, one is usable
#define BUFFER_FULL          1  // one byte must stay empty, so we don't have to ckeck between "Full" and "Empty"

typedef struct
{
	volatile uint8_t headIdx;
	volatile uint8_t tailIdx;
	volatile uint8_t bufferData[BLOCK_RING_BUFFER_SZ];
	volatile uint8_t bufferState;
} BlockRingBuffer;


void BlockRingBufferInitialize(BlockRingBuffer* r)
{
	r->headIdx = r->tailIdx = 0;
	r->bufferState = 0;
}

/*
uint8_t BlockRingBufferReady( BlockRingBuffer *r )
{
  uint8_t y= r->headIdx;
  ROLLOVER( y, BLOCK_RING_BUFFER_SZ );
  
    //return (r->tailIdx ^ y);		// 0 = busy
    if (r->tailIdx ^ y)
    {
        // buffer is not full
        if (BlockRingBufferDepth(r)  == (BLOCK_RING_BUFFER_SZ - 2)) return (BUFFER_NEARLY_FULL); else return (BUFFER_READY);
    } else {
        return (BUFFER_FULL);		// 0 = busy
    }
}
*/

uint8_t BlockRingBufferDepth(BlockRingBuffer* r)
{
	//return((r->headIdx - r->tailIdx + BLOCK_RING_BUFFER_SZ) % BLOCK_RING_BUFFER_SZ); // this works only because C calculates in int type.
	if (r->headIdx >= r->tailIdx)
	{
	    return (r->headIdx - r->tailIdx);
	} else {
	    return (BLOCK_RING_BUFFER_SZ - (r->tailIdx - r->headIdx));
	}
}

uint8_t BlockRingBufferSpace(BlockRingBuffer *r)
{
    uint8_t tmp;
    
    if (r->headIdx >= r->tailIdx)
	{
        // buffer used
	    tmp = r->headIdx - r->tailIdx;
	} else {
	    // buffer used
	    tmp = BLOCK_RING_BUFFER_SZ - (r->tailIdx - r->headIdx);
	}
	
	// free space
	tmp = BLOCK_RING_BUFFER_SZ - tmp;
	
	return (tmp);
}

    
void BlockRingBufferPush(BlockRingBuffer* r, uint8_t data)
{
	
	if (BlockRingBufferSpace(r) == BUFFER_NEARLY_FULL)
	{
        r->bufferData[r->headIdx] = END;
        r->bufferState += 1;
	} else if (BlockRingBufferSpace(r) == BUFFER_FULL) {
	    return;
	} else {
	    r->bufferData[r->headIdx] = data;
	    if (data == END) r->bufferState += 1;
	}

    ROLLOVER( r->headIdx, BLOCK_RING_BUFFER_SZ);
	
	/*
	//while (!(BlockRingBufferReady(r)));
	
	if (BlockRingBufferReady(r) == BUFFER_FULL)
	{
	    return;
	} else if (BlockRingBufferReady(r) == BUFFER_NEARLY_FULL) {
	   r->bufferData[r->headIdx] = END;
	   r->bufferState += 1;
	} else
        r->bufferData[r->headIdx] = data;
    }
//	if (!(BlockRingBufferReady(r))) return;
//	r->bufferData[r->headIdx] = data;

	*/

}

uint8_t BlockRingBufferPop(BlockRingBuffer* r)
{
	uint8_t data;
	
	if (BlockRingBufferDepth(r))
	{
        data = r->bufferData[r->tailIdx];
        ROLLOVER( r->tailIdx, BLOCK_RING_BUFFER_SZ);
        return(data);
	} else {
        return(END);       // if there wasn't any data, this routine shouldn't have been called; if it happened, the safest thing we can do is to return an END
	}
}



#endif