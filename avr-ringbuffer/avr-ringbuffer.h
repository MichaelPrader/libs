#ifndef _AVR_RINGBUFFER_H_
#define _AVR_RINGBUFFER_H_

#ifndef RING_BUFFER_SZ
#define RING_BUFFER_SZ  128
#endif

#ifndef ROLLOVER
#define ROLLOVER( x, max )	x = ++x >= max ? 0 : x
					// count up and wrap around
#endif
 
typedef struct
{
	volatile uint8_t headIdx;
	volatile uint8_t tailIdx;
	volatile uint8_t bufferData[RING_BUFFER_SZ];
} RingBuffer;


void ringBufferInitialize(RingBuffer* r)
{
	r->headIdx = r->tailIdx = 0;
}

uint8_t ringBufferReady( RingBuffer *r )
{
  uint8_t i = r->headIdx;

  ROLLOVER( i, RING_BUFFER_SZ );
  return (r->tailIdx ^ i);		// 0 = busy
}


uint8_t ringBufferDepth(RingBuffer* r)
{
	return((r->headIdx - r->tailIdx ) % RING_BUFFER_SZ);
}

void ringBufferPush(RingBuffer* r, uint8_t data)
{
	while (!(ringBufferReady(r)));
	r->bufferData[r->headIdx] = data;
	ROLLOVER( r->headIdx, RING_BUFFER_SZ );
}

uint8_t ringBufferPop(RingBuffer* r)
{
	uint8_t data = r->bufferData[r->tailIdx];
	ROLLOVER( r->tailIdx, RING_BUFFER_SZ);
	return(data);
}





#endif