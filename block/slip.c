
/* SLIP special character codes
*/
#define END             0300    /* indicates end of packet 0xC0 */
#define ESC             0333    /* indicates byte stuffing 0xDB */
#define ESC_END         0334    /* ESC ESC_END means END data byte 0xDC */
#define ESC_ESC         0335    /* ESC ESC_ESC means ESC data byte 0xDD */

/* SEND_PACKET: sends a packet of length "len", starting at
* location "p".
*/
void slip_encode(unsigned char data, BlockRingBuffer *output_buffer)
{


    switch (data)
    {
         /* if it's the same code as an END character, we send a
            * special two character code so as not to make the
            * receiver think we sent an END
            */
        case END:
                   BlockRingBufferPush(output_buffer, ESC);
                   BlockRingBufferPush(output_buffer, ESC_END);
                   break;

             /* if it's the same code as an ESC character,
            * we send a special two character code so as not
            * to make the receiver think we sent an ESC
            */
        case ESC:
                   BlockRingBufferPush(output_buffer, ESC);
                   BlockRingBufferPush(output_buffer, ESC_ESC);
                   break;
        default:
            BlockRingBufferPush(output_buffer, data);
    }
}





/* RECV_PACKET: receives a packet into the buffer located at "p".
*      If more than len bytes are received, the packet will
*      be truncated.
*      Returns the number of bytes stored in the buffer.
*/


// call only if buffer is not empty
// get one byte
unsigned char slip_decode(BlockRingBuffer *inbuffer, uint8_t *outbuffer )
{
    unsigned char c;

    c = BlockRingBufferPop(inbuffer);
        
    switch(c)
    {
        case END:
            // we completeted this packet
            return 1;
            break;
        
        case ESC:
            c = BlockRingBufferPop(inbuffer);
            if (c == ESC_END)
                {
                    c = END;
                } else if (c == ESC_ESC) {
                    c = ESC;
                }

         default:
            *outbuffer = c;
            // not END, therefore packet is not complete
            return 0;
    }
}


