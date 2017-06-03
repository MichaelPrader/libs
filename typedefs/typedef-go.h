
#ifndef TYPEDEF_GO_H
#define TYPEDEF_GO_H




struct TGenericOutput_ {
    
     
    /*  Internal states */
    uint8_t St_State;
};

typedef struct TGenericOutput_ TGenericOutput;


/***************************** DEFINITION FOR OUTPUT STATES ***************************/
#define GO_IO_STATE_OFF     0x00
#define GO_IO_STATE_ON      0x01
#define GO_IO_STATE_FLASH   0x02
#define GO_IO_STATE_FL_ALT  0x03


#endif