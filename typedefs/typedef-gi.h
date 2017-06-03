
#ifndef TYPEDEF_GI_H
#define TYPEDEF_GI_H




struct TGenericInput_ {
    
     
    /*  Internal states */
    uint8_t St_State;
	uint8_t St_OldState;
};

typedef struct TGenericInput_ TGenericInput;


/***************************** DEFINITION FOR INPUT STATES ***************************/
#define GI_IO_STATE_OFF     0x00
#define GI_IO_STATE_ON      0x01

#endif