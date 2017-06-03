#ifndef TYPEDEF_EE_OPERATION_H
#define TYPEDEF_EE_OPERATION_H



struct TEe_Operation_ {
    

    /*  State variable. */
    uint8_t St_State;
    uint8_t Val_Value;
	uint8_t Src_Source0;
	uint8_t Src_Source1;
	uint8_t Src_Source2;
	uint8_t Add_Address;
    
};

typedef struct TEe_Operation_ TEe_Operation;


#define EE_STATE_READ	0x01
#define EE_STATE_WRITE	0x02
#define EE_STATE_NO_OP	0x04

#endif