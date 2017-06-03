#ifndef TYPEDEF_ERRORCODE_H
#define TYPEDEF_ERRORCODE_H



struct TErrorCode_ {
    

    /*  State variable. */
    uint8_t Type;
    uint8_t ListIndex;
    
};

typedef struct TErrorCode_ TErrorCode;


#endif