
#ifndef TYPEDEF_IR_H
#define TYPEDEF_IR_H


/* IR sensors can be used at circuit boundaries for detection of
cars that cannot draw current. Therefore, they can influence
up to two logical circuits. */

struct TIR_Sensor_ {
    
    /* Debounces the release */
    uint8_t Co_Debounce;
    
    /*  Holds the state of the sensor. */
    uint8_t St_State;

};

typedef struct TIR_Sensor_ TIR_Sensor;

// #define IR_ACTIVE       0x80

#endif
