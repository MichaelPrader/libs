
#ifndef TYPEDEF_CIRC_H
#define TYPEDEF_CIRC_H

/* Define the logical track circuits. Each logical track circuit consists
of one current sensor.
IR sensors can be used at circuit boundaries for detection of
cars that cannot draw current - see the IR objects. */

struct TCircuit_ {


    /* Debounces the release */
    uint8_t Co_Debounce;
    
    
    uint8_t St_State;
    uint8_t St_OldState;
	
	

};

typedef struct TCircuit_ TCircuit;


/* Various masks.
The circuit object is not only used for the interlocking circuits,
but also for the block circuits, therefore it also provides
status masks for adjacent block signal/circuits. Additionally,
the specific functions for 1) releasing a current sensor occupancy
and 2) forced occupancy are included.
The IR_ACTIVE mask shows whether occupancy should be set because
of an active IR sensor detecting non-current drawing rolling stock. */

#define THIS_OCC        0x01
#define ADJ_OCC         0x02
#define THIS_CLEAR      0x04
#define ADJ_CLEAR       0x08
#define F_REL_ACTIVE    0x10
#define F_OCC_ACTIVE    0x20
#define SENSOR_ACTIVE   0x40
#define IR_ACTIVE       0x80

#endif