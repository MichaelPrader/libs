
#ifndef TYPEDEF_TO_H
#define TYPEDEF_TO_H



struct TTurnout_ {
    

    /*  State variable. */
    uint8_t St_State;
    uint8_t St_OldState;
	
	// holds the state of the circuits
	uint8_t St_TrackCircuitTurnout;
	
	uint8_t Co_PositionLockout;
    
};

typedef struct TTurnout_ TTurnout;


/* Status masks of a turnout. TO_REQ_REVERSE is
used by the logic for commanding the position, requiring
the lower levels to send the appropriate commands to
the field device. */

#define TO_NORMAL           0x01
#define TO_REVERSE          0x02
#define TO_FLEET_LOCKED     0x04
#define TO_LOCKED           0x08

#define TO_LOCKED_BY_OTHER_TURNOUT  0x10
#define TO_USED         0x20
#define TO_REQ_NORMAL   0x40
#define TO_REQ_REVERSE  0x80
    
#define TO_POS_MASK     (TO_NORMAL | TO_REVERSE)
#define TO_REQ_MASK     (TO_REQ_NORMAL | TO_REQ_REVERSE)

#define TO_REQ_POS_SHIFT    6

#define TO_IS_VIRTUAL   0x01

#endif