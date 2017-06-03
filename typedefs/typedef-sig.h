
#ifndef TYPEDEF_SIG_H
#define TYPEDEF_SIG_H




struct TSignal_ {
    
     
    /*  Internal states */
    // state of the next APB signal (actually the APB interface)
    uint8_t St_APBnext;
    

    // the track circuit state we get from the adjoining block signal
	// states are BLOCK_TRACK_CIRCUIT_FREE, BLOCK_TRACK_CIRCUIT_OCCUPIED
    uint8_t St_TrackCircuitNext;
    
    // state of the virtual APB signal state (headblock)
    uint8_t St_APBvirtual;
    
    // the APB interface we put out to the adjoining block signal
    uint8_t St_APBout;
	uint8_t St_OldAPBout;
    
    // the track circuit state we put out to the adjoining block signal
	// states are BLOCK_TRACK_CIRCUIT_FREE, BLOCK_TRACK_CIRCUIT_OCCUPIED
    uint8_t St_TrackCircuitOut;
	uint8_t St_OldTrackCircuitOut;
	
	// the track circuit state of circuit A
	// states are BLOCK_TRACK_CIRCUIT_FREE, BLOCK_TRACK_CIRCUIT_OCCUPIED
	uint8_t St_TrackCircuitA;
	uint8_t St_OldTrackCircuitA;
	
	
	// the track circuit state of the drop circuit
	// states are BLOCK_TRACK_CIRCUIT_FREE, BLOCK_TRACK_CIRCUIT_OCCUPIED
	uint8_t St_TrackCircuitDrop;
	uint8_t St_OldTrackCircuitDrop;
    
    // the aspect (blinking yellow, yellow-over-green, ...) of the signal
    uint8_t St_Aspect;
	uint8_t St_OldAspect;
    
    // the aspect (blinking yellow, yellow-over-green, ?) of the virtual headblock signal
    uint8_t St_VirtualAspect;
    
    // state and control variable
    uint8_t St_Control;
	uint8_t St_OldControl;
    
    /*  If a signal gets dropped and the approach circuit is occupied,
        the signal gets knocked down immediately, but all locks and
        opposing tumbledowns are retained until "time runs out". */
    uint8_t Co_RunningTime;
    
    /*  Index into the global routes. This variable
        contains the global index number of the route currently set. */
    uint8_t Nx_RouteSet;
    
    /*  Index into the global routes. This variable
        contains the global index number of the route that is currently being requested. */
    uint8_t Nx_RequestedRoute;
    

 };
 

typedef struct TSignal_ TSignal;

// masks of St_Control
#define SIG_RED             0x00
#define SIG_CLEAR           0x01
#define SIG_RUNNING_TIME    0x02
#define SIG_FLEET           0x04
#define ENTRY_REQUEST       0x08
#define EXIT_REQUEST        0x10
//#define                   0x20
//#define                   0x40
//#define                   0x80


//masks of APBout, APBvirtual
// APB block interface
#define ASP_STOP    0
#define ASP_APPR    1
#define ASP_DIV     2
#define ASP_SLOW    3
#define ASP_MEDIUM  4
#define ASP_LIMITED 5
#define ASP_RESTR   6
#define ASP_CLEAR   7

#define ASP_FILTER      0x07
#define STICK           0x08
#define OCC             0x10
#define CLEARED         0x20
//#define               0x40
#define ROUTE_ENDPOINT  0x80


#define NO_ROUTE	0xFE

/***************************** DEFINITION FOR IO SIGNAL STATES ***************************/
#define SIG_IO_STATE_OFF    0x00
#define SIG_IO_STATE_ON     0x01
#define SIG_IO_STATE_FLASH  0x02
#define SIG_IO_STATE_FL_ALT 0x03


#endif