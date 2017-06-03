
#ifndef TYPEDEF_RT_H
#define TYPEDEF_RT_H


struct TRouteActivator_ {
    
    /* This object is responsible for activating a route. */

    /*  The timeout indicates both the time to live, as well as the
        fact that this object has been activated. */
        
    uint8_t Co_TimeToLive;
};

typedef struct TRouteActivator_ TRouteActivator;
    
    /*
#define SPEED_STEPS     6
//defines for Speed
#define SPEED_NORMAL        0
#define SPEED_DIVERGING     1
#define SPEED_RESTRICTING   2
#define SPEED_SLOW          3
#define SPEED_MEDIUM        4
#define SPEED_LIMITED       5
*/


#endif