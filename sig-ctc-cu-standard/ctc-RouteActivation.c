
// Functions for activation of the routes

#include "ctc-CTCCmdProcesses.h"


void RouteActivationTimeout( uint8_t RouteNumber)
{
    uint8_t entry, exit, i, turnout;
    
    if (RouteNumber >= N_ROUTES) return;
    
    entry = pgm_read_byte(&conf_safe_rt_list[RouteNumber][CONF_SAFE_RT_ENTRY_SIGNAL]);
    exit = pgm_read_byte(&conf_safe_rt_list[RouteNumber][CONF_SAFE_RT_EXIT_SIGNAL]);
    
    if (entry >= N_SIGNALS || exit >= N_SIGNALS) return;
    
    
    for (i = 0; i < CONF_SAFE_RT_TO_POS_LIST_SIZE; ++i)
    {
        if (    (pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_ENTRY_SIGNAL]) == entry) &&
                (pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_EXIT_SIGNAL]) == exit))
        {
            // this list item "i" is for the route from "entry" to "exit"
            turnout = pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_TURNOUT]);
            if (turnout >= N_TURNOUTS)
            {
                // generate error
                ErrorCode.Type = CONF_SAFE_RT_TO_POS_ERROR_CODE;
                ErrorCode.ListIndex = i;
                break;
            } else {
                //number is correct
                // reset TO_USED
                ObjTurnout[turnout].St_State &= ~TO_USED;
             
            } // if turnout < N_TURNOUTS
        } // if entry and exit
    } // for list
    
    ObjSignal[entry].St_Control &= ~ENTRY_REQUEST;
    ObjSignal[exit].St_Control &= ~EXIT_REQUEST;
    ObjSignal[entry].Nx_RequestedRoute = NO_ROUTE;
    
    if (RouteActivator[RouteNumber].Co_TimeToLive) RouteActivator[RouteNumber].Co_TimeToLive = 0;
    
}





void RouteActivationStartRoute( uint8_t entry, uint8_t exit)
{
    uint8_t i, RouteNumber = NO_ROUTE, Conflict, turnout;
    
    if (( entry >= N_SIGNALS) || (exit >= N_SIGNALS)) return;
    
    // check
    for (i = 0; i < N_ROUTES; ++i)
    {
        if ( (pgm_read_byte(&conf_safe_rt_list[i][CONF_SAFE_RT_ENTRY_SIGNAL]) == entry) && (pgm_read_byte(&conf_safe_rt_list[i][CONF_SAFE_RT_EXIT_SIGNAL]) == exit))
        {
            // we found the route
            RouteNumber = i;
            break;
        }
    }
    // we haven't found a valid route; terminate function.
    if (RouteNumber == NO_ROUTE) return;
    
    // now check the state of the turnouts requested by this route
    Conflict = 0;
    
    for (i = 0; i < CONF_SAFE_RT_TO_POS_LIST_SIZE; ++i)
    {
        if (    (pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_ENTRY_SIGNAL]) == entry) &&
                (pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_EXIT_SIGNAL]) == exit))
        {
            // this list item "i" is for the route from "entry" to "exit"
            turnout = pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_TURNOUT]);
            if (turnout >= N_TURNOUTS)
            {
                // generate error
				Conflict = 1;
                ErrorCode.Type = CONF_SAFE_RT_TO_POS_ERROR_CODE;
                ErrorCode.ListIndex = i;
                break;
            } else {
                //number is correct, now check state
                
                if (    (((ObjTurnout[turnout].St_State & TO_REQ_MASK) >> TO_REQ_POS_SHIFT) != pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_POSITION])) &&
                        (ObjTurnout[turnout].St_State & (TO_USED | TO_LOCKED | TO_FLEET_LOCKED | TO_LOCKED_BY_OTHER_TURNOUT))   )
                {
                    // switch is wrong, and either used, locked, fleet locked, or locked by other turnout; we cannot throw it.
                    ++Conflict;
                    
                } else if (((ObjTurnout[turnout].St_State & TO_REQ_MASK) >> TO_REQ_POS_SHIFT) != pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_POSITION])) {
                    // turnout is wrong, but we can throw it
                    // set turnout
                    if (pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_POSITION]) == TO_NORMAL)
                    {
                        SetTurnout(turnout, BLOCK_BROADCAST_COMMAND_CMD_NORMAL);
                    } else if (pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_POSITION]) == TO_REVERSE) {
                        SetTurnout(turnout, BLOCK_BROADCAST_COMMAND_CMD_REVERSE);
                    }
                    // else .... do nothing
                    
                } else if (((ObjTurnout[turnout].St_State & TO_REQ_MASK) >> TO_REQ_POS_SHIFT) == pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_POSITION])) {
                    // position of turnout is correct
                    // the TO can additionally be locked, fleet locked, locked by other turnout or marked "used"
                    continue;
                }
                
            } // if turnout < N_TURNOUTS
        } // if entry and exit
    } // for list
    
    
    // we searched the whole turnout list for the route from "entry" to "exit"
    // if Conflict is 0, the turnouts are in or have been commanded into the correct position for this route
    // if Conflict is 1 or greater, we were not able to throw a wrong turnout, therefore we cannot start this route.
    if (Conflict) return;


    if (ObjSignal[entry].St_APBout & ROUTE_ENDPOINT)  return;   // entry signal is already used
    if (((ObjSignal[exit].St_APBvirtual & ASP_FILTER) == ASP_STOP) && !(ObjSignal[exit].St_APBvirtual & STICK)) return; // opposing movement set
    if (ObjSignal[entry].Nx_RequestedRoute != NO_ROUTE) return; // no other route must be active
    
    
    // start marking the turnouts as TO_USED
    for (i = 0; i < CONF_SAFE_RT_TO_POS_LIST_SIZE; ++i)
    {
        if (    (pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_ENTRY_SIGNAL]) == entry) &&
                (pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_EXIT_SIGNAL]) == exit))
        {
            // this list item "i" is for the route from "entry" to "exit"
            turnout = pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_TURNOUT]);
            if (turnout >= N_TURNOUTS)
            {
                // generate error
				Conflict = 1;
                ErrorCode.Type = CONF_SAFE_RT_TO_POS_ERROR_CODE;
                ErrorCode.ListIndex = i;
                break;
            } else {
                //number is correct
                ObjTurnout[turnout].St_State |= TO_USED;
                
            } // if turnout < N_TURNOUTS
        } // if entry and exit
    } // for list
    
    
    ObjSignal[entry].St_Control |= ENTRY_REQUEST;
    ObjSignal[exit].St_Control |= EXIT_REQUEST;
    ObjSignal[entry].Nx_RequestedRoute = RouteNumber;
    RouteActivator[RouteNumber].Co_TimeToLive = RT_ACTIVATION_TIMER_PRESET;

}




void RouteActivationMonitor( uint8_t RouteNumber)
{
     uint8_t entry, exit, i, turnout, Conflict;

     if (RouteNumber >= N_ROUTES) return;
    
    if (!(RouteActivator[RouteNumber].Co_TimeToLive)) return;
    
    entry = pgm_read_byte(&conf_safe_rt_list[RouteNumber][CONF_SAFE_RT_ENTRY_SIGNAL]);
    exit = pgm_read_byte(&conf_safe_rt_list[RouteNumber][CONF_SAFE_RT_EXIT_SIGNAL]);
    
    if (entry >= N_SIGNALS || exit >= N_SIGNALS) return;
    
    if (((ObjSignal[exit].St_APBvirtual & ASP_FILTER) == ASP_STOP) && !(ObjSignal[exit].St_APBvirtual & STICK))
    {
        RouteActivationTimeout(RouteNumber); // opposing movement set
        return;
    }
   
    // check if turnouts have arrived in their requested state
    
    Conflict = 0;
    
    for (i = 0; i < CONF_SAFE_RT_TO_POS_LIST_SIZE; ++i)
    {
        if (    (pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_ENTRY_SIGNAL]) == entry) &&
                (pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_EXIT_SIGNAL]) == exit))
        {
            // this list item "i" is for the route from "entry" to "exit"
            turnout = pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_TURNOUT]);
            if (turnout >= N_TURNOUTS)
            {
                // generate error
				Conflict = 1;
                ErrorCode.Type = CONF_SAFE_RT_TO_POS_ERROR_CODE;
                ErrorCode.ListIndex = i;
                break;
            } else {
                //number is correct
                if((ObjTurnout[turnout].St_State & TO_POS_MASK) != pgm_read_byte(&conf_safe_rt_to_pos_list[i][CONF_SAFE_RT_TO_POS_POSITION]))
				{
					Conflict++;
				}

            } // if turnout < N_TURNOUTS
        } // if entry and exit
    } // for list
    
    if (Conflict)
	{
		return;
	} else {
		RouteActivationTimeout(RouteNumber);
		ClearSignal(entry);
	}
        
}
