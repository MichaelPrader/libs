/*

        
*/

void ClearSignal(uint8_t signal)
{
	unsigned char ClearingConflict;
	unsigned char i, j, k, turnout;

	if (signal >= N_SIGNALS) return;
	
	if (ObjSignal[signal].St_Control & SIG_RUNNING_TIME) return;
	
	if (ObjSignal[signal].St_Control & EXIT_REQUEST) return;
	
	/* Handle clearing request */
	/* Identify route: The input is the signal,
        we have to find out which route we
        want to activate. In future this may be modified
        to handle a route-based control. */
        
        
    ClearingConflict = 0; // necessary for installations without turnouts
    
    for (i = 0; i < CONF_SAFE_RT_LIST_SIZE; i++)
    {
        /*  Go into the route list, and look for the entry signal,
            If is is found, read the associated exit signal, and go into
            the turnout position list. If for the
            specific combination there are incompatibilties,
            increase ClearingConflict. If ClearingConflict is 0, there
            are no incompatibilities and we found the route.
            Break the loops and continue with ClearingConflict == 0. */
        
        if (pgm_read_byte(&conf_safe_rt_list[i][CONF_SAFE_RT_ENTRY_SIGNAL]) == signal)
        {
            ClearingConflict = 0;
			// we found a line for the entry signal
            // select exit signal, and store in j
            j = pgm_read_byte(&conf_safe_rt_list[i][CONF_SAFE_RT_EXIT_SIGNAL]);
            if (j >= N_SIGNALS)
            {
                ClearingConflict = 1;
				// generate error
                ErrorCode.Type = CONF_SAFE_RT_ERROR_CODE;
                ErrorCode.ListIndex = i;
                break;
            }
            // SIGNAL and J define a valid route.
            // we can now go into turnout position list and
            // check all turnouts
            
            for (k = 0; k < CONF_SAFE_RT_TO_POS_LIST_SIZE; ++k)
            {
                if (    (pgm_read_byte(&conf_safe_rt_to_pos_list[k][CONF_SAFE_RT_TO_POS_ENTRY_SIGNAL]) == signal) &&
                        (pgm_read_byte(&conf_safe_rt_to_pos_list[k][CONF_SAFE_RT_TO_POS_EXIT_SIGNAL]) == j))
                {
                    // this list item "k" is for the route from "signal" to "j"
                    turnout = pgm_read_byte(&conf_safe_rt_to_pos_list[k][CONF_SAFE_RT_TO_POS_TURNOUT]);
                    if (turnout >= N_TURNOUTS)
                    {
                        // generate error
						ClearingConflict = 1;
                        ErrorCode.Type = CONF_SAFE_RT_TO_POS_ERROR_CODE;
                        ErrorCode.ListIndex = k;
                        break;
                    } else {
                        //number is correct, now check state; if position is incorrect, increase ClearingConflict
                        if((ObjTurnout[turnout].St_State & TO_POS_MASK) != pgm_read_byte(&conf_safe_rt_to_pos_list[k][CONF_SAFE_RT_TO_POS_POSITION]))
						{
							ClearingConflict++;
						}
                    }
                }
            }
            
            // we searched the whole turnout list for the route from "signal" to "j"
            // if ClearingConflict is 0, the turnouts are in the correct position for this route
            // that means we assume that there is no other route from "signal", that can lead to
            // other exit signals; therefore, we break out and don't search for other routes in the list
            if (ClearingConflict == 0)
            {
                ObjSignal[signal].Nx_RouteSet = i;
                break;
            }
        }
    }
	
    if (ClearingConflict)
	{
		return;
	}
	
	
		    
    // first check whether we shall ignore ROUTE_ENDPOINT on the entry signal
	/* 	      (ObjSignal[j].St_APBout & ROUTE_ENDPOINT)   &&*/
    
    if (ObjSignal[signal].St_APBout & ROUTE_ENDPOINT)	// if ClearingConflict is zero, we end up here. It means that j has a sensible value.
    {
        // entry signal is marked as route endpoint, increase ClearingConflict
        ++ClearingConflict;
        
        // now check whether we shall ignore
	    for (i = 0; i < CONF_SAFE_RT_ROUTE_ENDPOINT_IGNORE_LIST_SIZE; ++i)
	    {
	        k = pgm_read_byte(&conf_safe_rt_route_endpoint_ignore_list[i]);
            
	        if (k >= CONF_SAFE_RT_LIST_SIZE)
	        {
	            // generate error
                ErrorCode.Type = CONF_SAFE_RT_ROUTE_ENDPOINT_IGNORE_ERROR_CODE;
                ErrorCode.ListIndex = i;
                continue;
	        }
	        
	        if (k == ObjSignal[signal].Nx_RouteSet) // entry is identical to the selected route -> we can ignore a ROUTE_ENDPOINT marker
	        {
                --ClearingConflict; // decrease by what we added above; break the loop to be sure to substract only once
                break;
	        }

        }
    }
	
	if (ClearingConflict) return;
	
	
	      
	
	/* Chech block conditions, OS occupancy, route allowance */
	/* ClearingConflict is zero if route setting is allowed */
	// read exit signal (could reuse j from above, but it's better to read it again)
	if (ObjSignal[signal].Nx_RouteSet < CONF_SAFE_RT_LIST_SIZE)
		j = pgm_read_byte(&conf_safe_rt_list[ObjSignal[signal].Nx_RouteSet][CONF_SAFE_RT_EXIT_SIGNAL]);
	
	if (j >= N_SIGNALS) return;
	
	if ( ((ObjSignal[j].St_APBvirtual & ASP_FILTER) == ASP_STOP) && !(ObjSignal[j].St_APBvirtual & STICK)) return; // opposing movement set
	    
	
	// check free track circuits
    for (i = 0; i < CONF_SAFE_RT_TC_FREE_LIST_SIZE; ++i)
    {
        if ( pgm_read_byte(&conf_safe_rt_tc_free_list[i][CONF_SAFE_RT_TC_FREE_ROUTE]) == ObjSignal[signal].Nx_RouteSet)
        {
            // circuit entry for the current route was found
            k = pgm_read_byte(&conf_safe_rt_tc_free_list[i][CONF_SAFE_RT_TC_FREE_CIRCUIT]);
            if ( k >= N_TRACK_CIRCUITS)
            {
                // generate error
                ErrorCode.Type = CONF_SAFE_RT_TC_FREE_ERROR_CODE;
                ErrorCode.ListIndex = i;
                continue;
            //  j is the exit signal, k is specific circuit, i is the list counter, signal is entry signal
            } else {
                if (ObjCircuit[k].St_State & THIS_OCC)
                {
                    ++ClearingConflict;
                    break;
                }
            }
        }
    }
    
    if (ClearingConflict) return;
    
    
	// ----------------------- All conditions checked and met -------------------
	// ----------------------- start manipulations of objects -------------------
	
	//ClearingConflict is OK,
    // no errors on either switches or circuits, block conditions, ...
    for (i = 0; i < CONF_SAFE_RT_TO_LOCKS_LIST_SIZE; ++i)
    {
        if (pgm_read_byte(&conf_safe_rt_to_locks_list[i][CONF_SAFE_RT_TO_LOCKS_ROUTE]) == ObjSignal[signal].Nx_RouteSet)
        {
            // entry found, we must lock this turnout
            turnout = pgm_read_byte(&conf_safe_rt_to_locks_list[i][CONF_SAFE_RT_TO_LOCKS_TURNOUT]);
            if (turnout >= N_TURNOUTS)
            {
                ErrorCode.Type = CONF_SAFE_RT_TO_LOCKS_ERROR_CODE;
                ErrorCode.ListIndex = turnout;
                continue;
            } else {
                ObjTurnout[turnout].St_State |= TO_LOCKED;
                ObjTurnout[turnout].St_State &=  ~TO_USED;
				
				for (k = 0; k < CONF_SAFE_TO_CIRCUIT_LIST_SIZE; ++k)
				{
					if ((pgm_read_byte(&conf_safe_to_circuit_list[k][CONF_SAFE_TO_CIRCUIT_TURNOUT]) == turnout) &&
						(pgm_read_byte(&conf_safe_to_circuit_list[k][CONF_SAFE_TO_CIRCUIT_CIRCUIT]) < N_TRACK_CIRCUITS))
					{
						ObjCircuit[pgm_read_byte(&conf_safe_to_circuit_list[k][CONF_SAFE_TO_CIRCUIT_CIRCUIT])].St_State |= THIS_CLEAR;
					}
				}
            }
        }
    }

      
    //start tumbledown, if not to be ignored
    k = 0; // use as marker
    for (i = 0; i < CONF_SAFE_RT_INHIBIT_ACTIVE_TUMBLEDOWN_LIST_SIZE; ++i)
    {
        
        if (pgm_read_byte(&conf_safe_rt_inhibit_active_tumbledown_list[i]) == ObjSignal[signal].Nx_RouteSet)
        {
            k = 1;
            break;
        }
        // found an entry, so set marker k and skip
    }
    if (!k)
    {
        ObjSignal[j].St_APBout = (ObjSignal[j].St_APBout & ~ASP_FILTER) | ASP_STOP;
        ObjSignal[j].St_APBout &= ~STICK;
    }
    
    
    // mark the exit signal as used as the endpoint of the route
    ObjSignal[j].St_APBout |= ROUTE_ENDPOINT;
    
    
    // check whether a filter is to be set
    for (i = 0; i < CONF_SAFE_RT_ASPECT_FILTER_LIST_SIZE; ++i)
    {
        if (ObjSignal[signal].Nx_RouteSet == pgm_read_byte(&conf_safe_rt_aspect_filter_list[i][CONF_SAFE_RT_ASPECT_FILTER_ROUTE]))
        {
            // entry is for this route found
            k = pgm_read_byte(&conf_safe_rt_aspect_filter_list[i][CONF_SAFE_RT_ASPECT_FILTER_TURNOUT]);
            if (k >= N_TURNOUTS)
            {
                ErrorCode.Type = CONF_SAFE_RT_ASPECT_FILTER_ERROR_CODE;
                ErrorCode.ListIndex = i;
                continue;
            } else {
                if ((ObjTurnout[k].St_State & TO_POS_MASK) != pgm_read_byte(&conf_safe_rt_aspect_filter_list[i][CONF_SAFE_RT_ASPECT_FILTER_POSITION]))
                {
                    // filter must be applied
                    ObjSignal[signal].St_APBout = (ObjSignal[signal].St_APBout & ~ASP_FILTER) | pgm_read_byte(&conf_safe_rt_aspect_filter_list[i][CONF_SAFE_RT_ASPECT_FILTER_APB_OUT]);
                    ObjSignal[signal].St_Aspect = pgm_read_byte(&conf_safe_rt_aspect_filter_list[i][CONF_SAFE_RT_ASPECT_FILTER_ASPECT]);
                    break;
                }
            }
        }
    }
    
    // select what we want to show on the signal
    for (i = 0; i < CONF_SAFE_RT_ASPECT_TRANSFER_LIST_SIZE; ++i)
    {
        if (    (ObjSignal[signal].Nx_RouteSet == pgm_read_byte(&conf_safe_rt_aspect_transfer_list[i][CONF_SAFE_RT_ASPECT_TRANSFER_ROUTE])) &&
                (ObjSignal[j].St_VirtualAspect == pgm_read_byte(&conf_safe_rt_aspect_transfer_list[i][CONF_SAFE_RT_ASPECT_TRANSFER_EXIT_ASPECT])) )
        {
            // list entry found
            ObjSignal[signal].St_APBout = (ObjSignal[signal].St_APBout & ~ASP_FILTER) | pgm_read_byte(&conf_safe_rt_aspect_transfer_list[i][CONF_SAFE_RT_ASPECT_TRANSFER_ENTRY_APB_ASPECT]);
            ObjSignal[signal].St_Aspect = pgm_read_byte(&conf_safe_rt_aspect_transfer_list[i][CONF_SAFE_RT_ASPECT_TRANSFER_ENTRY_ASPECT]);
            break;
        }
    }
    
    
    
    //ObjSignal[signal].Nx_RequestedRoute = NO_ROUTE;

    //ObjSignal[signal].St_Control &= ~ENTRY_REQUEST;
    //ObjSignal[j].St_Control &= ~EXIT_REQUEST;
    
    // and highball
    ObjSignal[signal].St_Control |= SIG_CLEAR;
            
}

void DropSignal(uint8_t signal)
{
    uint8_t i, j, k;
	
	if (signal >= N_SIGNALS) return;
	
	if (ObjSignal[signal].St_Control & ENTRY_REQUEST)
	{
        if (ObjSignal[signal].Nx_RequestedRoute < N_ROUTES)
    	    RouteActivationTimeout( ObjSignal[signal].Nx_RequestedRoute);
	}

    // if the signal runs time, or is not clear, there is no point to continue with this routine -> return
	if ((ObjSignal[signal].St_Control & SIG_RUNNING_TIME)  || !(ObjSignal[signal].St_Control & SIG_CLEAR)) return;

    // check whether the APB_A circuit of signal is occupied
    // read the list
	if (	(ObjSignal[signal].St_TrackCircuitA != BLOCK_TRACK_CIRCUIT_FREE) ||
			(ObjSignal[signal].St_TrackCircuitNext != BLOCK_TRACK_CIRCUIT_FREE))
	{
		/* Adjacent block occupied: a train might be approaching, so run time on the signal */
		ObjSignal[signal].St_Aspect = RED;
		ObjSignal[signal].St_APBout = (ObjSignal[signal].St_APBout & ~ASP_FILTER) | ASP_STOP | STICK;
		ObjSignal[signal].St_Control &= ~(SIG_CLEAR | SIG_FLEET);
		ObjSignal[signal].St_Control |= SIG_RUNNING_TIME;
		ObjSignal[signal].Co_RunningTime = RUNNING_TIME;
		
		
		for (i = 0; i < CONF_SAFE_RT_TO_LOCKS_LIST_SIZE; ++i)
		{
			if (pgm_read_byte(&conf_safe_rt_to_locks_list[i][CONF_SAFE_RT_TO_LOCKS_ROUTE]) == ObjSignal[signal].Nx_RouteSet)
			{
				// entry found, we must release this turnout
				j = pgm_read_byte(&conf_safe_rt_to_locks_list[i][CONF_SAFE_RT_TO_LOCKS_TURNOUT]);
				if (j >= N_TURNOUTS)
				{
					ErrorCode.Type = CONF_SAFE_RT_TO_LOCKS_ERROR_CODE;
					ErrorCode.ListIndex = j;
					continue;
				} else {
					ObjTurnout[j].St_State &= ~TO_FLEET_LOCKED;
				}
			}
		}
	} else {
		/* No running time */
		ObjSignal[signal].St_Aspect = RED;
		ObjSignal[signal].St_APBout = (ObjSignal[signal].St_APBout & ~ASP_FILTER) | ASP_STOP | STICK;
		ObjSignal[signal].St_Control &= ~(SIG_CLEAR | SIG_FLEET);
		
		for (i = 0; i < CONF_SAFE_RT_TO_LOCKS_LIST_SIZE; ++i)
		{
			if (pgm_read_byte(&conf_safe_rt_to_locks_list[i][CONF_SAFE_RT_TO_LOCKS_ROUTE]) == ObjSignal[signal].Nx_RouteSet)
			{
				// entry found, we must release this turnout
				j = pgm_read_byte(&conf_safe_rt_to_locks_list[i][CONF_SAFE_RT_TO_LOCKS_TURNOUT]);
				if (j >= N_TURNOUTS)
				{
					ErrorCode.Type = CONF_SAFE_RT_TO_LOCKS_ERROR_CODE;
					ErrorCode.ListIndex = j;
				} else {
					ObjTurnout[j].St_State &= ~(TO_LOCKED | TO_FLEET_LOCKED);
					
					for (k = 0; k < CONF_SAFE_TO_CIRCUIT_LIST_SIZE; ++k)
					{
						if ((pgm_read_byte(&conf_safe_to_circuit_list[k][CONF_SAFE_TO_CIRCUIT_TURNOUT]) == j) &&
							(pgm_read_byte(&conf_safe_to_circuit_list[k][CONF_SAFE_TO_CIRCUIT_CIRCUIT]) < N_TRACK_CIRCUITS))
						{
							ObjCircuit[pgm_read_byte(&conf_safe_to_circuit_list[k][CONF_SAFE_TO_CIRCUIT_CIRCUIT])].St_State &= ~THIS_CLEAR;
						}
					}
				}
			}
		}
		
		if (ObjSignal[signal].Nx_RouteSet < CONF_SAFE_RT_LIST_SIZE)
		{
		    j = pgm_read_byte(&conf_safe_rt_list[ObjSignal[signal].Nx_RouteSet][CONF_SAFE_RT_EXIT_SIGNAL]);
            if (j >= N_SIGNALS)
    		{
    			ErrorCode.Type = CONF_SAFE_RT_ERROR_CODE;
    			ErrorCode.ListIndex = j;
    		} else {
    			ObjSignal[j].St_APBout |= STICK;
    			ObjSignal[j].St_APBout &= ~ROUTE_ENDPOINT;
    		}
		}
		ObjSignal[signal].Nx_RouteSet = NO_ROUTE;
	}
}



void SetFleet(uint8_t signal)
{
    uint8_t i, j;
	
	if (signal >= N_SIGNALS) return;
    
    if (    (ObjSignal[signal].St_Control & SIG_CLEAR)  &&
            !(ObjSignal[signal].St_Control & SIG_RUNNING_TIME) &&
            !(ObjSignal[signal].St_Control & SIG_FLEET))
    {
        ObjSignal[signal].St_Control |= SIG_FLEET;
        
        for (i = 0; i < CONF_SAFE_RT_TO_LOCKS_LIST_SIZE; ++i)
	    {
	        if (pgm_read_byte(&conf_safe_rt_to_locks_list[i][CONF_SAFE_RT_TO_LOCKS_ROUTE]) == ObjSignal[signal].Nx_RouteSet)
	        {
	            // entry found, we must lock this turnout
	            j = pgm_read_byte(&conf_safe_rt_to_locks_list[i][CONF_SAFE_RT_TO_LOCKS_TURNOUT]);
	            if (j >= N_TURNOUTS)
	            {
	                ErrorCode.Type = CONF_SAFE_RT_TO_LOCKS_ERROR_CODE;
                    ErrorCode.ListIndex = j;
	            } else {
	                ObjTurnout[j].St_State |= TO_FLEET_LOCKED;
	            }
	        }
	    }
    }
}

void DropFleet(uint8_t signal)
{
    uint8_t i, j;

	if (signal >= N_SIGNALS) return;

    
    if (    (ObjSignal[signal].St_Control & SIG_FLEET)  &&
            !(ObjSignal[signal].St_Control & SIG_RUNNING_TIME))
    {
        ObjSignal[signal].St_Control &= ~SIG_FLEET;
        
        for (i = 0; i < CONF_SAFE_RT_TO_LOCKS_LIST_SIZE; ++i)
	    {
	        if (pgm_read_byte(&conf_safe_rt_to_locks_list[i][CONF_SAFE_RT_TO_LOCKS_ROUTE]) == ObjSignal[signal].Nx_RouteSet)
	        {
	            // entry found, we must lock this turnout
	            j = pgm_read_byte(&conf_safe_rt_to_locks_list[i][CONF_SAFE_RT_TO_LOCKS_TURNOUT]);
	            if (j >= N_TURNOUTS)
	            {
	                ErrorCode.Type = CONF_SAFE_RT_TO_LOCKS_ERROR_CODE;
                    ErrorCode.ListIndex = j;
	            } else {
	                ObjTurnout[j].St_State &= ~TO_FLEET_LOCKED;
	            }
	        }
	    }
    }
}


void SetTurnout(uint8_t turnout, uint8_t command)
{
    uint8_t i;
    
    
	if(turnout >= N_TURNOUTS) return;

	if (ObjTurnout[turnout].St_TrackCircuitTurnout != BLOCK_TRACK_CIRCUIT_FREE) return;
	
	if (ObjTurnout[turnout].St_State & (TO_LOCKED | TO_FLEET_LOCKED)) return;
	if (ObjTurnout[turnout].St_State & TO_USED) return;
	if (ObjTurnout[turnout].St_State & TO_LOCKED_BY_OTHER_TURNOUT) return;

	/*
	for (i = 0; i < CONF_SAFE_TO_LOCKED_LIST_SIZE; ++i)
	{
        if (pgm_read_byte(&conf_safe_to_locked_list[i][CONF_SAFE_TO_LOCKED_TURNOUT]) == turnout)
        {
            j = pgm_read_byte(&conf_safe_to_locked_list[i][CONF_SAFE_TO_LOCKED_LOCKING_TURNOUT]);
            if (j >= N_TURNOUTS)
            {
                ErrorCode.Type = CONF_SAFE_TO_LOCKED_ERROR_CODE;
                ErrorCode.ListIndex = i;
                continue;
            }
            if (ObjTurnout[j].St_State & (TO_LOCKED | TO_FLEET_LOCKED)) return;
		}
	}
	*/
	
	if (!(ObjTurnout[turnout].St_State & (TO_LOCKED | TO_FLEET_LOCKED)))
	{
	    if (command == BLOCK_BROADCAST_COMMAND_CMD_NORMAL)
        {
            ObjTurnout[turnout].St_State &= ~TO_REQ_REVERSE;
            ObjTurnout[turnout].St_State |= TO_REQ_NORMAL;
        } else if (command == BLOCK_BROADCAST_COMMAND_CMD_REVERSE) {
            ObjTurnout[turnout].St_State |= TO_REQ_REVERSE;
            ObjTurnout[turnout].St_State &= ~TO_REQ_NORMAL;
        }
    }
    
    // forces a delay on valid position input
     for (i = 0; i < CONF_IO_TO_LIST_SIZE; ++i)
    {
        if (pgm_read_byte(&conf_io_to_list[i][CONF_IO_TO_TURNOUT]) == turnout)
        {
            if (!(pgm_read_byte(&conf_io_to_list[i][CONF_IO_TO_ISVIRTUAL])))
            {
                ObjTurnout[turnout].Co_PositionLockout = TO_POSITION_LOCKOUT_TIMER;
                ObjTurnout[turnout].St_State &= ~TO_POS_MASK;
            }
            break;
        }
    }
    
    
}
