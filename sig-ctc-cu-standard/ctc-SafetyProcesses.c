/**************************  APB BLOCK INTERFACES AND VIRTUAL SIGNALS *********************/
void APBSafetyProcess(void)
{
  
    uint8_t i, j, k, index;
    uint8_t entry, exit;
    
    //      TODO: track circuits without turnout must also be handled by the CTC routine
    //              lock, unlock with occupancy, unlock with running time, ....  -> November 13, 2015: future modification .... !!!
    

    // Map IR sensor to circuits
	// set to defined state
	for (i = 0; i < N_TRACK_CIRCUITS; ++i) ObjCircuit[i].St_State &= ~IR_ACTIVE;
	// trace into table
    for (i = 0; i < N_IR_SENSORS; ++i)
    {
        for (j = 0; j < CONF_SAFE_IR_MAP_LIST_SIZE; ++j)
        {
            if (pgm_read_byte(&conf_safe_ir_map_list[j][CONF_SAFE_IR_MAP_SENSOR]) == i)
            {
                index = pgm_read_byte(&conf_safe_ir_map_list[j][CONF_SAFE_IR_MAP_CIRCUIT]);
                if (index >= N_TRACK_CIRCUITS)
                {
                    // generate error
                    ErrorCode.Type = CONF_SAFE_IR_MAP_ERROR_CODE;
                    ErrorCode.ListIndex = j;
                } else {
                    if (ObjIR_Sensor[i].St_State & IR_ACTIVE) ObjCircuit[index].St_State |= IR_ACTIVE;
                }
            }
        }
    }

    // Calculate occupancy on circuits
    for (i = 0; i < N_TRACK_CIRCUITS; ++i)
    {
        ObjCircuit[i].St_State &= ~THIS_OCC;
        if (ObjCircuit[i].St_State & F_OCC_ACTIVE)
        {
            ObjCircuit[i].St_State |= THIS_OCC;
        } else if (ObjCircuit[i].St_State & IR_ACTIVE) {
            ObjCircuit[i].St_State |= THIS_OCC;
            if (ObjCircuit[i].St_State & F_REL_ACTIVE) ObjCircuit[i].St_State &= ~F_REL_ACTIVE;
        } else if (ObjCircuit[i].St_State & SENSOR_ACTIVE) {
            if (!(ObjCircuit[i].St_State & F_REL_ACTIVE)) ObjCircuit[i].St_State |= THIS_OCC;
        }
    }
	
	// get occupancy and clearance from adjacent node
	for (i = 0; i < N_SIGNALS; ++i)
	{
        for (j = 0; j < CONF_SAFE_SIG_APB_A_LIST_SIZE; ++j)
        {
            if (pgm_read_byte(&conf_safe_sig_apb_a_list[j][CONF_SAFE_SIG_APB_A_SIGNAL]) == i)
            {
                //signal found - check if circuit index is out of boundary
                index = pgm_read_byte(&conf_safe_sig_apb_a_list[j][CONF_SAFE_SIG_APB_A_CIRCUIT]);
                if ( index >= N_TRACK_CIRCUITS)
                {
                    // generate error
                    ErrorCode.Type = CONF_SAFE_SIG_APB_A_ERROR_CODE;
                    ErrorCode.ListIndex = j;
                } else {
                    if (ObjSignal[i].St_TrackCircuitNext != BLOCK_TRACK_CIRCUIT_FREE) ObjCircuit[index].St_State |= ADJ_OCC; else ObjCircuit[index].St_State &=  ~ADJ_OCC;
                    //if (ObjSignal[i].St_APBnext & CLEARED) ObjCircuit[index].St_State |= ADJ_CLEAR; else ObjCircuit[index].St_State &=  ~ADJ_CLEAR;
                    // if the next signal is tumbled down, there's probably a movement lined into our block/circuit
                    if (!(ObjSignal[i].St_APBnext & STICK) &&
                        ((ObjSignal[i].St_APBnext & ASP_FILTER) == ASP_STOP)) ObjCircuit[index].St_State |= ADJ_CLEAR; else ObjCircuit[index].St_State &=  ~ADJ_CLEAR;
                }
            }
        }
	}
	
    // Calculate support functions
	if (!(ForceReleaseTimeLock))
	{
	    for (i = 0; i < N_SIGNALS; ++i)
	    {
            for (j = 0; j < CONF_SAFE_SIG_APB_A_LIST_SIZE; ++j)
            {
                if (   pgm_read_byte(&conf_safe_sig_apb_a_list[j][CONF_SAFE_SIG_APB_A_SIGNAL]) == i    )
                {
                    index = pgm_read_byte(&conf_safe_sig_apb_a_list[j][CONF_SAFE_SIG_APB_A_CIRCUIT]);
                    if (index >= N_TRACK_CIRCUITS)
                    {
                        // generate error
                        ErrorCode.Type = CONF_SAFE_SIG_APB_A_ERROR_CODE;
                        ErrorCode.ListIndex = j;
                    } else {
                        // the two support functions can't be active at the same time, cancel both
                        if ( (ObjCircuit[index].St_State & F_REL_ACTIVE) && (ObjCircuit[index].St_State & F_OCC_ACTIVE)) ObjCircuit[index].St_State &= ~(F_REL_ACTIVE | F_OCC_ACTIVE);
                        if ( (ObjCircuit[index].St_State & F_REL_ACTIVE) && (ObjSignal[i].St_TrackCircuitNext != BLOCK_TRACK_CIRCUIT_FREE) ) ObjCircuit[index].St_State &= ~F_REL_ACTIVE;
                    }
                }
            }
	    }
	}
	
	
	/* synchronize the circuit states */
	for (i = 0; i < N_SIGNALS; ++i)
	{
	    // read the list of APB A circuits
	    for (j = 0; j < CONF_SAFE_SIG_APB_A_LIST_SIZE; ++j)
        {
            if (   pgm_read_byte(&conf_safe_sig_apb_a_list[j][CONF_SAFE_SIG_APB_A_SIGNAL]) == i    )
            {
                // entry found - check if circuit index is out of boundary
                index = pgm_read_byte(&conf_safe_sig_apb_a_list[j][CONF_SAFE_SIG_APB_A_CIRCUIT]);
                if ( index >= N_TRACK_CIRCUITS)
                {
                    // generate error
                    ErrorCode.Type = CONF_SAFE_SIG_APB_A_ERROR_CODE;
                    ErrorCode.ListIndex = j;
					continue;
				}
                // get the state of the circuit
                if (ObjCircuit[index].St_State & THIS_OCC) ObjSignal[i].St_TrackCircuitA = BLOCK_TRACK_CIRCUIT_OCCUPIED; else ObjSignal[i].St_TrackCircuitA = BLOCK_TRACK_CIRCUIT_FREE;
			}
		}
		
		// read the list of DROP circuits
	    for (j = 0; j < CONF_SAFE_SIG_DROP_LIST_SIZE; ++j)
        {
            if (   pgm_read_byte(&conf_safe_sig_drop_list[j][CONF_SAFE_SIG_DROP_SIGNAL]) == i    )
            {
                // entry found - check if circuit index is out of boundary
                index = pgm_read_byte(&conf_safe_sig_drop_list[j][CONF_SAFE_SIG_APB_A_CIRCUIT]);
                if ( index >= N_TRACK_CIRCUITS)
                {
                    // generate error
                    ErrorCode.Type = CONF_SAFE_SIG_DROP_ERROR_CODE;
                    ErrorCode.ListIndex = j;
					continue;
				}
                // get the state of the circuit
                if (ObjCircuit[index].St_State & THIS_OCC) ObjSignal[i].St_TrackCircuitDrop = BLOCK_TRACK_CIRCUIT_OCCUPIED; else ObjSignal[i].St_TrackCircuitDrop = BLOCK_TRACK_CIRCUIT_FREE;
			}
		}
	}
	
	 /* output occupancy to next nodes */
	for(i = 0; i < N_SIGNALS; ++i)
	{
        if (ObjSignal[i].St_TrackCircuitA != BLOCK_TRACK_CIRCUIT_FREE)
		{
			ObjSignal[i].St_TrackCircuitOut = BLOCK_TRACK_CIRCUIT_OCCUPIED;
			ObjSignal[i].St_APBout |= OCC;
		} else {
			ObjSignal[i].St_TrackCircuitOut = BLOCK_TRACK_CIRCUIT_FREE;
			ObjSignal[i].St_APBout &= ~OCC;
		}
	}
	
	/* calculate the state of the virtual APB signal */
	for (i = 0; i < N_SIGNALS; ++i)
	{
	    if (   	((ObjSignal[i].St_APBvirtual & ASP_FILTER) != ASP_STOP) &&
				(ObjSignal[i].St_TrackCircuitA != BLOCK_TRACK_CIRCUIT_FREE))
	    {
	        ObjSignal[i].St_APBvirtual |= STICK;
		}
		
        if (	(ObjSignal[i].St_TrackCircuitA != BLOCK_TRACK_CIRCUIT_FREE) ||
				(ObjSignal[i].St_TrackCircuitNext != BLOCK_TRACK_CIRCUIT_FREE) ||
				(((ObjSignal[i].St_APBnext & ASP_FILTER) == ASP_STOP) && !(ObjSignal[i].St_APBnext & STICK))  )
		{
			ObjSignal[i].St_APBvirtual = (ObjSignal[i].St_APBvirtual & ~ASP_FILTER) | ASP_STOP;
			ObjSignal[i].St_VirtualAspect = RED;
			
			if (ObjSignal[i].St_APBvirtual & STICK)
			{
				ObjSignal[i].St_APBvirtual = (ObjSignal[i].St_APBvirtual & ~ASP_FILTER) | ASP_RESTR;
				ObjSignal[i].St_VirtualAspect = BL_RED;
			}
		} else {
			index = ObjSignal[i].St_APBnext & ASP_FILTER;
			if (index <= ASP_FILTER)
			{
				ObjSignal[i].St_APBvirtual = (ObjSignal[i].St_APBvirtual & ~ASP_FILTER) | pgm_read_byte(&APBAspectSequence[index][0]);
				ObjSignal[i].St_VirtualAspect = pgm_read_byte(&APBAspectSequence[index][1]);
				
				ObjSignal[i].St_APBvirtual &= ~STICK;
			} else {
				ErrorCode.Type = APB_ASPECT_SEQUENCE_ERROR_CODE;
				ErrorCode.ListIndex = i;
			}
		}
	}
	
#ifdef AUTOMATION_USE_LOCKOUT
	for (i = 0; i < N_SIGNALS; ++i)
	{
		if (	(ObjSignal[i].St_TrackCircuitA != BLOCK_TRACK_CIRCUIT_FREE) &&
				(ObjSignal[i].St_OldTrackCircuitA == BLOCK_TRACK_CIRCUIT_FREE) &&
				(ObjSignal[i].St_TrackCircuitDrop != BLOCK_TRACK_CIRCUIT_FREE) &&
				(ObjSignal[i].St_APBout & ROUTE_ENDPOINT)   )
		{
			AutomationLockout[i] = 1;
		} else if (	(AutomationLockout[i]) &&
					(ObjSignal[i].St_TrackCircuitA == BLOCK_TRACK_CIRCUIT_FREE) &&
					(ObjSignal[i].St_TrackCircuitNext == BLOCK_TRACK_CIRCUIT_FREE))
		{
			// lockout was set, and train left the block section; clear lockout
			AutomationLockout[i] = 0;
        }
	}
#endif

	
	/* Cleanup when train enters adjacent section */
	for (i = 0; i < N_SIGNALS; ++i)
	{
		if (	(ObjSignal[i].St_TrackCircuitA != BLOCK_TRACK_CIRCUIT_FREE) &&
				(ObjSignal[i].St_TrackCircuitDrop != BLOCK_TRACK_CIRCUIT_FREE) &&
				//!(ObjSignal[i].St_APBout & STICK) &&                          // DEBUG - for APB - OK for other signal nodes??
																				// yes should be OK, didn't give flaws in Lauffen
				(ObjSignal[i].St_APBout & ROUTE_ENDPOINT) )
		{
			// this route resets a tumbledown; if no tumbledown was in effect (APB signal),
			// the check for !STICK above is negative and this routine is not called
			ObjSignal[i].St_APBout |= STICK;
			ObjSignal[i].St_APBout &= ~ROUTE_ENDPOINT;
		}
	}
	
	/*
		Function that passes tumbledown from exit signal to entry signal
	*/
	for (i = 0; i < CONF_SAFE_RT_LIST_SIZE; ++i)
	{
	    // for every route
	    entry = pgm_read_byte(&conf_safe_rt_list[i][CONF_SAFE_RT_ENTRY_SIGNAL]);
	    exit = pgm_read_byte(&conf_safe_rt_list[i][CONF_SAFE_RT_EXIT_SIGNAL]);
	    
	    for (j = 0; j < CONF_SAFE_RT_PASS_TUMBLEDOWN_LIST_SIZE; ++j)
	    {
            k = pgm_read_byte(&conf_safe_rt_pass_tumbledown_list[j]);
            
            if (k >= CONF_SAFE_RT_LIST_SIZE)
            {
                // generate error
                ErrorCode.Type = CONF_SAFE_RT_PASS_TUMBLEDOWN_ERROR_CODE;
                ErrorCode.ListIndex = i;
                continue;
	        }
            if (k == i) // entry found for this route
            {
                // check if virtual exit signal has tumbledown - this is independent of possible turnouts
                if ( ((ObjSignal[exit].St_APBvirtual & ASP_FILTER) == ASP_STOP) && !(ObjSignal[exit].St_APBvirtual & STICK))
                {
                    // tumbledown must be passed
                    ObjSignal[entry].St_APBout = (ObjSignal[entry].St_APBout & ~ASP_FILTER) | ASP_STOP;
                    ObjSignal[entry].St_APBout &= ~STICK;
                    ObjSignal[entry].St_Aspect = RED;
                    // and cancel highball/running time
                    ObjSignal[entry].St_Control &= ~(SIG_CLEAR | SIG_RUNNING_TIME);
					// clear the ROUTE_ENDPOINT on the exit, as this is normally done by the DropSignal routine
					// which is not called later on, as St_Control is already !SIG_CLEAR
					ObjSignal[exit].St_APBout &= ~ROUTE_ENDPOINT;
                    
                    // exit is not tumbled down, check whether we have to restore
                } else if (     ((ObjSignal[entry].St_APBvirtual & ASP_FILTER) == ASP_STOP) &&      // stop
                                !(ObjSignal[entry].St_APBvirtual & STICK) &&                        // no stick
                                !(ObjSignal[entry].St_APBout & ROUTE_ENDPOINT)) {                   // no route endpoint
                    /*  The entry signal must be restored if condition is lost and the entry is tumbled down.
                        Normally, the STICK is picked up by the CLEANUP routine above,
                        so this operation here must not be done if the entry is marked as route endpoint.
                        In that case, the cleanup routine will handle it. Automatic APB signals
                        will not perform this code, as the entry is always marked as ROUTE_ENDPOINT
                        of the opposing signal. The ClearSignal() routine handles restoring to
                        non-tumbledown state.
                    */
                    ObjSignal[entry].St_APBout = (ObjSignal[entry].St_APBout & ~ASP_FILTER) | ASP_STOP;
                    ObjSignal[entry].St_APBout |= STICK;
                    ObjSignal[entry].St_Aspect = RED;
                }
            }
		}
	}
	

	
}

/************************** BEGIN CTC ROUTINES *****************************/
void CTCSafetyProcess(void) //// from CTC     Hazeltine East
{

    uint8_t i, j, index, k;
    uint8_t result;
    
    

    for (i = 0; i < CONF_SAFE_TO_LOCKED_LIST_SIZE; ++i)
	{
        j = pgm_read_byte(&conf_safe_to_locked_list[i][CONF_SAFE_TO_LOCKED_TURNOUT]);
        k = pgm_read_byte(&conf_safe_to_locked_list[i][CONF_SAFE_TO_LOCKED_LOCKING_TURNOUT]);
        
        if (j >= N_TURNOUTS || k >= N_TURNOUTS)
        {
            ErrorCode.Type = CONF_SAFE_TO_LOCKED_ERROR_CODE;
            ErrorCode.ListIndex = i;
            continue;
        }

        if (ObjTurnout[j].St_State & (TO_LOCKED | TO_FLEET_LOCKED))
        {
            ObjTurnout[k].St_State |= TO_LOCKED_BY_OTHER_TURNOUT;
        } else {
            ObjTurnout[k].St_State &= ~TO_LOCKED_BY_OTHER_TURNOUT;
            ObjTurnout[k].St_State &= ~TO_USED;
        }
	}
	
	
    for (i = 0; i < N_ROUTES; ++i)
    {
        RouteActivationMonitor(i);
    }
    
    
	for(i = 0; i < N_SIGNALS; ++i)
	{
        // First handle all operations depending only from the signal, not from the individual route
        // Set signal to Stop when OS section becomes regularly occupied (no running time)
        if (ObjSignal[i].St_Control & SIG_CLEAR)
        {
			if (ObjSignal[i].St_TrackCircuitDrop != BLOCK_TRACK_CIRCUIT_FREE)
			{
				ObjSignal[i].St_Aspect = RED;
                ObjSignal[i].St_APBout = (ObjSignal[i].St_APBout & ~ASP_FILTER) | ASP_STOP | STICK;
                ObjSignal[i].St_Control &= ~SIG_CLEAR;
				ObjSignal[i].Nx_RouteSet = NO_ROUTE;
			}
		}
        
        /*    Signal upgrade/downgrade logic: call clearing routine */
        if (    (ObjSignal[i].St_Control & SIG_CLEAR)   &&
                !(ObjSignal[i].St_Control & SIG_RUNNING_TIME))
        {
            ClearSignal(i);
        }
        
        /* Reclear signal if fleeting is enabled */
        if  (   !(ObjSignal[i].St_Control & SIG_RUNNING_TIME)   &&
                (ObjSignal[i].St_Control & SIG_FLEET)           &&
                !(ObjSignal[i].St_Control & SIG_CLEAR))
        {
           /* result = 0;
            for (j = 0; j < CONF_SAFE_RT_TC_FREE_LIST_SIZE; ++j)
            {
                if ( pgm_read_byte(&conf_safe_rt_tc_free_list[j][CONF_SAFE_RT_TC_FREE_ROUTE) == ObjSignal[i].Nx_RouteSet)
                {
                    // circuit entry for the current route was found
                    index = pgm_read_byte(&conf_safe_rt_tc_free_list[j][CONF_SAFE_RT_TC_FREE_CIRCUIT]);
                    if ( index >= N_CIRCUIT)
                    {
                        // generate error
                        ErrorCode.Type = CONF_SAFE_RT_TC_FREE_LIST_ERROR_CODE;
                        ErrorCode.ListIndex = j;
                    } else {
                        if (ObjCircuit[index].St_State & THIS_OCC) result = 1;
                    }
                }
            }
            if (!(result)) */ ClearSignal(i); // maybe this is all that is needed TODO test NNNOOOO, there are  Problems!!
        }
        
        /* Cyclic control of route requirements: knock down signal */
        if     (ObjSignal[i].St_Control & (SIG_CLEAR | SIG_FLEET))
        {
            
			if (ObjSignal[i].Nx_RouteSet < CONF_SAFE_RT_LIST_SIZE)
			{
				// read the exit signal from the route table
				index = pgm_read_byte(&conf_safe_rt_list[ObjSignal[i].Nx_RouteSet][CONF_SAFE_RT_EXIT_SIGNAL]);
				if (index >= N_SIGNALS)
				{
					// generate error
					ErrorCode.Type = CONF_SAFE_RT_ERROR_CODE;
					ErrorCode.ListIndex = ObjSignal[i].Nx_RouteSet;
				} else {
					if (    ((ObjSignal[index].St_APBvirtual & ASP_FILTER) == ASP_STOP) &&
							!(ObjSignal[index].St_APBvirtual & STICK))
					{
						DropSignal(i);
					}
				}
			}
        }
	 
        /* Cyclic control, check track circuits */
        if     (ObjSignal[i].St_Control & SIG_CLEAR)
        {
            for (j = 0; j < CONF_SAFE_RT_TC_FREE_LIST_SIZE; ++j)
            {
                if ( pgm_read_byte(&conf_safe_rt_tc_free_list[j][CONF_SAFE_RT_TC_FREE_ROUTE]) == ObjSignal[i].Nx_RouteSet)
                {
                    // circuit entry for the current route was found
                    index = pgm_read_byte(&conf_safe_rt_tc_free_list[j][CONF_SAFE_RT_TC_FREE_CIRCUIT]);
                    if ( index >= N_TRACK_CIRCUITS)
                    {
                        // generate error
                        ErrorCode.Type = CONF_SAFE_RT_TC_FREE_ERROR_CODE;
                        ErrorCode.ListIndex = j;
                    } else {
                        if (ObjCircuit[index].St_State & THIS_OCC)
                        {
                            DropSignal(i);
                            break;
                        }
                    }
                }
            }
        }
        
        
        /* Knock down signal when turnout position is not correct anymore */
      if    (ObjSignal[i].St_Control & (SIG_CLEAR | SIG_FLEET))
      {
            // this signal i is clear or fleeted
            // check whether all turnout it requests
            // uses: i, j, index, result
            
            j = ObjSignal[i].Nx_RouteSet;   // extract the route that is set
            // we now need to find the exit signal, in order to get into the conf_safe_rt_to_pos_list
            if (j >= CONF_SAFE_RT_LIST_SIZE)
            {
                // generate error
                ErrorCode.Type = CONF_SAFE_RT_ERROR_CODE;
                ErrorCode.ListIndex = i;        // this actually doesn't give an incorrect entry, but tells that the Route Set is out of bounds, and indicates the signal.
            } else {
                index =  pgm_read_byte(&conf_safe_rt_list[j][CONF_SAFE_RT_EXIT_SIGNAL]);
                // OK we now know which exit signal this route leads to.
                // We can go into the conf_safe_rt_to_pos table.
                // i is the entry signal, index is the exit signal.
                
                for (j = 0; j < CONF_SAFE_RT_TO_POS_LIST_SIZE; ++j)
                {
                    if (    (pgm_read_byte(&conf_safe_rt_to_pos_list[j][CONF_SAFE_RT_TO_POS_ENTRY_SIGNAL]) == i) &&
                            (pgm_read_byte(&conf_safe_rt_to_pos_list[j][CONF_SAFE_RT_TO_POS_EXIT_SIGNAL]) == index))
                    {
                        // if these are true, then we found a turnout for this route i->index
                        // read the number and store in result
                        result = pgm_read_byte(&conf_safe_rt_to_pos_list[j][CONF_SAFE_RT_TO_POS_TURNOUT]);
                        if (result >= N_TURNOUTS)
                         {
                            // generate error
                            ErrorCode.Type = CONF_SAFE_RT_TO_POS_ERROR_CODE;
                            ErrorCode.ListIndex = j;
                        } else {
                            // number is correct, now check state
                            if ((ObjTurnout[result].St_State & TO_POS_MASK) != pgm_read_byte(&conf_safe_rt_to_pos_list[j][CONF_SAFE_RT_TO_POS_POSITION])) DropSignal(i);
                        }
                    }
                }
            }
      }

        // Running time clean up routine
        if (    (ObjSignal[i].St_Control & SIG_RUNNING_TIME) &&
		  	   !(ObjSignal[i].Co_RunningTime))
        {
            // running_time is over
            ObjSignal[i].St_Control &= ~SIG_RUNNING_TIME;
            
            // get the route that is set on signal i, extract
            // from the conf_safe_rt_list the exit signal
            // note: this should already be a checked value!
            if (ObjSignal[i].Nx_RouteSet < CONF_SAFE_RT_LIST_SIZE)
			{
                j = pgm_read_byte(&conf_safe_rt_list[ObjSignal[i].Nx_RouteSet][CONF_SAFE_RT_EXIT_SIGNAL]);
        		if (j >= N_SIGNALS)
        		{
        			ErrorCode.Type = CONF_SAFE_RT_ERROR_CODE;
        			ErrorCode.ListIndex = j;
        		} else {
        			ObjSignal[j].St_APBout |= STICK;
        			ObjSignal[j].St_APBout &= ~ROUTE_ENDPOINT;
        		}
				
				
				for (j = 0; j < CONF_SAFE_RT_TO_LOCKS_LIST_SIZE; ++j)
				{
					if (pgm_read_byte(&conf_safe_rt_to_locks_list[j][CONF_SAFE_RT_TO_LOCKS_ROUTE]) == ObjSignal[i].Nx_RouteSet)
					{
						// found an entry for the specific route
						index = pgm_read_byte(&conf_safe_rt_to_locks_list[j][CONF_SAFE_RT_TO_LOCKS_TURNOUT]);
						if ( index >= N_TURNOUTS)
						{
							// generate error
							ErrorCode.Type = CONF_SAFE_RT_TO_LOCKS_ERROR_CODE;
							ErrorCode.ListIndex = j;
						} else {
							ObjTurnout[index].St_State &= ~TO_LOCKED;
							for (k = 0; k < CONF_SAFE_TO_CIRCUIT_LIST_SIZE; ++k)
        					{
        						if ((pgm_read_byte(&conf_safe_to_circuit_list[k][CONF_SAFE_TO_CIRCUIT_TURNOUT]) == index) &&
        							(pgm_read_byte(&conf_safe_to_circuit_list[k][CONF_SAFE_TO_CIRCUIT_CIRCUIT]) < N_TRACK_CIRCUITS))
        						{
        							ObjCircuit[pgm_read_byte(&conf_safe_to_circuit_list[k][CONF_SAFE_TO_CIRCUIT_CIRCUIT])].St_State &= ~THIS_CLEAR;
        						}
        					}
							
						}
					}
				}
			}
			
			ObjSignal[i].Nx_RouteSet = NO_ROUTE;
        }
        
        
        
		
        
        /* end of ObjSignal[i] loop */
	}
	
	/* Turnout cleanup after train leaves */
    for (i = 0; i < N_TURNOUTS; ++i)
    {
        /*  When occupancy of the turnout's circuits change from
            occupied to free, some locks are removed.
            At least one circuit  must have been occupied, and all
            must be free.
            This is checked comparing every single circuit.
        */
        
        result = 0;
	    for (j = 0; j < CONF_SAFE_TO_CIRCUIT_LIST_SIZE; ++j)
	    {
	        if (pgm_read_byte(&conf_safe_to_circuit_list[j][CONF_SAFE_TO_CIRCUIT_TURNOUT]) == i)
	        {
	            // entry for turnout found
	            index = pgm_read_byte(&conf_safe_to_circuit_list[j][CONF_SAFE_TO_CIRCUIT_CIRCUIT]);
	            if ( index >= N_TRACK_CIRCUITS)
	            {
	                // generate error
                    ErrorCode.Type = CONF_SAFE_TO_CIRCUIT_ERROR_CODE;
                    ErrorCode.ListIndex = j;
	            } else {
	                
                    if (ObjCircuit[index].St_State & THIS_OCC)
                    {
                        // there is an occupied circuit - no need to investigate further
                        result = 0;
						ObjTurnout[i].St_TrackCircuitTurnout = BLOCK_TRACK_CIRCUIT_OCCUPIED;
                        break;
                    } else if (     (ObjCircuit[index].St_OldState & THIS_OCC)  &&
	                               !(ObjCircuit[index].St_State & THIS_OCC))
                    {
                        // circuit was occupied, but now is free
                        // memorize this event
                        result = 1;
						ObjTurnout[i].St_TrackCircuitTurnout = BLOCK_TRACK_CIRCUIT_FREE;
                    }
	            }
	        }
	    }
	    if (result)
	    {
	        ObjTurnout[i].St_State &= ~TO_LOCKED;
    	    for (j = 0; j < CONF_SAFE_TO_CIRCUIT_LIST_SIZE; ++j)
    		{
    			if ((pgm_read_byte(&conf_safe_to_circuit_list[j][CONF_SAFE_TO_CIRCUIT_TURNOUT]) == i) &&
    				(pgm_read_byte(&conf_safe_to_circuit_list[j][CONF_SAFE_TO_CIRCUIT_CIRCUIT]) < N_TRACK_CIRCUITS))
    			{
    				ObjCircuit[pgm_read_byte(&conf_safe_to_circuit_list[j][CONF_SAFE_TO_CIRCUIT_CIRCUIT])].St_State &= ~THIS_CLEAR;
    			}
    		}
	    }
    }
	
	
	
}

/******************************* END CTC ROUTINES *************************/


