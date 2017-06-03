

void APBClearIndication(void)
{
	uint8_t ClearIndicationSigIndex;
	uint8_t ClearIndicationCounter;
	uint8_t ClearIndicationCircuitIndex;
	/* Non-vital clearance indications */
	/* CODE IS SUPERSEDED AND INTEGRATED BELOW

	if (    (ObjSignal[i].St_APBout & ROUTE_ENDPOINT) &&
			((ObjSignal[i].St_Control & (SIG_CLEAR | SIG_RUNNING_TIME | SIG_FLEET)) == SIG_RED)   )
	// By looking also at the state of the opposing signal, on APB signals the green line
	// shows only when a direction is set.
	{
		ObjSignal[i].St_APBout |= CLEARED;
	} else {
		ObjSignal[i].St_APBout &= ~CLEARED;
	}
	*/

	for (ClearIndicationSigIndex = 0; ClearIndicationSigIndex < N_SIGNALS; ++ClearIndicationSigIndex)
	{
		
		// Pass clearance to circuit A
		for (ClearIndicationCounter = 0; ClearIndicationCounter < CONF_SAFE_SIG_APB_A_LIST_SIZE; ++ClearIndicationCounter)
		{
			if (   pgm_read_byte(&conf_safe_sig_apb_a_list[ClearIndicationCounter][CONF_SAFE_SIG_APB_A_SIGNAL]) == ClearIndicationSigIndex   )
			{
				// entry found - check if circuit index is out of boundary
				ClearIndicationCircuitIndex = pgm_read_byte(&conf_safe_sig_apb_a_list[ClearIndicationCounter][CONF_SAFE_SIG_APB_A_CIRCUIT]);
				if ( ClearIndicationCircuitIndex >= N_TRACK_CIRCUITS)
				{
					// generate error
					ErrorCode.Type = CONF_SAFE_SIG_APB_A_ERROR_CODE;
					ErrorCode.ListIndex = ClearIndicationCounter;
					// get the state of the circuit
				} else {
					if (    (ObjSignal[ClearIndicationSigIndex].St_APBout & ROUTE_ENDPOINT) &&
						   ((ObjSignal[ClearIndicationSigIndex].St_Control & (SIG_CLEAR | SIG_RUNNING_TIME | SIG_FLEET)) == SIG_RED)	)
							// By looking also at the state of the opposing signal, on APB signals the green line
							// shows only when a direction is set.
						ObjCircuit[ClearIndicationCircuitIndex].St_State |= THIS_CLEAR; else ObjCircuit[ClearIndicationCircuitIndex].St_State &= ~THIS_CLEAR;
						//if (ObjSignal[i].St_APBout & CLEARED) ObjCircuit[index].St_State |= THIS_CLEAR; else ObjCircuit[index].St_State &= ~THIS_CLEAR;
				}
			}
		}
	}
}