
/************************** CALCULATION OF STATES AND ITEMS *******************************/


void MapInputStates (void)
{
    
    uint8_t i, j, byte, hilo;
    uint8_t mask;
 
    // map io of IR sensors
    for (i = 0; i < N_IR_SENSORS; ++i)
    {

        for (j = 0; j < CONF_IO_IR_LIST_SIZE; ++j)
        {
            if (pgm_read_byte(&conf_io_ir_list[j][CONF_IO_IR_SENSOR]) == i)
            {
                byte = pgm_read_byte(&conf_io_ir_list[j][CONF_IO_IR_INBYTE]);
                mask = 0 | (1 << pgm_read_byte(&conf_io_ir_list[j][CONF_IO_IR_INBIT]));
                
                if (byte >= IO_INPUT_SIZE)
                {
                    // out of boundary of input array - generate error
                    ErrorCode.Type = CONF_IO_IR_ERROR_CODE;
                    ErrorCode.ListIndex = j;
                } else {
                    /*
                        es gibt zwei Variablen, also vier Kombinationen/Kreuze
                        was ist der effizienteste Weg diese zu verschneiden?
                        HI      + 0     => 0
                        LO      + 0     => 1
                        HI      + 1     => 1
                        LO      + 1     => 0
                    */
                 //   if (    (io_input[byte] & mask)) ObjIR_Sensor[i].Co_Debounce = IR_SENSOR_DEBOUNCE_TIME;
                    mask = io_input[byte] & mask;
                    hilo = pgm_read_byte(&conf_io_ir_list[j][CONF_IO_IR_HILO]);
                    if (    ((hilo == HI) && mask) ||
                            ((hilo == LO) && !mask))
                    {
                        // input is positive; in any case we set the timer
                        ObjIR_Sensor[i].Co_Debounce = IR_SENSOR_DEBOUNCE_TIME;
                    } else {
                        // input is negative; if we don't have to debounce,
                        // we immediately reset the timer; otherwise we let
                        // the timing routine count back to Zero.
                        if (pgm_read_byte(&conf_io_ir_list[j][CONF_IO_IR_DEBOUNCE]) == NO_DEBOUNCE)
                            ObjIR_Sensor[i].Co_Debounce = 0;
                    }
                }
            }
        }
        
        if (ObjIR_Sensor[i].Co_Debounce) ObjIR_Sensor[i].St_State |= IR_ACTIVE; else ObjIR_Sensor[i].St_State &= ~IR_ACTIVE;
        
    }
    
    
    
    // map io of track circuits
    for (i = 0; i < N_TRACK_CIRCUITS; ++i)
    {
        for (j = 0; j < CONF_IO_TC_LIST_SIZE; ++j)
        {
            if (pgm_read_byte(&conf_io_tc_list[j][CONF_IO_TC_CIRCUIT]) == i)
            {
                byte = pgm_read_byte(&conf_io_tc_list[j][CONF_IO_TC_INBYTE]);
                mask = 0 | (1 << pgm_read_byte(&conf_io_tc_list[j][CONF_IO_TC_INBIT]));
                if (byte >= IO_INPUT_SIZE)
                {
                    // out of boundary of input array - generate error
                    ErrorCode.Type = CONF_IO_TC_ERROR_CODE;
                    ErrorCode.ListIndex = j;
                } else {
                    
                    mask = io_input[byte] & mask;
                    hilo = pgm_read_byte(&conf_io_tc_list[j][CONF_IO_TC_HILO]);
                    if (    ((hilo == HI) && mask) ||
                            ((hilo == LO) && !mask))
                    {
                        // input is positive; in any case we set the timer
                        ObjCircuit[i].Co_Debounce = TC_SENSOR_DEBOUNCE_TIME;
                    } else {
                        // input is negative; if we don't have to debounce,
                        // we immediately reset the timer; otherwise we let
                        // the timing routine count back to Zero.
                        if (pgm_read_byte(&conf_io_tc_list[j][CONF_IO_TC_DEBOUNCE]) == NO_DEBOUNCE)
                            ObjCircuit[i].Co_Debounce = 0;
                    }
                    
                    if (ObjCircuit[i].Co_Debounce) ObjCircuit[i].St_State |= SENSOR_ACTIVE; else ObjCircuit[i].St_State &= ~SENSOR_ACTIVE;
                }
            }
        }
    }
    
    // map input of turnouts
    for (i = 0; i < N_TURNOUTS; ++i)
    {
        
        for (j = 0; j < CONF_IO_TO_LIST_SIZE; ++j)
        {
            if (pgm_read_byte(&conf_io_to_list[j][CONF_IO_TO_TURNOUT]) == i)
            {
                if (pgm_read_byte(&conf_io_to_list[j][CONF_IO_TO_ISVIRTUAL]))
                {
                    if (ObjTurnout[i].St_State & TO_REQ_REVERSE)
                    {
                        ObjTurnout[i].St_State |= TO_REVERSE;
                        ObjTurnout[i].St_State &= ~TO_NORMAL;
                    } else {
                        ObjTurnout[i].St_State |= TO_NORMAL;
                        ObjTurnout[i].St_State &= ~TO_REVERSE;
                    }
                } else {
                    
                    // normal position
                    byte = pgm_read_byte(&conf_io_to_list[j][CONF_IO_TO_N_INBYTE]);
                    mask = 0 | (1 << pgm_read_byte(&conf_io_to_list[j][CONF_IO_TO_N_INBIT]));
                    if (byte >= IO_INPUT_SIZE)
                    {
                        // out of boundary of input array - generate error
                        ErrorCode.Type = CONF_IO_TO_ERROR_CODE;
                        ErrorCode.ListIndex = j;
                    } else {
//                        if (io_input[byte] & mask) ObjTurnout[i].St_State |= TO_NORMAL;
                        mask = io_input[byte] & mask;
                        hilo = pgm_read_byte(&conf_io_to_list[j][CONF_IO_TO_N_INHILO]);
                        if (    ((hilo == HI) && mask) ||
                                ((hilo == LO) && !mask))
                        {
                            ObjTurnout[i].St_State |= TO_NORMAL;
                        } else {
                            ObjTurnout[i].St_State &= ~TO_NORMAL;
                        }
                    }
                    // reverse position
                    byte = pgm_read_byte(&conf_io_to_list[j][CONF_IO_TO_R_INBYTE]);
                    mask = 0 | (1 << pgm_read_byte(&conf_io_to_list[j][CONF_IO_TO_R_INBIT]));
                    if (byte >= IO_INPUT_SIZE)
                    {
                        // out of boundary of input array - generate error
                        ErrorCode.Type = CONF_IO_TO_ERROR_CODE;
                        ErrorCode.ListIndex = j;
                    } else {
//                        if (io_input[byte] & mask) ObjTurnout[i].St_State |= TO_REVERSE;
                        mask = io_input[byte] & mask;
                        hilo = pgm_read_byte(&conf_io_to_list[j][CONF_IO_TO_R_INHILO]);
                        if (    ((hilo == HI) && mask) ||
                                ((hilo == LO) && !mask))
                        {
                            ObjTurnout[i].St_State |= TO_REVERSE;
                        } else {
                            ObjTurnout[i].St_State &= ~TO_REVERSE;
                        }
                    }
                    
                    // invalidate input if Position Lockout is active
                    if (ObjTurnout[i].Co_PositionLockout)
                    {
                        ObjTurnout[i].St_State &= ~TO_POS_MASK;
                    }
                }
            }
        }
    }
  
    // map io generic inputs
    for (i = 0; i < N_GENERIC_INPUTS; ++i)
    {
        for (j = 0; j < CONF_IO_GI_LIST_SIZE; ++j)
        {
            if (pgm_read_byte(&conf_io_gi_list[j][CONF_IO_GI_INPUT]) == i)
            {
                byte = pgm_read_byte(&conf_io_gi_list[j][CONF_IO_GI_BYTE]);
                mask = 0 | (1 << pgm_read_byte(&conf_io_gi_list[j][CONF_IO_GI_BIT]));
                if (byte >= IO_INPUT_SIZE)
                {
                    // out of boundary of input array - generate error
                    ErrorCode.Type = CONF_IO_GI_ERROR_CODE;
                    ErrorCode.ListIndex = j;
                } else {
                    
                    mask = io_input[byte] & mask;
                    hilo = pgm_read_byte(&conf_io_gi_list[j][CONF_IO_GI_HILO]);
                    if (    ((hilo == HI) && mask) ||
                            ((hilo == LO) && !mask))
                    {
                        ObjGenericInput[i].St_State = GI_IO_STATE_ON;
                    } else {
                        ObjGenericInput[i].St_State = GI_IO_STATE_OFF;
                    }
                }
            }
        }
    }
    
    
    
    for (i = 0; i < N_BLOCKINTERFACES; ++i)
    {
        for (j = 0; j < CONF_IO_BLOCK_BROADCAST_SHUTOFF_LIST_SIZE; ++j)
        {
            if (pgm_read_byte(&conf_io_block_broadcast_shutoff_list[j][CONF_IO_BLOCK_BROADCAST_SHUTOFF_BLOCKINTERFACE]) == i)
            {
                // block interface found
                byte = pgm_read_byte(&conf_io_block_broadcast_shutoff_list[j][CONF_IO_BLOCK_BROADCAST_SHUTOFF_INBYTE]);
                mask = 0 | (1 << pgm_read_byte(&conf_io_block_broadcast_shutoff_list[j][CONF_IO_BLOCK_BROADCAST_SHUTOFF_INBIT]));
                if (byte >= IO_INPUT_SIZE)
                {
                    // out of boundary of input array - generate error
                    ErrorCode.Type = CONF_IO_BLOCK_BROADCAST_SHUTOFF_LIST_ERROR_CODE;
                    ErrorCode.ListIndex = j;
                } else {
                    // list entry for broadcast shutoff
                    mask = io_input[byte] & mask;
                    hilo = pgm_read_byte(&conf_io_block_broadcast_shutoff_list[j][CONF_IO_BLOCK_BROADCAST_SHUTOFF_HILO]);
                    if (    ((hilo == HI) && mask) ||
                            ((hilo == LO) && !mask))
                    {
                        ObjBlockInterface[i].St_Config |= BROADCAST_SHUT_OFF;
                    } else {
                        ObjBlockInterface[i].St_Config &= ~BROADCAST_SHUT_OFF;
                    }
             //   if (io_input[byte] & mask) ObjBlockInterface[i].St_Config &= ~BROADCAST_SHUT_OFF; else  ObjBlockInterface[i].St_Config |= BROADCAST_SHUT_OFF;
                }
            }
        }
    }

}


void MapOutputStates (uint8_t timing)
{

    uint8_t i, j, byte, hilo;
    uint8_t mask;

    
    timing = (timing / 2) % 2;
 

    // map output of turnouts
    for (i = 0; i < N_TURNOUTS; ++i)
    {
        for (j = 0; j < CONF_IO_TO_LIST_SIZE; ++j)
        {
            if (pgm_read_byte(&conf_io_to_list[j][CONF_IO_TO_TURNOUT]) == i)
            {
                if (!(pgm_read_byte(&conf_io_to_list[j][CONF_IO_TO_ISVIRTUAL])))
                {
                    // normal position
                    byte = pgm_read_byte(&conf_io_to_list[j][CONF_IO_TO_OUTBYTE]);
                    mask = 0 | (1 << pgm_read_byte(&conf_io_to_list[j][CONF_IO_TO_OUTBIT]));
                    if (byte >= IO_OUTPUT_SIZE)
                    {
                        // out of boundary of input array - generate error
                        ErrorCode.Type = CONF_IO_TO_ERROR_CODE;
                        ErrorCode.ListIndex = j;
                    } else {
                        hilo = pgm_read_byte(&conf_io_to_list[j][CONF_IO_TO_OUTHILO]);
                        if (ObjTurnout[i].St_State & TO_REQ_REVERSE)
                        {
                            if (hilo == HI) io_output[byte] |= mask; else io_output[byte] &= ~mask;
                        } else {
                            if (hilo == HI) io_output[byte] &= ~mask; else io_output[byte] |= mask;
                        }
                    }
                }
            }
        }
    }
    
    for (i = 0; i < N_SIGNALS; ++i)
    {
        for (j = 0; j < CONF_IO_SIG_ASPECT_LIST_SIZE; ++j)
        {
            if (pgm_read_byte(&conf_io_sig_aspect_list[j][CONF_IO_SIG_ASPECT_SIGNAL]) == i)
            {
                // signal found
                byte = pgm_read_byte(&conf_io_sig_aspect_list[j][CONF_IO_SIG_ASPECT_BYTE]);
                mask = 0 | (1 << pgm_read_byte(&conf_io_sig_aspect_list[j][CONF_IO_SIG_ASPECT_BIT]));
                if (byte >= IO_OUTPUT_SIZE)
                {
                    // out of boundary of input array - generate error
                    ErrorCode.Type = CONF_IO_SIG_ASPECT_ERROR_CODE;
                    ErrorCode.ListIndex = j;
                } else {
                    if (ObjSignal[i].St_Aspect == pgm_read_byte(&conf_io_sig_aspect_list[j][CONF_IO_SIG_ASPECT_NAME]))
                    {
                        // this signal has an aspect where there is a list entry
                        // check state
                        
                        hilo = pgm_read_byte(&conf_io_sig_aspect_list[j][CONF_IO_SIG_ASPECT_HILO]);
                       
                        if (pgm_read_byte(&conf_io_sig_aspect_list[j][CONF_IO_SIG_ASPECT_STATE]) == SIG_IO_STATE_ON)
                        {
                            if (hilo == HI) io_output[byte] |= mask; else /*(hilo == LO)*/ io_output[byte] &= ~mask;
                        } else if (pgm_read_byte(&conf_io_sig_aspect_list[j][CONF_IO_SIG_ASPECT_STATE]) == SIG_IO_STATE_OFF) {
                            if (hilo == HI) io_output[byte] &= ~mask; else /*(hilo == LO)*/ io_output[byte] |= mask;
                        } else if (pgm_read_byte(&conf_io_sig_aspect_list[j][CONF_IO_SIG_ASPECT_STATE]) == SIG_IO_STATE_FLASH) {
                            if (timing)
                            {
                                if (hilo == HI) io_output[byte] |= mask; else /*(hilo == LO)*/ io_output[byte] &= ~mask;
                            } else {
                                if (hilo == HI) io_output[byte] &= ~mask; else /*(hilo == LO)*/ io_output[byte] |= mask;
                            }
                        } else if (pgm_read_byte(&conf_io_sig_aspect_list[j][CONF_IO_SIG_ASPECT_STATE]) == SIG_IO_STATE_FL_ALT) {
                            if (!(timing))
                            {
                                if (hilo == HI) io_output[byte] |= mask; else /*(hilo == LO)*/ io_output[byte] &= ~mask;
                            } else {
                                if (hilo == HI) io_output[byte] &= ~mask; else /*(hilo == LO)*/ io_output[byte] |= mask;
                            }
                        }
                    }
                }
            }
        }
    }
    
 
    for (i = 0; i < N_GENERIC_OUTPUTS; ++i)
    {
        for (j = 0; j < CONF_IO_GO_LIST_SIZE; ++j)
        {
            if (pgm_read_byte(&conf_io_go_list[j][CONF_IO_GO_OUTPUT]) == i)
            {
                // generic output found
                byte = pgm_read_byte(&conf_io_go_list[j][CONF_IO_GO_BYTE]);
                mask = 0 | (1 << pgm_read_byte(&conf_io_go_list[j][CONF_IO_GO_BIT]));
                if (byte >= IO_OUTPUT_SIZE)
                {
                    // out of boundary of input array - generate error
                    ErrorCode.Type = CONF_IO_GO_ERROR_CODE;
                    ErrorCode.ListIndex = j;
                } else {
                    // list entry for generic output found
                    // check state
                    hilo = pgm_read_byte(&conf_io_go_list[j][CONF_IO_GO_HILO]);
                       
                        if (pgm_read_byte(&conf_io_sig_aspect_list[j][CONF_IO_SIG_ASPECT_STATE]) == SIG_IO_STATE_ON)
                        {
                            if (hilo == HI) io_output[byte] |= mask; else /*(hilo == LO)*/ io_output[byte] &= ~mask;
                        } else if (pgm_read_byte(&conf_io_sig_aspect_list[j][CONF_IO_SIG_ASPECT_STATE]) == SIG_IO_STATE_OFF) {
                            if (hilo == HI) io_output[byte] &= ~mask; else /*(hilo == LO)*/ io_output[byte] |= mask;
                        } else if (pgm_read_byte(&conf_io_sig_aspect_list[j][CONF_IO_SIG_ASPECT_STATE]) == SIG_IO_STATE_FLASH) {
                            if (timing)
                            {
                                if (hilo == HI) io_output[byte] |= mask; else /*(hilo == LO)*/ io_output[byte] &= ~mask;
                            } else {
                                if (hilo == HI) io_output[byte] &= ~mask; else /*(hilo == LO)*/ io_output[byte] |= mask;
                            }
                        } else if (pgm_read_byte(&conf_io_sig_aspect_list[j][CONF_IO_SIG_ASPECT_STATE]) == SIG_IO_STATE_FL_ALT) {
                            if (!(timing))
                            {
                                if (hilo == HI) io_output[byte] |= mask; else /*(hilo == LO)*/ io_output[byte] &= ~mask;
                            } else {
                                if (hilo == HI) io_output[byte] &= ~mask; else /*(hilo == LO)*/ io_output[byte] |= mask;
                            }
                        }
                    
                    if (ObjGenericOutput[i].St_State == SIG_IO_STATE_ON)
                    {
                        if (hilo == HI) io_output[byte] |= mask; else /*(hilo == LO)*/ io_output[byte] &= ~mask;
                    } else if (ObjGenericOutput[i].St_State == SIG_IO_STATE_OFF) {
                        if (hilo == HI) io_output[byte] &= ~mask; else /*(hilo == LO)*/ io_output[byte] |= mask;
                    } else if (ObjGenericOutput[i].St_State == SIG_IO_STATE_FLASH) {
                        if (timing)
                        {
                            if (hilo == HI) io_output[byte] |= mask; else /*(hilo == LO)*/ io_output[byte] &= ~mask;
                        } else {
                            if (hilo == HI) io_output[byte] &= ~mask; else /*(hilo == LO)*/ io_output[byte] |= mask;
                        }
                    } else if (ObjGenericOutput[i].St_State == SIG_IO_STATE_FL_ALT) {
                        if (!(timing))
                        {
                            if (hilo == HI) io_output[byte] |= mask; else /*(hilo == LO)*/ io_output[byte] &= ~mask;
                        } else {
                            if (hilo == HI) io_output[byte] &= ~mask; else /*(hilo == LO)*/ io_output[byte] |= mask;
                        }
                    }
                }
            }
        }
    }

            
}