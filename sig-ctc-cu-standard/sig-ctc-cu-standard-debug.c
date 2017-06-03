/*
    APB signaling Central Interlocking Unit
    
    Version 2.0, BETA 12.10.2012
    - complete rework for free configurability
    - MISSING: Packet Handler and Internal bus
    
    Version 2.0, BETA 23.02.2013:
    - some documentation added.
    
    Version 2.1, 14.02.2015
	- first test application for communication and local input/output
	- first test application for communication and local input/output
	
    Copyright 2009-2015 by Michael Prader
	Licensed under Creative Commons Attribution-Noncommercial-Share Alike 3.0
	
	Original MRBus concept and source code © by Nathan Holmes (www.mrbus.org),
	licensed under the GNU GPL v2.
	Thanks to Carsten Lundsten for his help.
*/

/********************************   EEPROM ERROR LOG  *************************************/

#define ERRORCODE_TYPE_EEPROM_LOCATION		0x01
#define ERRORCODE_LISTINDEX_EEPROM_LOCATION	0x02


/*********************************** TIMERS*********************************************/

#define PRESCALER           64      // prescaler for timer is 64
#define FREQUENCY           100     // value is in Hz
#define TCNT1_RELOAD_VAL    (0xFFFF - F_CPU/(PRESCALER*FREQUENCY)) //0xF3CB//(0xFFFF - F_CPU/(PRESCALER*FREQUENCY))

#define RUNNING_TIME			        (4*10)  // 10 secs
#define FORCE_RELEASE_TIME_LOCK_PRESET  (4*6)  // 6 sec
#define IR_SENSOR_DEBOUNCE_TIME         (4*2)   // 2 secs
#define TC_SENSOR_DEBOUNCE_TIME         (4*1)   // 1 sec
#define TO_POSITION_LOCKOUT_TIMER       (4*6)   // 6 secs
#define RT_ACTIVATION_TIMER_PRESET     (4*10)  // 10 secs



/*********************************** END OF CONFIGURATION *******************************/

/********************************* SYSTEM DEFINES AND INCLUDES ***************************/



#define WORK_BUFFER_SIZE    40



/***************************** COMMUNICATION-RELATED DEFINES AND INCLUDES ****************/


#include "../../libs/block/block-defines.h"
#include "../../libs/block/slip.c"

/*  Defines for events that require sending update packets. */
#define SEND_SAFETY_PACKET      0x01
#define SEND_EXT_SIG_PACKET     0x02
#define SEND_EXT_CIRC_PACKET    0x04
#define SEND_EXT_TO_PACKET      0x08
#define SEND_EXT_GI_PACKET		0x10

// these are internal command updates to the field devices
//#define SEND_INT_SIG_PACKET     0x20
#define RESTART_FLAG		    0x40
#define QUARTER_SEC             0x80

volatile unsigned char changed = 0;


#include "../../libs/typedefs/typedef-ee_operation.h"


/*****************************************  SAFETY LOGIC VARIABLES ***************************/
uint8_t ForceReleaseTimeLock;

// Global variables
TSignal ObjSignal[N_SIGNALS];
TBlockInterface ObjBlockInterface[N_BLOCKINTERFACES];
TRouteActivator RouteActivator[N_ROUTES];
TCircuit ObjCircuit[N_TRACK_CIRCUITS];
TIR_Sensor ObjIR_Sensor[N_IR_SENSORS];
TTurnout ObjTurnout[N_TURNOUTS];
TGenericOutput ObjGenericOutput[N_GENERIC_OUTPUTS];
TGenericInput ObjGenericInput[N_GENERIC_INPUTS];
TPhysicalBlockInterface ObjPhysicalBlockInterface[N_PHYSICAL_BLOCK_INTERFACES];
#ifdef AUTOMATION_USE_LOCKOUT
uint8_t AutomationLockout[N_SIGNALS];
#endif

#ifdef AUTOMATION_USE_AUTOMATION_FILE
#include "automation.h"
#endif

TErrorCode ErrorCode;


/******************************** PHYSICAL BLOCK HANDLING ROUTINES *********************/

#include "../../libs/block/block-master.c"


/***************************** INCLUDE GENERIC CTC COMMAND PROCESSES *******************/
#include "ctc-RouteActivation.c"

#include "ctc-CTCCmdProcesses.c"


/***************************** INCLUDE GENERIC SAFETY LOGIC PROCESSES *******************/

#include "ctc-SafetyProcesses.c"




#include "ctc-Handle-IO.c"

TEe_Operation EepromOperation;

uint8_t workbuffer[WORK_BUFFER_SIZE];
void OriginateBroadcastPacket(uint8_t *buffer, uint8_t BytesToSend);
void BroadcastPacketHandler(uint8_t *buffer, uint8_t ReceivedBytes);




void init(void);



// ******** Start 100 Hz Timer - Very Accurate Version

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks;
volatile uint8_t quartersecs;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCCR1A = 0;
	TCCR1B = _BV(CS11) | _BV(CS10);	// clk/64
	TCCR1C = 0;
	TIMSK1 |= _BV(TOIE1);
	ticks = 0;
}

ISR(TIMER1_OVF_vect)
{
	
	TCNT1 = TCNT1_RELOAD_VAL;
	
	++ticks;
	if (ticks >= 25)
	{
	   ticks = 0;
	   changed |= QUARTER_SEC;
	}
	
}


uint8_t IsOurAddress(uint8_t *addressField)
{
    if (    (*(addressField +0) == OUR_ADDR0) &&
            (*(addressField +1) == OUR_ADDR1) &&
            (*(addressField +2) == OUR_ADDR2)   )
    {
        // match on the address
        return 1;
        
    } else if (     (*(addressField +0) == 0xFF) &&
                    (*(addressField +1) == 0xFF) &&
                    (*(addressField +2) == 0xFF)    )
    {
        // match on the broadcast
        return 2;
        
    } else {
        // no match
        return 0;
    }
}




/******************************************************************************************/

int main(void)
{
	unsigned char i, j, k, l, m, n, o, p;
	
	volatile uint8_t *port_pointer;
	
RESTART:
	
	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	EepromOperation.St_State = EE_STATE_NO_OP;

    BlockUartInitialize();

    
    ErrorCode.Type = 0;

        
    // initialize block interfaces
    for (i = 0; i < N_BLOCKINTERFACES; ++i)
    {
        
        BlockRingBufferInitialize(&(BlockTXBuffer[i]));
        BlockRingBufferInitialize(&(BlockRXBuffer[i]));

        ObjBlockInterface[i].St_Config = 0;
        
        ObjBlockInterface[i].Add_I2C_Address = pgm_read_byte(&conf_io_block_list[i][CONF_IO_BLOCK_ADDRESS]);
        ObjBlockInterface[i].Nx_PhysicalNumber = pgm_read_byte(&conf_io_block_list[i][CONF_IO_BLOCK_PHYSICAL_NUMBER]);
        if ( pgm_read_byte(&conf_io_block_list[i][CONF_IO_BLOCK_IS_ON_I2C]) == IS_ON_I2C)
        {
			// this block is on I2c
            ObjBlockInterface[i].St_Config |= IS_ON_I2C;
        } else if (pgm_read_byte(&conf_io_block_list[i][CONF_IO_BLOCK_IS_ON_I2C]) == BLOCK_IS_VIRTUAL) {
			// this is a virtual block interface
			ObjBlockInterface[i].St_Config |= BLOCK_IS_VIRTUAL;
		} else {
            // this block is local
            if (ObjBlockInterface[i].Nx_PhysicalNumber >= N_PHYSICAL_BLOCK_INTERFACES)
            {
                 ErrorCode.Type = CONF_IO_BLOCK_LIST_ERROR_CODE;
                 ErrorCode.ListIndex = i;
                 for (j = 0; j < N_PHYSICAL_BLOCK_INTERFACES; ++j) ObjPhysicalBlockInterface[j].Nx_ReferencedLogicalBlock = 0;
                 continue;
            }
            
            ObjPhysicalBlockInterface[ObjBlockInterface[i].Nx_PhysicalNumber].Nx_ReferencedLogicalBlock = i;
		}
    }
            
            

    
	for (i = 0; i < N_GENERIC_OUTPUTS; ++i)
	{
		ObjGenericOutput[i].St_State = SIG_IO_STATE_ON;
	}

    for (i = 0; i < N_SIGNALS; ++i)
    {
        ObjSignal[i].St_APBout = ASP_STOP | STICK;
        ObjSignal[i].St_Control = 0;
        ObjSignal[i].Co_RunningTime = 0;
		ObjSignal[i].St_APBnext = ASP_CLEAR;
		ObjSignal[i].St_TrackCircuitNext = BLOCK_TRACK_CIRCUIT_FREE;
		ObjSignal[i].Nx_RouteSet = NO_ROUTE;
		ObjSignal[i].Nx_RequestedRoute = NO_ROUTE;
    }
    
    for (i = 0; i < N_TRACK_CIRCUITS; ++i)
    {
        ObjCircuit[i].St_State = 0;
		ObjCircuit[i].Co_Debounce = 0;
    }
    
    for (i = 0; i < N_TURNOUTS; ++i)
    {
        ObjTurnout[i].St_State = 0;
        ObjTurnout[i].Co_PositionLockout = 0;
    }
    
    for (i = 0; i < N_IR_SENSORS; ++i)
    {
        ObjIR_Sensor[i].Co_Debounce = 0;
        ObjIR_Sensor[i].St_State = 0;
    }
	
	
	
	/// TODO ::: IR sensors - how are they handled? Who debounces them? Are they already debounced/delayed?
	
	// Answer 28 November: if we use the new type of sensor from Nathan Holmes, turnouf delay
	// is already built-in, so we just need to read the new state.
	// It would be nice if this was configurable, in order to mix old and new sensors. At the moment,
	// sensor input is debounced by the logic.

        
    // Initialization of ports, field devices (I2), and so on must be done here!
    // configure local outputs
    for ( i = 0; i < CONF_IO_LOCAL_OUT_LIST_SIZE; ++i)
    {
        // read the index of the port
        j = pgm_read_byte(&conf_io_local_out_list[i][CONF_IO_LOCAL_OUT_PORT]);
        // read the pin
        k = pgm_read_byte(&conf_io_local_out_list[i][CONF_IO_LOCAL_OUT_PORTPIN]);
        if (    (k >= 8) || (j >= (HIGHEST_PORT_USED+1)))
        {
            // generate error
            ErrorCode.Type = CONF_IO_LOCAL_OUT_LIST_ERROR;
            ErrorCode.ListIndex = i;
            continue;
        }
        
        // valid port and pin found
        // configure as output
        port_pointer = gio_ddr_register[j];
        *port_pointer |= (1<<k);
    }
    

#ifdef AUTOMATION_USE_AUTOMATION_FILE
		Automation(0);
#endif



	PORTB &= ~_BV(4);
	
	changed = 0;
	
	UDR0 = 'B';

	/********************** START PRIMARY LOOP ********************************************************/
	while(1)
	{
		
		if (changed & RESTART_FLAG)
		{
			changed &= ~RESTART_FLAG;
			goto RESTART;
		}
		
		UDR0 = 'P';
		
		/*************************** HANDLE EEPROM ***************************************************/
		if (EepromOperation.St_State == EE_STATE_READ)
		{
		    if (!eeprom_is_ready()) eeprom_busy_wait();
		    EepromOperation.Val_Value = eeprom_read_byte((uint8_t *)EepromOperation.Add_Address);
		    
		    workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		    workbuffer[BLOCK_BROADCAST_HOPS] = 0;
		    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_EE_READ_RESP;
		    workbuffer[BLOCK_BROADCAST_EE_READ_ADDR] = EepromOperation.Add_Address;
		    workbuffer[BLOCK_BROADCAST_EE_READ_VAL] = EepromOperation.Val_Value;
		    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
		    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
		    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
		    workbuffer[BLOCK_BROADCAST_DEST0] = EepromOperation.Src_Source0;
		    workbuffer[BLOCK_BROADCAST_DEST1] = EepromOperation.Src_Source1;
		    workbuffer[BLOCK_BROADCAST_DEST2] = EepromOperation.Src_Source2;
		    
		    OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_EE_READ_RESP_SIZE);
		    
		    EepromOperation.St_State = EE_STATE_NO_OP;
		    
		} else if (EepromOperation.St_State == EE_STATE_WRITE) {
		    
		    if (!eeprom_is_ready()) eeprom_busy_wait();
		    eeprom_write_byte((uint8_t *)EepromOperation.Add_Address, EepromOperation.Val_Value);
		    
		    workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		    workbuffer[BLOCK_BROADCAST_HOPS] = 0;
		    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_EE_WRITE_RESP;
		    workbuffer[BLOCK_BROADCAST_EE_WRITE_ADDR] = EepromOperation.Add_Address;
		    workbuffer[BLOCK_BROADCAST_EE_WRITE_VAL] = EepromOperation.Val_Value;
		    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
		    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
		    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
		    workbuffer[BLOCK_BROADCAST_DEST0] = EepromOperation.Src_Source0;
		    workbuffer[BLOCK_BROADCAST_DEST1] = EepromOperation.Src_Source1;
		    workbuffer[BLOCK_BROADCAST_DEST2] = EepromOperation.Src_Source2;
		    
		    OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_EE_WRITE_RESP_SIZE);
		    
		    EepromOperation.St_State = EE_STATE_NO_OP;
		}
		
		/***************************************** TIMED EVENTS *************************/
        
        if (changed & QUARTER_SEC)
        {
            changed &= ~QUARTER_SEC;
			++quartersecs;
            
            // count down the locks for the force release function
            if (ForceReleaseTimeLock) --ForceReleaseTimeLock;

            // count down the Running Time on signals
            for (i = 0; i < N_SIGNALS; ++i)
            {
                if (ObjSignal[i].Co_RunningTime) --ObjSignal[i].Co_RunningTime;
            }
            
            
            for (i = 0; i < N_ROUTES; ++i)
            {
                // decrease the timeout on an active route activator
                if (RouteActivator[i].Co_TimeToLive)
                {
                    --RouteActivator[i].Co_TimeToLive;
                    // if last decrease passed to zero, call the Timeout routine with the route index
                    if (RouteActivator[i].Co_TimeToLive == 0)
                        RouteActivationTimeout(i);
                }
            }
            
            
            for (i = 0; i < N_IR_SENSORS; ++i)
            {
                // debounce the IR sensor inputs
                if (ObjIR_Sensor[i].Co_Debounce) --ObjIR_Sensor[i].Co_Debounce;
            }
            
			
			for (i = 0; i < N_TRACK_CIRCUITS; ++i)
			{
				// debounce the circuit inputs
				if (ObjCircuit[i].Co_Debounce) --ObjCircuit[i].Co_Debounce;
			}
			
            for (i = 0; i < N_TURNOUTS; ++i)
            {
                // count down the position lockout after a SetTurnout() command
                if (ObjTurnout[i].Co_PositionLockout) --ObjTurnout[i].Co_PositionLockout;
            }
            
            
            // Events that happen every 8 seconds
    		if (quartersecs >= 32) {
                cli();
    		  changed = SEND_SAFETY_PACKET | SEND_EXT_SIG_PACKET | SEND_EXT_TO_PACKET | SEND_EXT_CIRC_PACKET;
    		  quartersecs = 0;
    		  sei();

    	    }
    	}
    	
    	/**************************** HANDLE INPUT ************************************************************/
    	    

        // read local inputs
		for (i = 0; i < CONF_IO_LOCAL_IN_LIST_SIZE; ++i)
		{
			// read the index of the port
			j = pgm_read_byte(&conf_io_local_in_list[i][CONF_IO_LOCAL_IN_PORT]);
			// read the pin
			k = pgm_read_byte(&conf_io_local_in_list[i][CONF_IO_LOCAL_IN_PORTPIN]);
			// read the output byte
			l = pgm_read_byte(&conf_io_local_in_list[i][CONF_IO_LOCAL_IN_BYTE]);
			// read the output bit
			m = pgm_read_byte(&conf_io_local_in_list[i][CONF_IO_LOCAL_IN_BIT]);
			
			if (	(k >= 8) || (j >= (HIGHEST_PORT_USED+1)) ||	// error on the port
					(l >= IO_INPUT_SIZE) || (m >= 8) )				// error on the input byte
			{
				// generate error
				ErrorCode.Type = CONF_IO_LOCAL_IN_LIST_ERROR;
				ErrorCode.ListIndex = i;
				continue;
			}
			
			port_pointer = gio_pin_register[j];
			
			if (*port_pointer & (1<<k)) io_input[l] |= (1<<m); else io_input[l] &= ~(1<<m);
		}
           
            
			
		
        /*****************************  CALL SAFETY PROCESSES ***********************************************/
		MapInputStates();
		
		APBSafetyProcess();
	    CTCSafetyProcess();
	    
       MapOutputStates(quartersecs);
    
         /**************************** HANDLE OUTPUT ************************************************************/
    


		
		
		
	    // Output of local ports
		for ( i = 0; i < CONF_IO_LOCAL_OUT_LIST_SIZE; ++i)
		{
			// read the index of the port
			j = pgm_read_byte(&conf_io_local_out_list[i][CONF_IO_LOCAL_OUT_PORT]);
			// read the pin
			k = pgm_read_byte(&conf_io_local_out_list[i][CONF_IO_LOCAL_OUT_PORTPIN]);
			// read the output byte
			l = pgm_read_byte(&conf_io_local_out_list[i][CONF_IO_LOCAL_OUT_BYTE]);
			// read the output bit
			m = pgm_read_byte(&conf_io_local_out_list[i][CONF_IO_LOCAL_OUT_BIT]);
			
			if (    (k >= 8) || (j >= (HIGHEST_PORT_USED+1)) || 	// error on the port
					(l >= IO_OUTPUT_SIZE) || (m >= 8) )			// error on the output byte
			{
				// generate error
				ErrorCode.Type = CONF_IO_LOCAL_OUT_LIST_ERROR;
				ErrorCode.ListIndex = i;
				continue;
			}
			
			// valid port and pin found
			port_pointer = gio_port_register[j];
			
			// check requested state and set output
			if (io_output[l] & (1<<m)) *port_pointer |= (1<<k); else *port_pointer &= ~(1<<k);
		}
            
    
		/*********************  END SAFETY SECTION  *************************************************/
        
#ifdef AUTOMATION_USE_AUTOMATION_FILE
		Automation(1);
#endif
		/* Test if something changed from the last time
		   around the loop - we need to send an update
		   packet if it did */
		
		
        for (i = 0; i < N_TURNOUTS; ++i)
        {
            if (ObjTurnout[i].St_State != ObjTurnout[i].St_OldState)
            {
                ObjTurnout[i].St_OldState = ObjTurnout[i].St_State;
                changed |= SEND_EXT_TO_PACKET;
            }
        }
        
        for (i = 0; i < N_TRACK_CIRCUITS; ++i)
        {
            if (ObjCircuit[i].St_State != ObjCircuit[i].St_OldState)
            {
                ObjCircuit[i].St_OldState = ObjCircuit[i].St_State;
                changed |= SEND_EXT_CIRC_PACKET;
            }
        }
        
		
		for (i = 0; i < N_SIGNALS; ++i)
        {
            if (ObjSignal[i].St_Control != ObjSignal[i].St_OldControl)
            {
                ObjSignal[i].St_OldControl = ObjSignal[i].St_Control;
                changed |= SEND_EXT_SIG_PACKET;
            }
            
			if (ObjSignal[i].St_OldTrackCircuitA != ObjSignal[i].St_TrackCircuitA) ObjSignal[i].St_OldTrackCircuitA = ObjSignal[i].St_TrackCircuitA;
			if (ObjSignal[i].St_OldTrackCircuitDrop != ObjSignal[i].St_TrackCircuitDrop) ObjSignal[i].St_OldTrackCircuitDrop = ObjSignal[i].St_TrackCircuitDrop;
			
            if ((ObjSignal[i].St_APBout != ObjSignal[i].St_OldAPBout) ||
                (ObjSignal[i].St_TrackCircuitOut != ObjSignal[i].St_OldTrackCircuitOut))
            {
                ObjSignal[i].St_OldAPBout = ObjSignal[i].St_APBout;
                ObjSignal[i].St_OldTrackCircuitOut = ObjSignal[i].St_TrackCircuitOut;

                 // immediately send update on block interface i
                // encode to SLIP and push
				if (ObjBlockInterface[i].St_Config & BLOCK_IS_VIRTUAL) continue;
				
                BlockRingBufferPush(&(BlockTXBuffer[i]), END);
                slip_encode(BLOCK_TYPE_TRACK_CIRCUIT, &(BlockTXBuffer[i]));
                slip_encode(ObjSignal[i].St_TrackCircuitOut, &(BlockTXBuffer[i]));
                BlockRingBufferPush(&(BlockTXBuffer[i]), END);
                
                BlockRingBufferPush(&(BlockTXBuffer[i]), END);
                slip_encode(BLOCK_TYPE_SIG_STATE, &(BlockTXBuffer[i]));
                slip_encode(ObjSignal[i].St_APBout, &(BlockTXBuffer[i]));
                BlockRingBufferPush(&(BlockTXBuffer[i]), END);
				
                
            }
        }
		
		for (i = 0; i < N_GENERIC_INPUTS; ++i)
		{
			if (ObjGenericInput[i].St_State != ObjGenericInput[i].St_OldState) ObjGenericInput[i].St_OldState = ObjGenericInput[i].St_State;
			changed |= SEND_EXT_GI_PACKET;
		}
		
		// something changed
	    if (changed & SEND_SAFETY_PACKET)
		{
			for (i = 0; i < N_SIGNALS; ++i)
            {
                // send update on block interface i
                // encode to SLIP and push
                if (ObjBlockInterface[i].St_Config & BLOCK_IS_VIRTUAL) continue;
				
				BlockRingBufferPush(&(BlockTXBuffer[i]), END);
                slip_encode(BLOCK_TYPE_TRACK_CIRCUIT, &(BlockTXBuffer[i]));
                slip_encode(ObjSignal[i].St_TrackCircuitOut, &(BlockTXBuffer[i]));
                BlockRingBufferPush(&(BlockTXBuffer[i]), END);
                
                BlockRingBufferPush(&(BlockTXBuffer[i]), END);
                slip_encode(BLOCK_TYPE_SIG_STATE, &(BlockTXBuffer[i]));
                slip_encode(ObjSignal[i].St_APBout, &(BlockTXBuffer[i]));
                BlockRingBufferPush(&(BlockTXBuffer[i]), END);
                
            }
			changed &= ~SEND_SAFETY_PACKET;
	   
        /* Start of the new update routine
		} else if (changed & (SEND_EXT_CIRC_PACKET | SEND_EXT_TO_PACKET | SEND_EXT_SIG_PACKET)) {
		    workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		    workbuffer[BLOCK_BROADCAST_HOPS] = 0;
		    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_STATE;
		    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
		    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
		    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
		    workbuffer[BLOCK_BROADCAST_DEST0] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_DEST1] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_DEST2] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_STATE_TYPE] = BLOCK_BROADCAST_STATE_TYPE_CTC;
		    
		    // in any case write all three type quantities
		    workbuffer[BLOCK_BROADCAST_STATE_DATA +0] = N_TRACK_CIRCUITS;
		    workbuffer[BLOCK_BROADCAST_STATE_DATA +1] = N_TURNOUTS;
		    workbuffer[BLOCK_BROADCAST_STATE_DATA +2] = N_SIGNALS;
		    
		    //first put all track circuits
		    for (i = 0; i < N_TRACK_CIRCUITS; ++i)
		        workbuffer[BLOCK_BROADCAST_STATE_DATA +3 +i] = ObjCircuit[i].St_State;
		    
		    // hop over the track circuits and put all turnouts
		    for (i = 0; i < N_TURNOUTS; ++i)
                workbuffer[BLOCK_BROADCAST_STATE_DATA +3 + N_TRACK_CIRCUITS + i] = ObjTurnout[i].St_State;
            
            // after track circuits and turnouts put all signal controls
            for (i = 0; i < N_SIGNALS; ++i)
                workbuffer[BLOCK_BROADCAST_STATE_DATA + 3 + N_TRACK_CIRCUITS + N_TURNOUTS + i] = ObjSignal[i].St_Control;
                
            OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_STATE_DATA + 3 + N_TRACK_CIRCUITS + N_TURNOUTS + N_SIGNALS);
            
            changed &=  ~(SEND_EXT_CIRC_PACKET | SEND_EXT_TO_PACKET | SEND_EXT_SIG_PACKET);
		    
        */
		} else if ((changed & SEND_EXT_CIRC_PACKET) && (N_TRACK_CIRCUITS)){
			
			workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		    workbuffer[BLOCK_BROADCAST_HOPS] = 0;
		    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_STATE;
		    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
		    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
		    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
		    workbuffer[BLOCK_BROADCAST_DEST0] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_DEST1] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_DEST2] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_STATE_TYPE] = BLOCK_BROADCAST_STATE_TYPE_CIRCUIT;
		    
		    for (i = 0; i < N_TRACK_CIRCUITS; ++i)
		    {
                workbuffer[BLOCK_BROADCAST_STATE_DATA + i] = ObjCircuit[i].St_State;
		    }
		    OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_STATE_DATA + N_TRACK_CIRCUITS);
			
			changed &= ~SEND_EXT_CIRC_PACKET;
			
		/*} else if ((changed & SEND_EXT_TO_PACKET) && (N_TURNOUTS)) {
			workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		    workbuffer[BLOCK_BROADCAST_HOPS] = 0;
		    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_STATE;
		    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
		    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
		    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
		    workbuffer[BLOCK_BROADCAST_DEST0] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_DEST1] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_DEST2] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_STATE_TYPE] = BLOCK_BROADCAST_STATE_TYPE_TO;
		    
		    for (i = 0; i < N_TURNOUTS; ++i)
		    {
                workbuffer[BLOCK_BROADCAST_STATE_DATA + i] = ObjTurnout[i].St_State;
		    }
		    OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_STATE_DATA + N_TURNOUTS);
		    
		    changed &= ~SEND_EXT_TO_PACKET;
		  */
		} else if ((changed & SEND_EXT_SIG_PACKET) && (N_SIGNALS)) {
            workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		    workbuffer[BLOCK_BROADCAST_HOPS] = 0;
		    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_STATE;
		    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
		    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
		    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
		    workbuffer[BLOCK_BROADCAST_DEST0] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_DEST1] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_DEST2] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_STATE_TYPE] = BLOCK_BROADCAST_STATE_TYPE_SIG;
		    
		    for (i = 0; i < N_SIGNALS; ++i)
		    {
                workbuffer[BLOCK_BROADCAST_STATE_DATA + i] = ObjSignal[i].St_Control;
		    }
		    OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_STATE_DATA + N_SIGNALS);

			
			changed &= ~SEND_EXT_SIG_PACKET;
			
		//    Till here: old update routine.
		//    It is heavier on the block interfaces' load,
		//    therefore better for testing the IIC bus.  */
		
/*		} else if ((changed & SEND_EXT_GI_PACKET) && (N_GENERIC_INPUTS)) {
			
			
			workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		    workbuffer[BLOCK_BROADCAST_HOPS] = 0;
		    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_STATE;
		    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
		    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
		    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
		    workbuffer[BLOCK_BROADCAST_DEST0] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_DEST1] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_DEST2] = 0xFF;
		    workbuffer[BLOCK_BROADCAST_STATE_TYPE] = BLOCK_BROADCAST_STATE_TYPE_GI;
			
			for (i = 0; i < N_GENERIC_INPUTS; ++i)
		    {
                workbuffer[BLOCK_BROADCAST_STATE_DATA + i] = ObjGenericInput[i].St_State;
		    }
			
			OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_STATE_DATA + N_GENERIC_INPUTS );
			
			changed &= ~SEND_EXT_GI_PACKET;
			*/
		}
		
		
        
		// write last error code
		if (ErrorCode.Type)
	   {
    		if (!eeprom_is_ready()) eeprom_busy_wait();
    		eeprom_write_byte((uint8_t *)ERRORCODE_TYPE_EEPROM_LOCATION, ErrorCode.Type);
    		
    		if (!eeprom_is_ready()) eeprom_busy_wait();
    		eeprom_write_byte((uint8_t *)ERRORCODE_LISTINDEX_EEPROM_LOCATION, ErrorCode.ListIndex);
    		
    		ErrorCode.Type = 0;
	   }
		// If we have a packet to be transmitted, try to send it here
		if (BlockRingBufferDepth(&(BlockTXBuffer[ObjPhysicalBlockInterface[0].Nx_ReferencedLogicalBlock]))) ACTIVATE_PHYSICAL_BLOCK0_TX;
	#if (N_PHYSICAL_BLOCK_INTERFACES >= 2)
		if (BlockRingBufferDepth(&(BlockTXBuffer[ObjPhysicalBlockInterface[1].Nx_ReferencedLogicalBlock]))) ACTIVATE_PHYSICAL_BLOCK1_TX;
	#endif
	
	}
}

void OriginateBroadcastPacket(uint8_t *buffer, uint8_t BytesToSend)
{
    uint8_t i, index;
    uint16_t crc = 0;
    
    // we need to calculate the CRC16
	for (index = BLOCK_BROADCAST_FIRST_DATA; index < BytesToSend; ++index)
		crc = mrbusCRC16Update(crc, buffer[index]);
	buffer[BLOCK_BROADCAST_CRC_H] = UINT16_HIGH_BYTE(crc);
	buffer[BLOCK_BROADCAST_CRC_L] = UINT16_LOW_BYTE(crc);
    
    for (i = 0; i < N_BLOCKINTERFACES; ++i)
	{
        // only on interfaces that have broadcast enabled
        if (!(ObjBlockInterface[i].St_Config & BROADCAST_SHUT_OFF) && !(ObjBlockInterface[i].St_Config & BLOCK_IS_VIRTUAL))
        {
            // encode to SLIP and push
            BlockRingBufferPush(&(BlockTXBuffer[i]), END);
            for (index = 0; index < BytesToSend; ++index) slip_encode(buffer[index], &(BlockTXBuffer[i]));
            BlockRingBufferPush(&(BlockTXBuffer[i]), END);
        }
	}
}



void BroadcastPacketHandler(uint8_t *buffer, uint8_t ReceivedBytes)
{
    // Broadcast packet, check CRC
    uint8_t i, index;
	uint16_t crc = 0;
    uint8_t src[3];
	
	for (i = BLOCK_BROADCAST_FIRST_DATA; i < ReceivedBytes; ++i)
		crc = mrbusCRC16Update(crc, buffer[i]);

	if ((UINT16_HIGH_BYTE(crc) != buffer[BLOCK_BROADCAST_CRC_H]) || (UINT16_LOW_BYTE(crc) != buffer[BLOCK_BROADCAST_CRC_L])) goto PktExit;
    // => Error in CRC

    if (buffer[BLOCK_BROADCAST_SUBTYPE] == BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_APB_OUT)
    {
        // read DataDepth of ringbuffers
        src[0] = workbuffer[BLOCK_BROADCAST_SRC0];
        src[1] = workbuffer[BLOCK_BROADCAST_SRC1];
        src[2] = workbuffer[BLOCK_BROADCAST_SRC2];
        
        workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		workbuffer[BLOCK_BROADCAST_HOPS] = 0;
	    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_APB_OUT + 0xA0;
	    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
	    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
	    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
	    workbuffer[BLOCK_BROADCAST_DEST0] = src[0];
	    workbuffer[BLOCK_BROADCAST_DEST1] = src[1];
	    workbuffer[BLOCK_BROADCAST_DEST2] = src[2];
        for (i = 0; i < N_SIGNALS; ++i)
            workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA+i] = ObjSignal[i].St_APBout;
	    
	    OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_FIRST_USER_DATA+N_SIGNALS);
		
		
    } else if (buffer[BLOCK_BROADCAST_SUBTYPE] == BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_SIGNAL_STATE) {
        // read DataDepth of ringbuffers
        src[0] = workbuffer[BLOCK_BROADCAST_SRC0];
        src[1] = workbuffer[BLOCK_BROADCAST_SRC1];
        src[2] = workbuffer[BLOCK_BROADCAST_SRC2];
        
        workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		workbuffer[BLOCK_BROADCAST_HOPS] = 0;
	    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_SIGNAL_STATE + 0xA0;
	    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
	    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
	    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
	    workbuffer[BLOCK_BROADCAST_DEST0] = src[0];
	    workbuffer[BLOCK_BROADCAST_DEST1] = src[1];
	    workbuffer[BLOCK_BROADCAST_DEST2] = src[2];
        for (i = 0; i < N_SIGNALS; ++i)
            workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA+i] = ObjSignal[i].St_Aspect;
	    
	    OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_FIRST_USER_DATA+N_SIGNALS);
        
    } else if (buffer[BLOCK_BROADCAST_SUBTYPE] == BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_DEPTH)
    {
        // read DataDepth of ringbuffers
        src[0] = workbuffer[BLOCK_BROADCAST_SRC0];
        src[1] = workbuffer[BLOCK_BROADCAST_SRC1];
        src[2] = workbuffer[BLOCK_BROADCAST_SRC2];
        
        workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		workbuffer[BLOCK_BROADCAST_HOPS] = 0;
	    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_DEPTH + 0xA0;
	    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
	    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
	    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
	    workbuffer[BLOCK_BROADCAST_DEST0] = src[0];
	    workbuffer[BLOCK_BROADCAST_DEST1] = src[1];
	    workbuffer[BLOCK_BROADCAST_DEST2] = src[2];
        for (i = 0; i < N_BLOCKINTERFACES; ++i)
            workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA+i] = BlockRingBufferDepth(&BlockTXBuffer[i]);
	    
	    OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_FIRST_USER_DATA+N_BLOCKINTERFACES);
        
    } else if (buffer[BLOCK_BROADCAST_SUBTYPE] == BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_TWI) {

#ifdef USES_I2C

        // read TWI_Operation and status reg
        src[0] = workbuffer[BLOCK_BROADCAST_SRC0];
        src[1] = workbuffer[BLOCK_BROADCAST_SRC1];
        src[2] = workbuffer[BLOCK_BROADCAST_SRC2];
        
        workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		workbuffer[BLOCK_BROADCAST_HOPS] = 0;
	    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_TWI + 0xA0;
	    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
	    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
	    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
	    workbuffer[BLOCK_BROADCAST_DEST0] = src[0];
	    workbuffer[BLOCK_BROADCAST_DEST1] = src[1];
	    workbuffer[BLOCK_BROADCAST_DEST2] = src[2];
        workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA] = TWI_Operation.Operation;
        workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA+1] = TWI_Operation.Suboperation;
        workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA+2] = TWI_Operation.ElementNumber;
        workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA+3] = TWI_Operation.ReadLength;
		workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA+4] = TWI_statusReg.all;
		workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA+5] = TWI_statusReg.lastTransOK;
	    
	    OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_FIRST_USER_DATA+6);
        
#endif
    
    } else if (buffer[BLOCK_BROADCAST_SUBTYPE] == BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_OUTPUTS) {


        // read TWI_Operation and status reg
        src[0] = workbuffer[BLOCK_BROADCAST_SRC0];
        src[1] = workbuffer[BLOCK_BROADCAST_SRC1];
        src[2] = workbuffer[BLOCK_BROADCAST_SRC2];
        
        workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		workbuffer[BLOCK_BROADCAST_HOPS] = 0;
	    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_OUTPUTS + 0xA0;
	    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
	    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
	    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
	    workbuffer[BLOCK_BROADCAST_DEST0] = src[0];
	    workbuffer[BLOCK_BROADCAST_DEST1] = src[1];
	    workbuffer[BLOCK_BROADCAST_DEST2] = src[2];
		
		for (i = 0; i < IO_OUTPUT_SIZE; ++i)
			workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA+i] = io_output[i];
			
	    OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_FIRST_USER_DATA+IO_OUTPUT_SIZE);
               
	} else if (buffer[BLOCK_BROADCAST_SUBTYPE] == BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_INPUTS) {


        // read TWI_Operation and status reg
        src[0] = workbuffer[BLOCK_BROADCAST_SRC0];
        src[1] = workbuffer[BLOCK_BROADCAST_SRC1];
        src[2] = workbuffer[BLOCK_BROADCAST_SRC2];
        
        workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		workbuffer[BLOCK_BROADCAST_HOPS] = 0;
	    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_INPUTS + 0xA0;
	    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
	    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
	    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
	    workbuffer[BLOCK_BROADCAST_DEST0] = src[0];
	    workbuffer[BLOCK_BROADCAST_DEST1] = src[1];
	    workbuffer[BLOCK_BROADCAST_DEST2] = src[2];
		
		for (i = 0; i < IO_INPUT_SIZE; ++i)
			workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA+i] = io_input[i];
			
	    OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_FIRST_USER_DATA+IO_INPUT_SIZE);
                
    } else if (buffer[BLOCK_BROADCAST_SUBTYPE] == BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_WRITE_TWI) {
        
#ifdef USES_I2C
		// set TWI
        TWI_Operation.Operation = workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA];
        TWI_Operation.Suboperation = workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA+1];
        TWI_Operation.ElementNumber = workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA+2];
#endif
        
    } else if (buffer[BLOCK_BROADCAST_SUBTYPE] == BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_RESTART) {
	
        changed  |= RESTART_FLAG;
        
    } else if (buffer[BLOCK_BROADCAST_SUBTYPE] == BLOCK_BROADCAST_SUBTYPE_AMERICAN_EE_READ_CMD) {
        // store operation parameters
        if (EepromOperation.St_State == EE_STATE_NO_OP)
        {
            EepromOperation.Add_Address = buffer[BLOCK_BROADCAST_EE_READ_ADDR];
            EepromOperation.Val_Value = 0;
            EepromOperation.Src_Source0 = buffer[BLOCK_BROADCAST_SRC0];
            EepromOperation.Src_Source1 = buffer[BLOCK_BROADCAST_SRC1];
            EepromOperation.Src_Source2 =  buffer[BLOCK_BROADCAST_SRC2];
            EepromOperation.St_State = EE_STATE_READ;
        }
    } else if (buffer[BLOCK_BROADCAST_SUBTYPE] == BLOCK_BROADCAST_SUBTYPE_AMERICAN_EE_WRITE_CMD) {
        // store operation parameters
        if (EepromOperation.St_State == EE_STATE_NO_OP)
        {
            EepromOperation.Add_Address = buffer[BLOCK_BROADCAST_EE_WRITE_ADDR];
            EepromOperation.Val_Value = buffer[BLOCK_BROADCAST_EE_WRITE_VAL];
            EepromOperation.Src_Source0 = buffer[BLOCK_BROADCAST_SRC0];
            EepromOperation.Src_Source1 = buffer[BLOCK_BROADCAST_SRC1];
            EepromOperation.Src_Source2 =  buffer[BLOCK_BROADCAST_SRC2];
            EepromOperation.St_State = EE_STATE_WRITE;
        }
    } else if (buffer[BLOCK_BROADCAST_SUBTYPE] == BLOCK_BROADCAST_SUBTYPE_AMERICAN_PING) {
            src[0] = workbuffer[BLOCK_BROADCAST_SRC0];
            src[1] = workbuffer[BLOCK_BROADCAST_SRC1];
            src[2] = workbuffer[BLOCK_BROADCAST_SRC2];

            workbuffer[BLOCK_TYPE] = BLOCK_TYPE_BROADCAST;
		    workbuffer[BLOCK_BROADCAST_HOPS] = 0;
		    workbuffer[BLOCK_BROADCAST_SUBTYPE] = BLOCK_BROADCAST_SUBTYPE_AMERICAN_PING_RESPONSE;
		    workbuffer[BLOCK_BROADCAST_SRC0] = OUR_ADDR0;
		    workbuffer[BLOCK_BROADCAST_SRC1] = OUR_ADDR1;
		    workbuffer[BLOCK_BROADCAST_SRC2] = OUR_ADDR2;
		    workbuffer[BLOCK_BROADCAST_DEST0] = src[0];
		    workbuffer[BLOCK_BROADCAST_DEST1] = src[1];
		    workbuffer[BLOCK_BROADCAST_DEST2] = src[2];
		    workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA] = 0xAA;
		    
		    OriginateBroadcastPacket(workbuffer, BLOCK_BROADCAST_FIRST_USER_DATA+1);
		    
		    
    } else if (buffer[BLOCK_BROADCAST_SUBTYPE] == BLOCK_BROADCAST_SUBTYPE_AMERICAN_COMMAND) {

        switch (buffer[BLOCK_BROADCAST_COMMAND_TYPE])
        {
            case BLOCK_BROADCAST_COMMAND_TYPE_GO:
                if (buffer[BLOCK_BROADCAST_COMMAND_ELEMENT] >= N_GENERIC_OUTPUTS)  break;
                switch (buffer[BLOCK_BROADCAST_COMMAND_CMD])
                {
                    case BLOCK_BROADCAST_COMMAND_CMD_ON: ObjGenericOutput[buffer[BLOCK_BROADCAST_COMMAND_ELEMENT]].St_State = SIG_IO_STATE_ON; break;
                    case BLOCK_BROADCAST_COMMAND_CMD_OFF: ObjGenericOutput[buffer[BLOCK_BROADCAST_COMMAND_ELEMENT]].St_State = SIG_IO_STATE_OFF; break;
                    case BLOCK_BROADCAST_COMMAND_CMD_FLASH: ObjGenericOutput[buffer[BLOCK_BROADCAST_COMMAND_ELEMENT]].St_State = SIG_IO_STATE_FLASH; break;
                    case BLOCK_BROADCAST_COMMAND_CMD_FL_ALT: ObjGenericOutput[buffer[BLOCK_BROADCAST_COMMAND_ELEMENT]].St_State = SIG_IO_STATE_FL_ALT; break;
                }
                break;
            
            case BLOCK_BROADCAST_COMMAND_TYPE_CLEARANCE:
                if (buffer[BLOCK_BROADCAST_COMMAND_ELEMENT] >= N_SIGNALS)  break;
                switch (buffer[BLOCK_BROADCAST_COMMAND_CMD])
                {
                    case BLOCK_BROADCAST_COMMAND_CMD_CLEAR: ClearSignal(buffer[BLOCK_BROADCAST_COMMAND_ELEMENT]); break;
                    case BLOCK_BROADCAST_COMMAND_CMD_DROP: DropSignal(buffer[BLOCK_BROADCAST_COMMAND_ELEMENT]); break;
                    case BLOCK_BROADCAST_COMMAND_CMD_FLEET: SetFleet(buffer[BLOCK_BROADCAST_COMMAND_ELEMENT]); break;
                    case BLOCK_BROADCAST_COMMAND_CMD_NFLEET: DropFleet(buffer[BLOCK_BROADCAST_COMMAND_ELEMENT]); break;
                    default: break;
                }
                break;
				
			case BLOCK_BROADCAST_COMMAND_TYPE_ROUTE:
				RouteActivationStartRoute(buffer[BLOCK_BROADCAST_COMMAND_ROUTE_ENTRY], buffer[BLOCK_BROADCAST_COMMAND_ROUTE_EXIT]);
				break;

            case BLOCK_BROADCAST_COMMAND_TYPE_TURNOUT:
                if (buffer[BLOCK_BROADCAST_COMMAND_ELEMENT] >= N_TURNOUTS)  break;
                SetTurnout(buffer[BLOCK_BROADCAST_COMMAND_ELEMENT], buffer[BLOCK_BROADCAST_COMMAND_CMD]);
                break;

            case BLOCK_BROADCAST_COMMAND_TYPE_SAFETY:
                if (buffer[BLOCK_BROADCAST_COMMAND_ELEMENT] >= N_SIGNALS)  break;
                // first find circuit A for the signal
                for (i = 0; i < CONF_SAFE_SIG_APB_A_LIST_SIZE; ++i)
                {
                    if (pgm_read_byte(&conf_safe_sig_apb_a_list[i][CONF_SAFE_SIG_APB_A_SIGNAL]) == buffer[BLOCK_BROADCAST_COMMAND_ELEMENT])
                    {
                        //signal found - check if circuit index is out of boundary
                        index = pgm_read_byte(&conf_safe_sig_apb_a_list[i][CONF_SAFE_SIG_APB_A_CIRCUIT]);
                        if ( index >= N_TRACK_CIRCUITS)
                        {
                            // generate error
                            ErrorCode.Type = CONF_SAFE_SIG_APB_A_ERROR_CODE;
                            ErrorCode.ListIndex = i;
                            continue;
                        }
                        switch (buffer[BLOCK_BROADCAST_COMMAND_CMD])
                        {
                            case BLOCK_BROADCAST_COMMAND_CMD_SET_OCC: ObjCircuit[index].St_State |= F_OCC_ACTIVE; break;
                            case BLOCK_BROADCAST_COMMAND_CMD_CLEAR_OCC: ObjCircuit[index].St_State &= ~F_OCC_ACTIVE; break;
                            case BLOCK_BROADCAST_COMMAND_CMD_SET_REL: ObjCircuit[index].St_State |= F_REL_ACTIVE; break;
                            case BLOCK_BROADCAST_COMMAND_CMD_CLEAR_REL: ObjCircuit[index].St_State &= ~F_REL_ACTIVE; break;
                            default: break;
                        }
                    }
                }
                break;

            case BLOCK_BROADCAST_COMMAND_TYPE_APPR_LIT:
                // for future expansion of functionality
                break;
            
            default: break;
        }
    }


    PktExit:
	asm("nop");
    
}

void init(void)
{
	ASSR = 0;
  //  GIMSK = 0x00;
	
	ACSR = _BV(ACD);
	ADCSRA = 0;
	
#ifdef USES_I2C
	/* configure TWI pins */
	TWI_DDR &= ~(TWI_INT | TWI_SDA | TWI_SCL);
	TWI_PORT &= ~(TWI_SDA | TWI_SCL | TWI_INT); // disable pull-ups and set low
	
#endif

}
