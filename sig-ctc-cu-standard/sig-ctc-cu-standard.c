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

/********************************   EEPROM LOCATIONS  *************************************/

#define ERRORCODE_TYPE_EEPROM_LOCATION		0x01
#define ERRORCODE_LISTINDEX_EEPROM_LOCATION	0x02



/*********************************** TIMERS*********************************************/
#define ACCURATE_TIMER
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


#ifdef USES_I2C
#include "../../libs/twi-master/TWI_master.c"
TTWI_Operation TWI_Operation;
#endif





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

#include "ctc-APBClearIndication.c"


#include "ctc-Handle-IO.c"

TEe_Operation EepromOperation;

uint8_t workbuffer[WORK_BUFFER_SIZE];
void OriginateBroadcastPacket(uint8_t *buffer, uint8_t BytesToSend);
void BroadcastPacketHandler(uint8_t *buffer, uint8_t ReceivedBytes);



void SendPacket(uint8_t *buffer, uint8_t BytesToSend, unsigned char i);



void init(void);

#ifdef ACCURATE_TIMER

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
	
	TCNT1 += TCNT1_RELOAD_VAL;
	
	++ticks;
	if (ticks >= 25)
	{
	   ticks = 0;
	   changed |= QUARTER_SEC;
	}
}
#endif

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



#ifdef USES_I2C

uint8_t LastErrorMessage = 0xFF;

unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg )
{
                    // A failure has occurred, use TWIerrorMsg to determine the nature of the failure
                    // and take appropriate actions.
                    // Se header file for a list of possible failure messages.
                    
                    // Here is a simple sample, where if received a NACK on the slave address,
                    // then a retransmission will be initiated.

 
    if (TWIerrorMsg == TWI_MTX_ADR_NACK)
          TWI_Start_Transceiver();
    else if ( TWIerrorMsg == TWI_BUS_ERROR) {
        TWI_Master_Initialise();
    }
  
  //TWI_statusReg.lastTransOK = 1;
/*
    TWI_Operation.Operation = NO_OP;
    TWI_Operation.Suboperation = FALSE;
    TWI_Operation.ElementNumber = 0;
    TWI_statusReg.lastTransOK = 1;
*/
    

  return TWIerrorMsg;
}
#endif

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
            
            
    
    #ifdef USES_I2C
        TWI_Master_Initialise();
        TWI_statusReg.lastTransOK = TRUE;
        TWI_Operation.Operation = NO_OP;
    #endif

    
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


  #ifdef USES_I2C
	// delay for  slave startup, gives  25-49 ms
	changed &= ~QUARTER_SEC;
	sei();
	
	
	while (!(changed & QUARTER_SEC));
    
    // Initialize all I2C devices here
    // Activate DDRx registers on I2C
	TWI_Operation.Operation = TWI_ACTIVATE_OUTPUTS;
   // TWI_Operation.Operation = NO_OP;
    TWI_Operation.ElementNumber = 0;
    TWI_Operation.Suboperation = W_CMD_ACTIVATE_OUTPUTS;
 //   TWI_Operation.Suboperation = FALSE;
    
    
    while(1)
    {

        
        if ( ! TWI_Transceiver_Busy() )
        {
            // Check if the last operation was successful
            if ( TWI_statusReg.lastTransOK )
            {
                // Determine what action to take now
                if (TWI_Operation.Operation == TWI_ACTIVATE_OUTPUTS)
                {
                    if (TWI_Operation.ElementNumber < CONF_IO_I2C_DEV_LIST_SIZE)
                    {
                        if (TWI_Operation.Suboperation == W_CMD_ACTIVATE_OUTPUTS)
                        {
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                            j = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_HIGHEST_PORT]) +1;
                            
                            workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);
                            workbuffer[I2C_CMD] = W_CMD_ACTIVATE_OUTPUTS;
                            workbuffer[I2C_LEN] = W_CMD_ACTIVATE_OUTPUTS_LEN + j;
							
                            // clear all data bytes first, so we can simply set the individual bits
                            for ( k = (W_CMD_ACTIVATE_OUTPUTS_LEN); k < sizeof(workbuffer); ++k)  workbuffer[k] = 0;
                            
                            for (k = 0; k < CONF_IO_I2C_OUT_LIST_SIZE; ++k)
                            {
                                l = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_DEVICE_ADDRESS]);
                                m = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_GIO]);
                                n = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_PORTPIN]);
                                o = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_BYTE]);
                                p = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_BIT]);
                                
                                // check that pin is not higher than 7, port not higher than the declared highest port of the device,
                                // the byte not higher than the output list, and the bit not higher than 7
                                if (    (n >= 8) || (m >= j) || ( o >= IO_OUTPUT_SIZE) || (p >= 8))
                                {
                                    ErrorCode.Type = CONF_IO_I2C_OUT_LIST_ERROR_CODE;
                                    ErrorCode.ListIndex = k;
                                    continue;
                                }
                                
                                if (l == i) // line matches current device
                                {
                                    // o is the byte, p the bit; m is the port, n the pin
                                    // should be this one here:
                                    workbuffer[W_CMD_ACTIVATE_OUTPUTS_LEN + m] |= (1 << n); // the byte m is equal to the port; the port pin n is masked. Slave puts this onto DDRx
                                }
                            }
                            
                            // Start transceiver
                            TWI_Start_Transceiver_With_Data(workbuffer, W_CMD_ACTIVATE_OUTPUTS_LEN + j);
                            
                            // set next operation
							TWI_Operation.Suboperation = W_CMD_ACTIVATE_INPUTS;
							
                        } else if (TWI_Operation.Suboperation == W_CMD_ACTIVATE_INPUTS) {
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                            j = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_HIGHEST_PORT]) +1;
                            
                            workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);
                            workbuffer[I2C_CMD] = W_CMD_ACTIVATE_INPUTS;
                            workbuffer[I2C_LEN] = W_CMD_ACTIVATE_INPUTS_LEN + j;
							
                            // clear all data bytes first, so we can simply set the individual bits
                            for ( k = (W_CMD_ACTIVATE_INPUTS_LEN); k < sizeof(workbuffer); ++k)  workbuffer[k] = 0;
                            
                            for (k = 0; k < CONF_IO_I2C_IN_LIST_SIZE; ++k)
                            {
                                l = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_DEVICE_ADDRESS]);
                                m = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_GIO]);
                                n = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_PORTPIN]);
                                o = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_BYTE]);
                                p = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_BIT]);
                                
                                // check that pin is not higher than 7, port not higher than the declared highest port of the device,
                                // the byte not higher than the input list, and the bit not higher than 7
                                if (    (n >= 8) || (m >= j) || ( o >= IO_INPUT_SIZE) || (p >= 8))
                                {
                                    ErrorCode.Type = CONF_IO_I2C_IN_LIST_ERROR_CODE;
                                    ErrorCode.ListIndex = k;
                                    continue;
                                }
                                
                                if (l == i) // line matches current device
                                {
                                    // o is the byte, p the bit; m is the port, n the pin
                                    // should be this one here:
                                    workbuffer[W_CMD_ACTIVATE_INPUTS_LEN + m] |= (1 << n); // the byte m is equal to the port; the port pin n is masked. Slave puts this onto DDRx
                                }
                            }
                            
                            // Start transceiver
                            TWI_Start_Transceiver_With_Data(workbuffer, W_CMD_ACTIVATE_INPUTS_LEN + j);
                            
                            // all operations on this element are concluded, hop to next Element
                            ++TWI_Operation.ElementNumber;
                            
						}
						
                    } else {
                            // we are out of bounds for the devices => go back to idle state
                            TWI_Operation.Operation = NO_OP;
                            TWI_Operation.ElementNumber = 0;
                            TWI_Operation.Suboperation = FALSE;
                            goto I2C_INIT_END;  // we have concluded all operations of the initialization, so break the while(1) loop
                    }
                }
            } else { // Got an error during the last transmission
                // Use TWI status information to detemine cause of failure and take appropriate actions.
                TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info( ) );
            }
        // end of I2C configuration init
        }
    } // end of while() loop;
    
    I2C_INIT_END:
                
    #endif
	
    sei();
 

	changed = 0;

	/********************** START PRIMARY LOOP ********************************************************/
	while(1)
	{
		
		if (changed & RESTART_FLAG)
		{
			changed &= ~RESTART_FLAG;
			goto RESTART;
		}
		
		
		
		
		
		
		
		
		
		
		
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
		
		
		


/********************/
		
		
		/*************************** READ BLOCK DATA ***************************************************/
		for (i = 0; i < N_BLOCKINTERFACES; ++i)
		{
			uint8_t RxIdx, TxIdx;
			
		    if (BlockRXBuffer[i].bufferState)
			{
				// we received an END indicator byte, process data; decrease by one
				cli();
				BlockRXBuffer[i].bufferState -= 1;
				sei();
				RxIdx = 0;
				
				while (BlockRingBufferDepth(&(BlockRXBuffer[i])))
				{
					// there is data in the block input buffer
					j = slip_decode(&(BlockRXBuffer[i]), &(workbuffer[RxIdx]));
					if (!j) RxIdx += 1;
					if (RxIdx >= WORK_BUFFER_SIZE) RxIdx = WORK_BUFFER_SIZE - 1;
					
					
					// if j is 1, the routine completed copying the packet into the workbuffer;
					// at the end of the following block, we processed the packet, thus break the while, and wait
					// for a new END byte to be signaled by the INT routine (or start processing it, if already available)
					if (j)
					{
						// check whether this was a packet or a flush END
						if (RxIdx == 0) break;
						
						
						// we received at least one byte; therefore, we may suppose
						// it is the TYPE indicator, and check for it (BLOCK_TYPE is 0 - see block-com.h)
						if (  (workbuffer[BLOCK_TYPE] == BLOCK_TYPE_BROADCAST) &&
						      (RxIdx >= BLOCK_BROADCAST_MIN_PACKET_SIZE)  )
						  
						{
							
							if (IsOurAddress(&workbuffer[BLOCK_BROADCAST_DEST0]) == 1)
							{
								// packet is for us, inspect it
								BroadcastPacketHandler(workbuffer, RxIdx);
								
							}  else if (IsOurAddress(&workbuffer[BLOCK_BROADCAST_DEST0]) == 2) {
							    // packet is a broadcast
                                // first rebroadcast, then inspect it; content should be equal
                                // increase hops
								workbuffer[BLOCK_BROADCAST_HOPS] += 1;
								for (k = 0; k < N_BLOCKINTERFACES; ++k)
								{
									// only on interfaces that have broadcast enabled,
									// and not on the one the packet came from
									// no output on virtual block interface
									if (ObjBlockInterface[i].St_Config & BLOCK_IS_VIRTUAL) continue;
									
									if ((i != k) &&
										!(ObjBlockInterface[k].St_Config & BROADCAST_SHUT_OFF))
									{
										// encode to SLIP and push
										BlockRingBufferPush(&(BlockTXBuffer[k]), END);
										for (TxIdx = 0; TxIdx < RxIdx; ++TxIdx)
											slip_encode(workbuffer[TxIdx], &(BlockTXBuffer[k]));
										BlockRingBufferPush(&(BlockTXBuffer[k]), END);
									}
								}
                                BroadcastPacketHandler(workbuffer, RxIdx);
							    
							} else if (IsOurAddress(&workbuffer[BLOCK_BROADCAST_SRC0]) == 0)  {
								// if we are not the destination,
								// and not the source (so it's not a looped packet),
								// re-broadcast packet
							
								//increase hops
								workbuffer[BLOCK_BROADCAST_HOPS] += 1;
								for (k = 0; k < N_BLOCKINTERFACES; ++k)
								{
									// only on interfaces that have broadcast enabled,
									// and not on the one the packet came from
									// no output on virtual block interface
									if (ObjBlockInterface[i].St_Config & BLOCK_IS_VIRTUAL) continue;
									
									if ((i != k) &&
										!(ObjBlockInterface[k].St_Config & BROADCAST_SHUT_OFF))
									{
										// encode to SLIP and push
										BlockRingBufferPush(&(BlockTXBuffer[k]), END);
										for (TxIdx = 0; TxIdx < RxIdx; ++TxIdx)
											slip_encode(workbuffer[TxIdx], &(BlockTXBuffer[k]));
										BlockRingBufferPush(&(BlockTXBuffer[k]), END);
									}
								}
						   }
						} else if (workbuffer[BLOCK_TYPE] == BLOCK_TYPE_TRACK_CIRCUIT) {
							// we got a track circuit packet on this interface
							if (RxIdx == BLOCK_TRACK_CIRCUIT_PACKET_SIZE) ObjSignal[i].St_TrackCircuitNext = workbuffer[BLOCK_TRACK_CIRCUIT_STATE];
						} else if (workbuffer[BLOCK_TYPE] == BLOCK_TYPE_SIG_STATE) {
							// we got a signal state on this interface
							if (RxIdx == BLOCK_SIG_STATE_PACKET_SIZE) ObjSignal[i].St_APBnext = workbuffer[BLOCK_SIG_STATE_STATE];
						} // no packet caught, add different types here
					
    					// stop execution of the "while" loop, in order not to read data after an END character;
    					// wait for next END character to be signaled by the interrupt routine
    					break;
    					
					} // if (j)
				}
			}
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
    		if (quartersecs >= ((OUR_ADDR0 + OUR_ADDR1 + OUR_ADDR2)/3)) {
                cli();
    		  changed = SEND_SAFETY_PACKET | SEND_EXT_SIG_PACKET | SEND_EXT_TO_PACKET | SEND_EXT_CIRC_PACKET;
    		  quartersecs = 0;
    		  sei();
    		  



#ifdef USES_I2C
			  
			  if (TWI_Operation.Operation == NO_OP)
			  {
				TWI_Operation.Operation = TWI_SET_OUTPUTS;
				TWI_Operation.Suboperation = W_CMD_OUTPUTS_SET;
				TWI_Operation.ElementNumber = 0;
			  }
#endif
    	    }
    	}
    	
    	/**************************** HANDLE INPUT ************************************************************/
    	    
#ifdef USES_I2C
		
		// TWI_Operation:
        //      Operation:          ActivateBlock, WriteBlock, ReadBlock, ActivateOutputs, ReadInputs, WriteOutputs, FALSE
        //      Element Number:     Number of block interface, Number of device list entry (conf_io_i2c_dev_list)

        
        /* Description of single commands
                                    
                Operation:          TWI_WRITE_BLOCK
                Suboperation:       W_CMD_BLOCK_WRITE_DATA
                Length:             W_CMD_BLOCK_WRITE_DATA_LEN + n
                Command example:    [I2C_ADDRESS]  [I2C_CMD]=W_CMD_BLOCK_WRITE_DATA  [I2C_WRITE_BLOCK_REMOTE_NUMBER]=0 [n data bytes]
                                    TWIStartReceiver(workbuffer, W_CMD_BLOCK_ACTIVATE_LEN + n)  NB: the +1 comes from the address byte
                                    Tells the node on the given address to send the data onto his Block Interface #0


                Operation:          TWI_READ_BLOCK
                Suboperation:       W_CMD_BLOCK_PREPARE_DATA_LENGTH
                Length:             W_CMD_BLOCK_PREPARE_DATA_LENGTH_LEN
                Command example:    [I2C_ADDRESS]  [I2C_CMD]=W_CMD_BLOCK_PREPARE_DATA_LENGTH  [I2C_WRITE_BLOCK_REMOTE_NUMBER]=0
                                    TWIStartReceiver(workbuffer, W_CMD_BLOCK_PREPARE_DATA_LENGTH_LEN)  NB: the +1 comes from the address byte
                                    Tells the node on the given address to prepare for reading the size of the Buffer #0
                Operation:          TWI_READ_BLOCK
                Suboperation:       R_CMD_BLOCK_GET_DATA_LENGTH
                Length:             R_CMD_BLOCK_GET_DATA_LENGTH_LEN
                Command example:    [I2C_ADDRESS]
                                    TWIStartReceiver(workbuffer, R_CMD_BLOCK_GET_DATA_LENGTH_LEN)
                                    Reads the data length (one byte) on the Buffer selected before
                Operation:          TWI_READ_BLOCK
                Suboperation:       R_CMD_BLOCK_EXTRACT_DATA_LENGTH
                Length:             R_CMD_BLOCK_GET_DATA_LENGTH_LEN
                                    ----
                Operation:          TWI_READ_BLOCK
                Suboperation:       R_CMD_BLOCK_EXTRACT_DATA
                Length:             TWI_Operation.ReadLength
                Command example:    [I2C_ADDRESS]
                                    Reads the data (len bytes as read before) on the Buffer selected before
                                   
                
                Operation:          TWI_ACTIVATE_OUTPUTS
                Suboperation:       W_CMD_OUTPUTS_CONFIGURE
                Length:             W_CMD_OUTPUTS_CONFIGURE_LEN + n port bytes
                Command example:    [I2C_ADDRESS]   [I2C_CMD] = W_CMD_OUTPUTS_CONFIGURE  [n port bytes with masks]
                                    TWIStartReceiver( workbuffer, W_CMD_OUTPUTS_CONFIGURE_LEN + n)

                                    
                Operation:          TWI_SET_OUTPUTS
                Suboperation:       W_CMD_OUTPUTS_SET
                Length:             W_CMD_OUTPUTS_SET_LEN + highest port used  (of the single chip)
                Command example:    [I2C_ADDRESS]  [I2C_CMD] = W_CMD_OUTPUTS_SET  [n port bytes with masks]
                                    TWIStartReceiver( workbuffer, W_CMD_OUTPUTS_SET_LEN + n)
                                    
                                    
                Operation:          TWI_GET_INPUTS
                Suboperation:       W_CMD_INPUTS_PREPARE
                Length:             W_CMD_INPUTS_PREPARE_LEN
                Command example:    [I2C_ADDRESS]  [I2C_CMD]=W_CMD_INPUTS_PREPARE
                                    TWIStartReceiver(workbuffer, W_CMD_INPUTS_PREPARE_LEN)  NB: the +1 comes from the address byte
                                    Tells the node on the given address to prepare for reading its inputs
                Operation:          TWI_GET_INPUTS
                Suboperation:       R_CMD_INPUTS_GET
                Length:             R_CMD_INPUTS_GET_LEN
                Command example:    [I2C_ADDRESS]
                                    TWIStartReceiver(workbuffer, highest portused +1)
                                    Reads the inputs of the node, quantity is "highest port used"
                

     */
        
        // /TWI_INT is low, and we are idle; start the reading process
        if (!(TWI_PIN & TWI_INT) && (TWI_Operation.Operation == NO_OP))
		{
			TWI_Operation.Operation = TWI_READ_BLOCK; // begin reading the inputs from the first block interface, then proceed with inputs;
			TWI_Operation.ElementNumber = 0;
			TWI_Operation.Suboperation = W_CMD_BLOCK_PREPARE_DATA_LENGTH;
		}
//    	    READ ALL I2C
        if ( ! TWI_Transceiver_Busy() )
        {
            // Check if the last operation was successful
            if ( TWI_statusReg.lastTransOK )
            {
                // Determine what action to take now
                if (TWI_Operation.Operation == TWI_ACTIVATE_OUTPUTS)
                {
                    if (TWI_Operation.ElementNumber < CONF_IO_I2C_DEV_LIST_SIZE)
                    {
                        if (TWI_Operation.Suboperation == W_CMD_ACTIVATE_OUTPUTS)
                        {
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                            j = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_HIGHEST_PORT]) +1;
                            
                            workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);
                            workbuffer[I2C_CMD] = W_CMD_ACTIVATE_OUTPUTS;
                            workbuffer[I2C_LEN] = W_CMD_ACTIVATE_OUTPUTS_LEN + j;
							
                            // clear all data bytes first, so we can simply set the individual bits
                            for ( k = (W_CMD_ACTIVATE_OUTPUTS_LEN); k < sizeof(workbuffer); ++k)  workbuffer[k] = 0;
                            
                            for (k = 0; k < CONF_IO_I2C_OUT_LIST_SIZE; ++k)
                            {
                                l = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_DEVICE_ADDRESS]);
                                m = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_GIO]);
                                n = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_PORTPIN]);
                                o = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_BYTE]);
                                p = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_BIT]);
                                
                                // check that pin is not higher than 7, port not higher than the declared highest port of the device,
                                // the byte not higher than the output list, and the bit not higher than 7
                                if (    (n >= 8) || (m >= j) || ( o >= IO_OUTPUT_SIZE) || (p >= 8))
                                {
                                    ErrorCode.Type = CONF_IO_I2C_OUT_LIST_ERROR_CODE;
                                    ErrorCode.ListIndex = k;
                                    continue;
                                }
                                
                                if (l == i) // line matches current device
                                {
                                    // o is the byte, p the bit; m is the port, n the pin
                                    // should be this one here:
                                    workbuffer[W_CMD_ACTIVATE_OUTPUTS_LEN + m] |= (1 << n); // the byte m is equal to the port; the port pin n is masked. Slave puts this onto DDRx
                                }
                            }
                            
                            // Start transceiver
                            TWI_Start_Transceiver_With_Data(workbuffer, W_CMD_ACTIVATE_OUTPUTS_LEN + j);
                            
                            TWI_Operation.Operation = NO_OP;
                            
                            //
                        } else if (TWI_Operation.Suboperation == W_CMD_ACTIVATE_INPUTS) {
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                            j = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_HIGHEST_PORT]) +1;
                            
                            workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);
                            workbuffer[I2C_CMD] = W_CMD_ACTIVATE_INPUTS;
                            workbuffer[I2C_LEN] = W_CMD_ACTIVATE_INPUTS_LEN + j;
							
                            // clear all data bytes first, so we can simply set the individual bits
                            for ( k = (W_CMD_ACTIVATE_INPUTS_LEN); k < sizeof(workbuffer); ++k)  workbuffer[k] = 0;
                            
                            for (k = 0; k < CONF_IO_I2C_IN_LIST_SIZE; ++k)
                            {
                                l = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_DEVICE_ADDRESS]);
                                m = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_GIO]);
                                n = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_PORTPIN]);
                                o = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_BYTE]);
                                p = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_BIT]);
                                
                                // check that pin is not higher than 7, port not higher than the declared highest port of the device,
                                // the byte not higher than the input list, and the bit not higher than 7
                                if (    (n >= 8) || (m >= j) || ( o >= IO_INPUT_SIZE) || (p >= 8))
                                {
                                    ErrorCode.Type = CONF_IO_I2C_IN_LIST_ERROR_CODE;
                                    ErrorCode.ListIndex = k;
                                    continue;
                                }
                                
                                if (l == i) // line matches current device
                                {
                                    // o is the byte, p the bit; m is the port, n the pin
                                    // should be this one here:
                                    workbuffer[W_CMD_ACTIVATE_INPUTS_LEN + m] |= (1 << n); // the byte m is equal to the port; the port pin n is masked. Slave puts this onto DDRx
                                }
                            }
                            
                            // Start transceiver
                            TWI_Start_Transceiver_With_Data(workbuffer, W_CMD_ACTIVATE_INPUTS_LEN + j);
                            
                            TWI_Operation.Operation = NO_OP;
                            
                            //
                        }
						
                    } else {
                            // we are out of bounds for the devices => go back to idle state
                            TWI_Operation.Operation = NO_OP;
                            TWI_Operation.ElementNumber = 0;
                            TWI_Operation.Suboperation = FALSE;
                     //       goto I2C_INIT_END;  // we have concluded all operations of the initialization, so break the while(1) loop
                    }
                }
                
                // Determine what action to take now
                if (TWI_Operation.Operation == TWI_READ_BLOCK)
                { // Send data to slave
    
                    if (TWI_Operation.ElementNumber < CONF_IO_BLOCK_LIST_SIZE)
                    {
                        // we are working on the block "ElementNumber"
                        if (pgm_read_byte(&conf_io_block_list[TWI_Operation.ElementNumber][CONF_IO_BLOCK_IS_ON_I2C]) == IS_NOT_ON_I2C)
                        {
                            // element is not on I2C, directly go to next object
							// this also covers a virtual interface
                            // TODO
                            ++TWI_Operation.ElementNumber;
                            //TWI_Operation.Operation = NO_OP;
                            
                        } else {
                            if (TWI_Operation.Suboperation == W_CMD_BLOCK_PREPARE_DATA_LENGTH)
                            {
                                // interface is on I2C: check whether there is buffer data
                                i = pgm_read_byte(&conf_io_block_list[TWI_Operation.ElementNumber][CONF_IO_BLOCK_ADDRESS]);
                                j = pgm_read_byte(&conf_io_block_list[TWI_Operation.ElementNumber][CONF_IO_BLOCK_PHYSICAL_NUMBER]);
                                                    
                                workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);
                                workbuffer[I2C_CMD] = W_CMD_BLOCK_PREPARE_DATA_LENGTH;
                                workbuffer[I2C_LEN] = W_CMD_BLOCK_PREPARE_DATA_LENGTH_LEN;
                                workbuffer[I2C_BLOCK_REMOTE_NUMBER] = j;
                                
                                // Start transceiver
                                TWI_Start_Transceiver_With_Data(workbuffer, W_CMD_BLOCK_PREPARE_DATA_LENGTH_LEN);
                                
                                // Set next operation
                                TWI_Operation.Suboperation = R_CMD_BLOCK_GET_DATA_LENGTH;
                                //TWI_Operation.Operation = NO_OP;
                                
                            } else if (TWI_Operation.Suboperation == R_CMD_BLOCK_GET_DATA_LENGTH) {
                                // next suobperation on the same Element: read the depth of the buffer on the remote device
                                i = pgm_read_byte(&conf_io_block_list[TWI_Operation.ElementNumber][CONF_IO_BLOCK_ADDRESS]);
                                //j = pgm_read_byte(&conf_io_block_list[TWI_Operation.ElementNumber][CONF_IO_BLOCK_PHYSICAL_NUMBER]);
                                                    
                                workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (TRUE << TWI_READ_BIT);
                                // Start transceiver
                                TWI_Start_Transceiver_With_Data(workbuffer, R_CMD_BLOCK_GET_DATA_LENGTH_LEN);
                                
                                // Set next operation
                                TWI_Operation.Suboperation = R_CMD_BLOCK_EXTRACT_DATA_LENGTH;
                                //    TWI_Operation.Operation = NO_OP;
                                    
                            } else if (TWI_Operation.Suboperation == R_CMD_BLOCK_EXTRACT_DATA_LENGTH) {
                                TWI_Get_Data_From_Transceiver( workbuffer, R_CMD_BLOCK_EXTRACT_DATA_LENGTH_LEN);
                                if (workbuffer[I2C_BLOCK_GET_DATA_LENGTH])
                                {
                                    if (workbuffer[I2C_BLOCK_GET_DATA_LENGTH] <= (WORK_BUFFER_SIZE-1)) TWI_Operation.ReadLength = workbuffer[I2C_BLOCK_GET_DATA_LENGTH]; else TWI_Operation.ReadLength = WORK_BUFFER_SIZE -1;
                                    
                                    i = pgm_read_byte(&conf_io_block_list[TWI_Operation.ElementNumber][CONF_IO_BLOCK_ADDRESS]);
                                    workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (TRUE << TWI_READ_BIT);
                                    TWI_Start_Transceiver_With_Data(workbuffer, TWI_Operation.ReadLength +1);

                                    // TWI_SetNextOperation(TTWI_Operation *twi, uint8_t WasError, uint8_t DataLength)
                                    TWI_Operation.Suboperation = R_CMD_BLOCK_EXTRACT_DATA;
                                    //TWI_Operation.Operation = NO_OP;
                                } else {
                                    // slave has no data, skip to next element
                                    // TODO
                                    ++TWI_Operation.ElementNumber;
									TWI_Operation.Suboperation = W_CMD_BLOCK_PREPARE_DATA_LENGTH;
                                    TWI_Operation.Operation = TWI_READ_BLOCK;
                                    //TWI_Operation.Operation = NO_OP;
                                }

                            } else if (TWI_Operation.Suboperation == R_CMD_BLOCK_EXTRACT_DATA) {
                                // read all the data from the TWI buffer
                                TWI_Get_Data_From_Transceiver( workbuffer, TWI_Operation.ReadLength +1);
                                // the workbuffer now holds the address byte and all data bytes
                                for (i = 1; i < TWI_Operation.ReadLength +1; ++i)
                                {
                                	//if (workbuffer[i] == END) BlockRXBuffer[TWI_Operation.ElementNumber].bufferState += 1;
                                	BlockRingBufferPush(&(BlockRXBuffer[TWI_Operation.ElementNumber]), workbuffer[i]);
                                }
                                // we completed the reading procedure for this remote Block Interface
                                // increase the element, and start anew
                                ++TWI_Operation.ElementNumber;
                                TWI_Operation.Suboperation = W_CMD_BLOCK_PREPARE_DATA_LENGTH;
                                TWI_Operation.Operation = TWI_READ_BLOCK;
                                //TWI_Operation.Operation = NO_OP;
                            }
                        }
                    } else {
                        // we are out of bounds for the block list => set next operation
                        TWI_Operation.Operation = TWI_GET_INPUTS;
                        TWI_Operation.ElementNumber = 0;
                        TWI_Operation.Suboperation = W_CMD_INPUTS_PREPARE;
                        //TWI_Operation.Operation = NO_OP;
                   }    // this bracket closes the ElementNumber check, next bracket below closes the whole TWI_READ_BLOCK part
                    
                } else if (TWI_Operation.Operation == TWI_GET_INPUTS) {
                    
                    if (TWI_Operation.ElementNumber < CONF_IO_I2C_DEV_LIST_SIZE)
                    {
                        // we are working on the device "ElementNumber", we read every device, even it doesn't have any inputs
                        if (TWI_Operation.Suboperation == W_CMD_INPUTS_PREPARE)
                        {
                            // interface is on I2C: check whether there is buffer data
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                                                
                            workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);
                            workbuffer[I2C_CMD] = W_CMD_INPUTS_PREPARE;
                            workbuffer[I2C_LEN] = W_CMD_INPUTS_PREPARE_LEN;
                            
                            // Start transceiver
                            TWI_Start_Transceiver_With_Data(workbuffer, W_CMD_INPUTS_PREPARE_LEN);
                            
                            // TODO
                            
                            // Set next operation
                            TWI_Operation.Suboperation = R_CMD_INPUTS_GET;
                            //TWI_Operation.Operation = NO_OP;
                            
                        } else if (TWI_Operation.Suboperation == R_CMD_INPUTS_GET) {
                            // next suobperation on the same Element: read all ports on the remote device
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                            j = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_HIGHEST_PORT]) +1;
                                                
                            workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (TRUE << TWI_READ_BIT);
                            // Start transceiver
                            TWI_Start_Transceiver_With_Data(workbuffer, j +1);
                            
                            // Set next operation
                            TWI_Operation.Suboperation = R_CMD_INPUTS_EXTRACT;
							TWI_Operation.ReadLength = j;
                            

                         } else if (TWI_Operation.Suboperation == R_CMD_INPUTS_EXTRACT) {
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                            //j = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_HIGHEST_PORT]) +1;
							j = TWI_Operation.ReadLength;  // net quantity of user bytes
							
                            TWI_Get_Data_From_Transceiver( workbuffer, j +1);
	
							// the workbuffer now holds the address byte and all data bytes
                            // we need to map the data onto the inputs
                            
                            for (k = 0; k < CONF_IO_I2C_IN_LIST_SIZE; ++k)
                            {
                                l = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_DEVICE_ADDRESS]);
                                m = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_GIO]);
                                n = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_PORTPIN]);
                                o = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_BYTE]);
                                p = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_BIT]);
                                
                                // check that pin is not higher than 7, port not higher than the declared highest port of the device,
                                // the byte not higher than the input list, and the bit not higher than 7
                                if (    (n >= 8) || (m >= j) || ( o >= IO_INPUT_SIZE) || (p >= 8))
                                {
                                    ErrorCode.Type = CONF_IO_I2C_IN_LIST_ERROR_CODE;
                                    ErrorCode.ListIndex = k;
                                    continue;
                                }
                                if (l == i) // line matches current device
                                {
                                    // o is the byte, p the bit; m is the port, n the pin
                                    if (workbuffer[m+1] & (1<<n)) io_input[o] |= (1<<p); else io_input[o] &= ~(1<<p);
                                }
                            }
                            // we completed the reading procedure for this device
                            // increase the element, and start anew
                            // TODO
                            ++TWI_Operation.ElementNumber;
                            TWI_Operation.Suboperation = W_CMD_INPUTS_PREPARE;
                            TWI_Operation.Operation = TWI_GET_INPUTS;
                            //TWI_Operation.Operation = NO_OP;
							
/*
						} else if (TWI_Operation.Suboperation == W_CMD_OLD_INPUTS_PREP) {
							// interface is on I2C: check whether there is buffer data
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                                                
                            workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);
                            workbuffer[I2C_CMD] = W_CMD_OLD_INPUTS_PREP;
                            workbuffer[I2C_LEN] = W_CMD_INPUTS_PREPARE_LEN;
                            
                            // Start transceiver
                            TWI_Start_Transceiver_With_Data(workbuffer, W_CMD_INPUTS_PREPARE_LEN);
							
							TWI_Operation.Operation = NO_OP;
                            
                        } else if (TWI_Operation.Suboperation == R_CMD_OLD_INPUTS_GET) {
						
						// next suobperation on the same Element: read all ports on the remote device
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                            j = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_HIGHEST_PORT]) +1;
                                                
                            workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (TRUE << TWI_READ_BIT);
                            // Start transceiver
                            TWI_Start_Transceiver_With_Data(workbuffer, j +1);
						
							TWI_Operation.Operation = NO_OP;
							
						} else if (TWI_Operation.Suboperation == W_CMD_CHANGED_PREP) {
							// interface is on I2C: check whether there is buffer data
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                                                
                            workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);
                            workbuffer[I2C_CMD] = W_CMD_CHANGED_PREP;
                            workbuffer[I2C_LEN] = W_CMD_INPUTS_PREPARE_LEN;
                            
                            // Start transceiver
                            TWI_Start_Transceiver_With_Data(workbuffer, W_CMD_INPUTS_PREPARE_LEN);
							
							TWI_Operation.Operation = NO_OP;
                            
                        } else if (TWI_Operation.Suboperation == R_CMD_CHANGED_GET) {
						
						// next suobperation on the same Element: read all ports on the remote device
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                            //j = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_HIGHEST_PORT]) +1;
                                                
                            workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (TRUE << TWI_READ_BIT);
                            // Start transceiver
                            TWI_Start_Transceiver_With_Data(workbuffer, 3);
						
							TWI_Operation.Operation = NO_OP;
*/
						}
					
                    } else {
                        // we are out of bounds for the device list => set next operation
                        TWI_Operation.Operation = TWI_ACTIVATE_OUTPUTS;
                        TWI_Operation.ElementNumber = 0;
                        TWI_Operation.Suboperation = W_CMD_ACTIVATE_OUTPUTS;
						
						
                   }    // this bracket closes the ElementNumber check
                   
                } // closing TWI_READ_INPUTS block
                
                
                
                if (TWI_Operation.Operation == TWI_ACTIVATE_OUTPUTS)
                {
                    if (TWI_Operation.ElementNumber < CONF_IO_I2C_DEV_LIST_SIZE)
                    {
                        if (TWI_Operation.Suboperation == W_CMD_ACTIVATE_OUTPUTS)
                        {
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                            j = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_HIGHEST_PORT]) +1;
                            
                            workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);
                            workbuffer[I2C_CMD] = W_CMD_ACTIVATE_OUTPUTS;
                            workbuffer[I2C_LEN] = W_CMD_ACTIVATE_OUTPUTS_LEN + j;
							
                            // clear all data bytes first, so we can simply set the individual bits
                            for ( k = (W_CMD_ACTIVATE_OUTPUTS_LEN); k < sizeof(workbuffer); ++k)  workbuffer[k] = 0;
                            
                            for (k = 0; k < CONF_IO_I2C_OUT_LIST_SIZE; ++k)
                            {
                                l = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_DEVICE_ADDRESS]);
                                m = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_GIO]);
                                n = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_PORTPIN]);
                                o = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_BYTE]);
                                p = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_BIT]);
                                
                                // check that pin is not higher than 7, port not higher than the declared highest port of the device,
                                // the byte not higher than the output list, and the bit not higher than 7
                                if (    (n >= 8) || (m >= j) || ( o >= IO_OUTPUT_SIZE) || (p >= 8))
                                {
                                    ErrorCode.Type = CONF_IO_I2C_OUT_LIST_ERROR_CODE;
                                    ErrorCode.ListIndex = k;
                                    continue;
                                }
                                
                                if (l == i) // line matches current device
                                {
                                    // o is the byte, p the bit; m is the port, n the pin
                                    // should be this one here:
                                    workbuffer[W_CMD_ACTIVATE_OUTPUTS_LEN + m] |= (1 << n); // the byte m is equal to the port; the port pin n is masked. Slave puts this onto DDRx
                                }
                            }
                            
                            // Start transceiver
                            TWI_Start_Transceiver_With_Data(workbuffer, W_CMD_ACTIVATE_OUTPUTS_LEN + j);
                            
                            // set next operation
							TWI_Operation.Suboperation = W_CMD_ACTIVATE_INPUTS;
							
                        } else if (TWI_Operation.Suboperation == W_CMD_ACTIVATE_INPUTS) {
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                            j = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_HIGHEST_PORT]) +1;
                            
                            workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);
                            workbuffer[I2C_CMD] = W_CMD_ACTIVATE_INPUTS;
                            workbuffer[I2C_LEN] = W_CMD_ACTIVATE_INPUTS_LEN + j;
							
                            // clear all data bytes first, so we can simply set the individual bits
                            for ( k = (W_CMD_ACTIVATE_INPUTS_LEN); k < sizeof(workbuffer); ++k)  workbuffer[k] = 0;
                            
                            for (k = 0; k < CONF_IO_I2C_IN_LIST_SIZE; ++k)
                            {
                                l = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_DEVICE_ADDRESS]);
                                m = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_GIO]);
                                n = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_PORTPIN]);
                                o = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_BYTE]);
                                p = pgm_read_byte(&conf_io_i2c_in_list[k][CONF_IO_I2C_IN_BIT]);
                                
                                // check that pin is not higher than 7, port not higher than the declared highest port of the device,
                                // the byte not higher than the input list, and the bit not higher than 7
                                if (    (n >= 8) || (m >= j) || ( o >= IO_INPUT_SIZE) || (p >= 8))
                                {
                                    ErrorCode.Type = CONF_IO_I2C_IN_LIST_ERROR_CODE;
                                    ErrorCode.ListIndex = k;
                                    continue;
                                }
                                
                                if (l == i) // line matches current device
                                {
                                    // o is the byte, p the bit; m is the port, n the pin
                                    // should be this one here:
                                    workbuffer[W_CMD_ACTIVATE_INPUTS_LEN + m] |= (1 << n); // the byte m is equal to the port; the port pin n is masked. Slave puts this onto DDRx
                                }
                            }
                            
                            // Start transceiver
                            TWI_Start_Transceiver_With_Data(workbuffer, W_CMD_ACTIVATE_INPUTS_LEN + j);
                            
                            // all operations on this element are concluded, hop to next Element
                            ++TWI_Operation.ElementNumber;
                            
						}
						
                    } else {
                            // we are out of bounds for the devices => go back to idle state
                            TWI_Operation.Operation = NO_OP;
                            TWI_Operation.ElementNumber = 0;
                            TWI_Operation.Suboperation = FALSE;
                    }
                }
                
                
            } else { // Got an error during the last transmission
                // Use TWI status information to detemine cause of failure and take appropriate actions.
                TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info( ) );
                
            }
        // end of I2C reading init
        }
#endif

           
            
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
	    
		
#ifdef AUTOMATION_USE_AUTOMATION_FILE
		Automation(1);
#endif
		APBClearIndication();
		
		
		/*********************  END SAFETY SECTION  *************************************************/
		
		
        MapOutputStates(quartersecs);
    
         /**************************** HANDLE OUTPUT ************************************************************/
    
#ifdef USES_I2C

//write all I2C
// we should do this only if the state of the bytes changed
        if (TWI_Operation.Operation == NO_OP)
        {
			// check if outputs have been modified
			for (i = 0; i < IO_OUTPUT_SIZE; ++i)
			{
				if (io_output[i] != io_old_output[i])
				{
					io_old_output[i] = io_output[i];
					// some output has been modified, write data to all
					TWI_Operation.Operation = TWI_SET_OUTPUTS;
					TWI_Operation.ElementNumber = 0;
					TWI_Operation.Suboperation = W_CMD_OUTPUTS_SET;
					break;
				}
			}
        }
        if (TWI_Operation.Operation == NO_OP)
        {
            // check whether a block interface on I2C has Depth
            for (i = 0; i < CONF_IO_BLOCK_LIST_SIZE; ++i)
            {
                j = pgm_read_byte(&conf_io_block_list[i][CONF_IO_BLOCK_IS_ON_I2C]);
                if (j == IS_ON_I2C)
                {
                    k = BlockRingBufferDepth(&BlockTXBuffer[i]);
                    
                    if (k)
                    {
                        // the block i is on I2C and there is data for it
                        // select it and start sending data
                        TWI_Operation.Operation = TWI_WRITE_BLOCK;
                        TWI_Operation.ElementNumber = i;
                        TWI_Operation.Suboperation = W_CMD_BLOCK_WRITE_DATA;

                        break;
                    }
                }
            }
        }
        
    
        if ( ! TWI_Transceiver_Busy() )
        {
            // Check if the last operation was successful
            if ( TWI_statusReg.lastTransOK )
            {
                // Determine what action to take now
                if (TWI_Operation.Operation == TWI_WRITE_BLOCK)
                { // Send data to slave
                    
                    // check whether we are out of bounds for the I2C devices
                    if ((TWI_Operation.ElementNumber < CONF_IO_BLOCK_LIST_SIZE) &&
                        (TWI_Operation.Suboperation == W_CMD_BLOCK_WRITE_DATA) &&
                        (pgm_read_byte(&conf_io_block_list[TWI_Operation.ElementNumber][CONF_IO_BLOCK_IS_ON_I2C]) == IS_ON_I2C))
                    {
                        // interface is on I2C
                        i = pgm_read_byte(&conf_io_block_list[TWI_Operation.ElementNumber][CONF_IO_BLOCK_ADDRESS]);
                        j = pgm_read_byte(&conf_io_block_list[TWI_Operation.ElementNumber][CONF_IO_BLOCK_PHYSICAL_NUMBER]);
                        k = BlockRingBufferDepth(&(BlockTXBuffer[TWI_Operation.ElementNumber]));
                        
						if ((k + W_CMD_BLOCK_WRITE_LEN) > WORK_BUFFER_SIZE) k = WORK_BUFFER_SIZE - W_CMD_BLOCK_WRITE_LEN; // detracts 4 places from the buffer; k is the length
                        
                        workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);
                        workbuffer[I2C_CMD] = W_CMD_BLOCK_WRITE_DATA;
                        if ((k + W_CMD_BLOCK_WRITE_LEN) > WORK_BUFFER_SIZE) workbuffer[I2C_LEN] = WORK_BUFFER_SIZE; else workbuffer[I2C_LEN] = k + W_CMD_BLOCK_WRITE_LEN;
                        workbuffer[I2C_BLOCK_REMOTE_NUMBER] = j;
                        
						for (l = (I2C_BLOCK_REMOTE_NUMBER+1); l < workbuffer[I2C_LEN]; ++l)
							workbuffer[l] = BlockRingBufferPop(&(BlockTXBuffer[TWI_Operation.ElementNumber]));
						
                        // Start transceiver
                        TWI_Start_Transceiver_With_Data(workbuffer, workbuffer[I2C_LEN]);
                        
                        // Set next operation
                        TWI_Operation.Operation = NO_OP;
                        TWI_Operation.Suboperation = FALSE;
                        TWI_Operation.ElementNumber = 0;

                    } else {
                        // somehow we are out of bounds for the block list, reset to NO_OP
                        // Set next operation
                        TWI_Operation.Operation = NO_OP;
                        TWI_Operation.Suboperation = FALSE;
                        TWI_Operation.ElementNumber = 0;
                    }
                } else if (TWI_Operation.Operation == TWI_SET_OUTPUTS) {
                    
                    if (TWI_Operation.ElementNumber < CONF_IO_I2C_DEV_LIST_SIZE)
                    {
                        if (TWI_Operation.Suboperation == W_CMD_OUTPUTS_SET)
                        {
                            i = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_ADDR]);
                            j = pgm_read_byte(&conf_io_i2c_dev_list[TWI_Operation.ElementNumber][CONF_IO_I2C_DEV_HIGHEST_PORT]) +1;
                            
                            workbuffer[I2C_ADDRESS] = (i << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);
                            workbuffer[I2C_CMD] = W_CMD_OUTPUTS_SET;
                            workbuffer[I2C_LEN] = W_CMD_OUTPUTS_SET_LEN + j;
    
                            // clear all data bytes first, so we can simply set the individual bits
                            for ( k = W_CMD_OUTPUTS_SET_LEN; k < sizeof(workbuffer); ++k)  workbuffer[k] = 0;
                            
                            for (k = 0; k < CONF_IO_I2C_OUT_LIST_SIZE; ++k)
                            {
                                l = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_DEVICE_ADDRESS]);
                                m = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_GIO]);
                                n = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_PORTPIN]);
                                o = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_BYTE]);
                                p = pgm_read_byte(&conf_io_i2c_out_list[k][CONF_IO_I2C_OUT_BIT]);
                                
                                // check that pin is not higher than 7, port not higher than the declared highest port of the device,
                                // the byte not higher than the output list, and the bit not higher than 7
                                if (    (n >= 8) || (m >= j) || ( o >= IO_OUTPUT_SIZE) || (p >= 8))
                                {
                                    ErrorCode.Type = CONF_IO_I2C_OUT_LIST_ERROR_CODE;
                                    ErrorCode.ListIndex = k;
                                    continue;
                                }
                                
                                if (l == i) // line k matches current device
                                {
                                    // o is the byte, p the bit; m is the port, n the pin
                                    if (io_output[o] & (1<<p)) workbuffer[W_CMD_OUTPUTS_SET_LEN + m] |= (1<<n); else workbuffer[W_CMD_OUTPUTS_SET_LEN + m] &= ~(1<<n);
                                }
                            }
                            
                            // Start transceiver
                            TWI_Start_Transceiver_With_Data(workbuffer, W_CMD_OUTPUTS_SET_LEN + j);
                            
                            // Set next operation
                           // TWI_Operation.Operation = TWI_SET_OUTPUTS;
                           // TWI_Operation.Suboperation = W_CMD_OUTPUTS_SET;
                           // ++TWI_Operation.ElementNumber;
						   TWI_Operation.Operation = NO_OP;

                        }
                    } else {
                        // Set next operation
                        TWI_Operation.Operation = NO_OP;
                        TWI_Operation.Suboperation = FALSE;
                        TWI_Operation.ElementNumber = 0;
                    }
                }
            } else { // Got an error during the last transmission
                // Use TWI status information to detemine cause of failure and take appropriate actions.
                TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info( ) );
            }
        // end of I2C output init
        }
#endif

		
		
		
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
            
    
		/*********************  OUTPUT  *************************************************/
        
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
                slip_encode(ObjSignal[i].St_APBout & (ASP_FILTER | STICK), &(BlockTXBuffer[i]));
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
                slip_encode(ObjSignal[i].St_APBout  & (ASP_FILTER | STICK), &(BlockTXBuffer[i]));
                BlockRingBufferPush(&(BlockTXBuffer[i]), END);
                
            }
			changed &= ~SEND_SAFETY_PACKET;
	   
		} else if ((changed & SEND_EXT_GI_PACKET) && (N_GENERIC_INPUTS)) {
			
			
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
		}
		
	    /*********** NEW SINGLE PACKET DATA UPDATE *********************/
	    if (changed & (SEND_EXT_CIRC_PACKET | SEND_EXT_TO_PACKET | SEND_EXT_SIG_PACKET))
	    {
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


void SendPacket(uint8_t *buffer, uint8_t BytesToSend, unsigned char i)
{
    uint8_t index;
    uint16_t crc = 0;
    
    // we need to calculate the CRC16
	for (index = BLOCK_BROADCAST_FIRST_DATA; index < BytesToSend; ++index)
		crc = mrbusCRC16Update(crc, buffer[index]);
	buffer[BLOCK_BROADCAST_CRC_H] = UINT16_HIGH_BYTE(crc);
	buffer[BLOCK_BROADCAST_CRC_L] = UINT16_LOW_BYTE(crc);
    
   if (i < N_BLOCKINTERFACES)
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
            workbuffer[BLOCK_BROADCAST_FIRST_USER_DATA+i] = ObjSignal[i].St_APBout  & (ASP_FILTER | STICK);
	    
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
