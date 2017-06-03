#ifndef _BLOCK_COM_H
#define _BLOCK_COM_H


    
#define BLOCK_TYPE		0                   // position


#define BLOCK_TYPE_SIG_STATE        0x2F        // value
#define BLOCK_SIG_STATE_STATE       1           // position
#define BLOCK_SIG_STATE_PACKET_SIZE  2           // value, to discard shortened packets

#define BLOCK_TYPE_PLUG_PRAY        0x30    // value

#define BLOCK_TYPE_TRACK_CIRCUIT    0x35    // value
#define BLOCK_TRACK_CIRCUIT_STATE   1       // position
#define BLOCK_TRACK_CIRCUIT_OCCUPIED    0x01
#define BLOCK_TRACK_CIRCUIT_FREE        0x00
#define BLOCK_TRACK_CIRCUIT_UNDEF       0xFF
#define BLOCK_TRACK_CIRCUIT_PACKET_SIZE 2   // value, to discard shortened packets


#define BLOCK_TYPE_BROADCAST        0x37    // value
#define BLOCK_BROADCAST_HOPS        1       // position
#define BLOCK_BROADCAST_SUBTYPE	    2       // position
#define BLOCK_BROADCAST_MIN_PACKET_SIZE 3   // value, to discard shortened packets

// different BROADCAST packets
#define BLOCK_BROADCAST_SUBTYPE_ZLV					        1   // value
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_COMMAND            2   // value
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_STATE              3   // value
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_EE_READ_CMD        4   // value
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_EE_READ_RESP       5   // value
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_EE_WRITE_CMD       6   // value
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_EE_WRITE_RESP      7   // value
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_PING               8   // value
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_PING_RESPONSE      9   // value
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_DEPTH    0x10
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_TWI      0x11
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_WRITE_TWI    0x12
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_OUTPUTS	0x13
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_INPUTS	0x14
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_APB_OUT		0x15
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_GET_SIGNAL_STATE	0x16
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_DEBUG_RESTART          0x17
#define BLOCK_BROADCAST_SUBTYPE_AMERICAN_TWI_ERROR_STATE        0x20


// americaN subtypes
#define BLOCK_BROADCAST_CRC_H       3   // position
#define BLOCK_BROADCAST_CRC_L       4   // position
#define BLOCK_BROADCAST_FIRST_DATA	5   // position
#define BLOCK_BROADCAST_SRC0        5   // position
#define BLOCK_BROADCAST_SRC1        6   // position
#define BLOCK_BROADCAST_SRC2        7   // position
#define BLOCK_BROADCAST_DEST0       8   // position
#define BLOCK_BROADCAST_DEST1       9   // position
#define BLOCK_BROADCAST_DEST2       10  // position
#define BLOCK_BROADCAST_FIRST_USER_DATA 11  // position

// americaN command
#define BLOCK_BROADCAST_COMMAND_TYPE        11     // position
#define BLOCK_BROADCAST_COMMAND_ELEMENT     12      // position
#define BLOCK_BROADCAST_COMMAND_CMD         13      // position
#define BLOCK_BROADCAST_COMMAND_ROUTE_ENTRY	12		// position
#define BLOCK_BROADCAST_COMMAND_ROUTE_EXIT	13		// position

#define BLOCK_BROADCAST_COMMAND_TYPE_CLEARANCE  'C' // value
#define BLOCK_BROADCAST_COMMAND_TYPE_ROUTE		'R'	// value
#define BLOCK_BROADCAST_COMMAND_TYPE_TURNOUT    'T' // value
#define BLOCK_BROADCAST_COMMAND_TYPE_SAFETY     'A' // value
#define BLOCK_BROADCAST_COMMAND_TYPE_GO         'O' // value
#define BLOCK_BROADCAST_COMMAND_TYPE_APPR_LIT   'L' // value

#define BLOCK_BROADCAST_COMMAND_CMD_CLEAR       'C' // value
#define BLOCK_BROADCAST_COMMAND_CMD_DROP        'N' // value
#define BLOCK_BROADCAST_COMMAND_CMD_FLEET       'F' // value
#define BLOCK_BROADCAST_COMMAND_CMD_NFLEET      'L' // value

#define BLOCK_BROADCAST_COMMAND_CMD_NORMAL      'N' // value
#define BLOCK_BROADCAST_COMMAND_CMD_REVERSE     'R' // value

#define BLOCK_BROADCAST_COMMAND_CMD_SET_OCC     'O' // value
#define BLOCK_BROADCAST_COMMAND_CMD_CLEAR_OCC   'P' // value
#define BLOCK_BROADCAST_COMMAND_CMD_SET_REL     'R' // value
#define BLOCK_BROADCAST_COMMAND_CMD_CLEAR_REL   'S' // value

#define BLOCK_BROADCAST_COMMAND_CMD_OFF         0x00 // value
#define BLOCK_BROADCAST_COMMAND_CMD_ON          0x01 // value
#define BLOCK_BROADCAST_COMMAND_CMD_FLASH       0x02 // value
#define BLOCK_BROADCAST_COMMAND_CMD_FL_ALT      0x03 // value

#define BLOCK_BROADCAST_COMMAND_CMD_APPR_LIT_ON		0x01 // value
#define BLOCK_BROADCAST_COMMAND_CMD_APPR_LIT_OFF	0x00 // value


// americaN state
#define BLOCK_BROADCAST_STATE_TYPE          11  // position
#define BLOCK_BROADCAST_STATE_DATA          12  // position
#define BLOCK_BROADCAST_STATE_TYPE_CIRCUIT   0  // values
#define BLOCK_BROADCAST_STATE_TYPE_TO        1  // values
#define BLOCK_BROADCAST_STATE_TYPE_SIG       2  // values
#define BLOCK_BROADCAST_STATE_TYPE_GI		 3  // values
#define BLOCK_BROADCAST_STATE_TYPE_CTC      0x53 // values

// americaN EEPROM read
#define BLOCK_BROADCAST_EE_READ_ADDR        11  // position
#define BLOCK_BROADCAST_EE_READ_VAL         12  // position
#define BLOCK_BROADCAST_EE_READ_RESP_SIZE   13  // size

// americaN EEPROM write
#define BLOCK_BROADCAST_EE_WRITE_ADDR       11  // position
#define BLOCK_BROADCAST_EE_WRITE_VAL        12  // position
#define BLOCK_BROADCAST_EE_WRITE_RESP_SIZE  13 // size





#endif