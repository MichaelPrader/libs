
#ifndef I2C_COM_DEFINES_H
#define I2C_COM_DEFINES_H

// I2C communication states and typedefs

struct TTWI_Operation_ {
    uint8_t Operation;
    uint8_t ElementNumber;
    uint8_t Suboperation;
    uint8_t Cmd_Length;
    uint8_t ReadLength;
    
    // TWI_Operation:
        //      Operation:          ReadBlock, ReadInputs, WriteBlock, WriteOutputs, FALSE
        //      Element Number:     Number of block interface, Number of device list entry (conf_io_i2c_dev_list)
        //      Suboperation:       W_CMD_BLOCK_PREPARE_LENGTH, R_CMD_BLOCK_GET_DATA_LENGTH, R_CMD_BLOCK_EXTRACT_DATA_LENGTH,
        //                          R_CMD_BLOCK_GET_DATA, R_CMD_BLOCK_EXTRACT_DATA,  W_CMD_BLOCK_ACTIVATE
        //                          W_CMD_OUTPUTS_ACTIVATE, W_CMD_OUTPUTS_SET, W_CMD_INPUTS_PREPARE_DATA, R_CMD_INPUTS_GET_DATA,
        //                          R_CMD_INPUTS_EXTRACT_DATA
        //      CmdLength:          the length of the command, or the header with data
        //      ReadLength:         the length of the data to be read
};

typedef struct TTWI_Operation_ TTWI_Operation;

#ifndef WORK_BUFFER_SIZE
#define WORK_BUFFER_SIZE	40
#endif


#define I2C_MAX_MESSAGE_LENGTH      (WORK_BUFFER_SIZE-1) // ???
#define TWI_BUFFER_SIZE         (I2C_MAX_MESSAGE_LENGTH + 1)



// positions of bytes
#define I2C_ADDRESS     			0
#define I2C_CMD         			1
#define I2C_LEN         			2
#define I2C_DATA        			3
#define I2C_BLOCK_REMOTE_NUMBER 	3
#define I2C_BLOCK_GET_DATA_LENGTH	1


// commands
#define     NO_OP                       0
#define     TWI_WRITE_BLOCK             1
#define     TWI_READ_BLOCK              2
#define     TWI_ACTIVATE_OUTPUTS        3
#define     TWI_SET_OUTPUTS             4
#define     TWI_GET_INPUTS              5

// suboperations
#define W_CMD_BLOCK_WRITE_DATA          0
#define W_CMD_BLOCK_PREPARE_DATA_LENGTH 1
#define R_CMD_BLOCK_GET_DATA_LENGTH     2
#define R_CMD_BLOCK_EXTRACT_DATA_LENGTH 3
#define R_CMD_BLOCK_EXTRACT_DATA        4
#define W_CMD_OUTPUTS_SET               5
#define W_CMD_INPUTS_PREPARE            6
#define R_CMD_INPUTS_GET                7
#define R_CMD_INPUTS_EXTRACT            8
#define W_CMD_ACTIVATE_OUTPUTS			9
#define W_CMD_OLD_INPUTS_PREP			0x0A
#define R_CMD_OLD_INPUTS_GET			0x0B
#define W_CMD_CHANGED_PREP				0x0C
#define R_CMD_CHANGED_GET				0x0D
#define W_CMD_ACTIVATE_INPUTS			        0x0E
#define W_CMD_BLOCK_WRITE_WAIT_FOR_COMPLETION   0x0F        // new for better block write handling

// lengths
#define W_CMD_BLOCK_WRITE_LEN                   4
#define W_CMD_BLOCK_PREPARE_DATA_LENGTH_LEN     4
#define R_CMD_BLOCK_GET_DATA_LENGTH_LEN     2
#define R_CMD_BLOCK_EXTRACT_DATA_LENGTH_LEN 2
#define W_CMD_OUTPUTS_SET_LEN               3
#define W_CMD_INPUTS_PREPARE_LEN            3
#define W_CMD_ACTIVATE_OUTPUTS_LEN			3
#define W_CMD_ACTIVATE_INPUTS_LEN			3



// internal defines
#define I2C_GIO_PA	0
#define I2C_GIO_PB	1
#define I2C_GIO_PC	2
#define I2C_GIO_PD	3
#define I2C_GIO_PE	4
#define I2C_GIO_PF	5

// define the device types for easier usage // TODO ::: check and populate
#define I2C_ATMEGA_8    I2C_GIO_PC  // ATmega8 doesn't have a PORTA; correct address pointers are given in the slave's file
#define I2C_ATMEGA_324  I2C_GIO_PD  // has four full ports, PORTA-PORTD


#endif
