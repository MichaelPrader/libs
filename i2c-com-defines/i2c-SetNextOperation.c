

void TWI_SetNextOperation(TTWI_Operation *twi, uint8_t WasError, uint8_t DataLength)
{
	// Variables
	//uint8_t Operation;
    //uint8_t ElementNumber;
    //uint8_t Suboperation;
    //uint8_t Cmd_Length;
    //uint8_t ReadLength;

    switch (twi->Operation)
    {
        case TWI_SET_OUTPUTS:
            if (twi->ElementNumber < CONF_IO_I2C_DEV_LIST_SIZE)
            {
                twi->ElementNumber += 1;
            } else {
                twi->Operation = NO_OP;
                twi->Suboperation = FALSE;
                twi->ElementNumber = 0;
            }
            break;
            
        
        case TWI_ACTIVATE_OUTPUTS:
            if (twi->ElementNumber < CONF_IO_I2C_DEV_LIST_SIZE)
            {
                twi->ElementNumber += 1;
            } else {
                twi->Operation = NO_OP;
                twi->Suboperation = FALSE;
                twi->ElementNumber = 0;
            }
            break;
        
        
        case TWI_WRITE_BLOCK:
            twi->Operation = NO_OP;
            twi->Suboperation = FALSE;
            twi->ElementNumber = 0;
            break;
            
            
        case TWI_READ_BLOCK:
            
            if (twi->ElementNumber >= CONF_IO_BLOCK_LIST_SIZE)
            {
                twi->Operation = // read inputs
                twi->ElementNumber =
                twi->Suboperation =
                
            } else {
                switch (twi->Suboperation)
                {
                    case W_CMD_BLOCK_PREPARE_DATAL_LENGTH:
                        twi->Suboperation = R_CMD_BLOCK_GET_DATA_LENGTH;
                        break;
                    
                    case R_CMD_BLOCK_GET_DATA_LENGTH:
                        twi->Suboperation = R_CMD_BLOCK_EXTRACT_DATA_LENGTH;
                        break;
                    
                    case R_CMD_BLOCK_EXTRACT_DATA_LENGTH:
                        if (DataLength)
                        {
                            twi->Suboperation = R_CMD_BLOCK_EXTRACT_DATA;
                        } else {

	
	
	// TWI_READ_BLOCK
	//		W_CMD_BLOCK_PREPARE_DATA_LENGTH
	//		R_CMD_BLOCK_GET_DATA_LENGTH
	//		R_CMD_BLOCK_EXTRACT_DATA_LENGTH
	//		... and here we actually read block data
	//		R_CMD_BLOCK_EXTRACT_DATA

	
	// TWI_GET_INPUTS
	//		W_CMD_INPUTS_PREPARE
	//		W_CMD_INPUTS_GET
	//		W_CMD_INPUTS_EXTRACT
	
	
}