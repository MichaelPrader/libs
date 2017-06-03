
#ifndef TYPEDEF_BLOCKINTERFACE_H
#define TYPEDEF_BLOCKINTERFACE_H




struct TBlockInterface_ {
    
     
    /*  Internal states */
    // Configuration of the block interface
    uint8_t St_Config;
    uint8_t Add_I2C_Address;
    uint8_t Nx_PhysicalNumber;
 

 };
 

typedef struct TBlockInterface_ TBlockInterface;

// masks of St_Config
#define     BROADCAST_SHUT_OFF  0x01
#define     IS_ON_I2C           0x02
#define     IS_NOT_ON_I2C       0x00
#define		BLOCK_IS_VIRTUAL	0x04


struct TPhysicalBlockInterface_ {
    
    // the connector between the logical block
    // and the physical block on the main controller
    
    uint8_t Nx_ReferencedLogicalBlock;
    
};

typedef struct TPhysicalBlockInterface_ TPhysicalBlockInterface;


#endif