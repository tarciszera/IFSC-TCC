

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MMC3416_DEF_H
#define __MMC3416_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

// MMC3416 I2C slave address, the 3 LSB can change with the order code, (example) order code = 4, so the 3 LSB will be 010
#define MMC3416_SLAVE_ADDR				  		0x30

// MMC3416 registers address
typedef enum MMC_regs {
	MMC3416REG_X_L = 0,
	MMC3416REG_X_H,
	MMC3416REG_Y_L,
	MMC3416REG_Y_H,
	MMC3416REG_Z_L,
	MMC3416REG_Z_H,
	MMC3416REG_STATUS,
	MMC3416REG_CTRL0,
	MMC3416REG_CTRL1,
	MMC3416REG_PRODUCTID = 0x20
}MMC_REGS;



// Convert the 16 bits two's complement representation to uT
// Value uT = raw16BitsValue*100/MMC3416_Scale_Adjust;

#define MMC3416_Scale_Adjust_12Bits 	    	128
#define MMC3416_Scale_Adjust_14Bits 		  	512
#define MMC3416_Scale_Adjust_16Bits				2048

#ifdef __cplusplus
}
#endif

#endif /* __MMC3416_DEF_H */
