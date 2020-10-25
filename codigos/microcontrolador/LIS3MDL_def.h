

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIS3MDL_DEF_H
#define __LIS3MDL_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

// LIS3MDL I2C slave address, the LSB depends on SDO pin
#define LIS3MDL_SLAVE_ADDR 		  0b0011100 		// 7 bits address (SDO = GND)
//#define LIS3MDL_SLAVE_ADDR 		  0b0011101 		// 7 bits address (SDO = VDD)

// LIS3MDL registers address
#define LISREG_OFFSET_X_L 	  	0x05
#define LISREG_OFFSET_X_H 	  	0x06
#define LISREG_OFFSET_Y_L 	  	0x07
#define LISREG_OFFSET_Y_H   		0x08
#define LISREG_OFFSET_Z_L   		0x09
#define LISREG_OFFSET_Z_H 	   	0x0A
#define LISREG_WHO_I_AM		    	0x0F
#define LISREG_CTRL_REG1 	    	0x20
#define LISREG_CTRL_REG2 	    	0x21
#define LISREG_CTRL_REG3 	    	0x22
#define LISREG_CTRL_REG4 	    	0x23
#define LISREG_CTRL_REG5 	    	0x24
#define LISREG_STATUS_REG 	   	0x27
#define LISREG_OUT_X_L	 	    	0x28
#define LISREG_OUT_X_H	  		  0x29
#define LISREG_OUT_Y_L	  		  0x2A
#define LISREG_OUT_Y_H	  		  0x2B
#define LISREG_OUT_Z_L	  		  0x2C
#define LISREG_OUT_Z_H 			    0x2D
#define LISREG_TEMP_OUT_L 		  0x2E
#define LISREG_TEMP_OUT_H 		  0x2F
#define LISREG_INT_CFG	 		    0x30
#define LISREG_INT_SRC	  		  0x31
#define LISREG_INT_THS_L 		    0x32
#define LISREG_INT_THS_H 		    0x33

// Convert the 16 bits two's complement representation to uT
// Value uT = raw16BitsValue/LIS_Scale_Adjust;

#define LIS_Scale_Adjust_4 	    68.42
#define LIS_Scale_Adjust_8 		  32.21
#define LIS_Scale_Adjust_12 		22.81
#define LIS_Scale_Adjust_16 		17.11

#ifdef __cplusplus
}
#endif

#endif /* __LIS3MDL_DEF_H */
