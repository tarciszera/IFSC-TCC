

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM303D_DEF_H
#define __LSM303D_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

// LSM303D I2C slave address, the LSB depends on SDO pin
#define LSM303D_SLAVE_ADDR_SDOH		  		0x1D 		// 7 bits address (SDO = HG)
#define LSM303D_SLAVE_ADDR_SDOL 	 		0x1E 		// 7 bits address (SDO = LW)

// LSM303D registers address
#define LSM303DREG_TEMP_OUT_L 	  			0x05
#define LSM303DREG_TEMP_OUT_H				0x06
#define LSM303DREG_STATUS_M 				0x07
#define LSM303DREG_OUT_X_L_M				0x08
#define LSM303DREG_OUT_X_H_M				0x09
#define LSM303DREG_OUT_Y_L_M				0x0A
#define LSM303DREG_OUT_Y_H_M				0x0B
#define LSM303DREG_OUT_Z_L_M				0x0C
#define LSM303DREG_OUT_Z_H_M				0x0D
#define LSM303DREG_WHO_I_AM					0x0F
#define LSM303DREG_INT_CTRL_M				0x12
#define LSM303DREG_INT_SRC_M				0x13
#define LSM303DREG_INT_THS_L_M				0x14
#define LSM303DREG_INT_THS_H_M				0x15
#define LSM303DREG_OFFSET_X_L_M				0x16
#define LSM303DREG_OFFSET_X_H_M				0x17
#define LSM303DREG_OFFSET_Y_L_M				0x18
#define LSM303DREG_OFFSET_Y_H_M				0x19
#define LSM303DREG_OFFSET_Z_L_M				0x1A
#define LSM303DREG_OFFSET_Z_H_M				0x1B
#define LSM303DREG_REFERENCE_X				0x1C
#define LSM303DREG_REFERENCE_Y				0x1D
#define LSM303DREG_REFERENCE_Z				0x1E
#define LSM303DREG_CTRL_0					0x1F
#define LSM303DREG_CTRL_1					0x20
#define LSM303DREG_CTRL_2					0x21
#define LSM303DREG_CTRL_3					0x22
#define LSM303DREG_CTRL_4					0x23
#define LSM303DREG_CTRL_5					0x24
#define LSM303DREG_CTRL_6					0x25
#define LSM303DREG_CTRL_7					0x26
#define LSM303DREG_STATUS_A					0x27
#define LSM303DREG_OUT_X_L_A				0x28
#define LSM303DREG_OUT_X_H_A				0x29
#define LSM303DREG_OUT_Y_L_A				0x2A
#define LSM303DREG_OUT_Y_H_A				0x2B
#define LSM303DREG_OUT_Z_L_A				0x2C
#define LSM303DREG_OUT_Z_H_A				0x2D
#define LSM303DREG_FIFO_CTRL				0x2E
#define LSM303DREG_FIFO_SRC					0x2F
#define LSM303DREG_IG_CFG1					0x30
#define LSM303DREG_IG_SRC1					0x31
#define LSM303DREG_IG_THS1					0x32
#define LSM303DREG_IG_DUR1					0x33
#define LSM303DREG_IG_CFG2					0x34
#define LSM303DREG_IG_SRC2					0x35
#define LSM303DREG_IG_THS2					0x36
#define LSM303DREG_IG_DUR2					0x37
#define LSM303DREG_CLICK_CFG				0x38
#define LSM303DREG_CLICK_SRC				0x39
#define LSM303DREG_CLICK_THS				0x3A
#define LSM303DREG_TIME_LIMIT				0x3B
#define LSM303DREG_TIME_LATENCY				0x3C
#define LSM303DREG_TIME_WINDOW				0x3D
#define LSM303DREG_ACT_THX					0x3E
#define LSM303DREG_ACT_DUR					0x3F

// Convert the 16 bits two's complement representation to uT
// Value uT = raw16BitsValue*LSM303D_Scale_Adjust;

#define LSM303D_Scale_Adjust_2					0.008
#define LSM303D_Scale_Adjust_4 	    			0.016
#define LSM303D_Scale_Adjust_8 		  			0.032
#define LSM303D_Scale_Adjust_12 				0.0479

#ifdef __cplusplus
}
#endif

#endif /* __LSM303D_DEF_H */
