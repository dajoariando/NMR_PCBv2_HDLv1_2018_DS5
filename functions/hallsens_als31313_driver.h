#define EEP02_OFST		(0x02)
	#define EEP02_BW_Select_OFST 			(21) // Used to control the sample rate of the device. 0 = Slowest sample rate, highest resolution. 7 = Fastest sample rate, lowest resolution
	#define EEP02_Hall_Mode_OFST 			(19) // Controls the operation mode of the Hall plates. 0 = Single-Ended Hall Mode. 1 = Differential Hall Mode. 2 = Common Hall Mode. 2 = Common Hall Mode Modes for each conversion per enabled axis.
	#define EEP02_I2C_CRC_Enable_OFST 		(18) // I2C Cyclic Redundancy Check (CRC) output byte enabled. Enable CRC for applications that require high data integrity. 0 = disabled. 1 = enabled.
	#define EEP02_Disable_Slave_ADC_OFST 	(17) // Disable the external slave address pins. When set, the EEPROM setting in Slave Address is used to determine the slave address.
	#define EEP02_Slave_Address_OFST 		(10) // Used to set the slave address for the device when either Disable Slave ADC is set, or the voltages on the slave address pins are set to VCC.
	#define EEP02_I2C_Threshold_OFST 		(9) // Enables 1.8 V or 3 V compatible I2C. 0 = 3 V compatible mode (Increases threshold for logic input high level). 1 = 1.8 V compatible mode
	#define EEP02_ChannelZ_Enable_OFST 		(8) // Enables the Z channel. Disable for faster update rate if this axis is not needed.
	#define EEP02_ChannelY_Enable_OFST 		(7) // Enables the Y channel. Disable for faster update rate if this axis is not needed.
	#define EEP02_ChannelX_Enable_OFST 		(6) // Enables the X channel. Disable for faster update rate if this axis is not needed.
	#define EEP02_INT_Latch_Enable_OFST 	(5) // Enables volatile latching of the INT signal. When set, if an interrupt event occurs, the INT status bit and INT output will both remain latched even after the event goes away
	#define EEP02_Customer_EEPROM_OFST 		(0) // Customer non-volatile EEPROM. Can be used to store any customer information. Does not affect device operation
		// BASEMASK : the mask of the register after the value is shifted to bit-0 instead of its original bit position
		#define EEP02_BW_Select_BASEMASK 			(0b111)
		#define EEP02_Hall_Mode_BASEMASK 			(0b11)
		#define EEP02_I2C_CRC_Enable_BASEMASK 		(0b1)
		#define EEP02_Disable_Slave_ADC_BASEMASK 	(0b1)
		#define EEP02_Slave_Address_BASEMASK 		(0b1111111)
		#define EEP02_I2C_Threshold_BASEMASK 		(0b1)
		#define EEP02_ChannelZ_Enable_BASEMASK 		(0b1)
		#define EEP02_ChannelY_Enable_BASEMASK 		(0b1)
		#define EEP02_ChannelX_Enable_BASEMASK 		(0b1)
		#define EEP02_INT_Latch_Enable_BASEMASK 	(0b1)
		#define EEP02_Customer_EEPROM_BASEMASK 		(0b11111)

#define EEP03_OFST		(0x03)
	#define EEP03_Signed_INT_Enable_OFST 	(24) // Controls if the interrupt threshold(s) are absolute or signed. In absolute mode, an interrupt is triggered if the applied field crosses the threshold in either the positive or negative direction. In signed mode, an interrupt is only triggered if the applied field passes the threshold in a single direction specified by the user. 0 = Absolute. 1 = Signed
	#define EEP03_INT_Mode_OFST				(23) // Controls the behavior of INT. 0 = Threshold Mode. Compares the sensor’s most recent measurement to the  specified event conditions. 1 = Delta Mode. Used in combination with LPDCM. Compares the sensor’s most recent measurement to the first measurement when the device entered LPDCM and the specified event conditions.
	#define EEP03_INT_EEPROM_Status_OFST	(22) // Non-volatile EEPROM storage to indicate an interrupt event has occurred.
	#define EEP03_INT_EEPROM_Enable_OFST	(21) // If set, INT EEPROM Status will be automatically written when an interrupt event occurs
	#define EEP03_Z_INT_Enable_OFST			(20) // INT enable for Z axis.
	#define EEP03_Y_INT_Enable_OFST			(19) // INT enable for Y axis.
	#define EEP03_X_INT_Enable_OFST			(18) // INT enable for X axis.
	#define EEP03_Z_INT_Threshold_OFST		(12) // INT threshold for Z axis. Affected by Signed INT Enable.
	#define EEP03_Y_INT_Threshold_OFST		(6)  // INT threshold for Y axis. Affected by Signed INT Enable.
	#define EEP03_X_INT_Threshold_OFST		(0)  // INT threshold for X axis. Affected by Signed INT Enable.
		// BASEMASK : the mask of the register after the value is shifted to bit-0 instead of its original bit position
		#define EEP03_Signed_INT_Enable_BASEMASK 	(0b1)
		#define EEP03_INT_Mode_BASEMASK				(0b1)
		#define EEP03_INT_EEPROM_Status_BASEMASK	(0b1)
		#define EEP03_INT_EEPROM_Enable_BASEMASK	(0b1)
		#define EEP03_Z_INT_Enable_BASEMASK			(0b1)
		#define EEP03_Y_INT_Enable_BASEMASK			(0b1)
		#define EEP03_X_INT_Enable_BASEMASK			(0b1)
		#define EEP03_Z_INT_Threshold_BASEMASK		(0b111111)
		#define EEP03_Y_INT_Threshold_BASEMASK		(0b111111)
		#define EEP03_X_INT_Threshold_BASEMASK		(0b111111)

#define EEP0d_OFST		(0x0d) // Customer non-volatile EEPROM space
	#define EEP0d_MSK		(0b11111111111111111111111111)
#define EEP0e_OFST		(0x0e) // Customer non-volatile EEPROM space
	#define EEP0e_MSK		(0b11111111111111111111111111)
#define EEP0f_OFST		(0x0f) // Customer non-volatile EEPROM space
	#define EEP0f_MSK		(0b11111111111111111111111111)

#define Vola27_OFST		(0x27)
	#define Vola27_Low_Power_Mode_Count_Max_OFST 	(4) // Sets max counter for inactive time during low-power duty cycle mode. ALS31313 offers 8 discrete time frames for inactive time.
	#define Vola27_I2C_Loop_Mode_OFST				(2) // Sets I2C readback mode to single read, fast loop, or full loop mode.
	#define Vola27_Sleep_OFST 						(0) // Sets device operating mode to full active, ultralow power sleep mode, or low-power duty cycle mode.
		// BASEMASK : the mask of the register after the value is shifted to bit-0 instead of its original bit position
		#define Vola27_Low_Power_Mode_Count_Max_BASEMASK 	(0b111)
		#define Vola27_I2C_Loop_Mode_BASEMASK 				(0b11)
		#define Vola27_Sleep_BASEMASK 						(0b11)

#define Vola28_OFST		(0x28)
	#define Vola28_X_Axis_MSBs_OFST 		(24) // MSBs of the register proportional to the field strength in the X direction.
	#define Vola28_Y_Axis_MSBs_OFST 		(16) // MSBs of the register proportional to the field strength in the Y direction.
	#define Vola28_Z_Axis_MSBs_OFST 		(8)  // MSBs of the register proportional to the field strength in the Z direction.
	#define Vola28_New_Data_OFST			(7) // New data update flag for XYZ. Cleared when read. Set when a new update is available. Use this bit when sampling the device faster than the update rate to avoid averaging the same sample twice. This bit clears when address 0x28 is read.
	#define Vola28_Interrupt_OFST			(6) // Set when the interrupt thresholds are crossed. Latched if INT Latch Enable is set. In latched mode, latch can be cleared by writing a 1 to this bit location.
	#define Vola28_Temperature_MSBs_OFST	(0) // MSBs of the temperature register proportional to the absolute temperature
		// BASEMASK : the mask of the register after the value is shifted to bit-0 instead of its original bit position
		#define Vola28_X_Axis_MSBs_BASEMASK 		(0b11111111)
		#define Vola28_Y_Axis_MSBs_BASEMASK 		(0b11111111)
		#define Vola28_Z_Axis_MSBs_BASEMASK 		(0b11111111)
		#define Vola28_New_Data_BASEMASK			(0b1)
		#define Vola28_Interrupt_BASEMASK 			(0b1)
		#define Vola28_Temperature_MSBs_BASEMASK 	(0b111111)

#define Vola29_OFST		(0x29)
	#define Vola29_Interrupt_Write_OFST		(20) // Status bit to indicate if an interrupt write is in progress. Will be set if Interrupt EEPROM Enable is set and an interrupt event has occurred. This field will be set while the device is writing the Interrupt EEPROM Status bit in address 0x03. When the writing is complete, this bit will clear automatically.
	#define Vola29_X_Axis_LSBs_OFST			(16) // LSBs of the register proportional to the field-strength in the X direction.
	#define Vola29_Y_Axis_LSBs_OFST			(12) // LSBs of the register proportional to the field-strength in the Y direction.
	#define Vola29_Z_Axis_LSBs_OFST			(8)  // LSBs of the register proportional to the field-strength in the Z direction.
	#define Vola29_Hall_Mode_Status_OFST	(6)  // The Hall mode of the current readout. Will be primarily used if 0x02 Hall mode is set to alternating mode. See Application Information section on Hall modes.0 = Value measured in Single-Ended Hall Mode. 1 = Value measured in Differential Hall Mode. 2 = Value measured in Common Hall Mode
	#define Vola29_Temperature_LSBs_OFST	(0) // LSBs of the temperature register proportional to the absolute temperature
		// BASEMASK : the mask of the register after the value is shifted to bit-0 instead of its original bit position
		#define Vola29_Interrupt_Write_BASEMASK		(0b1)
		#define Vola29_X_Axis_LSBs_BASEMASK			(0b1111)
		#define Vola29_Y_Axis_LSBs_BASEMASK			(0b1111)
		#define Vola29_Z_Axis_LSBs_BASEMASK			(0b1111)
		#define Vola29_Hall_Mode_Status_BASEMASK	(0b11)
		#define Vola29_Temperature_LSBs_BASEMASK	(0b111111)
