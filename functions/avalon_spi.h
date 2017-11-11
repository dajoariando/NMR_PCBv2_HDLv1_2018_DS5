#define SPI_RXDATA_offst 0
#define SPI_TXDATA_offst 1
#define SPI_STATUS_offst 2
	#define status_EOP_bit	9
	#define status_E_bit	8
	#define status_RRDY_bit	7
	#define status_TRDY_bit	6
	#define status_TMT_bit	5
	#define status_TOE_bit	4
	#define status_ROE_bit	3
#define SPI_CONTROL_offst 3
	#define control_SSO_bit	10
	#define control_IE_bit	8
	#define control_IRRDY_bit	7
	#define control_ITRDY_bit	6
	#define control_ITOE_bit	4
	#define control_IROE_bit	3
#define SPI_SLAVESELECT_offst 5