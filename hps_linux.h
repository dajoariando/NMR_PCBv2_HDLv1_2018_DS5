#ifndef HPS_LINUX_H_
#define HPS_LINUX_H_

#include <stdbool.h>
#include <stdint.h>

#include "socal/hps.h"
#include "hps_soc_system.h"
#include "functions/general.h"

// |=============|==========|==============|==========|
// | Signal Name | HPS GPIO | Register/bit | Function |
// |=============|==========|==============|==========|
// |   HPS_LED   |  GPIO53  |   GPIO1[24]  |    I/O   |
// |=============|==========|==============|==========|
#define HPS_LED_IDX      (ALT_GPIO_1BIT_53)                      // GPIO53
#define HPS_LED_PORT     (alt_gpio_bit_to_pid(HPS_LED_IDX))      // ALT_GPIO_PORTB
#define HPS_LED_PORT_BIT (alt_gpio_bit_to_port_pin(HPS_LED_IDX)) // 24 (from GPIO1[24])
#define HPS_LED_MASK     (1 << HPS_LED_PORT_BIT)

// |=============|==========|==============|==========|
// | Signal Name | HPS GPIO | Register/bit | Function |
// |=============|==========|==============|==========|
// |  HPS_KEY_N  |  GPIO54  |   GPIO1[25]  |    I/O   |
// |=============|==========|==============|==========|
#define HPS_KEY_N_IDX      (ALT_GPIO_1BIT_54)                        // GPIO54
#define HPS_KEY_N_PORT     (alt_gpio_bit_to_pid(HPS_KEY_N_IDX))      // ALT_GPIO_PORTB
#define HPS_KEY_N_PORT_BIT (alt_gpio_bit_to_port_pin(HPS_KEY_N_IDX)) // 25 (from GPIO1[25])
#define HPS_KEY_N_MASK     (1 << HPS_KEY_N_PORT_BIT)

// physical memory file descriptor
int fd_dev_mem = 0;

// memory-mapped peripherals
void   *hps_gpio     = NULL;
size_t hps_gpio_span = ALT_GPIO1_UB_ADDR - ALT_GPIO1_LB_ADDR + 1;
size_t hps_gpio_ofst = ALT_GPIO1_OFST;

void   *h2f_lw_axi_master     = NULL;
size_t h2f_lw_axi_master_span = ALT_LWFPGASLVS_UB_ADDR - ALT_LWFPGASLVS_LB_ADDR + 1;
size_t h2f_lw_axi_master_ofst = ALT_LWFPGASLVS_OFST;

void *fpga_leds = NULL;
void *fpga_switches = NULL;

// general input / output fsm control addresses
void *h2p_ctrl_out_addr					= NULL; // control output signal for NMR FSM
void *h2p_ctrl_in_addr					= NULL; // control input signal for NMR FSM

// general i/o addresses
// volatile unsigned int is used when computing the offset address is needed
// for example, if the address is pointing to 0, *(volatile unsigned int + 1) will result in address of 4 (because one integer uses 32 bits or 4 bytes)
// while *(void+1) will result in address 1, which is incorrect
// this is due to the system in qsys usually uses byte addresses instead of word addresses. With one word is usually 4 bytes or 32 bits
void *h2p_adcdata_addr							= NULL; // gpio for adc high speed
void *h2p_led_addr								= NULL; // gpio for LEDs
volatile unsigned long *h2p_i2ccommon_addr		= NULL; // gpio for i2c (used for relay control through io expander chip TCA9555PWR, and also rx gain selector)
volatile unsigned int *h2p_dac_addr			= NULL; // gpio for dac (spi)
// void *h2p_i2ccommon_addr		= NULL; // gpio for i2c (used for relay control through io expander chip TCA9555PWR, and also rx gain selector)
// void *h2p_dac_addr			= NULL; // gpio for dac (spi)


// pll reconfig address for NMR transmitter
void *h2p_nmr_pll_addr 							= NULL; // PLL reconfiguration control for the NMR transmitter

// NMR sequence fsm parameter addresses
void *h2p_pulse1_addr						= NULL; // PLL 90-deg length
void *h2p_pulse2_addr						= NULL; // PLL 180-deg length
void *h2p_delay1_addr						= NULL; // PLL delay length after 90-deg signal
void *h2p_delay2_addr						= NULL; // PLL delay length after 180-deg signal
void *h2p_echo_per_scan_addr 				= NULL; // the amount of echoes on 1 NMR scan
void *h2p_nmr_pll_rst_dly_addr				= NULL; // the amount of delay between resetting the pll and starting the nmr fsm

// adc addresses
void *h2p_adc_fifo_addr									= NULL; // ADC FIFO output data address
// void *h2p_adc_fifo_status_addr		= NULL; // ADC FIFO status address
// void *h2p_adc_str_fifo_status_addr	= NULL; // ADC streaming FIFO status address
volatile unsigned int *h2p_adc_fifo_status_addr		= NULL; // ADC FIFO status address
volatile unsigned int *h2p_adc_str_fifo_status_addr	= NULL; // ADC streaming FIFO status address
void *h2p_adc_samples_per_echo_addr						= NULL; // The number of ADC capture per echo
void *h2p_init_adc_delay_addr							= NULL; // The cycle number for delay in an echo after pulse 180 is done. The idea is to put adc capture in the middle of echo window and giving some freedom to move the ADC capture window within the echo window




void open_physical_memory_device();
void close_physical_memory_device();
void mmap_hps_peripherals();
void munmap_hps_peripherals();
void mmap_fpga_peripherals();
void munmap_fpga_peripherals();
void mmap_peripherals();
void munmap_peripherals();
void setup_hps_gpio();
void setup_fpga_leds();
void handle_hps_led();
void handle_fpga_leds();

// FUNCTIONS
void create_measurement_folder();								// create a folder in the system for the measurement data
int exit_program();												// terminate the program
void init_dac_ad5722r ();
void print_warning_ad5722r();
void init_default_system_param();								// initialize the system with tuned default parameter
void write_i2c_relay_cnt (uint8_t c_shunt, uint8_t c_series);	// program the capacitance with the relay
void write_i2c_rx_gain (uint8_t rx_gain);						// program the final receiver gain. 0x00 means minimum gain (not really zero gain) and 0x0F means max gain (infinite/open circuit). So effective value is 0x00 to 0x0E
void test_dac_ad5722r();										// easy method to give number to dac
void sweep_matching_network();									// sweep the capacitance in matching network by sweeping the relay (FOREVER LOOP)
void write_vbias (double vbias);								// write vbias as voltage format
void write_vvarac (double vvarac);								// write vvarac as voltage format
void write_vbias_int(int16_t dac_v_bias);						// write vbias with a value
void sweep_vbias();												// sweep the vbias
void write_vvarac_int(int16_t dac_v_varac);						// write v_varactor with a value
void sweep_vvarac ();											// sweep the v_varactor
void sweep_rx_gain ();											// sweep the rx gain (FOREVER LOOP)
void CPMG_Sequence (
	double cpmg_freq,
	double pulse1_us,
	double pulse2_us,
	double pulse1_dtcl,
	double pulse2_dtcl,
	double echo_spacing_us,
	long unsigned scan_spacing_us,
	unsigned int samples_per_echo,
	unsigned int echoes_per_scan,
	double init_adc_delay_compensation,
	uint32_t ph_cycl_en,
	char * filename,
	char * avgname,
	uint32_t enable_message
);
void tx_sampling(double freq, double samples_per_echo, char * filename);


// global variables
FILE	*fptr;
long i;
long j;
unsigned int rddata [150000];
unsigned int rddata_16[150000];
char foldername[50]; // variable to store folder name of the measurement data
char pathname[60];

// FPGA control signal address
uint32_t ctrl_out = CNT_OUT_default;					// default variable to store the current control state

#endif