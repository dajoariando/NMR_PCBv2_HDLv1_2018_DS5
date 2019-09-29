#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include <alt_generalpurpose_io.h>
#include <hwlib.h>
#include <socal/alt_gpio.h>
#include <socal/hps.h>
#include <socal/socal.h>

#include "hps_linux.h"
#include "functions/general.h"
#include "functions/avalon_i2c.h"
#include "functions/tca9555_driver.h"
#include "functions/avalon_spi.h"
#include "functions/dac_ad5722r_driver.h"
#include "functions/hallsens_als31313_driver.h"
#include "functions/general.h"
#include "functions/reconfig_functions.h"
#include "functions/pll_param_generator.h"
#include "functions/adc_functions.h"
#include "functions/cpmg_functions.h"
#include "functions/AlteraIP/altera_avalon_fifo_regs.h"
#include "functions/nmr_table.h"
#include "functions/avalon_dma.h"
#include "hps_soc_system.h"

void open_physical_memory_device() {
    // We need to access the system's physical memory so we can map it to user
    // space. We will use the /dev/mem file to do this. /dev/mem is a character
    // device file that is an image of the main memory of the computer. Byte
    // addresses in /dev/mem are interpreted as physical memory addresses.
    // Remember that you need to execute this program as ROOT in order to have
    // access to /dev/mem.

    fd_dev_mem = open("/dev/mem", O_RDWR | O_SYNC);
    if(fd_dev_mem  == -1) {
        printf("ERROR: could not open \"/dev/mem\".\n");
        printf("    errno = %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }
}

void close_physical_memory_device() {
    close(fd_dev_mem);
}

void mmap_hps_peripherals() {
    hps_gpio = mmap(NULL, hps_gpio_span, PROT_READ | PROT_WRITE, MAP_SHARED, fd_dev_mem, hps_gpio_ofst);
    if (hps_gpio == MAP_FAILED) {
        printf("Error: hps_gpio mmap() failed.\n");
        printf("    errno = %s\n", strerror(errno));
        close(fd_dev_mem);
        exit(EXIT_FAILURE);
    }
}

void munmap_hps_peripherals() {
    if (munmap(hps_gpio, hps_gpio_span) != 0) {
        printf("Error: hps_gpio munmap() failed\n");
        printf("    errno = %s\n", strerror(errno));
        close(fd_dev_mem);
        exit(EXIT_FAILURE);
    }

    hps_gpio = NULL;
}

void mmap_fpga_peripherals() {
    // IMPORTANT: If you try to only mmap the fpga leds, it is possible for the
    // operation to fail, and you will get "Invalid argument" as errno. The
    // mmap() manual page says that you can only map a file from an offset which
    // is a multiple of the system's page size.

    // In our specific case, our fpga leds are located at address 0xFF200000,
    // which is a multiple of the page size, however this is due to luck because
    // the fpga leds are the only peripheral connected to the h2f_lw_axi_master.
    // The typical page size in Linux is 0x1000 bytes.

    // So, generally speaking, you will have to mmap() the closest address which
    // is a multiple of your page size and access your peripheral by a specific
    // offset from the mapped address.

    h2f_lw_axi_master = mmap(NULL, h2f_lw_axi_master_span, PROT_READ | PROT_WRITE, MAP_SHARED, fd_dev_mem, h2f_lw_axi_master_ofst);
    if (h2f_lw_axi_master == MAP_FAILED) {
        printf("Error: h2f_lw_axi_master mmap() failed.\n");
        printf("    errno = %s\n", strerror(errno));
        close(fd_dev_mem);
        exit(EXIT_FAILURE);
    }

    h2f_axi_master = mmap(NULL, h2f_axi_master_span, PROT_READ | PROT_WRITE, MAP_SHARED, fd_dev_mem, h2f_axi_master_ofst);
	if (h2f_axi_master == MAP_FAILED) {
		printf("Error: h2f_axi_master mmap() failed.\n");
		printf("    errno = %s\n", strerror(errno));
		close(fd_dev_mem);
		exit(EXIT_FAILURE);
    }

	h2p_ctrl_out_addr				= h2f_lw_axi_master + CTRL_OUT_BASE;
	h2p_ctrl_in_addr				= h2f_lw_axi_master + CTRL_IN_BASE;
	h2p_pulse1_addr					= h2f_lw_axi_master + NMR_PARAMETERS_PULSE_90DEG_BASE;
	h2p_pulse2_addr					= h2f_lw_axi_master + NMR_PARAMETERS_PULSE_180DEG_BASE;
	h2p_delay1_addr					= h2f_lw_axi_master + NMR_PARAMETERS_DELAY_NOSIG_BASE;
	h2p_delay2_addr					= h2f_lw_axi_master + NMR_PARAMETERS_DELAY_SIG_BASE;
	h2p_nmr_sys_pll_addr			= h2f_lw_axi_master + NMR_SYS_PLL_RECONFIG_BASE;
	h2p_echo_per_scan_addr			= h2f_lw_axi_master + NMR_PARAMETERS_ECHOES_PER_SCAN_BASE;
	h2p_i2c_ext_addr				= h2f_lw_axi_master + I2C_EXT_BASE;
	h2p_i2c_int_addr				= h2f_lw_axi_master + I2C_INT_BASE;
	h2p_adc_fifo_addr				= h2f_lw_axi_master + ADC_FIFO_MEM_OUT_BASE;
	h2p_adc_fifo_status_addr		= h2f_lw_axi_master + ADC_FIFO_MEM_IN_CSR_BASE;
	h2p_adc_samples_per_echo_addr	= h2f_lw_axi_master + NMR_PARAMETERS_SAMPLES_PER_ECHO_BASE;
	h2p_init_adc_delay_addr			= h2f_lw_axi_master + NMR_PARAMETERS_INIT_DELAY_BASE;
	h2p_rx_delay_addr				= h2f_lw_axi_master + NMR_PARAMETERS_RX_DELAY_BASE;
	h2p_dac_addr					= h2f_lw_axi_master + DAC_PREAMP_BASE;
	h2p_analyzer_pll_addr			= h2f_lw_axi_master + ANALYZER_PLL_RECONFIG_BASE;
	h2p_t1_pulse					= h2f_lw_axi_master + NMR_PARAMETERS_PULSE_T1_BASE;
	h2p_t1_delay					= h2f_lw_axi_master + NMR_PARAMETERS_DELAY_T1_BASE;

	h2p_dma_addr					= h2f_lw_axi_master + DMA_FIFO_BASE;
	h2p_sdram_addr					= h2f_axi_master + SDRAM_BASE;
	h2p_switches_addr				= h2f_axi_master + SWITCHES_BASE;

}

void munmap_fpga_peripherals() {


	if (munmap(h2f_lw_axi_master, h2f_lw_axi_master_span) != 0) {
        printf("Error: h2f_lw_axi_master munmap() failed\n");
        printf("    errno = %s\n", strerror(errno));
        close(fd_dev_mem);
        exit(EXIT_FAILURE);
    }

    h2f_lw_axi_master	= NULL;
    fpga_leds			= NULL;
    fpga_switches		= NULL;

}

void mmap_peripherals() {
    mmap_hps_peripherals();
    mmap_fpga_peripherals();
}

void munmap_peripherals() {
    munmap_hps_peripherals();
    munmap_fpga_peripherals();
}

void setup_hps_gpio() {
    // Initialize the HPS PIO controller:
    //     Set the direction of the HPS_LED GPIO bit to "output"
    //     Set the direction of the HPS_KEY_N GPIO bit to "input"
    void *hps_gpio_direction = ALT_GPIO_SWPORTA_DDR_ADDR(hps_gpio);
    alt_setbits_word(hps_gpio_direction, ALT_GPIO_PIN_OUTPUT << HPS_LED_PORT_BIT);
    alt_setbits_word(hps_gpio_direction, ALT_GPIO_PIN_INPUT << HPS_KEY_N_PORT_BIT);
}

void setup_fpga_leds() {
    // Switch on first LED only
    alt_write_word(h2p_led_addr, 0xF0);
}

void handle_hps_led() {
    void *hps_gpio_data = ALT_GPIO_SWPORTA_DR_ADDR(hps_gpio);
    void *hps_gpio_port = ALT_GPIO_EXT_PORTA_ADDR(hps_gpio);

    uint32_t hps_gpio_input = alt_read_word(hps_gpio_port) & HPS_KEY_N_MASK;

    // HPS_KEY_N is active-low
    bool toggle_hps_led = (~hps_gpio_input & HPS_KEY_N_MASK);

    if (toggle_hps_led) {
        uint32_t hps_led_value = alt_read_word(hps_gpio_data);
        hps_led_value >>= HPS_LED_PORT_BIT;
        hps_led_value = !hps_led_value;
        hps_led_value <<= HPS_LED_PORT_BIT;
        alt_replbits_word(hps_gpio_data, HPS_LED_MASK, hps_led_value);
    }
}

void create_measurement_folder(char * foldertype) {
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	char command[60];
	sprintf(foldername,"%04d_%02d_%02d_%02d_%02d_%02d_%s",tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec,foldertype);
	sprintf(command,"mkdir %s",foldername);
	system(command);

	// copy the executable file to the folder
	// sprintf(command,"cp ./thesis_nmr_de1soc_hdl2.0 %s/execfile",foldername);
	// system(command);
}

void check_i2c_isr_stat (volatile unsigned long * i2c_addr, uint8_t en_mesg) {
	uint32_t isr_status;

	isr_status = alt_read_word( i2c_addr+ISR_OFST);
	if (isr_status & RX_OVER_MSK) {
		printf("\t[ERROR] Receive data FIFO has overrun condition, new data is lost\n");
		alt_write_word( (i2c_addr+ISR_OFST) , RX_OVER_MSK ); // clears receive overrun
	}
	else {
		if (en_mesg) printf("\t[NORMAL] No receive overrun\n");
	}
	if (isr_status & ARBLOST_DET_MSK) {
		printf("\t[ERROR] Core has lost bus arbitration\n");
		alt_write_word( (i2c_addr+ISR_OFST) , ARBLOST_DET_MSK ); // clears receive overrun
	}
	else {
		if (en_mesg) printf("\t[NORMAL] No arbitration lost\n");
	}
	if (isr_status & NACK_DET_MSK) {
		printf("\t[ERROR] NACK is received by the core\n");
		alt_write_word( (i2c_addr+ISR_OFST) , NACK_DET_MSK ); // clears receive overrun
	}
	else {
		if (en_mesg) printf("\t[NORMAL] ACK has been received\n");
	}
	if (isr_status & RX_READY_MSK) {
		printf("\t[WARNING] RX_DATA_FIFO level is equal or more than its threshold\n");
	}
	else {
		if (en_mesg) printf("\t[NORMAL] RX_DATA_FIFO level is less than its threshold\n");
	}
	if (isr_status & TX_READY_MSK) { // NOTE THAT IF THERE'S NO DELAY BETWEEN WRITING TFR_CMD REGISTER AND THIS check_i2c_isr_stat FUNCTION CALL, THE I2C MIGHT NOT FINISH ITS OPERATION YET, WHICH LIKELY WILL GENERATE WARNING HERE
		printf("\t[WARNING] TFR_CMD level is equal or more than its threshold\n");
	}
	else {
		if (en_mesg) printf("\t[NORMAL] TFR_CMD level is less than its threshold\n");
	}

	if (en_mesg) printf("\t --- \n");
}

uint32_t i2c_tfr_cmd_fifo_lvl (volatile unsigned long * i2c_addr) {
	return (   alt_read_word(i2c_addr+TFR_CMD_FIFO_LVL_OFST)  &   TFR_CMD_FIFO_LVL_MSK   );
}

uint32_t i2c_rxdata_fifo_lvl (volatile unsigned long * i2c_addr) {
	return (   alt_read_word(i2c_addr+RX_DATA_FIFO_LVL_OFST)  &   RX_DATA_FIFO_LVL_MSK   );
}

void i2c_rxdata_flush (volatile unsigned long * i2c_addr, uint8_t en_mesg) {
	uint32_t rx_lvl = i2c_rxdata_fifo_lvl (i2c_addr);

	if (en_mesg) {
		printf("i2c_rxdata_flush :: RX-level: %d\n", rx_lvl);
	}

	while (i2c_rxdata_fifo_lvl (i2c_addr) > 0) {
		uint8_t flush = alt_read_word(h2p_i2c_ext_addr+RX_DATA_OFST);
		if (en_mesg) {
			printf("i2c_rxdata_flush :: flushed data: 0x%x\n", flush);
		}
	}
}

void i2c_wait_transfer(volatile unsigned long * i2c_addr) {
	while (! ( (alt_read_word( i2c_addr+ISR_OFST) & TX_READY_MSK) == TX_READY_MSK) ); // wait until TX_READY signal is asserted
}


void write_i2c_relay_cnt (uint8_t c_shunt, uint8_t c_series, uint8_t en_mesg) {
	uint8_t i2c_addr_relay = 0x40;	// i2c address for TCA9555PWR used by the relay
	i2c_addr_relay >>= 1;			// shift by one because the LSB address is not used as an address (controlled by the Altera I2C IP)

	// reorder port2 data for CR2 because the schematic switched the order (LSB was switched to MSB, etc)
	uint8_t c_series_reorder = 0;
	c_series_reorder =
		((c_series & 0b00000001)<<7) |
		((c_series & 0b00000010)<<5) |
		((c_series & 0b00000100)<<3) |
		((c_series & 0b00001000)<<1) |
		((c_series & 0b00010000)>>1) |
		((c_series & 0b00100000)>>3) |
		((c_series & 0b01000000)>>5) |
		((c_series & 0b10000000)>>7);

	// reorder port1 data for CR1 because the schematic switched the order (LSB was switched to MSB, etc)
	uint8_t c_shunt_reorder = 0;
	c_shunt_reorder =
		((c_shunt & 0b00000001)<<7) |
		((c_shunt & 0b00000010)<<5) |
		((c_shunt & 0b00000100)<<3) |
		((c_shunt & 0b00001000)<<1) |
		((c_shunt & 0b00010000)>>1) |
		((c_shunt & 0b00100000)>>3) |
		((c_shunt & 0b01000000)>>5) |
		((c_shunt & 0b10000000)>>7);



	alt_write_word( (h2p_i2c_ext_addr+ISR_OFST) , RX_OVER_MSK|ARBLOST_DET_MSK|NACK_DET_MSK ); // RESET THE I2C FROM PREVIOUS ERRORS

	alt_write_word( (h2p_i2c_ext_addr+CTRL_OFST), 1<<CORE_EN_SHFT); // enable i2c core

	alt_write_word( (h2p_i2c_ext_addr+SCL_LOW_OFST), 250); // set the SCL_LOW_OFST to 250 for 100 kHz with 50 MHz clock
	alt_write_word( (h2p_i2c_ext_addr+SCL_HIGH_OFST), 250); // set the SCL_HIGH_OFST to 250 for 100 kHz with 50 MHz clock
	alt_write_word( (h2p_i2c_ext_addr+SDA_HOLD_OFST), 1); // set the SDA_HOLD_OFST to 1 as the default (datasheet requires min 0 ns hold time)
    
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (1<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_relay<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_CONF_PORT0 & I2C_DATA_MSK) );
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((0x00) & I2C_DATA_MSK) );				// set port 0 as output
                                                      
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (1<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_relay<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_CONF_PORT1 & I2C_DATA_MSK) );
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((0x00) & I2C_DATA_MSK) );					// set port 1 as output
                                                      
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_relay<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_OUT_PORT0 & I2C_DATA_MSK) );
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((c_shunt_reorder) & I2C_DATA_MSK) );				// set output on port 0
                                                      
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_relay<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_OUT_PORT1 & I2C_DATA_MSK) );
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((c_series_reorder) & I2C_DATA_MSK) );	// set output on port 1

	if (en_mesg) {
		printf("Status for i2c transactions:\n");
	}

	uint32_t isr_status;
	isr_status = alt_read_word( h2p_i2c_ext_addr+ISR_OFST);
	if (isr_status & RX_OVER_MSK) {
		printf("\t[ERROR] Receive data FIFO has overrun condition, new data is lost\n");
		alt_write_word( (h2p_i2c_ext_addr+ISR_OFST) , RX_OVER_MSK ); // clears receive overrun
	}
	else {
		if (en_mesg) printf("\t[NORMAL] No receive overrun\n");
	}
	if (isr_status & ARBLOST_DET_MSK) {
		printf("\t[ERROR] Core has lost bus arbitration\n");
		alt_write_word( (h2p_i2c_ext_addr+ISR_OFST) , ARBLOST_DET_MSK ); // clears receive overrun
	}
	else {
		if (en_mesg) printf("\t[NORMAL] No arbitration lost\n");
	}
	if (isr_status & NACK_DET_MSK) {
		printf("\t[ERROR] NACK is received by the core\n");
		alt_write_word( (h2p_i2c_ext_addr+ISR_OFST) , NACK_DET_MSK ); // clears receive overrun
	}
	else {
		if (en_mesg) printf("\t[NORMAL] ACK has been received\n");
	}
	if (isr_status & RX_READY_MSK) {
		printf("\t[WARNING] RX_DATA_FIFO level is equal or more than its threshold\n");
	}
	else {
		if (en_mesg) printf("\t[NORMAL] RX_DATA_FIFO level is less than its threshold\n");
	}
	if (isr_status & TX_READY_MSK) {
		printf("\t[WARNING] TFR_CMD level is equal or more than its threshold\n");
	}
	else {
		if (en_mesg) printf("\t[NORMAL] TFR_CMD level is less than its threshold\n");
	}

	alt_write_word( (h2p_i2c_ext_addr+CTRL_OFST), 0<<CORE_EN_SHFT); // disable i2c core

	usleep(10000);
}

uint32_t rd_hall_sens (uint8_t reg_addr, uint8_t en_mesg) { // read command for the hall sensor
	uint8_t i2c_addr_relay = 96;	// hardwired address of ALS31313 is 96
	uint32_t dataout = 0;

	alt_write_word( (h2p_i2c_ext_addr+ISR_OFST) , RX_OVER_MSK|ARBLOST_DET_MSK|NACK_DET_MSK ); // RESET THE I2C FROM PREVIOUS ERRORS
	check_i2c_isr_stat (h2p_i2c_ext_addr, en_mesg);
	i2c_rxdata_flush(h2p_i2c_ext_addr, en_mesg);

	alt_write_word( (h2p_i2c_ext_addr+CTRL_OFST), 1<<CORE_EN_SHFT); // enable i2c core

	alt_write_word( (h2p_i2c_ext_addr+SCL_LOW_OFST), 250); // set the SCL_LOW_OFST to 250 for 100 kHz with 50 MHz clock
	alt_write_word( (h2p_i2c_ext_addr+SCL_HIGH_OFST), 250); // set the SCL_HIGH_OFST to 250 for 100 kHz with 50 MHz clock
	alt_write_word( (h2p_i2c_ext_addr+SDA_HOLD_OFST), 1); // set the SDA_HOLD_OFST to 1 as the default (datasheet requires min 0 ns hold time)

	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (1<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_relay<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );	//check_i2c_isr_stat (h2p_i2c_ext_addr, en_mesg);
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (reg_addr & I2C_DATA_MSK) ); 								//check_i2c_isr_stat (h2p_i2c_ext_addr, en_mesg);

	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (1<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_relay<<AD_SHFT) | (RD_I2C<<RW_D_SHFT) );	//check_i2c_isr_stat (h2p_i2c_ext_addr, en_mesg);
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | ((0x00) & I2C_DATA_MSK) );							//check_i2c_isr_stat (h2p_i2c_ext_addr, en_mesg);
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | ((0x00) & I2C_DATA_MSK) );							//check_i2c_isr_stat (h2p_i2c_ext_addr, en_mesg);
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | ((0x00) & I2C_DATA_MSK) );							//check_i2c_isr_stat (h2p_i2c_ext_addr, en_mesg);
	alt_write_word( (h2p_i2c_ext_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((0x00) & I2C_DATA_MSK) );							//check_i2c_isr_stat (h2p_i2c_ext_addr, en_mesg);

	i2c_wait_transfer(h2p_i2c_ext_addr);
	usleep(10000); // this delay is important because i2c_wait_transfer apparently doesn't wait until transfer is complete. This can only be eliminated if interrupt is utilized.

	// read 32-bit data inside the rx register. Mind that there's limit in fifo length stated in QSYS
	dataout = (dataout << 8) | (alt_read_word(h2p_i2c_ext_addr+RX_DATA_OFST) & RX_DATA_MSK ) ; // read bit 31:24
	dataout = (dataout << 8) | (alt_read_word(h2p_i2c_ext_addr+RX_DATA_OFST) & RX_DATA_MSK ) ; // read bit 23:16
	dataout = (dataout << 8) | (alt_read_word(h2p_i2c_ext_addr+RX_DATA_OFST) & RX_DATA_MSK ) ; // read bit 15:8
	dataout = (dataout << 8) | (alt_read_word(h2p_i2c_ext_addr+RX_DATA_OFST) & RX_DATA_MSK ) ; // read bit 7:0

	alt_write_word( (h2p_i2c_ext_addr+CTRL_OFST), 0<<CORE_EN_SHFT); // disable i2c core

	if (en_mesg) printf("dataout: %x\n", dataout);

	return dataout;

}

void rd_hall_sens_stat (void) {
	uint32_t REG02 = rd_hall_sens (EEP02_OFST, DISABLE_MESSAGE);
	uint32_t REG03 = rd_hall_sens (EEP03_OFST, DISABLE_MESSAGE);
	uint32_t REG27 = rd_hall_sens (Vola27_OFST, DISABLE_MESSAGE);
	uint32_t REG28 = rd_hall_sens (Vola28_OFST, DISABLE_MESSAGE);
	uint32_t REG29 = rd_hall_sens (Vola29_OFST, DISABLE_MESSAGE);

	printf("REG02 = 0x%x\n", REG02);
		printf("\tEEP02_BW_Select = %d\n", (REG02>>EEP02_BW_Select_OFST) & EEP02_BW_Select_BASEMASK);
		printf("\tEEP02_Hall_Mode = %d\n", (REG02>>EEP02_Hall_Mode_OFST) & EEP02_Hall_Mode_BASEMASK);
		printf("\tEEP02_I2C_CRC_Enable = %d\n", (REG02>>EEP02_I2C_CRC_Enable_OFST) & EEP02_I2C_CRC_Enable_BASEMASK);
		printf("\tEEP02_Disable_Slave_ADC = %d\n", (REG02>>EEP02_Disable_Slave_ADC_OFST) & EEP02_Disable_Slave_ADC_BASEMASK);
		printf("\tEEP02_Slave_Address = %d\n", (REG02>>EEP02_Slave_Address_OFST) & EEP02_Slave_Address_BASEMASK);
		printf("\tEEP02_I2C_Threshold = %d\n", (REG02>>EEP02_I2C_Threshold_OFST) & EEP02_I2C_Threshold_BASEMASK);
		printf("\tEEP02_ChannelZ_Enable = %d\n", (REG02>>EEP02_ChannelZ_Enable_OFST) & EEP02_ChannelZ_Enable_BASEMASK);
		printf("\tEEP02_ChannelY_Enable = %d\n", (REG02>>EEP02_ChannelY_Enable_OFST) & EEP02_ChannelY_Enable_BASEMASK);
		printf("\tEEP02_ChannelX_Enable = %d\n", (REG02>>EEP02_ChannelX_Enable_OFST) & EEP02_ChannelX_Enable_BASEMASK);
		printf("\tEEP02_INT_Latch_Enable = %d\n", (REG02>>EEP02_INT_Latch_Enable_OFST) & EEP02_INT_Latch_Enable_BASEMASK);
		printf("\tEEP02_Customer_EEPROM = %d\n", (REG02>>EEP02_Customer_EEPROM_OFST) & EEP02_Customer_EEPROM_BASEMASK);


	printf("REG03 = 0x%x\n", REG03);
		printf("\tEEP03_Signed_INT_Enable   = %d\n", (REG03>>EEP03_Signed_INT_Enable_OFST) 		& EEP03_Signed_INT_Enable_BASEMASK);
		printf("\tEEP03_INT_Mode            = %d\n", (REG03>>EEP03_INT_Mode_OFST			) 	& EEP03_INT_Mode_BASEMASK);
		printf("\tEEP03_INT_EEPROM_Status   = %d\n", (REG03>>EEP03_INT_EEPROM_Status_OFST) 		& EEP03_INT_EEPROM_Status_BASEMASK);
		printf("\tEEP03_INT_EEPROM_Enable   = %d\n", (REG03>>EEP03_INT_EEPROM_Enable_OFST) 		& EEP03_INT_EEPROM_Enable_BASEMASK);
		printf("\tEEP03_Z_INT_Enable        = %d\n", (REG03>>EEP03_Z_INT_Enable_OFST		) 	& EEP03_Z_INT_Enable_BASEMASK);
		printf("\tEEP03_Y_INT_Enable        = %d\n", (REG03>>EEP03_Y_INT_Enable_OFST		) 	& EEP03_Y_INT_Enable_BASEMASK);
		printf("\tEEP03_X_INT_Enable        = %d\n", (REG03>>EEP03_X_INT_Enable_OFST		) 	& EEP03_X_INT_Enable_BASEMASK);
		printf("\tEEP03_Z_INT_Threshold     = %d\n", (REG03>>EEP03_Z_INT_Threshold_OFST	) 		& EEP03_Z_INT_Threshold_BASEMASK);
		printf("\tEEP03_Y_INT_Threshold     = %d\n", (REG03>>EEP03_Y_INT_Threshold_OFST	)	 	& EEP03_Y_INT_Threshold_BASEMASK);
		printf("\tEEP03_X_INT_Threshold     = %d\n", (REG03>>EEP03_X_INT_Threshold_OFST	) 		& EEP03_X_INT_Threshold_BASEMASK);

	printf("REG27 = 0x%x\n", REG27);
		printf("\tVola27_Low_Power_Mode_Count_Max  = %d\n", (REG27>>Vola27_Low_Power_Mode_Count_Max_OFST) 		& Vola27_Low_Power_Mode_Count_Max_BASEMASK);
		printf("\tVola27_I2C_Loop_Mode 			   = %d\n", (REG27>>Vola27_I2C_Loop_Mode_OFST			 ) 		& Vola27_I2C_Loop_Mode_BASEMASK 			);
		printf("\tVola27_Sleep 					   = %d\n", (REG27>>Vola27_Sleep_OFST 				 ) 		& Vola27_Sleep_BASEMASK 					);
	
	printf("REG28 = 0x%x\n", REG28);
		printf("\tVola28_X_Axis_MSBs      = %d\n", (REG28>>Vola28_X_Axis_MSBs_OFST 	) 		& Vola28_X_Axis_MSBs_BASEMASK 	);
		printf("\tVola28_Y_Axis_MSBs      = %d\n", (REG28>>Vola28_Y_Axis_MSBs_OFST 	) 		& Vola28_Y_Axis_MSBs_BASEMASK 	);
		printf("\tVola28_Z_Axis_MSBs      = %d\n", (REG28>>Vola28_Z_Axis_MSBs_OFST 	) 		& Vola28_Z_Axis_MSBs_BASEMASK 	);
		printf("\tVola28_New_Data         = %d\n", (REG28>>Vola28_New_Data_OFST		) 		& Vola28_New_Data_BASEMASK		);
		printf("\tVola28_Interrupt        = %d\n", (REG28>>Vola28_Interrupt_OFST		) 	& Vola28_Interrupt_BASEMASK 		);
		printf("\tVola28_Temperature_MSBs = %d\n", (REG28>>Vola28_Temperature_MSBs_OFST) 	& Vola28_Temperature_MSBs_BASEMASK);
	
	printf("REG29 = 0x%x\n", REG29);
		printf("\tVola29_Interrupt_Write    = %d\n", (REG29>>Vola29_Interrupt_Write_OFST	 	) 		& Vola29_Interrupt_Write_BASEMASK	 	);
		printf("\tVola29_X_Axis_LSBs        = %d\n", (REG29>>Vola29_X_Axis_LSBs_OFST		 	) 		& Vola29_X_Axis_LSBs_BASEMASK		 	);
		printf("\tVola29_Y_Axis_LSBs        = %d\n", (REG29>>Vola29_Y_Axis_LSBs_OFST		 	) 		& Vola29_Y_Axis_LSBs_BASEMASK		 	);
		printf("\tVola29_Z_Axis_LSBs        = %d\n", (REG29>>Vola29_Z_Axis_LSBs_OFST		 	) 		& Vola29_Z_Axis_LSBs_BASEMASK		 	);
		printf("\tVola29_Hall_Mode_Status   = %d\n", (REG29>>Vola29_Hall_Mode_Status_OFST 	) 		& Vola29_Hall_Mode_Status_BASEMASK 	);
		printf("\tVola29_Temperature_LSBs   = %d\n", (REG29>>Vola29_Temperature_LSBs_OFST 	) 		& Vola29_Temperature_LSBs_BASEMASK 	);

}

void rd_hall_sens_data (void) {
	uint32_t REG28 = rd_hall_sens (Vola28_OFST, DISABLE_MESSAGE);
	uint32_t REG29 = rd_hall_sens (Vola29_OFST, DISABLE_MESSAGE);

	int16_t XDigit, YDigit, ZDigit, TempDigit;
	double XGauss, YGauss, ZGauss, Temp;
	XDigit 		= (((REG28>>Vola28_X_Axis_MSBs_OFST) & Vola28_X_Axis_MSBs_BASEMASK)<<4) 			| ((REG29>>Vola29_X_Axis_LSBs_OFST) & Vola29_X_Axis_LSBs_BASEMASK); // combine MSB and LSB
	YDigit 		= (((REG28>>Vola28_Y_Axis_MSBs_OFST) & Vola28_Y_Axis_MSBs_BASEMASK)<<4)		 		| ((REG29>>Vola29_Y_Axis_LSBs_OFST) & Vola29_Y_Axis_LSBs_BASEMASK); // combine MSB and LSB
	ZDigit 		= (((REG28>>Vola28_Z_Axis_MSBs_OFST) & Vola28_Z_Axis_MSBs_BASEMASK)<<4) 			| ((REG29>>Vola29_Z_Axis_LSBs_OFST) & Vola29_Z_Axis_LSBs_BASEMASK); // combine MSB and LSB
	TempDigit 	= (((REG28>>Vola28_Temperature_MSBs_OFST) & Vola28_Temperature_MSBs_BASEMASK)<<6) 	| ((REG29>>Vola29_Temperature_LSBs_OFST) & Vola29_Temperature_LSBs_BASEMASK); // combine MSB and LSB

	// The integer container is 16-bit signed, so we need to shift by 4 because the data is 12-bit signed value. We need to maintain the sign at MSB.
	XDigit = XDigit << 4;
	YDigit = YDigit << 4;
	ZDigit = ZDigit << 4;
	TempDigit = TempDigit << 4;

	XGauss = ((double) XDigit/16) / 4; // divide by 4 is coming from datasheet. Divide by 16 is coming from shift by 4 factor above.
	YGauss = ((double) YDigit/16) / 4;
	ZGauss = ((double) ZDigit/16) / 4;
	Temp = 25 + (((double) TempDigit/16) / 8); // divide by 8 is coming from datasheet. Divide by 16 is coming from shift by 4 factor above.

	printf("X = %5.2f Gauss\n", XGauss);
	printf("Y = %5.2f Gauss\n", YGauss);
	printf("Z = %5.2f Gauss\n", ZGauss);
	printf("Temp = %5.2f %%C\n", Temp);
	printf("\n");

}

void write_i2c_cnt (uint32_t en, uint32_t addr_msk, uint8_t en_mesg) {
	// en		: 1 for enable the address mask given, 0 for disable the address mask given
	// addr_msk	: give 1 to the desired address mask that needs to be changed, and 0 to the one that doesn't need to be changed
	// en_msg	: enable error message

	uint8_t i2c_addr_cnt = 0x40;	// i2c address for TCA9555PWR used by the relay
	i2c_addr_cnt >>= 1;				// shift by one because the LSB address is not used as an address (controlled by the Altera I2C IP)

	uint8_t i2c_port0, i2c_port1;
	if (en) {
		ctrl_i2c = ctrl_i2c | (addr_msk & 0xFFFF);
	}
	else {
		ctrl_i2c = ctrl_i2c & ~(addr_msk & 0xFFFF);
	}

	i2c_port0 = ctrl_i2c & 0xFF;
	i2c_port1 = (ctrl_i2c >> 8) & 0xFF;

	alt_write_word( (h2p_i2c_int_addr+ISR_OFST) , RX_OVER_MSK|ARBLOST_DET_MSK|NACK_DET_MSK ); // RESET THE I2C FROM PREVIOUS ERRORS

	alt_write_word( (h2p_i2c_int_addr+CTRL_OFST), 1<<CORE_EN_SHFT); // enable i2c core

	alt_write_word( (h2p_i2c_int_addr+SCL_LOW_OFST), 250); // set the SCL_LOW_OFST to 250 for 100 kHz with 50 MHz clock
	alt_write_word( (h2p_i2c_int_addr+SCL_HIGH_OFST), 250); // set the SCL_HIGH_OFST to 250 for 100 kHz with 50 MHz clock
	alt_write_word( (h2p_i2c_int_addr+SDA_HOLD_OFST), 1); // set the SDA_HOLD_OFST to 1 as the default (datasheet requires min 0 ns hold time)

	// set port 0 as output
	alt_write_word( (h2p_i2c_int_addr+TFR_CMD_OFST) , (1<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_cnt<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word( (h2p_i2c_int_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_CONF_PORT0 & I2C_DATA_MSK) );
	alt_write_word( (h2p_i2c_int_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((0x00) & I2C_DATA_MSK) );

	// set port 1 as output
	alt_write_word( (h2p_i2c_int_addr+TFR_CMD_OFST) , (1<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_cnt<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word( (h2p_i2c_int_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_CONF_PORT1 & I2C_DATA_MSK) );
	alt_write_word( (h2p_i2c_int_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((0x00) & I2C_DATA_MSK) );

	// set output on port 0
	alt_write_word( (h2p_i2c_int_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_cnt<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word( (h2p_i2c_int_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_OUT_PORT0 & I2C_DATA_MSK) );
	alt_write_word( (h2p_i2c_int_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((i2c_port0) & I2C_DATA_MSK) );

	// set output on port 1
	alt_write_word( (h2p_i2c_int_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_cnt<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word( (h2p_i2c_int_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_OUT_PORT1 & I2C_DATA_MSK) );
	alt_write_word( (h2p_i2c_int_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((i2c_port1) & I2C_DATA_MSK) );

	if (en_mesg) {
		printf("Status for i2c transactions:\n");
	}

	uint32_t isr_status;
	isr_status = alt_read_word( h2p_i2c_int_addr+ISR_OFST);
	if (isr_status & RX_OVER_MSK) {
		printf("\t[ERROR] Receive data FIFO has overrun condition, new data is lost\n");
		alt_write_word( (h2p_i2c_int_addr+ISR_OFST) , RX_OVER_MSK ); // clears receive overrun
	}
	else {
		if (en_mesg) printf("\t[NORMAL] No receive overrun\n");
	}
	if (isr_status & ARBLOST_DET_MSK) {
		printf("\t[ERROR] Core has lost bus arbitration\n");
		alt_write_word( (h2p_i2c_int_addr+ISR_OFST) , ARBLOST_DET_MSK ); // clears receive overrun
	}
	else {
		if (en_mesg) printf("\t[NORMAL] No arbitration lost\n");
	}
	if (isr_status & NACK_DET_MSK) {
		printf("\t[ERROR] NACK is received by the core\n");
		alt_write_word( (h2p_i2c_int_addr+ISR_OFST) , NACK_DET_MSK ); // clears receive overrun
	}
	else {
		if (en_mesg) printf("\t[NORMAL] ACK has been received\n");
	}
	if (isr_status & RX_READY_MSK) {
		printf("\t[WARNING] RX_DATA_FIFO level is equal or more than its threshold\n");
	}
	else {
		if (en_mesg) printf("\t[NORMAL] RX_DATA_FIFO level is less than its threshold\n");
	}
	if (isr_status & TX_READY_MSK) {
		printf("\t[WARNING] TFR_CMD level is equal or more than its threshold\n");
	}
	else {
		if (en_mesg) printf("\t[NORMAL] TFR_CMD level is less than its threshold\n");
	}

	alt_write_word( (h2p_i2c_int_addr+CTRL_OFST), 0<<CORE_EN_SHFT); // disable i2c core

	usleep(10000); // delay to finish i2c operation
}

void sweep_matching_network() {
	uint8_t c_sw = 0;
	uint8_t c_sta = 36;
	uint8_t c_sto = 40;
	uint8_t c_spa = 1;
	while (1) {
		for (c_sw = c_sta; c_sw <= c_sto; c_sw += c_spa) {
			// put number between 1 and 255 (0 is when nothing is connected)
			write_i2c_relay_cnt(c_sw,75, DISABLE_MESSAGE); //(c_shunt, c_series)
			printf("c_match_ntwrk = %d\n",c_sw);
			usleep(2000000);
		}
	}
};

void init_dac_ad5722r () {
	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	// setup the control lines for dac clear and dac ldac
	ctrl_out = ctrl_out | DAC_LDAC_en | DAC_CLR;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(10);

	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|PWR_CNT_REG|DAC_A_PU|DAC_B_PU|REF_PU );	// power up reference voltage, dac A, and dac B
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));					// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|OUT_RANGE_SEL_REG|DAC_AB|PN50 );			// set range voltage to +/- 5.0V
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));					// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|CNT_REG|Other_opt|Clamp_en);				// enable the current limit clamp
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));					// wait for the spi command to finish

	// clear the DAC output
	ctrl_out = ctrl_out & (~DAC_CLR) ;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(1);

	// release the clear pin
	ctrl_out = ctrl_out | DAC_CLR;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(1);

}

void print_warning_ad5722r() {
	int dataread;

	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , RD_DAC|PWR_CNT_REG );					// read the power control register
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));			// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|CNT_REG|NOP );					// no operation (NOP)
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));			// wait for the spi command to finish
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_RRDY_bit) ));			// wait for the data to be ready
	dataread = alt_read_word( h2p_dac_addr + SPI_RXDATA_offst );								// read the data
	if (dataread & (1<<5) ) {
		printf("\nDevice is in thermal shutdown (TSD) mode!\n");
	}
	if (dataread & (1<<7)) {
		printf("DAC A (vvarac) overcurrent alert (OCa)!\n");
		usleep(50);
	}
	if (dataread & (1<<9)) {
		printf("DAC B (vbias) overcurrent alert (OCb)!\n");
		usleep(50);
	}

}

void write_vvarac (double vvarac) {
	int16_t vvarac_int;

	vvarac_int = (int16_t)((vvarac/5)*2048);
	if (vvarac_int > 2047) {
		vvarac_int = 2047;
	}

	write_vvarac_int(vvarac_int);
}

void write_vbias (double vbias) {
	int16_t vbias_int;

	vbias_int = (int16_t)((vbias/5)*2048);
	if (vbias_int > 2047) {
		vbias_int = 2047;
	}

	write_vbias_int(vbias_int);
}

void write_vbias_int(int16_t dac_v_bias) { //  easy method to write number to dac
	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	/* OLD DRIVER: DON'T DELETE
	// setup the control lines for dac clear and dac ldac
	ctrl_out = (ctrl_out & (~DAC_LDAC_en)) | DAC_CLR;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;

	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|PWR_CNT_REG|DAC_A_PU|DAC_B_PU|REF_PU );	// power up reference voltage, dac A, and dac B
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));				// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|OUT_RANGE_SEL_REG|DAC_AB|PN50 );			// set range voltage to +/- 5.0V
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));				// wait for the spi command to finish

	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|DAC_REG|DAC_B|((dac_v_bias & 0x0FFF)<<4) );			// set the dac B voltage
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));				// wait for the spi command to finish
	*/

	/*
	// write data register to DAC output
	ctrl_out = ctrl_out & ~DAC_LDAC_en;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(50);
	*/

	// NEW CODE
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|DAC_REG|DAC_B|((dac_v_bias & 0x0FFF)<<4) );			// set the dac B voltage
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));

	// CODE BELOW IS WRITTEN BECAUSE SOMETIMES THE DAC DOESN'T WORK REALLY WELL
	// DATA WRITTEN IS NOT THE SAME AS DATA READ FROM THE ADC
	// THEREFORE THE DATA IS READ AND VERIFIED BEFORE IT IS USED AS AN OUTPUT

	// read the data just written to the dac

	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , RD_DAC|DAC_REG|DAC_B|0x00 );			// read DAC B value
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));			// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|CNT_REG|NOP );					// no operation (NOP)
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));			// wait for the spi command to finish
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_RRDY_bit) ));			// wait for read data to be ready

	/* in this version, the MISO is not connected to FPGA
	int dataread;
	dataread = alt_read_word( h2p_dac_addr + SPI_RXDATA_offst );								// read the data
	printf("vbias: %4.3f V ",(double)dac_v_bias/2048*5); 										// print the voltage desired
	printf("(w:0x%04x)", (dac_v_bias & 0x0FFF) ); 												// print the integer dac_varac value, truncate to 12-bit signed integer value
	printf("(r:0x%04x)\n",dataread>>4);															// print the read value
	// find out if warning has been detected
	usleep(1);
	print_warning_ad5722r();
	// if the data written to the dac is different than data read back, rewrite the dac
	// recursively until the data is right
	if ( (dac_v_bias&0x0FFF) != (dataread>>4)) {
		write_vbias_int (dac_v_bias);
	}
	*/

	// write data register to DAC output
	ctrl_out = ctrl_out & ~DAC_LDAC_en;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(50);

	// disable LDAC one more time
	ctrl_out = ctrl_out | DAC_LDAC_en;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(50);

}

void write_vvarac_int(int16_t dac_v_varac) { //  easy method to write number to dac

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|DAC_REG|DAC_A|((dac_v_varac & 0x0FFF)<<4) );			// set the dac A voltage
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));



	// CODE BELOW IS WRITTEN BECAUSE SOMETIMES THE DAC DOESN'T WORK REALLY WELL
	// DATA WRITTEN IS NOT THE SAME AS DATA READ FROM THE ADC
	// THEREFORE THE DATA IS READ AND VERIFIED BEFORE IT IS USED AS AN OUTPUT

	// read the data just written to the dac
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , RD_DAC|DAC_REG|DAC_A|0x00 );			// read DAC A value
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));			// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|CNT_REG|NOP );					// no operation (NOP)
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));			// wait for the spi command to finish
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_RRDY_bit) ));			// wait for read data to be ready

	/* in this version, the MISO is not connected to FPGA
	int dataread;
	dataread = alt_read_word( h2p_dac_addr + SPI_RXDATA_offst );								// read the data
	printf("vvarac: %4.3f V ",(double)dac_v_varac/2048*5); 										// print the voltage desired
	printf("(w:0x%04x)", (dac_v_varac & 0x0FFF) ); 												// print the integer dac_varac value, truncate to 12-bit signed integer value
	printf("(r:0x%04x)\n",dataread>>4);															// print the read value
	// find out if warning has been detected
	usleep(1);
	print_warning_ad5722r();
	// if the data written to the dac is different than data read back, rewrite the dac
	// recursively until the data is right
	if ( (dac_v_varac & 0x0FFF) != (dataread>>4)) {
		write_vvarac_int (dac_v_varac);
	}
	*/

	// write data register to DAC output
	ctrl_out = ctrl_out & ~DAC_LDAC_en;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(50);

	// disable LDAC one more time
	ctrl_out = ctrl_out | DAC_LDAC_en;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(50);


}

void sweep_vbias (){
	double vbias_cur;
	double vbias_sta = -5;
	double vbias_sto = 0.5;
	double vbias_spa = 0.1;
	vbias_cur = vbias_sta;
	while (1) {
		write_vbias(vbias_cur);
		vbias_cur += vbias_spa;
		if (vbias_cur > vbias_sto) {
			vbias_cur = vbias_sta;
		}
		usleep(1000000);
	}
}

void sweep_vvarac () {
	int16_t dac_v_varac = -450;
	int16_t init_varac_val = 2047;
	int16_t final_varac_val = -2048;
	dac_v_varac = init_varac_val;

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	while (1) {
		// clear the DAC output
		// ctrl_out = ctrl_out & (~DAC_CLR) ;
		// alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;

		// release the clear pin
		// ctrl_out = ctrl_out | DAC_CLR;
		// alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;

		// write vvarac
		write_vvarac_int(dac_v_varac);

		dac_v_varac -= 100;
		if (dac_v_varac < final_varac_val) {
			dac_v_varac = init_varac_val;
		}
		usleep(1000000);
	}
}

void fifo_to_sdram_dma_trf (uint32_t transfer_length) {
	alt_write_word(h2p_dma_addr+DMA_CONTROL_OFST,	DMA_CTRL_SWRST_MSK); 	// write twice to do software reset
		alt_write_word(h2p_dma_addr+DMA_CONTROL_OFST,	DMA_CTRL_SWRST_MSK); 	// software resetted
		alt_write_word(h2p_dma_addr+DMA_STATUS_OFST,	0x0); 					// clear the DONE bit
		alt_write_word(h2p_dma_addr+DMA_READADDR_OFST,	ADC_FIFO_MEM_OUT_BASE); // set DMA read address
		alt_write_word(h2p_dma_addr+DMA_WRITEADDR_OFST,	SDRAM_BASE);			// set DMA write address
		alt_write_word(h2p_dma_addr+DMA_LENGTH_OFST,	transfer_length*4);		// set transfer length (in byte, so multiply by 4 to get word-addressing)
		alt_write_word(h2p_dma_addr+DMA_CONTROL_OFST,	(DMA_CTRL_WORD_MSK|DMA_CTRL_LEEN_MSK|DMA_CTRL_RCON_MSK)); // set settings for transfer
		alt_write_word(h2p_dma_addr+DMA_CONTROL_OFST,	(DMA_CTRL_WORD_MSK|DMA_CTRL_LEEN_MSK|DMA_CTRL_RCON_MSK|DMA_CTRL_GO_MSK)); // set settings & also enable transfer
}

void datawrite_with_dma (uint32_t transfer_length, uint8_t en_mesg) {
	int i_sd = 0;

	fifo_to_sdram_dma_trf (transfer_length);

	unsigned int dma_status;
	do {
		dma_status = alt_read_word(h2p_dma_addr+DMA_STATUS_OFST);
		if (en_mesg) {
			printf("\tstatus reg: 0x%x\n",dma_status);
			if (!(dma_status & DMA_STAT_DONE_MSK)) {
				printf("\tDMA transaction is not done.\n");
			}
			if (dma_status & DMA_STAT_BUSY_MSK) {
				printf("\tDMA is busy.\n");
			}
		}
		usleep(10); // wait time to prevent overloading the DMA bus arbitration request
	}
	while (!(dma_status & DMA_STAT_DONE_MSK) || (dma_status & DMA_STAT_BUSY_MSK)); // keep in the loop when the 'DONE' bit is '0' and 'BUSY' bit is '1'

	if (en_mesg) {
		if (dma_status & DMA_STAT_REOP_MSK) {
			printf("\tDMA transaction completed due to end-of-packet on read side.\n");
		}
		if (dma_status & DMA_STAT_WEOP_MSK) {
			printf("\tDMA transaction completed due to end-of-packet on write side.\n");
		}
		if (dma_status & DMA_STAT_LEN_MSK) {
			printf("\tDMA transaction completed due to length-register decrements to 0.\n");
		}
	}

	unsigned int fifo_data_read;
	for (i_sd=0; i_sd < transfer_length; i_sd++) {
		fifo_data_read = alt_read_word(h2p_sdram_addr+i_sd);

		// the data is 2 symbols-per-beat in the fifo.
		// And the symbol arrangement can be found in Altera Embedded Peripherals pdf.
		// The 32-bit data per beat is transfered from FIFO to the SDRAM with the same
		// format so this formatting should follow the FIFO format.
		rddata_16[i_sd*2] = fifo_data_read & 0x3FFF;
		rddata_16[i_sd*2+1] = (fifo_data_read>>16) & 0x3FFF;
	}
}

void tx_sampling(double tx_freq, double samp_freq, unsigned int tx_num_of_samples, char * filename) {

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	// KEEP THIS CODE AND ENABLE IT IF YOU USE C-ONLY, OPPOSED TO USING PYTHON
	// activate signal_coup path (from the directional coupler) for the receiver
	// write_i2c_cnt (DISABLE, RX_IN_SEL_1_msk, DISABLE_MESSAGE);
	// write_i2c_cnt (ENABLE, RX_IN_SEL_2_msk, DISABLE_MESSAGE);

	// set parameters for acquisition (using CPMG registers and CPMG sequence: not a good practice)
	alt_write_word( (h2p_pulse1_addr) , 100); // random safe number
	alt_write_word( (h2p_delay1_addr) , 100); // random safe number
	alt_write_word( (h2p_pulse2_addr) , 100); // random safe number
	alt_write_word( (h2p_delay2_addr) , tx_num_of_samples*4*2); // *4 is because the system clock is 4*ADC clock. *2 factor is to increase the delay_window to about 2*acquisition window for safety.
	alt_write_word( (h2p_init_adc_delay_addr) , (unsigned int)(tx_num_of_samples/2)); // put adc acquisition window exactly at the middle of the delay windo
	alt_write_word( (h2p_echo_per_scan_addr) , 1 );
	alt_write_word( (h2p_adc_samples_per_echo_addr) , tx_num_of_samples);
	// set the system frequency, which is sampling frequency*4
	Set_PLL (h2p_nmr_sys_pll_addr, 0, samp_freq*4, 0.5, DISABLE_MESSAGE);
	Reset_PLL (h2p_ctrl_out_addr, PLL_NMR_SYS_RST_ofst, ctrl_out);
	Set_DPS (h2p_nmr_sys_pll_addr, 0, 0, DISABLE_MESSAGE);
	Wait_PLL_To_Lock (h2p_ctrl_in_addr, PLL_NMR_SYS_lock_ofst);

	// set pll for the tx sampling
	Set_PLL (h2p_analyzer_pll_addr, 0, tx_freq, 0.5, DISABLE_MESSAGE);
	Set_PLL (h2p_analyzer_pll_addr, 1, tx_freq, 0.5, DISABLE_MESSAGE);
	Set_PLL (h2p_analyzer_pll_addr, 2, tx_freq, 0.5, DISABLE_MESSAGE);
	Set_PLL (h2p_analyzer_pll_addr, 3, tx_freq, 0.5, DISABLE_MESSAGE);
	Reset_PLL (h2p_ctrl_out_addr, PLL_ANALYZER_RST_ofst, ctrl_out);
	Wait_PLL_To_Lock (h2p_ctrl_in_addr, PLL_ANALYZER_lock_ofst);
	Set_DPS (h2p_analyzer_pll_addr, 0, 0, DISABLE_MESSAGE);
	Set_DPS (h2p_analyzer_pll_addr, 1, 90, DISABLE_MESSAGE);
	Set_DPS (h2p_analyzer_pll_addr, 2, 180, DISABLE_MESSAGE);
	Set_DPS (h2p_analyzer_pll_addr, 3, 270, DISABLE_MESSAGE);
	Wait_PLL_To_Lock (h2p_ctrl_in_addr, PLL_ANALYZER_lock_ofst);

	// reset buffer
	ctrl_out |= (0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);
	ctrl_out &= ~(0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);

	// enable PLL_analyzer path, disable RF gate path
	ctrl_out &= ~(NMR_CLK_GATE_AVLN);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);

	// start the state machine to capture data
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out | (0x01<<FSM_START_ofst) );
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out & ~(0x01<<FSM_START_ofst) );
	// wait until fsm stops
	while (alt_read_word(h2p_ctrl_in_addr) & (0x01<<NMR_SEQ_run_ofst) );
	usleep(10);

	// disable PLL_analyzer path and enable the default RF gate path
	ctrl_out |= NMR_CLK_GATE_AVLN;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);

	// KEEP THIS CODE AND ENABLE IT IF YOU USE C-ONLY, OPPOSED TO USING PYTHON
	// activate normal signal path for the receiver
	// write_i2c_cnt (ENABLE, RX_IN_SEL_1_msk, DISABLE_MESSAGE);
	// write_i2c_cnt (DISABLE, RX_IN_SEL_2_msk, DISABLE_MESSAGE);

	uint32_t fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG); // the fill level of FIFO memory
	for (i=0; fifo_mem_level>0; i++) {
		rddata[i] = alt_read_word(h2p_adc_fifo_addr);

		fifo_mem_level--;
		if (fifo_mem_level == 0) {
			fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG);
		}
	}

	if (i*2 == tx_num_of_samples) { // if the amount of data captured matched with the amount of data being ordered, then continue the process. if not, then don't process the datas (requesting empty data from the fifo will cause the FPGA to crash, so this one is to avoid that)
		// printf("number of captured data vs requested data : MATCHED\n");

		j=0;
		// FIFO is 32-bit, while 1-sample is only 16-bit. FIFO organize this automatically.
		for(i=0; i <  ( (long)tx_num_of_samples>>1 ); i++) {
			rddata_16[j++] = (rddata[i] & 0x3FFF);		// 14 significant bit
			rddata_16[j++] = ((rddata[i]>>16)&0x3FFF);	// 14 significant bit
		}

		// write the raw data from adc to a file
		sprintf(pathname,"%s/%s",foldername,filename);	// put the data into the data folder
		fptr = fopen(pathname, "w");
		if (fptr == NULL) {
			printf("File does not exists \n");
		}
		for(i=0; i < ( (long)tx_num_of_samples ); i++) {
			fprintf(fptr, "%d\n", rddata_16[i]);
		}
		fclose(fptr);

	}
	else { // if the amount of data captured didn't match the amount of data being ordered, then something's going on with the acquisition
		printf("number of data captured and data order : NOT MATCHED\nReconfigure the FPGA immediately\n");
	}
}

void noise_sampling (unsigned char signal_path, unsigned int num_of_samples, char * filename) {
	// signal path: the signal path used with the ADC, can be normal signal path or S11 signal path

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	alt_write_word( (h2p_init_adc_delay_addr) , 0 );	// don't need adc_delay for sampling the data
	alt_write_word( (h2p_adc_samples_per_echo_addr) , num_of_samples ); // the number of samples taken for tx sampling

	// KEEP THIS CODE IF YOU DON'T USE PYTHON
	//if (signal_path == SIG_NORM_PATH) {
		// activate normal signal path for the receiver
	//	write_i2c_cnt (ENABLE, RX_IN_SEL_1_msk, DISABLE_MESSAGE);
	//	write_i2c_cnt (DISABLE, RX_IN_SEL_2_msk, DISABLE_MESSAGE);

	//	printf("normal signal path is connected...\n");
	//}
	//else if (signal_path == SIG_S11_PATH) {
	//	// activate signal_coup path for the receiver
	//	write_i2c_cnt (DISABLE, RX_IN_SEL_1_msk, DISABLE_MESSAGE);
	//	write_i2c_cnt (ENABLE, RX_IN_SEL_2_msk, DISABLE_MESSAGE);
	//	printf("S11 signal path is connected...");
	//}
	//else {
	//	printf("signal path is not defined...");
	//}
	//usleep(100);

	// reset the selected ADC (ADC reset is omitted in new design)
	// alt_write_word( (h2p_ctrl_out_addr) , ctrl_out | (0x01<<ADC_LTC1746_RST_ofst) );
	// usleep(10);
	// alt_write_word( (h2p_ctrl_out_addr) , ctrl_out & ~(0x01<<ADC_LTC1746_RST_ofst) );
	// usleep(10);

	// reset buffer
	ctrl_out |= (0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);
	ctrl_out &= ~(0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);

	// send ADC start pulse signal
	ctrl_out |= ACTIVATE_ADC_AVLN; // this signal is connected to pulser, so it needs to be turned of as quickly as possible after it is turned on
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	ctrl_out &= ~ACTIVATE_ADC_AVLN; // turning off the ADC start signal
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10000); // delay for data acquisition


	uint32_t fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG); // the fill level of FIFO memory
	for (i=0; fifo_mem_level>0; i++) {
		rddata[i] = alt_read_word(h2p_adc_fifo_addr);

		fifo_mem_level--;
		if (fifo_mem_level == 0) {
			fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG);
		}
	}
	// usleep(100);
	// fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG);

	printf("i: %ld",i);

	if (i*2 == num_of_samples) { // if the amount of data captured matched with the amount of data being ordered, then continue the process. if not, then don't process the datas (requesting empty data from the fifo will cause the FPGA to crash, so this one is to avoid that)
		// printf("number of captured data vs requested data : MATCHED\n");

		j=0;
		// FIFO is 32-bit, while 1-sample is only 16-bit. FIFO organize this automatically.
		for(i=0; i <  ( (long)num_of_samples>>1 ); i++) {
			rddata_16[j++] = (rddata[i] & 0x3FFF);		// 14 significant bit
			rddata_16[j++] = ((rddata[i]>>16)&0x3FFF);	// 14 significant bit
		}

		// write the raw data from adc to a file
		sprintf(pathname,"%s/%s",foldername,filename);	// put the data into the data folder
		fptr = fopen(pathname, "w");
		if (fptr == NULL) {
			printf("File does not exists \n");
		}
		for(i=0; i < ( (long)num_of_samples ); i++) {
			fprintf(fptr, "%d\n", rddata_16[i]);
		}
		fclose(fptr);

	}
	else { // if the amount of data captured didn't match the amount of data being ordered, then something's going on with the acquisition
		printf("number of data captured and data order : NOT MATCHED\nReconfigure the FPGA immediately\n");
	}
}

// duty cycle is not functioning anymore
void CPMG_Sequence (double cpmg_freq, double pulse1_us, double pulse2_us, double pulse1_dtcl, double pulse2_dtcl, double echo_spacing_us, long unsigned scan_spacing_us, unsigned int samples_per_echo, unsigned int echoes_per_scan, double init_adc_delay_compensation, uint32_t ph_cycl_en, char * filename, char * avgname, uint32_t enable_message) {
	unsigned int cpmg_param [5];
	double adc_ltc1746_freq = cpmg_freq*4;
	double nmr_fsm_clkfreq = cpmg_freq*16;

	double init_delay_inherent = 2.25; // inherehent delay factor from the HDL structure, in ADC clock cycles

	// read settings
	uint8_t data_nowrite = 0; // do not write the data from fifo to text file (external reading mechanism should be implemented)
	uint8_t read_with_dma = 1; // else the program reads data directly from the fifo
	uint8_t wr_indv_data = 0;

	usleep(scan_spacing_us);

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	// local variables
	uint32_t fifo_mem_level; // the fill level of fifo memory

	usleep(100);

	cpmg_param_calculator_ltc1746(
		cpmg_param,
		nmr_fsm_clkfreq,
		cpmg_freq,
		adc_ltc1746_freq,
		init_adc_delay_compensation,
		pulse1_us,
		pulse2_us,
		echo_spacing_us,
		samples_per_echo
	);

	alt_write_word( (h2p_pulse1_addr) , cpmg_param[PULSE1_OFFST] );
	alt_write_word( (h2p_delay1_addr) , cpmg_param[DELAY1_OFFST] );
	alt_write_word( (h2p_pulse2_addr) , cpmg_param[PULSE2_OFFST] );
	alt_write_word( (h2p_delay2_addr) , cpmg_param[DELAY2_OFFST] );
	alt_write_word( (h2p_init_adc_delay_addr) , cpmg_param[INIT_DELAY_ADC_OFFST] );
	alt_write_word( (h2p_echo_per_scan_addr) , echoes_per_scan );
	alt_write_word( (h2p_adc_samples_per_echo_addr) , samples_per_echo );

	if (enable_message) {
		printf("CPMG Sequence Actual Parameter:\n");
		printf("\tPulse 1\t\t\t: %7.3f us (%d)\n", (double)cpmg_param[PULSE1_OFFST]/nmr_fsm_clkfreq, cpmg_param[PULSE1_OFFST]);
		printf("\tDelay 1\t\t\t: %7.3f us (%d)\n", (double)cpmg_param[DELAY1_OFFST]/nmr_fsm_clkfreq, cpmg_param[DELAY1_OFFST]);
		printf("\tPulse 2\t\t\t: %7.3f us (%d)\n", (double)cpmg_param[PULSE2_OFFST]/nmr_fsm_clkfreq, cpmg_param[PULSE2_OFFST]);
		printf("\tDelay 2\t\t\t: %7.3f us (%d)\n", (double)cpmg_param[DELAY2_OFFST]/nmr_fsm_clkfreq, cpmg_param[DELAY2_OFFST]);
		printf("\tADC init delay\t: %7.3f us (%d) -not-precise\n", ((double)cpmg_param[INIT_DELAY_ADC_OFFST]+init_delay_inherent)/adc_ltc1746_freq, cpmg_param[INIT_DELAY_ADC_OFFST]);
		printf("\tADC acq window\t: %7.3f us (%d)\n", ((double)samples_per_echo)/adc_ltc1746_freq, samples_per_echo);
	}
	if (cpmg_param[INIT_DELAY_ADC_OFFST] < 2) {
		printf("\tWARNING: Computed ADC_init_delay is less than 2, ADC_init_delay is force driven to 2 inside the HDL!");
	}

	// set pll for CPMG
	Set_PLL (h2p_nmr_sys_pll_addr, 0, nmr_fsm_clkfreq, 0.5, DISABLE_MESSAGE);
	Reset_PLL (h2p_ctrl_out_addr, PLL_NMR_SYS_RST_ofst, ctrl_out);
	// Set_DPS (h2p_nmr_sys_pll_addr, 0, 0, DISABLE_MESSAGE);
	Wait_PLL_To_Lock (h2p_ctrl_in_addr, PLL_NMR_SYS_lock_ofst);


	// cycle phase for CPMG measurement
	if (ph_cycl_en == ENABLE) {
		if (ctrl_out & (0x01<<PHASE_CYCLING_ofst)) {
			ctrl_out &= ~(0x01<<PHASE_CYCLING_ofst);
		}
		else {
			ctrl_out |= (0x01<<PHASE_CYCLING_ofst);
		}
		alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
		usleep(10);
	}

	// reset the selected ADC (the ADC reset was omitted)
	// alt_write_word( (h2p_ctrl_out_addr) , ctrl_out | (0x01<<ADC_LTC1746_RST_ofst) );
	// usleep(10);
	// alt_write_word( (h2p_ctrl_out_addr) , ctrl_out & ~(0x01<<ADC_LTC1746_RST_ofst) );
	// usleep(10);


	// reset buffer
	ctrl_out |= (0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);
	ctrl_out &= ~(0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);

	// start fsm
	// it will reset the pll as well, so it's important to set the phase
	// the pll_rst_dly should be longer than the delay coming from changing the phase
	// otherwise, the fsm will start with wrong relationship between 4 pll output clocks (1/2 pi difference between clock)
	// alt_write_word( (h2p_nmr_pll_rst_dly_addr) , 1000000 );	// set the amount of delay for pll reset (with 50MHz system clock, every tick means 20ns) -> default: 100000
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out | (0x01<<FSM_START_ofst) );
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out & ~(0x01<<FSM_START_ofst) );
	// shift the pll phase accordingly
	// Set_DPS (h2p_nmr_pll_addr, 0, 0, DISABLE_MESSAGE);
	// Set_DPS (h2p_nmr_pll_addr, 1, 90, DISABLE_MESSAGE);
	// Set_DPS (h2p_nmr_pll_addr, 2, 180, DISABLE_MESSAGE);
	// Set_DPS (h2p_nmr_pll_addr, 3, 270, DISABLE_MESSAGE);
	// usleep(scan_spacing_us);

	if (!data_nowrite) { // write data to text with C programming
		if (read_with_dma) { // if read with dma is intended
			datawrite_with_dma(samples_per_echo*echoes_per_scan/2,DISABLE_MESSAGE);
		}
		else { // if read from fifo is intended
			// wait until fsm stops
			while ( alt_read_word(h2p_ctrl_in_addr) & (0x01<<NMR_SEQ_run_ofst) );
			usleep(300);

			// PRINT # of DATAS in FIFO
			// fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG); // the fill level of FIFO memory
			// printf("num of data in fifo: %d\n",fifo_mem_level);

			// READING DATA FROM FIFO
			fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG); // the fill level of FIFO memory
			for (i=0; fifo_mem_level>0; i++) {			// FIFO is 32-bit, while 1-sample is only 16-bit. FIFO organize this automatically. So, fetch only amount_of_data shifted by 2 to get amount_of_data/2.
				rddata[i] = alt_read_word(h2p_adc_fifo_addr);

				fifo_mem_level--;
				if (fifo_mem_level == 0) {
					fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG);
				}
				//usleep(1);
			}
			usleep(100);

			if (i*2 == samples_per_echo*echoes_per_scan) { // if the amount of data captured matched with the amount of data being ordered, then continue the process. if not, then don't process the datas (requesting empty data from the fifo will cause the FPGA to crash, so this one is to avoid that)
				// printf("number of captured data vs requested data : MATCHED\n");

				j=0;
				for(i=0; i < ( ((long)samples_per_echo*(long)echoes_per_scan)>>1 ); i++) {
					rddata_16[j++] = (rddata[i] & 0x3FFF);		// 14 significant bit
					rddata_16[j++] = ((rddata[i]>>16)&0x3FFF);	// 14 significant bit
				}

			}
			else { // if the amount of data captured didn't match the amount of data being ordered, then something's going on with the acquisition
				printf("[ERROR] number of data captured (%ld) and data ordered (%d): NOT MATCHED\nData are flushed!\nReconfigure the FPGA immediately\n", i*2, samples_per_echo*echoes_per_scan);
			}
		}

		if (wr_indv_data) {
			// write the raw data from adc to a file
			sprintf(pathname,"%s/%s",foldername,filename);	// put the data into the data folder
			fptr = fopen(pathname, "w");
			if (fptr == NULL) {
				printf("File does not exists \n");
			}
			for(i=0; i < ( ((long)samples_per_echo*(long)echoes_per_scan) ); i++) {
				fprintf(fptr, "%d\n", rddata_16[i]);
			}
			fclose(fptr);

			// write the averaged data to a file
			unsigned int avr_data[samples_per_echo];
			// initialize array
			for (i=0; i<samples_per_echo; i++) {
				avr_data[i] = 0;
			};
			for (i=0; i<samples_per_echo; i++) {
				for (j=i; j<( ((long)samples_per_echo*(long)echoes_per_scan) ); j+=samples_per_echo) {
					avr_data[i] += rddata_16[j];
				}
			}
			sprintf(pathname,"%s/%s",foldername,avgname);	// put the data into the data folder
			fptr = fopen(pathname, "w");
			if (fptr == NULL) {
				printf("File does not exists \n");
			}
			for (i=0; i<samples_per_echo; i++) {
				fprintf(fptr, "%d\n", avr_data[i]);
			}
			fclose(fptr);
		}
	}
	else { // do not write data to text with C programming: external mechanism should be implemented
		fifo_to_sdram_dma_trf (samples_per_echo*echoes_per_scan/2); // start DMA process
		//while ( alt_read_word(h2p_ctrl_in_addr) & (0x01<<NMR_SEQ_run_ofst) ); // might not be needed as the system will wait until data is available anyway
	}

}

void CPMG_iterate (
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
	unsigned int number_of_iteration,
	uint32_t ph_cycl_en
){
	double nmr_fsm_clkfreq = 16*cpmg_freq;
	double adc_ltc1746_freq = 4*cpmg_freq;

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	create_measurement_folder("cpmg");
	// printf("Approximated measurement time : %.2f mins\n",( scan_spacing_us*(double)number_of_iteration) *1e-6/60);

	unsigned int cpmg_param [5];
	cpmg_param_calculator_ltc1746(
		cpmg_param,
		nmr_fsm_clkfreq,
		cpmg_freq,
		adc_ltc1746_freq,
		init_adc_delay_compensation,
		pulse1_us,
		pulse2_us,
		echo_spacing_us,
		samples_per_echo
	);
	// print general measurement settings
	sprintf(pathname,"%s/acqu.par",foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr,"b1Freq = %4.3f\n", cpmg_freq);
	fprintf(fptr,"p90LengthGiven = %4.3f\n", pulse1_us);
	fprintf(fptr,"p90LengthRun = %4.3f\n", (double)cpmg_param[PULSE1_OFFST]/nmr_fsm_clkfreq);
	fprintf(fptr,"p90LengthCnt = %d @ %4.3f MHz\n", cpmg_param[PULSE1_OFFST],nmr_fsm_clkfreq);
	fprintf(fptr,"d90LengthRun = %4.3f\n", (double)cpmg_param[DELAY1_OFFST]/nmr_fsm_clkfreq);
	fprintf(fptr,"d90LengthCnt = %d @ %4.3f MHz\n", cpmg_param[DELAY1_OFFST],nmr_fsm_clkfreq);
	fprintf(fptr,"p180LengthGiven = %4.3f\n", pulse2_us);
	fprintf(fptr,"p180LengthRun = %4.3f\n", (double)cpmg_param[PULSE2_OFFST]/nmr_fsm_clkfreq);
	fprintf(fptr,"p180LengthCnt =  %d @ %4.3f MHz\n", cpmg_param[PULSE2_OFFST],nmr_fsm_clkfreq);
	fprintf(fptr,"d180LengthRun = %4.3f\n", (double)cpmg_param[DELAY2_OFFST]/nmr_fsm_clkfreq);
	fprintf(fptr,"d180LengthCnt = %d @ %4.3f MHz\n", cpmg_param[DELAY2_OFFST],nmr_fsm_clkfreq);
	//fprintf(fptr,"p90_dtcl = %4.3f\n", pulse1_dtcl);
	//fprintf(fptr,"p180_dtcl = %4.3f\n", pulse2_dtcl);
	fprintf(fptr,"echoTimeRun = %4.3f\n", (double)(cpmg_param[PULSE2_OFFST]+cpmg_param[DELAY2_OFFST])/nmr_fsm_clkfreq );
	fprintf(fptr,"echoTimeGiven = %4.3f\n", echo_spacing_us);
	fprintf(fptr,"ieTime = %lu\n", scan_spacing_us/1000);
	fprintf(fptr,"nrPnts = %d\n", samples_per_echo);
	fprintf(fptr,"nrEchoes = %d\n", echoes_per_scan);
	fprintf(fptr,"echoShift = %4.3f\n", init_adc_delay_compensation);
	fprintf(fptr,"nrIterations = %d\n", number_of_iteration);
	fprintf(fptr,"dummyEchoes = 0\n");
	fprintf(fptr,"adcFreq = %4.3f\n", adc_ltc1746_freq);
	fprintf(fptr,"dwellTime = %4.3f\n", 1/adc_ltc1746_freq);
	fprintf(fptr,"usePhaseCycle = %d\n", ph_cycl_en);
	fclose(fptr);

	// print matlab script to analyze datas
	sprintf(pathname,"measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr,"compute_iterate([data_folder,'%s']);\n",foldername);
	fclose(fptr);

	// print matlab script to analyze datas
	sprintf(pathname,"current_folder.txt");
	fptr = fopen(pathname, "w");
	fprintf(fptr,"%s\n",foldername);
	fclose(fptr);

	int iterate = 1;

	int FILENAME_LENGTH = 100;
	char *name;
	name = (char*) malloc (FILENAME_LENGTH*sizeof(char));
	char *nameavg;
	nameavg = (char*) malloc (FILENAME_LENGTH*sizeof(char));

	int Asum[samples_per_echo*echoes_per_scan];
	for (i=0; i<samples_per_echo*echoes_per_scan; i++) Asum[i] = 0;

	for (iterate=1; iterate<=number_of_iteration; iterate++) {
		// printf("\n*** RUN %d ***\n",iterate);

		snprintf(name, FILENAME_LENGTH,"dat_%03d",iterate);
		snprintf(nameavg, FILENAME_LENGTH,"avg_%03d",iterate);

		CPMG_Sequence (
			cpmg_freq,						//cpmg_freq
			pulse1_us,						//pulse1_us
			pulse2_us,						//pulse2_us
			pulse1_dtcl,					//pulse1_dtcl
			pulse2_dtcl,					//pulse2_dtcl
			echo_spacing_us,				//echo_spacing_us
			scan_spacing_us,				//scan_spacing_us
			samples_per_echo,				//samples_per_echo
			echoes_per_scan,				//echoes_per_scan
			init_adc_delay_compensation,	//compensation delay number (counted by the adc base clock)
			ph_cycl_en,						//phase cycle enable/disable
			name,							//filename for data
			nameavg,						//filename for average data
			DISABLE_MESSAGE
		);

		// process the data
		if (ph_cycl_en) {
			if (iterate % 2 == 0)
				for (i=0; i<samples_per_echo*echoes_per_scan; i++) Asum[i]-=rddata_16[i];
			else
				for (i=0; i<samples_per_echo*echoes_per_scan; i++) Asum[i]+=rddata_16[i];
		}
		else {
			for (i=0; i<samples_per_echo*echoes_per_scan; i++) Asum[i]+=rddata_16[i];
		}

	}

	sprintf(pathname,"%s/%s",foldername,"asum");	// put the data into the data folder
	fptr = fopen(pathname, "w");
	for(i=0; i<samples_per_echo*echoes_per_scan; i++) fprintf(fptr, "%d\n", Asum[i]);
	fclose(fptr);


	free(name);
	free(nameavg);

}

void FID (double cpmg_freq, double pulse2_us, double pulse2_dtcl, long unsigned scan_spacing_us, unsigned int samples_per_echo, char * filename, uint32_t enable_message) {
	double adc_ltc1746_freq = cpmg_freq*4;
	double nmr_fsm_clkfreq = cpmg_freq*16;
	uint8_t read_with_dma = 1; // else the program reads data directly from the fifo

	usleep(scan_spacing_us);

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	// local variables
	uint32_t fifo_mem_level; // the fill level of fifo memory

	unsigned int pulse2_int = (unsigned int)(round(pulse2_us * nmr_fsm_clkfreq));	// the number of 180 deg pulse in the multiplication of cpmg pulse period (discrete value, no continuous number supported)
	unsigned int delay2_int = (unsigned int) (round(samples_per_echo*(nmr_fsm_clkfreq/adc_ltc1746_freq)*10));	// the number of delay after 180 deg pulse. It is simply samples_per_echo multiplied by (nmr_fsm_clkfreq/adc_ltc1746_freq) factor, as the delay2_int is counted by nmr_fsm_clkfreq, not by adc_ltc1746_freq. It is also multiplied by a constant 10 as safety factor to make sure the ADC acquisition is inside FSMSTAT (refer to HDL) 'on' window.
	unsigned int fixed_init_adc_delay = 2; // set to the minimum delay values, which is 2 (limited by HDL structure).
	unsigned int fixed_echo_per_scan = 1; // it must be 1, otherwise the HDL will go to undefined state.
	double init_delay_inherent; // inherehent delay factor from the HDL structure. The minimum is 2.25 no matter how small the delay is set. Look ERRATA
	if (fixed_init_adc_delay <= 2) {
		init_delay_inherent = 2.25;
	}
	else { // if fixed_init_adc_delay is more than 2
		init_delay_inherent = (double) fixed_init_adc_delay + 0.25; // look at ERRATA from the HDL to get 0.25
	}

	alt_write_word( (h2p_pulse1_addr) , 0 );
	alt_write_word( (h2p_delay1_addr) , 0 );
	alt_write_word( (h2p_pulse2_addr) , pulse2_int );
	alt_write_word( (h2p_delay2_addr) , delay2_int );
	alt_write_word( (h2p_init_adc_delay_addr) , fixed_init_adc_delay );
	alt_write_word( (h2p_echo_per_scan_addr) , fixed_echo_per_scan );
	alt_write_word( (h2p_adc_samples_per_echo_addr) , samples_per_echo );

	if (enable_message) {
		printf("CPMG Sequence Actual Parameter:\n");
		printf("\tPulse 2\t\t\t: %7.3f us (%d)\n", (double)pulse2_int/nmr_fsm_clkfreq, pulse2_int);
		printf("\tDelay 2\t\t\t: %7.3f us (%d)\n", (double)delay2_int/nmr_fsm_clkfreq, delay2_int);
		printf("\tADC init delay\t: %7.3f us (%d) --imprecise\n", init_delay_inherent/adc_ltc1746_freq, fixed_init_adc_delay );
		printf("\tADC acq window\t: %7.3f us (%d)\n", ((double)samples_per_echo)/adc_ltc1746_freq, samples_per_echo);
	}
	if (fixed_init_adc_delay < 2) {
		printf("\tWARNING: Computed ADC_init_delay is less than 2, ADC_init_delay is force driven to 2 inside the HDL!");
	}

	// set pll for CPMG system
	Set_PLL (h2p_nmr_sys_pll_addr, 0, nmr_fsm_clkfreq, 0.5, DISABLE_MESSAGE);	// set pll frequency
	Reset_PLL (h2p_ctrl_out_addr, PLL_NMR_SYS_RST_ofst, ctrl_out);				// reset pll, changes the phase
	Set_DPS (h2p_nmr_sys_pll_addr, 0, 0, DISABLE_MESSAGE); 						// set pll phase to 0 (might not be needed)
	Wait_PLL_To_Lock (h2p_ctrl_in_addr, PLL_NMR_SYS_lock_ofst);					// wait for pll to lock

	// set a fix phase cycle state
	ctrl_out &= ~(0x01<<PHASE_CYCLING_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);

	// reset the selected ADC (the ADC reset was omitted)
	// alt_write_word( (h2p_ctrl_out_addr) , ctrl_out | (0x01<<ADC_LTC1746_RST_ofst) );
	// usleep(10);
	// alt_write_word( (h2p_ctrl_out_addr) , ctrl_out & ~(0x01<<ADC_LTC1746_RST_ofst) );
	// usleep(10);


	// reset ADC buffer
	ctrl_out |= (0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);
	ctrl_out &= ~(0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);

	// start fsm
	// it will reset the pll as well, so it's important to set the phase
	// the pll_rst_dly should be longer than the delay coming from changing the phase
	// otherwise, the fsm will start with wrong relationship between 4 pll output clocks (1/2 pi difference between clock)
	// alt_write_word( (h2p_nmr_pll_rst_dly_addr) , 1000000 );	// set the amount of delay for pll reset (with 50MHz system clock, every tick means 20ns) -> default: 100000
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out | (0x01<<FSM_START_ofst) );
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out & ~(0x01<<FSM_START_ofst) );
	// shift the pll phase accordingly
	// Set_DPS (h2p_nmr_pll_addr, 0, 0, DISABLE_MESSAGE);
	// Set_DPS (h2p_nmr_pll_addr, 1, 90, DISABLE_MESSAGE);
	// Set_DPS (h2p_nmr_pll_addr, 2, 180, DISABLE_MESSAGE);
	// Set_DPS (h2p_nmr_pll_addr, 3, 270, DISABLE_MESSAGE);
	// usleep(scan_spacing_us);

	if (read_with_dma) { // if read with dma is intended
		datawrite_with_dma (samples_per_echo/2,enable_message); // divided by 2 to compensate 2 symbol per beat in the fifo interface
	}
	else { // if read from fifo is intended
		// wait until fsm stops
		while ( alt_read_word(h2p_ctrl_in_addr) & (0x01<<NMR_SEQ_run_ofst) );
		usleep(300);

		// PRINT # of DATAS in FIFO
		// fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG); // the fill level of FIFO memory
		// printf("num of data in fifo: %d\n",fifo_mem_level);

		// READING DATA FROM FIFO
		fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG); // the fill level of FIFO memory
		for (i=0; fifo_mem_level>0; i++) {			// FIFO is 32-bit, while 1-sample is only 16-bit. FIFO organize this automatically. So, fetch only amount_of_data shifted by 2 to get amount_of_data/2.
			rddata[i] = alt_read_word(h2p_adc_fifo_addr);

			fifo_mem_level--;
			if (fifo_mem_level == 0) {
				fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG);
			}
			//usleep(1);
		}
		usleep(100);

		if (i*2 == samples_per_echo) { // if the amount of data captured matched with the amount of data being ordered, then continue the process. if not, then don't process the datas (requesting empty data from the fifo will cause the FPGA to crash, so this one is to avoid that)
			// printf("number of captured data vs requested data : MATCHED\n");

			j=0;
			for(i=0; i < ( ((long)samples_per_echo)>>1 ); i++) {
				rddata_16[j++] = (rddata[i] & 0x3FFF);		// 14 significant bit
				rddata_16[j++] = ((rddata[i]>>16)&0x3FFF);	// 14 significant bit
			}

		}
		else { // if the amount of data captured didn't match the amount of data being ordered, then something's going on with the acquisition
			printf("[ERROR] number of data captured (%ld) and data ordered (%d): NOT MATCHED\nData are flushed!\nReconfigure the FPGA immediately\n", i*2, samples_per_echo);
		}
	}

	// write the raw data from adc to a file
	sprintf(pathname,"%s/%s",foldername,filename);	// put the data into the data folder
	fptr = fopen(pathname, "w");
	if (fptr == NULL) {
		printf("File does not exists \n");
	}
	for(i=0; i < ( ((long)samples_per_echo) ); i++) {
		fprintf(fptr, "%d\n", rddata_16[i]);
	}
	fclose(fptr);



}

void FID_iterate (
	double cpmg_freq,
	double pulse2_us,
	double pulse2_dtcl,
	long unsigned scan_spacing_us,
	unsigned int samples_per_echo,
	unsigned int number_of_iteration,
	uint32_t enable_message
){
	double nmr_fsm_clkfreq = 16*cpmg_freq;
	double adc_ltc1746_freq = 4*cpmg_freq;

	unsigned int fixed_init_adc_delay = 2; // set to the minimum delay values, which is 2 (limited by HDL structure).
	double init_delay_inherent; // inherehent delay factor from the HDL structure. The minimum is 2.25 no matter how small the delay is set. Look ERRATA
	if (fixed_init_adc_delay <= 2) {
		init_delay_inherent = 2.25;
	}
	else { // if fixed_init_adc_delay is more than 2
		init_delay_inherent = (double) fixed_init_adc_delay + 0.25; // look at ERRATA from the HDL to get 0.25
	}

	double init_adc_delay_compensation = init_delay_inherent /adc_ltc1746_freq;
	unsigned int pulse2_int = (unsigned int)(round(pulse2_us * nmr_fsm_clkfreq));	// the number of 180 deg pulse in the multiplication of cpmg pulse period (discrete value, no continuous number supported)
	unsigned int delay2_int = (unsigned int) (samples_per_echo*(nmr_fsm_clkfreq/adc_ltc1746_freq)*10);	// the number of delay after 180 deg pulse. It is simply samples_per_echo multiplied by (nmr_fsm_clkfreq/adc_ltc1746_freq) factor, as the delay2_int is counted by nmr_fsm_clkfreq, not by adc_ltc1746_freq. It is also multiplied by a constant 2 as safety factor to make sure the ADC acquisition is inside FSMSTAT (refer to HDL) 'on' window.

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	create_measurement_folder("fid");
	// printf("Approximated measurement time : %.2f mins\n",( scan_spacing_us*(double)number_of_iteration)*1e-6/60);

	// print general measurement settings
	sprintf(pathname,"%s/acqu.par",foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr,"b1Freq = %4.3f\n", cpmg_freq);
	fprintf(fptr,"p180LengthGiven = %4.3f\n", pulse2_us);
	fprintf(fptr,"p180LengthRun = %4.3f\n", (double)pulse2_int/nmr_fsm_clkfreq);
	fprintf(fptr,"p180LengthCnt =  %d @ %4.3f MHz\n", pulse2_int,nmr_fsm_clkfreq);
	fprintf(fptr,"d180LengthRun = %4.3f\n", (double)delay2_int/nmr_fsm_clkfreq);
	fprintf(fptr,"d180LengthCnt = %d @ %4.3f MHz\n", delay2_int,nmr_fsm_clkfreq);
	//fprintf(fptr,"p90_dtcl = %4.3f\n", pulse1_dtcl);
	//fprintf(fptr,"p180_dtcl = %4.3f\n", pulse2_dtcl);
	fprintf(fptr,"ieTime = %lu\n", scan_spacing_us/1000);
	fprintf(fptr,"nrPnts = %d\n", samples_per_echo);
	fprintf(fptr,"echoShift = %4.3f --imprecise\n", init_adc_delay_compensation);
	fprintf(fptr,"nrIterations = %d\n", number_of_iteration);
	fprintf(fptr,"dummyEchoes = 0\n");
	fprintf(fptr,"adcFreq = %4.3f\n", adc_ltc1746_freq);
	fprintf(fptr,"dwellTime = %4.3f\n", 1/adc_ltc1746_freq);
	fclose(fptr);

	// print matlab script to analyze datas
	sprintf(pathname,"measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr,"fid_iterate([data_folder,'%s']);\n",foldername);
	fclose(fptr);

	// print matlab script to analyze datas
	sprintf(pathname,"current_folder.txt");
	fptr = fopen(pathname, "w");
	fprintf(fptr,"%s\n",foldername);
	fclose(fptr);



	int FILENAME_LENGTH = 100;
	char *name;
	name = (char*) malloc (FILENAME_LENGTH*sizeof(char));

	int iterate = 1;
	for (iterate=1; iterate<=number_of_iteration; iterate++) {
		// printf("\n*** RUN %d ***\n",iterate);

		snprintf(name, FILENAME_LENGTH,"dat_%03d",iterate);

		FID (
			cpmg_freq,						//cpmg_freq
			pulse2_us,						//pulse2_us
			pulse2_dtcl,					//pulse2_dtcl
			scan_spacing_us,				//scan_spacing_us
			samples_per_echo,				//samples_per_echo
			name,							//filename for data
			enable_message
		);

	}

	free(name);

}

void noise (double cpmg_freq, long unsigned scan_spacing_us, unsigned int samples_per_echo, char * filename, uint32_t enable_message) {
	double adc_ltc1746_freq = cpmg_freq*4;
	double nmr_fsm_clkfreq = cpmg_freq*16;
	uint8_t read_with_dma = 0; // else the program reads data directly from the fifo

	usleep(scan_spacing_us);

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	// local variables
	uint32_t fifo_mem_level; // the fill level of fifo memory

	unsigned int delay2_int = (unsigned int) (round(samples_per_echo*(nmr_fsm_clkfreq/adc_ltc1746_freq)*10));
	unsigned int fixed_init_adc_delay = 2; // set to the minimum delay values, which is 2 (limited by HDL structure).
	unsigned int fixed_echo_per_scan = 1; // it must be 1, otherwise the HDL will go to undefined state.
	double init_delay_inherent; // inherehent delay factor from the HDL structure. The minimum is 2.25 no matter how small the delay is set. Look ERRATA
	if (fixed_init_adc_delay <= 2) {
		init_delay_inherent = 2.25;
	}
	else { // if fixed_init_adc_delay is more than 2
		init_delay_inherent = (double) fixed_init_adc_delay + 0.25; // look at ERRATA from the HDL to get 0.25
	}

	alt_write_word( (h2p_pulse1_addr) , 0 );
	alt_write_word( (h2p_delay1_addr) , 0 );
	alt_write_word( (h2p_pulse2_addr) , 0 );
	alt_write_word( (h2p_delay2_addr) , delay2_int );
	alt_write_word( (h2p_init_adc_delay_addr) , fixed_init_adc_delay );
	alt_write_word( (h2p_echo_per_scan_addr) , fixed_echo_per_scan );
	alt_write_word( (h2p_adc_samples_per_echo_addr) , samples_per_echo );

	if (enable_message) {
		printf("CPMG Sequence Actual Parameter:\n");
		printf("\tDelay 2\t\t\t: %7.3f us (%d)\n", (double)delay2_int/nmr_fsm_clkfreq, delay2_int);
		printf("\tADC init delay\t: %7.3f us (%d) --imprecise\n", init_delay_inherent/adc_ltc1746_freq, fixed_init_adc_delay );
		printf("\tADC acq window\t: %7.3f us (%d)\n", ((double)samples_per_echo)/adc_ltc1746_freq, samples_per_echo);
	}
	if (fixed_init_adc_delay < 2) {
		printf("\tWARNING: Computed ADC_init_delay is less than 2, ADC_init_delay is force driven to 2 inside the HDL!");
	}

	// set pll for CPMG system
	Set_PLL (h2p_nmr_sys_pll_addr, 0, nmr_fsm_clkfreq, 0.5, DISABLE_MESSAGE);	// set pll frequency
	Reset_PLL (h2p_ctrl_out_addr, PLL_NMR_SYS_RST_ofst, ctrl_out);				// reset pll, changes the phase
	Set_DPS (h2p_nmr_sys_pll_addr, 0, 0, DISABLE_MESSAGE); 						// set pll phase to 0 (might not be needed)
	Wait_PLL_To_Lock (h2p_ctrl_in_addr, PLL_NMR_SYS_lock_ofst);					// wait for pll to lock

	// set a fix phase cycle state
	ctrl_out &= ~(0x01<<PHASE_CYCLING_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);

	// reset the selected ADC (the ADC reset was omitted)
	// alt_write_word( (h2p_ctrl_out_addr) , ctrl_out | (0x01<<ADC_LTC1746_RST_ofst) );
	// usleep(10);
	// alt_write_word( (h2p_ctrl_out_addr) , ctrl_out & ~(0x01<<ADC_LTC1746_RST_ofst) );
	// usleep(10);


	// reset ADC buffer
	ctrl_out |= (0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);
	ctrl_out &= ~(0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);

	// start fsm
	// it will reset the pll as well, so it's important to set the phase
	// the pll_rst_dly should be longer than the delay coming from changing the phase
	// otherwise, the fsm will start with wrong relationship between 4 pll output clocks (1/2 pi difference between clock)
	// alt_write_word( (h2p_nmr_pll_rst_dly_addr) , 1000000 );	// set the amount of delay for pll reset (with 50MHz system clock, every tick means 20ns) -> default: 100000
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out | (0x01<<FSM_START_ofst) );
	usleep(10);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out & ~(0x01<<FSM_START_ofst) );
	// shift the pll phase accordingly
	// Set_DPS (h2p_nmr_pll_addr, 0, 0, DISABLE_MESSAGE);
	// Set_DPS (h2p_nmr_pll_addr, 1, 90, DISABLE_MESSAGE);
	// Set_DPS (h2p_nmr_pll_addr, 2, 180, DISABLE_MESSAGE);
	// Set_DPS (h2p_nmr_pll_addr, 3, 270, DISABLE_MESSAGE);
	// usleep(scan_spacing_us);

	if (read_with_dma) { // if read with dma is intended
		datawrite_with_dma(samples_per_echo/2,enable_message); // divided by 2 to compensate 2 symbol per beat in the fifo interface
	}
	else { // if read from fifo is intended
		// wait until fsm stops
		while ( alt_read_word(h2p_ctrl_in_addr) & (0x01<<NMR_SEQ_run_ofst) );
		usleep(300);

		// PRINT # of DATAS in FIFO
		// fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG); // the fill level of FIFO memory
		// printf("num of data in fifo: %d\n",fifo_mem_level);

		// READING DATA FROM FIFO
		fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG); // the fill level of FIFO memory
		for (i=0; fifo_mem_level>0; i++) {			// FIFO is 32-bit, while 1-sample is only 16-bit. FIFO organize this automatically. So, fetch only amount_of_data shifted by 2 to get amount_of_data/2.
			rddata[i] = alt_read_word(h2p_adc_fifo_addr);

			fifo_mem_level--;
			if (fifo_mem_level == 0) {
				fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG);
			}
			//usleep(1);
		}
		usleep(100);

		if (i*2 == samples_per_echo) { // if the amount of data captured matched with the amount of data being ordered, then continue the process. if not, then don't process the datas (requesting empty data from the fifo will cause the FPGA to crash, so this one is to avoid that)
			// printf("number of captured data vs requested data : MATCHED\n");

			j=0;
			for(i=0; i < ( ((long)samples_per_echo)>>1 ); i++) {
				rddata_16[j++] = (rddata[i] & 0x3FFF);		// 14 significant bit
				rddata_16[j++] = ((rddata[i]>>16)&0x3FFF);	// 14 significant bit
			}

		}
		else { // if the amount of data captured didn't match the amount of data being ordered, then something's going on with the acquisition
			printf("[ERROR] number of data captured (%ld) and data ordered (%d): NOT MATCHED\nData are flushed!\nReconfigure the FPGA immediately\n", i*2, samples_per_echo);
		}
	}

	// write the raw data from adc to a file
	sprintf(pathname,"%s/%s",foldername,filename);	// put the data into the data folder
	fptr = fopen(pathname, "w");
	if (fptr == NULL) {
		printf("File does not exists \n");
	}
	for(i=0; i < ( ((long)samples_per_echo) ); i++) {
		fprintf(fptr, "%d\n", rddata_16[i]);
	}
	fclose(fptr);



}

void noise_iterate (
	double cpmg_freq,
	long unsigned scan_spacing_us,
	unsigned int samples_per_echo,
	unsigned int number_of_iteration,
	uint32_t enable_message
){
	double nmr_fsm_clkfreq = 16*cpmg_freq;
	double adc_ltc1746_freq = 4*cpmg_freq;

	unsigned int fixed_init_adc_delay = 2; // set to the minimum delay values, which is 2 (limited by HDL structure).
	double init_delay_inherent; // inherehent delay factor from the HDL structure. The minimum is 2.25 no matter how small the delay is set. Look ERRATA
	if (fixed_init_adc_delay <= 2) {
		init_delay_inherent = 2.25;
	}
	else { // if fixed_init_adc_delay is more than 2
		init_delay_inherent = (double) fixed_init_adc_delay + 0.25; // look at ERRATA from the HDL to get 0.25
	}

	double init_adc_delay_compensation = init_delay_inherent /adc_ltc1746_freq;
	unsigned int delay2_int = (unsigned int) (samples_per_echo*(nmr_fsm_clkfreq/adc_ltc1746_freq)*10);	// the number of delay after 180 deg pulse. It is simply samples_per_echo multiplied by (nmr_fsm_clkfreq/adc_ltc1746_freq) factor, as the delay2_int is counted by nmr_fsm_clkfreq, not by adc_ltc1746_freq. It is also multiplied by a constant 2 as safety factor to make sure the ADC acquisition is inside FSMSTAT (refer to HDL) 'on' window.

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	create_measurement_folder("noise");
	// printf("Approximated measurement time : %.2f mins\n",( scan_spacing_us*(double)number_of_iteration)*1e-6/60);

	// print general measurement settings
	sprintf(pathname,"%s/acqu.par",foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr,"b1Freq = %4.3f\n", cpmg_freq);
	fprintf(fptr,"d180LengthRun = %4.3f\n", (double)delay2_int/nmr_fsm_clkfreq);
	fprintf(fptr,"d180LengthCnt = %d @ %4.3f MHz\n", delay2_int,nmr_fsm_clkfreq);
	fprintf(fptr,"ieTime = %lu\n", scan_spacing_us/1000);
	fprintf(fptr,"nrPnts = %d\n", samples_per_echo);
	fprintf(fptr,"echoShift = %4.3f --imprecise\n", init_adc_delay_compensation);
	fprintf(fptr,"nrIterations = %d\n", number_of_iteration);
	fprintf(fptr,"dummyEchoes = 0\n");
	fprintf(fptr,"adcFreq = %4.3f\n", adc_ltc1746_freq);
	fprintf(fptr,"dwellTime = %4.3f\n", 1/adc_ltc1746_freq);
	fclose(fptr);

	// print matlab script to analyze datas
	sprintf(pathname,"measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr,"fid_iterate([data_folder,'%s']);\n",foldername);
	fclose(fptr);

	// print matlab script to analyze datas
	sprintf(pathname,"current_folder.txt");
	fptr = fopen(pathname, "w");
	fprintf(fptr,"%s\n",foldername);
	fclose(fptr);



	int FILENAME_LENGTH = 100;
	char *name;
	name = (char*) malloc (FILENAME_LENGTH*sizeof(char));

	int iterate = 1;
	for (iterate=1; iterate<=number_of_iteration; iterate++) {
		// printf("\n*** RUN %d ***\n",iterate);

		snprintf(name, FILENAME_LENGTH,"dat_%03d",iterate);

		noise (
			cpmg_freq,						//cpmg_freq
			scan_spacing_us,				//scan_spacing_us
			samples_per_echo,				//samples_per_echo
			name,							//filename for data
			enable_message
		);

	}

	free(name);

}

void tune_board (double freq) {
	//double c_idx;
	//c_idx = (freq-mtch_ntwrk_freq_sta)/mtch_ntwrk_freq_spa;	// find index for C
	// round c_idx (integer conversion always floors the number (even for 0.99999), which causes trouble, so compensate for it)
	//if ( c_idx - (double)((uint16_t)c_idx) > 0.5 ) {
	//	c_idx += 0.5;
	//}
	// write_i2c_relay_cnt(cpar_tbl[(uint16_t)c_idx],cser_tbl[(uint16_t)c_idx]);	// find c values from table

	double vvarac_idx;
	vvarac_idx = (freq-vvarac_freq_sta)/vvarac_freq_spa;	// find index for the vvarac
	// round vvarac (integer conversion always floors the number (even for 0.99999), which causes trouble, so compensate for it)
	if ( vvarac_idx - (double)((uint16_t)vvarac_idx) > 0.5 ) {
		vvarac_idx += 0.5;
	}
	write_vvarac(vvarac_tbl[(uint16_t)vvarac_idx]);								// find vvarac from the table

	write_vbias(-1.25);															// minimum S11 value also max gain (32dB) : -1.25V
	//usleep(1000000);															// wait for the v_varac & v_bias to settle down
	usleep(10000);															// wait for the v_varac & v_bias to settle down
}

void wobble_function (double startfreq, double stopfreq, double spacfreq, double sampfreq, unsigned int wobb_samples) {

	// buffer in the fpga needs to be an even number, therefore the number of samples should be even as well
	if (wobb_samples % 2) {
		wobb_samples++;
	}

	create_measurement_folder("nmr_wobb");

	// print matlab script to analyze datas
	sprintf(pathname,"current_folder.txt");
	fptr = fopen(pathname, "w");
	fprintf(fptr,"%s\n",foldername);
	fclose(fptr);

	// print matlab script to analyze datas
	sprintf(pathname,"measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr,"wobble_plot([data_folder,'%s']);\n",foldername);
	fclose(fptr);

	// print the NMR acquired settings
	//sprintf(pathname,"%s/matlab_settings.txt",foldername);	// put the data into the data folder
	//fptr = fopen(pathname, "a");
	//fprintf(fptr,"%f\n", startfreq);
	//fprintf(fptr,"%f\n", stopfreq);
	//fprintf(fptr,"%f\n", spacfreq);
	//fprintf(fptr,"%d\n", wobb_samples);
	//fclose(fptr);

	//sprintf(pathname,"%s/readable_settings.txt",foldername);	// put the data into the data folder
	//fptr = fopen(pathname, "a");
	//fprintf(fptr,"Start frequency: %f\n", startfreq);
	//fprintf(fptr,"Stop frequency: %f\n", stopfreq);
	//fprintf(fptr,"Spacing frequency: %f\n", spacfreq);
	//fprintf(fptr,"Number of samples: %d\n", wobb_samples);
	//fclose(fptr);
	//

	// print general measurement settings
	sprintf(pathname,"%s/acqu.par",foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr,"freqSta = %4.3f\n", startfreq);
	fprintf(fptr,"freqSto = %4.3f\n", stopfreq);
	fprintf(fptr,"freqSpa = %4.3f\n", spacfreq);
	fprintf(fptr,"nSamples = %d\n", wobb_samples);
	fprintf(fptr,"freqSamp = %4.3f\n", sampfreq);
	fclose(fptr);


	char * wobbname;
	double wobbfreq = 0;
	wobbname = (char*) malloc (100*sizeof(char));
	stopfreq += (spacfreq/2); // the (spacfreq/2) factor is to compensate double comparison error. double cannot be compared with '==' operator !
	for (wobbfreq = startfreq; wobbfreq < stopfreq; wobbfreq += spacfreq) {
		snprintf(wobbname, 100,"wobbdata_%4.3f",wobbfreq);
		tx_sampling(wobbfreq, sampfreq, wobb_samples, wobbname);
		usleep(100);		// this delay is necessary. If it's not here, the system will not crash but the i2c will stop working (?), and the reading length is incorrect
	}

}

void noise_meas (unsigned int signal_path, unsigned int num_of_samples) {

	create_measurement_folder("noise");

	// print matlab script to analyze datas
	sprintf(pathname,"measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr,"noise_plot([data_folder,'%s']);\n",foldername);
	fclose(fptr);

	// print the NMR acquired settings
	sprintf(pathname,"%s/matlab_settings.txt",foldername);	// put the data into the data folder
	fptr = fopen(pathname, "a");
	fprintf(fptr,"%d\n", num_of_samples);
	fclose(fptr);

	sprintf(pathname,"%s/readable_settings.txt",foldername);	// put the data into the data folder
	fptr = fopen(pathname, "a");
	fprintf(fptr,"Number of samples: %d\n", num_of_samples);
	fclose(fptr);

	char * noisename; noisename = (char*) malloc (100*sizeof(char));
	snprintf(noisename, 100,"noisedata.o");
	noise_sampling (signal_path, num_of_samples, noisename);
}

void test_leds_and_switches () {
    // initialize gpio for the hps
    setup_hps_gpio();

    // switch on first led
    setup_fpga_leds();

    while (true) {
        handle_hps_led();
        // handle_fpga_leds(); LED is omitted in new design
        printf("%d\n",alt_read_word(fpga_switches));
        usleep(ALT_MICROSECS_IN_A_SEC / 10);
    }
}

void init_default_system_param() {

	// initialize control lines to default value
	ctrl_out = CNT_OUT_default;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(100);

	// initialize i2c default
	// ctrl_i2c = CNT_I2C_default;

	// set reconfig configuration for pll's
	Reconfig_Mode(h2p_nmr_sys_pll_addr,1); // polling mode for main pll
	Reconfig_Mode(h2p_analyzer_pll_addr,1); // polling mode for main pll

	//write_i2c_cnt (ENABLE, AMP_HP_LT1210_EN_msk, DISABLE_MESSAGE); // enable high-power transmitter
	//write_i2c_cnt (ENABLE, PSU_5V_ADC_EN_msk|PSU_5V_ANA_P_EN_msk|PSU_5V_ANA_N_EN_msk|PSU_5V_TX_N_EN_msk|PSU_15V_TX_P_EN_msk|PSU_15V_TX_N_EN_msk, DISABLE_MESSAGE);
	//write_i2c_cnt (ENABLE, PAMP_IN_SEL_RX_msk, DISABLE_MESSAGE);

	ctrl_out |= NMR_CLK_GATE_AVLN;						// enable RF gate path, disable PLL_analyzer path
	// ctrl_out &= ~NMR_CLK_GATE_AVLN;					// disable RF gate path, enable PLL analyzer path
	// alt_write_word(h2p_ctrl_out_addr, ctrl_out);		// write down the control
	// usleep(100);

	/* manual selection of cshunt and cseries (put breakpoint before running)
	int cshuntval = 120;
	int cseriesval = 215;
	while (1) {
		write_i2c_relay_cnt(cshuntval,cseriesval, DISABLE_MESSAGE);
		printf("put a breakpoint here!!!!");
	}
	*/
	// write_i2c_relay_cnt(19, 66, DISABLE_MESSAGE);

	// init_dac_ad5722r();		// power up the dac and init its operation
	/* manual selection of vbias and vvarac (put breakpoint before running)
	double vbias_test, vvarac_test;
	vbias_test = -3.174;
	vvarac_test = -1.77;
	while (1) {
		write_vbias(vbias_test);
		write_vvarac(vvarac_test);
		printf("put a breakpoint here!!!!");
	}
	*/
	// write_vbias(-3.35);		// default number for vbias is -3.35V (reflection at -20dB at 4.3MHz)
	// write_vvarac(-1.2);	// the default number for v_varactor is -1.2V (gain of 23dB at 4.3 MHz)


	// tune_board(4.3); 				// tune board frequency: input is frequency in MHz

	// reset controller (CAUTION: this will fix the problem of crashed controller temporarily but won't really eliminate the problem: fix the state machine instead)
	// the issue is the logic in ADC_WINGEN, where TOKEN is implemented to prevent retriggering. But also at the same time, if ADC_CLOCK is generated after ACQ_WND rises,
	// the TOKEN is not resetted to 0, which will prevent the state machine from running. It is fixed by having reset button implemented to reset the TOKEN to 0 just
	// before any acquisition.
	ctrl_out |= NMR_CNT_RESET;
	alt_write_word(h2p_ctrl_out_addr, ctrl_out);	// write down the control
	usleep(10);
	ctrl_out &= ~(NMR_CNT_RESET);
	alt_write_word(h2p_ctrl_out_addr, ctrl_out);	// write down the control

	// usleep(500000); // this delay is extremely necessary! or data will be bad in first cpmg scan. also used to wait for vvarac and vbias to settle down

}

void close_system () {
	write_i2c_relay_cnt(0,0, DISABLE_MESSAGE); //  disable all relays

	write_i2c_cnt (DISABLE, PAMP_IN_SEL_TEST_msk|PAMP_IN_SEL_RX_msk|PSU_15V_TX_P_EN_msk|PSU_15V_TX_N_EN_msk|AMP_HP_LT1210_EN_msk|PSU_5V_ANA_P_EN_msk|PSU_5V_ANA_N_EN_msk | PSU_5V_TX_N_EN_msk, DISABLE_MESSAGE);

}

// MAIN SYSTEM (ENABLE ONE AT A TIME)

/* Init default system param (rename the output to "init")
int main() {
    // printf("Init system\n");

    open_physical_memory_device();
    mmap_peripherals();
    init_default_system_param();

    alt_write_word( h2p_rx_delay_addr , 20 );

    munmap_peripherals();
    close_physical_memory_device();
    return 0;
}
*/

/* SPI for vbias and vvarac (rename the output to "preamp_tuning")
int main(int argc, char * argv[]) {
    // printf("Preamp tuning with SPI\n");

    // input parameters
    double vbias = atof(argv[1]);
    double vvarac = atof(argv[2]);

    open_physical_memory_device();
    mmap_peripherals();

	init_dac_ad5722r();			// power up the dac and init its operation
	write_vbias(vbias);			// default number for vbias is -3.35V (reflection at -20dB at 4.3MHz)
	write_vvarac(vvarac);		// the default number for v_varactor is -1.2V (gain of 23dB at 4.3 MHz)

    munmap_peripherals();
    close_physical_memory_device();
    return 0;
}
*/

/* I2C matching network control (rename the output to "i2c_mtch_ntwrk")
int main(int argc, char * argv[]) {
    // printf("Matching network control with I2C\n");

    // input parameters
    unsigned int cshunt = atoi(argv[1]);
    unsigned int cseries = atoi(argv[2]);

    open_physical_memory_device();
    mmap_peripherals();

    write_i2c_relay_cnt(cshunt, cseries, DISABLE_MESSAGE);

    munmap_peripherals();
    close_physical_memory_device();
    return 0;
}
*/

/* I2C general control (rename the output to "i2c_gnrl")
int main(int argc, char * argv[]) {
    // printf("General control with I2C\n");

    // input parameters
    unsigned int gnrl_cnt = atoi(argv[1]);

    open_physical_memory_device();
    mmap_peripherals();

    write_i2c_cnt (ENABLE, (gnrl_cnt & 0xFFFF), DISABLE_MESSAGE); // enable the toggled index

    munmap_peripherals();
    close_physical_memory_device();
    return 0;
}
*/

/* Wobble (rename the output to "wobble")
int main(int argc, char * argv[]) {
    printf("Wobble measurement starts\n");

    // input parameters
    double startfreq = atof(argv[1]);
    double stopfreq = atof(argv[2]);
	double spacfreq = atof(argv[3]);
	double sampfreq = atof(argv[4]);

    open_physical_memory_device();
    mmap_peripherals();
    init_default_system_param();

    //WOBBLE
	unsigned int wobb_samples 	= (unsigned int)(lround(sampfreq/spacfreq));	// the number of ADC samples taken
	wobble_function (
			startfreq,
			stopfreq,
			spacfreq,
			sampfreq,
			wobb_samples
	);

	// close_system();
    munmap_peripherals();
    close_physical_memory_device();
    return 0;
}
*/

/* CPMG Iterate (rename the output to "cpmg_iterate"). data_nowrite in CPMG_Sequence should 0
// if CPMG Sequence is used without writing to text file, rename the output to "cpmg_iterate_direct". Set this setting in CPMG_Sequence: data_nowrite = 1
int main(int argc, char * argv[]) {
    // printf("NMR system start\n");

    // input parameters
    double cpmg_freq = atof(argv[1]);
    double pulse1_us = atof(argv[2]);
	double pulse2_us = atof(argv[3]);
	double pulse1_dtcl = atof(argv[4]);
	double pulse2_dtcl = atof(argv[5]);
	double echo_spacing_us = atof(argv[6]);
	long unsigned scan_spacing_us = atoi(argv[7]);
	unsigned int samples_per_echo = atoi(argv[8]);
	unsigned int echoes_per_scan = atoi(argv[9]);
	double init_adc_delay_compensation = atof(argv[10]);
	unsigned int number_of_iteration = atoi(argv[11]);
	uint32_t ph_cycl_en = atoi(argv[12]);
	unsigned int pulse180_t1_int = atoi(argv[13]);
	unsigned int delay180_t1_int = atoi(argv[14]);

	rddata_16 = (unsigned int*)malloc(samples_per_echo*echoes_per_scan*sizeof(unsigned int)); 	//added malloc to this routine only - other routines will need to be updated when required
	rddata = (unsigned int *)malloc(samples_per_echo*echoes_per_scan/2*sizeof(unsigned int));	// petrillo 2Feb2019

    open_physical_memory_device();
    mmap_peripherals();
    // init_default_system_param();

    // write t1-IR measurement parameters (put both to 0 if IR is not desired)
    alt_write_word( h2p_t1_pulse , pulse180_t1_int );
    alt_write_word( h2p_t1_delay , delay180_t1_int );

    // printf("cpmg_freq = %0.3f\n",cpmg_freq);
	CPMG_iterate (
		cpmg_freq,
		pulse1_us,
		pulse2_us,
		pulse1_dtcl,
		pulse2_dtcl,
		echo_spacing_us,
		scan_spacing_us,
		samples_per_echo,
		echoes_per_scan,
		init_adc_delay_compensation,
		number_of_iteration,
		ph_cycl_en
	);

	// close_system();
    munmap_peripherals();
    close_physical_memory_device();

    free(rddata_16);	//freeing up allocated memory requried for multiple calls from host
    free(rddata);		//petrillo 2Feb2019

    return 0;
}
*/

/* FID Iterate (rename the output to "fid")
int main(int argc, char * argv[]) {

    // input parameters
    double cpmg_freq = atof(argv[1]);
	double pulse2_us = atof(argv[2]);
	double pulse2_dtcl = atof(argv[3]);
	long unsigned scan_spacing_us = atoi(argv[4]);
	unsigned int samples_per_echo = atoi(argv[5]);
	unsigned int number_of_iteration = atoi(argv[6]);

    open_physical_memory_device();
    mmap_peripherals();
    //init_default_system_param();

    FID_iterate (
    	cpmg_freq,
    	pulse2_us,
    	pulse2_dtcl,
    	scan_spacing_us,
    	samples_per_echo,
    	number_of_iteration,
    	ENABLE_MESSAGE
	);

	// close_system();
    munmap_peripherals();
    close_physical_memory_device();
    return 0;
}
*/

/* noise Iterate (rename the output to "noise")
int main(int argc, char * argv[]) {

    // input parameters
    double samp_freq = atof(argv[1]);
	long unsigned scan_spacing_us = atoi(argv[2]);
	unsigned int samples_per_echo = atoi(argv[3]);
	unsigned int number_of_iteration = atoi(argv[4]);

    open_physical_memory_device();
    mmap_peripherals();
    init_default_system_param();

    double cpmg_freq = samp_freq/4; // the building block that's used is still nmr cpmg, so the sampling frequency is fixed to 4*cpmg_frequency
    noise_iterate (
    	cpmg_freq,
    	scan_spacing_us,
    	samples_per_echo,
    	number_of_iteration,
    	ENABLE_MESSAGE
	);

	// close_system();
    munmap_peripherals();
    close_physical_memory_device();
    return 0;
}
*/

// hallsensor test
int main() {

    open_physical_memory_device();
    mmap_peripherals();
    init_default_system_param();

    alt_write_word( h2p_rx_delay_addr , 20 );

    // DAC write
    init_dac_ad5722r();			// power up the dac and init its operation
	write_vbias(2.2);			// default number for vbias is -3.35V (reflection at -20dB at 4.3MHz)
	write_vvarac(1.8);		// the default number for v_varactor is -1.2V (gain of 23dB at 4.3 MHz)

	// i2c write
	// write_i2c_relay_cnt(100,100, DISABLE_MESSAGE); //  disable all relays

	write_i2c_cnt (ENABLE, PAMP_IN_SEL_TEST_msk|PAMP_IN_SEL_RX_msk|PSU_15V_TX_P_EN_msk|PSU_15V_TX_N_EN_msk|AMP_HP_LT1210_EN_msk|PSU_5V_ANA_P_EN_msk|PSU_5V_ANA_N_EN_msk | PSU_5V_TX_N_EN_msk, DISABLE_MESSAGE);

	/* input parameters
	double cpmg_freq = 4.0;
	double pulse1_us = 10;
	double pulse2_us = pulse1_us*1.6;
	double pulse1_dtcl = 0.5;
	double pulse2_dtcl = 0.5;
	double echo_spacing_us = 300;
	long unsigned scan_spacing_us = 200000;
	unsigned int samples_per_echo = 1024;
	unsigned int echoes_per_scan = 128;
	double init_adc_delay_compensation = 6;
	unsigned int number_of_iteration = 1;
	uint32_t ph_cycl_en = 1;
	unsigned int pulse180_t1_int = 0;
	unsigned int delay180_t1_int = 0;

	// write t1-IR measurement parameters (put both to 0 if IR is not desired)
	alt_write_word( h2p_t1_pulse , pulse180_t1_int );
	alt_write_word( h2p_t1_delay , delay180_t1_int );

	printf("cpmg_freq = %0.3f\n",cpmg_freq);
	CPMG_iterate (
		cpmg_freq,
		pulse1_us,
		pulse2_us,
		pulse1_dtcl,
		pulse2_dtcl,
		echo_spacing_us,
		scan_spacing_us,
		samples_per_echo,
		echoes_per_scan,
		init_adc_delay_compensation,
		number_of_iteration,
		ph_cycl_en
	);
	*/

	write_i2c_cnt (DISABLE, PSU_15V_TX_P_EN_msk|PSU_15V_TX_N_EN_msk, DISABLE_MESSAGE);

	rd_hall_sens_stat(); // read hall sensor status

	// loop for reading hall sensor data and also temperature
	int i = 0;
	for (i = 0; i<1000; i++) {
		rd_hall_sens_data();
		usleep(500000);
	}

	close_system();
    munmap_peripherals();
    close_physical_memory_device();
    return 0;
}
//

/* standalone main
int main() {

    open_physical_memory_device();
    mmap_peripherals();
    init_default_system_param();

    alt_write_word( h2p_rx_delay_addr , 20 );

    // DAC write
    init_dac_ad5722r();			// power up the dac and init its operation
	write_vbias(2.2);			// default number for vbias is -3.35V (reflection at -20dB at 4.3MHz)
	write_vvarac(1.8);		// the default number for v_varactor is -1.2V (gain of 23dB at 4.3 MHz)

	// i2c write
	write_i2c_relay_cnt(100,100, DISABLE_MESSAGE); //  disable all relays

	write_i2c_cnt (ENABLE, PAMP_IN_SEL_TEST_msk|PAMP_IN_SEL_RX_msk|PSU_15V_TX_P_EN_msk|PSU_15V_TX_N_EN_msk|AMP_HP_LT1210_EN_msk|PSU_5V_ANA_P_EN_msk|PSU_5V_ANA_N_EN_msk | PSU_5V_TX_N_EN_msk, DISABLE_MESSAGE);

	// input parameters
	double cpmg_freq = 4.0;
	double pulse1_us = 10;
	double pulse2_us = pulse1_us*1.6;
	double pulse1_dtcl = 0.5;
	double pulse2_dtcl = 0.5;
	double echo_spacing_us = 300;
	long unsigned scan_spacing_us = 200000;
	unsigned int samples_per_echo = 1024;
	unsigned int echoes_per_scan = 128;
	double init_adc_delay_compensation = 6;
	unsigned int number_of_iteration = 1;
	uint32_t ph_cycl_en = 1;
	unsigned int pulse180_t1_int = 0;
	unsigned int delay180_t1_int = 0;

	// write t1-IR measurement parameters (put both to 0 if IR is not desired)
	alt_write_word( h2p_t1_pulse , pulse180_t1_int );
	alt_write_word( h2p_t1_delay , delay180_t1_int );

	printf("cpmg_freq = %0.3f\n",cpmg_freq);
	CPMG_iterate (
		cpmg_freq,
		pulse1_us,
		pulse2_us,
		pulse1_dtcl,
		pulse2_dtcl,
		echo_spacing_us,
		scan_spacing_us,
		samples_per_echo,
		echoes_per_scan,
		init_adc_delay_compensation,
		number_of_iteration,
		ph_cycl_en
	);


	write_i2c_cnt (DISABLE, PSU_15V_TX_P_EN_msk|PSU_15V_TX_N_EN_msk, DISABLE_MESSAGE);


	close_system();
    munmap_peripherals();
    close_physical_memory_device();
    return 0;
}
*/
