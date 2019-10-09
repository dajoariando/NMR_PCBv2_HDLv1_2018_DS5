/*
 * pwm_pca9685_driver.h
 *
 *  Created on: Oct 4, 2019
 *      Author: chenc
 */

#ifndef FUNCTIONS_PWM_PCA9685_DRIVER_H_
#define FUNCTIONS_PWM_PCA9685_DRIVER_H_

#define MODE1			(0x00)
	#define Restart_OFST	(7)
		#define Restart_MSK		(0b1)
#define MODE2			(0x01)
#define SUBADR1			(0x02)
#define SUBADR2			(0x03)
#define SUBADR3			(0x04)
#define ALLCALLADR		(0x05)


#define LED0_ON_L		(0x06) //led0
#define LED0_ON_H		(0x07)
#define LED0_OFF_L		(0x08)
#define LED0_OFF_H		(0x09)
#define LED1_ON_L		(0x0A) //led1
#define LED1_ON_H		(0x0B)
#define LED1_OFF_L		(0x0C)
#define LED1_OFF_H		(0x0D)
#define LED2_ON_L		(0x0E) //led2
#define LED2_ON_H		(0x0F)
#define LED2_OFF_L		(0x10)
#define LED2_OFF_H		(0x11)
#define LED3_ON_L		(0x12) //led3
#define LED3_ON_H		(0x13)
#define LED3_OFF_L		(0x14)
#define LED3_OFF_H		(0x15)
#define LED4_ON_L		(0x16) //led4
#define LED4_ON_H		(0x17)
#define LED4_OFF_L		(0x18)
#define LED4_OFF_H		(0x19)
#define LED5_ON_L		(0x1A) //led5
#define LED5_ON_H		(0x1B)
#define LED5_OFF_L		(0x1C)
#define LED5_OFF_H		(0x1D)
#define LED6_ON_L		(0x1E) //led6
#define LED6_ON_H		(0x1F)
#define LED6_OFF_L		(0x20)
#define LED6_OFF_H		(0x21)
#define LED7_ON_L		(0x22) //led7
#define LED7_ON_H		(0x23)
#define LED7_OFF_L		(0x24)
#define LED7_OFF_H		(0x25)
#define LED8_ON_L		(0x26) //led8
#define LED8_ON_H		(0x27)
#define LED8_OFF_L		(0x28)
#define LED8_OFF_H		(0x29)
#define LED9_ON_L		(0x2A) //led9
#define LED9_ON_H		(0x2B)
#define LED9_OFF_L		(0x2C)
#define LED9_OFF_H		(0x2D)
#define LED10_ON_L		(0x2E)//led10
#define LED10_ON_H		(0x2F)
#define LED10_OFF_L		(0x30)
#define LED10_OFF_H		(0x31)
#define LED11_ON_L		(0x32) //led11
#define LED11_ON_H		(0x33)
#define LED11_OFF_L		(0x34)
#define LED11_OFF_H		(0x35)
#define LED12_ON_L		(0x36)//led12
#define LED12_ON_H		(0x37)
#define LED12_OFF_L		(0x38)
#define LED12_OFF_H		(0x39)
#define LED13_ON_L		(0x3A) //led13
#define LED13_ON_H		(0x3B)
#define LED13_OFF_L		(0x3C)
#define LED13_OFF_H		(0x3D)
#define LED14_ON_L		(0x3E)//led14
#define LED14_ON_H		(0x3F)
#define LED14_OFF_L		(0x40)
#define LED14_OFF_H		(0x41)
#define LED15_ON_L		(0x42) //led15
#define LED15_ON_H		(0x43)
#define LED15_OFF_L		(0x44)
#define LED15_OFF_H		(0x45)

#define ALL_LED_ON_L	(0xFA) //led all
#define ALL_LED_ON_H	(0xFB)
#define ALL_LED_OFF_L	(0xFC)
#define ALL_LED_OFF_H	(0xFD)

#define PRE_SCALE		(0xFE)
#define TestMode		(0xFF)

#endif /* FUNCTIONS_PWM_PCA9685_DRIVER_H_ */
