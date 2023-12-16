 /*FileName:         GPIO.h                                                                                            */
 /* Dependencies:    None                                                                                              */
 /* Processor:       ESP32                                                                                             */
 /* Board:           ESP32-PANTHER48                                                                                   */
 /* Program version: 5.1                                                                                           */
 /* Company:         Espressif Systems                                                                                 */
 /* Description:     Incluye librerías, define ciertas macros,Prototipos de funciones.                                 */
 /* Authors:         Guadalupe Méndez Ontiveros, Manuel Francisco Zubiate Loya, Alan Alexis Rodriguez Hernandez.                                          */
 /*Updated:          12/2023                                                                                            */


#ifndef _GPIO_H_
#define _GPIO_H_
#include<stdio.h>
#include<stdlib.h>
#include "BSP.h"

#define DELAY_MS 500


#define HWREG32(x)        (*((volatile uint32_t *)(x)))

#define GPIO_IN_REG        HWREG32(0x3FF4403C)
#define GPIO_IN1_REG       HWREG32(0x3FF44040)

#define GPIO_ENABLE_REG    HWREG32(0x3FF44020)
#define GPIO_OUT_W1TS_REG  HWREG32(0x3FF44008)
#define GPIO_OUT_W1TC_REG  HWREG32(0x3FF4400C)

#define GPIO_ENABLE1_REG   HWREG32(0x3FF4402C)
#define GPIO_OUT1_W1TS_REG HWREG32(0x3FF44014)
#define GPIO_OUT1_W1TC_REG HWREG32(0x3FF44018)


 #define RTC_IO_TOUCH_PAD0_REG 0x3FF48494
 #define RTC_IO_TOUCH_PAD1_REG 0x3FF48498
 #define RTC_IO_TOUCH_PAD2_REG 0x3FF4849C
 #define RTC_IO_TOUCH_PAD3_REG 0x3FF484A0
 #define RTC_IO_TOUCH_PAD4_REG 0x3FF484A4
 #define RTC_IO_TOUCH_PAD5_REG 0x3FF484A8
 #define RTC_IO_TOUCH_PAD6_REG 0x3FF484AC
 #define RTC_IO_TOUCH_PAD7_REG 0x3FF484B0
 #define RTC_IO_TOUCH_PAD8_REG 0x3FF484B4
 #define RTC_IO_TOUCH_PAD9_REG 0x3FF484B8

 enum RTC_REG{
	RTCIO_TOUCH_PAD0=1, RTCIO_TOUCH_PAD1,RTCIO_TOUCH_PAD2,
	RTCIO_TOUCH_PAD3, RTCIO_TOUCH_PAD4, RTCIO_TOUCH_PAD5,
	RTCIO_TOUCH_PAD6, RTCIO_TOUCH_PAD7, RTCIO_TOUCH_PAD8,
	RTCIO_TOUCH_PAD9,


};

#define FUN_IE      9 
#define PULL_UP     8
#define PULL_DOWM   7

#define ALTO     1
#define BAJO     0

typedef enum state{
    GPIO_HIGH = ALTO,
    GPIO_LOW  = BAJO,

}gpio_state;

typedef volatile struct {
	union {
		struct {
			uint32_t mcu_oe:     1;
			uint32_t slp_sel:    1;
			uint32_t mcu_wpd:    1;
			uint32_t mcu_wpu:    1;
			uint32_t mcu_ie:     1;
			uint32_t mcu_drb:    2;
			uint32_t func_wpd:   1;
			uint32_t func_wpu:   1;
			uint32_t func_ie:    1;
			uint32_t func_drv:   2;
			uint32_t mcu_sel:    3;
			uint32_t reserved15: 17;
		};
		uint32_t val;
	};
} io_mux_reg_t;

typedef volatile struct {
	union {
		struct {
			uint32_t reserved1:  12;
			uint32_t to_gpio:    1;
			uint32_t fun_ie:     1;
			uint32_t slp_oe:     1;
			uint32_t slp_ie:     1;
			uint32_t slp_sel:    1;
			uint32_t func_sel:   2;
			uint32_t mux_sel:    1;
			uint32_t xpd:        1;
			uint32_t tie_opt:    1;
			uint32_t start:      1;
			uint32_t dac:        3;
            uint32_t reserved2:  1;
            uint32_t rue:        1;
            uint32_t rde:        1;
            uint32_t drv:        2; 
            uint32_t hold:       1;
		};
		uint32_t val;
	};
} rtcio_touch_reg_t;


extern volatile uint32_t GPIO_PINX_MUX_REG[];
extern volatile uint32_t dir_rtcio_touch_pad[];

extern void GpioDigitalWrite(uint32_t gpio,gpio_state state);
extern int GpioDigitalRead(uint32_t gpio);
extern void GpioPullUpEnable(uint32_t gpio);
extern void GpioPullDownEnable(uint32_t gpio);
extern void GpioModeOutput(uint32_t gpio);
extern void GpioModeInput(uint32_t gpio);
extern int identify_pin_rtc(uint32_t gpio);




#endif