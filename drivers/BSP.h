 /*FileName:         BSP.c                                                                                            */
 /* Dependencies:    None                                                                                              */
 /* Processor:       ESP32                                                                                             */
 /* Board:           ESP32-PANTHER48                                                                                   */
 /* Program version: 5.1                                                                                           */
 /* Company:         Espressif Systems                                                                                 */
 /* Description:     Definicion de unas macros y enumeraciones.                                 */
 /* Authors:         Guadalupe Méndez Ontiveros, Manuel Francisco Zubiate Loya, Alan Alexis Rodriguez Hernandez.                                          */
 /*Updated:          12/2023                                                                                            */


#ifndef _BSP_H_
#define _BSP_H_
#include "GPIO.h"

enum LEDS{
    LED1=17, LED2=16, LED3=4, LED4=2,
    LED5=14, LED6=12, LED7=13, LED8=5,
    LEDR=14, LEDG=13, LEDB=12,

};
enum GPIOS{
    GPIO0, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO7, GPIO8, GPIO9,
    GPIO10, GPIO11, GPIO12, GPIO13, GPIO14, GPIO15, GPIO16, GPIO17, GPIO18, GPIO19,
    GPIO20, GPIO21, GPIO22, GPIO23, GPIO24, GPIO25, GPIO26, GPIO27, GPIO28, GPIO29,
    GPIO30, GPIO31, GPIO32, GPIO33, GPIO34, GPIO35, GPIO36, GPIO37, GPIO38, GPIO39,
};

enum BTNS{
    BTN1=18,
    BTN2,
};

#define IO_MUX_GPIO0_REG  0x3FF49044
#define IO_MUX_GPIO1_REG  0x3FF49088
#define IO_MUX_GPIO2_REG  0x3FF49040
#define IO_MUX_GPIO3_REG  0x3FF49084
#define IO_MUX_GPIO4_REG  0x3FF49048
#define IO_MUX_GPIO5_REG  0x3FF4906C
#define IO_MUX_GPIO6_REG  0x3FF49060
#define IO_MUX_GPIO7_REG  0x3FF49064
#define IO_MUX_GPIO8_REG  0x3FF49068
#define IO_MUX_GPIO9_REG  0x3FF49054
#define IO_MUX_GPIO10_REG 0x3FF49058   
#define IO_MUX_GPIO11_REG 0x3FF4905C 
#define IO_MUX_GPIO12_REG 0x3FF49034   
#define IO_MUX_GPIO13_REG 0x3FF49038
#define IO_MUX_GPIO14_REG 0x3FF49030
#define IO_MUX_GPIO15_REG 0x3FF4903C
#define IO_MUX_GPIO16_REG 0x3FF4904C
#define IO_MUX_GPIO17_REG 0x3FF49050
#define IO_MUX_GPIO18_REG 0x3FF49070
#define IO_MUX_GPIO19_REG 0x3FF49074
#define IO_MUX_GPIO20_REG 0x3FF49078
#define IO_MUX_GPIO21_REG 0x3FF4907C
#define IO_MUX_GPIO22_REG 0x3FF49080
#define IO_MUX_GPIO23_REG 0x3FF4908C
#define IO_MUX_GPIO24_REG 0
#define IO_MUX_GPIO25_REG 0x3FF49024
#define IO_MUX_GPIO26_REG 0x3FF49028
#define IO_MUX_GPIO27_REG 0x3FF4902C
#define IO_MUX_GPIO28_REG 0
#define IO_MUX_GPIO29_REG 0
#define IO_MUX_GPIO30_REG 0
#define IO_MUX_GPIO31_REG 0
#define IO_MUX_GPIO32_REG 0x3FF4901C
#define IO_MUX_GPIO33_REG 0x3FF49020
#define IO_MUX_GPIO34_REG 0x3FF49014
#define IO_MUX_GPIO35_REG 0x3FF49018
#define IO_MUX_GPIO36_REG 0x3FF49004
#define IO_MUX_GPIO37_REG 0x3FF49008
#define IO_MUX_GPIO38_REG 0x3FF4900C
#define IO_MUX_GPIO39_REG 0x3FF49010












#endif