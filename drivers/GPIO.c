 /*FileName:         GPIO.c                                                                                            */
 /* Dependencies:    None                                                                                              */
 /* Processor:       ESP32                                                                                             */
 /* Board:           ESP32-PANTHER48                                                                                   */
 /* Program version: 5.1                                                                                           */
 /* Company:         Espressif Systems                                                                                 */
 /* Description:     Definicion de las funciones.                                 */
 /* Authors:         Guadalupe Méndez Ontiveros, Manuel Francisco Zubiate Loya, Alan Alexis Rodriguez Hernandez.                                          */
 /*Updated:          12/2023                                                                                            */
#include "GPIO.h"

volatile uint32_t GPIO_PINX_MUX_REG[] = {
    IO_MUX_GPIO0_REG,
    IO_MUX_GPIO1_REG,
    IO_MUX_GPIO2_REG,
    IO_MUX_GPIO3_REG,
    IO_MUX_GPIO4_REG,
    IO_MUX_GPIO5_REG,
    IO_MUX_GPIO6_REG,
    IO_MUX_GPIO7_REG,
    IO_MUX_GPIO8_REG,
    IO_MUX_GPIO9_REG,
    IO_MUX_GPIO10_REG,
    IO_MUX_GPIO11_REG,
    IO_MUX_GPIO12_REG,
    IO_MUX_GPIO13_REG,
    IO_MUX_GPIO14_REG,
    IO_MUX_GPIO15_REG,
    IO_MUX_GPIO16_REG,
    IO_MUX_GPIO17_REG,
    IO_MUX_GPIO18_REG,
    IO_MUX_GPIO19_REG,
    IO_MUX_GPIO20_REG, 
    IO_MUX_GPIO21_REG,
    IO_MUX_GPIO22_REG,
    IO_MUX_GPIO23_REG,
    0,
    IO_MUX_GPIO25_REG,
    IO_MUX_GPIO26_REG,
    IO_MUX_GPIO27_REG,
    0,
    0,
    0,
    0,
    IO_MUX_GPIO32_REG,
    IO_MUX_GPIO33_REG,
    IO_MUX_GPIO34_REG,
    IO_MUX_GPIO35_REG,
    IO_MUX_GPIO36_REG,
    IO_MUX_GPIO37_REG,
    IO_MUX_GPIO38_REG,
    IO_MUX_GPIO39_REG,
};

uint32_t rtc_pins[] = {
	RTCIO_TOUCH_PAD1, /*GPIO0*/
	0,                /*GPIO1*/
	RTCIO_TOUCH_PAD2, /*GPIO2*/
	0,                /*GPIO3*/
	RTCIO_TOUCH_PAD0, /*GPIO4*/
	0,                /*GPIO5*/
	0,                /*GPIO6*/
	0,                /*GPIO7*/
	0,                /*GPIO8*/
	0,                /*GPIO9*/
	0,                /*GPIO10*/
	0,                /*GPIO11*/
	RTCIO_TOUCH_PAD5, /*GPIO12*/
	RTCIO_TOUCH_PAD4, /*GPIO13*/
	RTCIO_TOUCH_PAD6, /*GPIO14*/
	RTCIO_TOUCH_PAD3, /*GPIO15*/
	0,                /*GPIO16*/
	0,                /*GPIO17*/
	0,                /*GPIO18*/
	0,                /*GPIO19*/
	0,             /*GPIO20*/
	0,                /*GPIO21*/
	0,                /*GPIO22*/
	0,                /*GPIO23*/
	0,             /*GPIO24*/
	1,                /*GPIO25*/
	1,                /*GPIO26*/
	RTCIO_TOUCH_PAD7, /*GPIO27*/
	0,             /*GPIO28*/
	0,             /*GPIO29*/
	0,             /*GPIO30*/
	0,             /*GPIO31*/
	1,                /*GPIO32*/
	1,                /*GPIO33*/
    1,                /*GPIO34*/
    1,                /*GPIO35*/
    1,                /*GPIO36*/
    1,                /*GPIO37*/
    1,                /*GPIO38*/
    1,                /*GPIO39*/

};

volatile uint32_t dir_rtcio_touch_pad[]={
    0,
    RTC_IO_TOUCH_PAD0_REG,
    RTC_IO_TOUCH_PAD1_REG,
    RTC_IO_TOUCH_PAD2_REG,
    RTC_IO_TOUCH_PAD3_REG,
    RTC_IO_TOUCH_PAD4_REG,
    RTC_IO_TOUCH_PAD5_REG,
    RTC_IO_TOUCH_PAD6_REG,
    RTC_IO_TOUCH_PAD7_REG,
    RTC_IO_TOUCH_PAD8_REG,
    RTC_IO_TOUCH_PAD9_REG,

};

/**************************************************************************
* Function: GpioModeOutput
* Preconditions: None
* Overview: Configura el gpio que se le pase como parametro para poder usarlo como salida.
* Input: Gpio 
* Output: None.
*
*****************************************************************************/

void GpioModeOutput(uint32_t gpio){

    /*VALIDAMOS GPIOS RESERVADOS O NO EXISTENTES*/
    if(GPIO_PINX_MUX_REG[gpio]==0){
        printf("Error el GPIO:%lu No se puede utilizar",gpio);
        exit(1);
    }

    /*VALIDACIONES*/
    if(gpio>=GPIO34 && gpio<=GPIO39){ /*ESTOS PINES SOLO PUEDEN USARSE COMO ENTRADAS*/
        printf("ERROR: El GPIO%lu SOLO PUEDE SER USADO COMO ENTRADA\n",gpio);
        exit(1);
    }

    /*DESACTIVAMOS EL PIN COMO ENTRADA*/
    HWREG32(GPIO_PINX_MUX_REG[gpio]) &= ~(1<<FUN_IE);

    /*SELECCIONAMOS LA FUNCION 2 GPIO*/
    HWREG32(GPIO_PINX_MUX_REG[gpio]) &= ~(1<<12);
    HWREG32(GPIO_PINX_MUX_REG[gpio]) |=  (1<<13);
    HWREG32(GPIO_PINX_MUX_REG[gpio]) &= ~(1<<14);


   /*ACTIVAMOS EL PIN COMO SALIDA*/          
   /*CASO ESPECIAL PARA EL GPIO32 Y 33*/                        /*ONLY INPUTS  I   I   I   I   I   I            */
    if((gpio==GPIO32)||(gpio==GPIO33)){                         /* GPIOS      39  38  37  36  35  34  33  32    */
        GPIO_ENABLE1_REG |= (1<<(gpio-32));                    /*ENABLE1_REG | X | X | X | X | X | X | X | X |  */
                                                                /*  BIT        B7  B6  B5  B4  B3  B2  B1  B0   */
    }
    else
        GPIO_ENABLE_REG |= (1<<gpio);           /*REGISTRO QUE TIENE LOS GPIO DEL 0 AL 31*/

}

/**************************************************************************
* Function: GpioModeInput
* Preconditions: None
* Overview: Configura el gpio que se le pase como parametro para poder usarlo como entrada.
* Input: Gpio 
* Output: None. 
*
*****************************************************************************/

void GpioModeInput(uint32_t gpio){

    /*VALIDAMOS GPIOS RESERVADOS*/
    if(GPIO_PINX_MUX_REG[gpio]==0){
        printf("Error el GPIO:%lu No se puede utilizar",gpio);
        exit(1);
    }

    /*DESACTIVAMOS EL PIN COMO SALIDA*/
    if(gpio>=GPIO32 && gpio<=GPIO39){
        GPIO_ENABLE1_REG &= ~(1<<(gpio-32));
    }
    else
        GPIO_ENABLE_REG &= ~(1<<gpio);

    /*ACTIVAMOS EL PIN COMO ENTRADA*/
    HWREG32(GPIO_PINX_MUX_REG[gpio]) |= (1<<FUN_IE);

    /*SELECCIONAMOS LA FUNCION 2 GPIO*/
    HWREG32(GPIO_PINX_MUX_REG[gpio]) &= ~(1<<12);
    HWREG32(GPIO_PINX_MUX_REG[gpio]) |=  (1<<13);
    HWREG32(GPIO_PINX_MUX_REG[gpio]) &= ~(1<<14);

}


/**************************************************************************
* Function: GpioPullUpEnable
* Preconditions: GpioInput para declarar el pin como entrada
* Overview: Activa la resistencia PullUp interna del gpio que se le pase como parametro, si es que tiene .
* Input: Gpio al cual se le quiere activar la resistencia pull up
* Output:None
*
*****************************************************************************/


void GpioPullUpEnable(uint32_t gpio){
    if(identify_pin_rtc(gpio)){
        
        HWREG32(dir_rtcio_touch_pad[identify_pin_rtc(gpio)]) |=(1<<27);
        HWREG32(dir_rtcio_touch_pad[identify_pin_rtc(gpio)]) &= ~(1<<28);
        
    }
    else{
           HWREG32(GPIO_PINX_MUX_REG[gpio]) |=(1<<PULL_UP); 
           HWREG32(GPIO_PINX_MUX_REG[gpio]) &= ~(1<<PULL_DOWM);   
    }

}


/**************************************************************************
* Function: GpioPullDownEnable
* Preconditions: GpioInput para declarar el pin como entrada.
* Overview: Activa la resistencia PullDown interna del gpio que se le pase como parametro, si es que tiene.
* Input: Gpio al cual se le quiere activar la resistencia pull down.
* Output: None.
*
*****************************************************************************/

void GpioPullDownEnable(uint32_t gpio){
    if((identify_pin_rtc(gpio))!=0){
        HWREG32(dir_rtcio_touch_pad[identify_pin_rtc(gpio)]) |=(1<<28);
        HWREG32(dir_rtcio_touch_pad[identify_pin_rtc(gpio)]) &= ~(1<<27);
    }
    else{
           HWREG32(GPIO_PINX_MUX_REG[gpio]) |=(1<<PULL_DOWM); 
           HWREG32(GPIO_PINX_MUX_REG[gpio]) &= ~(1<<PULL_UP);
    }

}


/**************************************************************************
* Function: GpioDigitalWrite
* Preconditions: Gpio Establecido como salida GpioModeOutput
* Overview: Establece un 1 o un 0 logico al puerto que recibe como paramentro.
* Input: GPIO que se desea poner en alto o bajo, el estado(1,0) que se desa poner al GPIO
* Output: None
*
*****************************************************************************/

void GpioDigitalWrite(uint32_t gpio,gpio_state state){
    /*OPCION:MANDAR UN ALTO HIGH*/
    if(state==GPIO_HIGH){
        if((gpio==GPIO32)||(gpio==GPIO33))                    /*    GPIOS      39  38  37  36  35  34  33  32    */
            GPIO_OUT1_W1TS_REG = (1<<(gpio-32));              /*OUT1_W1TS_REG | X | X | X | X | X | X | X | X |  */                                                     
        else                                                  /*     BIT        B7  B6  B5  B4  B3  B2  B1  B0   */
            GPIO_OUT_W1TS_REG = (1<<gpio);         /*REGISTRO QUE TIENE LOS GPIO DEL 0 AL 31*/
    }

        /*OPCION:MANDAR UN BAJO LOW*/
    else if(state == GPIO_LOW){
        if((gpio==GPIO32)||(gpio==GPIO33))
            GPIO_OUT1_W1TC_REG = (1<<(gpio-32));
        else
            GPIO_OUT_W1TC_REG = (1<<gpio);/*REGISTRO QUE TIENE LOS GPIO DEL 0 AL 31*/
    }
        
}


/**************************************************************************
* Function: GpioDigitalRead
* Preconditions: Gpio establecido como entrada GpioModeInput
* Overview: Lee el estado que tiene el Gpio que se pasa como argumento .
* Input: Gpio al que se desea leer el valor de entrada
* Output: Valor Leido en el GPIO 1 = HIGH  0 = LOW
*
*****************************************************************************/

int GpioDigitalRead(uint32_t gpio){
    if(gpio>=GPIO32 && gpio<=GPIO39)                             
        return ((GPIO_IN1_REG &(1<<(gpio-32)))?1:0);       /*    GPIOS      39  38  37  36  35  34  33  32    */
                                                           /*    IN1_REG   | X | X | X | X | X | X | X | X |  */
    else                                                   /*     BIT        B7  B6  B5  B4  B3  B2  B1  B0   */
        return ((GPIO_IN_REG &(1<<gpio))?1:0);
     /*PARA GPIOS DEL 0 AL 31 SE HACE EN ESTE REGISTRO*/

}


/**************************************************************************
* Function: identifi_pin_rtc
* Preconditions: None
* Overview: Me dice si un pin es RTC y en cual canal se maneja .
* Input: Gpio 
* Output: regresa el indeice de un arreglo donde se encuantran direcciones si el pin si es rtc si no regresa 0 
*
*****************************************************************************/

int identify_pin_rtc(uint32_t gpio){
    if(rtc_pins[gpio]!=0)
        return rtc_pins[gpio];
    else
        return 0;

}

