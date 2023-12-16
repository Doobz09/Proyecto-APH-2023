
#include "funciones.h"

char state[100];
char s_puerta[10];
char s_auto[10];
char s_cool[10];
uint32_t divisor = 4095/100;
uint32_t temp_corporal;



uint32_t BotonEncState =off;
uint32_t EstadoSistema =off;
uint32_t EstadoPuerta=off;
uint32_t mensaje_no_hay_espacio = off;
uint32_t adc_val1;
uint32_t adc_val2;

uint32_t mensaje_temperatura=off;
/*****************************************/
       /*ESTOS VALORES SE PUEDEN AJUSTAR Y MODIFICARIAN EL FUNCIONAMIENTO DEL PROGRAMA */

uint32_t tem_min=10;
uint32_t tem_max=40;
uint32_t set_point=30;
/***************************************/
uint32_t estado_alarma=off;

uint32_t estado_boton_auto=AUTO;
uint32_t estado_boton_cool= COOL;



double tv;
double tr;
double y;
double temp;


extern uint32_t espacio_total_personas=3;
uint32_t espacio_ahora_personas=0;



lv_disp_t * init_oled(void){
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
        .dc_bit_offset = 0,                     // According to SH1107 datasheet
        .flags =
        {
            .disable_control_phase = 1,
        }
#endif
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh1107(io_handle, &panel_config, &panel_handle));
#endif

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif

    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    lv_disp_t * disp = lvgl_port_add_disp(&disp_cfg);
    /* Register done callback for IO */
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, disp);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    ESP_LOGI(TAG, "Display LVGL Scroll Text");


    return(disp);
}

void init_gpios(void){
    /*ENTRADAS*/
    /*se configuran las esntradas y se activan pullUps*/

    GpioModeInput(BTN1);/*GPIO18*/
    GpioPullUpEnable(BTN1);

    GpioModeInput(BTN2); /*GPIO19*/
    GpioPullUpEnable(BTN2);

    GpioModeInput(GPIO15); /*GPIO15*/
    GpioPullUpEnable(GPIO15);


    GpioModeInput(GPIO23);
    GpioPullUpEnable(GPIO23);

    GpioModeInput(GPIO27);
    GpioPullUpEnable(GPIO27);





    /*SALIDAS*/
    GpioModeOutput(LED1); /*GPIO17*/
    GpioModeOutput(LED2); /*GPO16*/
    GpioModeOutput(LED3);/*GPIO4*/
    GpioModeOutput(LED4); /*GPIO2*/
    GpioModeOutput(LED8);

    /*RGB*/
    GpioModeOutput(LEDR);/*GPIO14*/
    GpioModeOutput(LEDG);/*GPIO13*/
    GpioModeOutput(LEDB);/*GPIO12*/

    /*EMPIEZAN APAGADOS LOS RGB*/
    GpioDigitalWrite(LEDR,GPIO_HIGH);
    GpioDigitalWrite(LEDG,GPIO_HIGH);
    GpioDigitalWrite(LEDB,GPIO_HIGH);
    /*establecemos el led 8 que es el de encendido del sistema para que empieze apgado*/
    GpioDigitalWrite(LED8,GPIO_LOW);


}
void init_adc(void){

    /*CONFIGURAMOS LOS ADC USAMOS EL ADC1 CANAL 0 DONDE ESTA EL SENSOR Y EL CANAL 4 DONDE SIMULAMOS EL OTRO */
    /*RESOLUCION DE 12 BITS MAXIMO VALOR 4095 ATENUACION 11 PARA TENER UN VALOR DE 3.3 EN VOLTAJE*/
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);/*pinVp*/
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);/*PIN32*/
    adc1_config_width(ADC_WIDTH_BIT_12);

}

void actualizar_entradas(void){

    if(!GpioDigitalRead(BTN1)){/*el btn de encendio es el btn1 tenemos un toogles para cambiar el valor de EstadoSistema cada vez que se presiona*/
        while(!GpioDigitalRead(BTN1));/*protegemos el btn para si se deja presionado se quede aqui hasta que lo suelta */
        BotonEncState=on;
        EstadoSistema=!EstadoSistema;/*toogle cambia el valor de la bandera cada que se presiona el btn1*/
    }

    /*Todo lo que sigue en el siguiente condicional solo se hara si la bandera EstadoSistema == on lo que indica que se presion el btn1*/

    if(EstadoSistema==on){
        /*LEEMOS LA TEMPERATURA DEL SALON */
        adc_val1 = adc1_get_raw(ADC1_CHANNEL_0);

        /*LEEMOS TEMPERATURA CORPORAL*/
        adc_val2 = adc1_get_raw(ADC1_CHANNEL_4);
/********************************************************************************************/        

        /*SENSOR S_IN*/


        if(!GpioDigitalRead(S_IN)){          /*Este condicional responde al sensor S_in que esta en el btn2 de la placa*/
            while(!GpioDigitalRead(S_IN));/*proteccion por si se mantiene presionado, tambien se podria tomar como antirebote */

            if(temp_corporal<tem_min || temp_corporal>tem_max){/*si el s_in detecto persona pero su temperatura corporal esta fuera de los limites*/
                mensaje_temperatura=on;                        /*activamos estas banderas la primera hara que se imprima el mensaje en oled y terminal*/
                estado_alarma=on;                              /*la segunda activara la alarma, parpadeo del led rgb esto se hace en otra tarea llamada alarma */
            }                                                  /*que se controla con la bandera estadoalarma*/
                                                               /*si esto se cumplio los otras 2 condiciones se omiten */
            else if(espacio_ahora_personas<espacio_total_personas){
                    espacio_ahora_personas++;                   /*si la primera condicion no se cumplio y si hay espacio disponible*/
                    EstadoPuerta=on;                            /*aumentamos el numero de personas en el local */
            }                                                   /*ademas de activar la bandera estadopuerta =on para que la imprima en la oled y terminal por 5s*/
            else{
                mensaje_no_hay_espacio=on;      /*si el local esta lleno no se cumple la condicion anterior y se mete a este else */
            }                                   /*que pondra en on la bandera mensaje_no_hay_esoacio que hara que se imprima en terminal y oled  */
                                                /*cuando llegue a la funcion imprimirterminal o imprimiroled checan el valor de estas banderas */
        }                                       /*y dependiendo el estado hacen una accion u otra */
/********************************************************************************************/

/********************************************************************************************/
          /*SENSOR S_OUT*/


        if(!GpioDigitalRead(S_OUT)){         /*Esta condicion comprueba si el S_OUT detecto a una persona*/
            while(!GpioDigitalRead(S_OUT));/*proteccion por si se mantiene presionado, tambien se podria tomar como antirebote */
            if(espacio_ahora_personas>0){ 
                espacio_ahora_personas--;     /*lo unico que se hace aqui es si el sensor detecta va descontando personas del local*/
            }                                 /*le ponemos una protecion para que no decremente si no hay personas dentro */
        }
/********************************************************************************************/

/********************************************************************************************/

        /*BOTONES AUTO/ON   Y  COOL/HEAT PARA CONTROLAR EL FAN*/


        if(!GpioDigitalRead(BTN_AUTO_ON)){          /*Si se presiona el btn_auto_on*/
            while(!GpioDigitalRead(BTN_AUTO_ON));    /*lo va hacer es intercambiar el valor que guarda cada vez que sea presionado*/
            estado_boton_auto=!estado_boton_auto;    /*y con esto hacemos el cambio entre auto y on por defecto tiene auto que es un 0*/
        }                                            /*si se presiona lo cambia a 1 que equivale a on */
                                                     /*esta bandera se usa en la fucion actuañizar salidas para dependiendo del valor que */
        if(!GpioDigitalRead(BTN_COOL_HEAT)){         /*contenga escribira en una variable(arreglo de caracteres) "auto" o "on" para poder imprimirlo */
            while(!GpioDigitalRead(BTN_COOL_HEAT)); /*ademas en la tarea FAN dependiendo el valor de la bandera se escogera un modo*/
            estado_boton_cool=!estado_boton_cool;   /*esto es lo mismo para el botn cool_heat */
        }

/*********************************************************************************************/

    }
    

}

void actualizar_salidas(void){
    /*aqui lo que se hace es dependiendo del valor de las banderas que se modifican en la funcion actualizar_entradas se realiza una accion*/

    /******************************************************************************************/

    static uint32_t bandera = 1;             
    if(EstadoSistema==on && bandera==1){   /*si la bandera estado sistea == on encendemos el led que indica encendido del sistema */
        GpioDigitalWrite(LED8,GPIO_HIGH);  /*y usamos una bandera interna para que no se este metiendo a la condicion siempre, solo una vez*/
        bandera=0;
    }
    /******************************************************************************************/

    /******************************************************************************************/
                /*APAGADO DEL SISTEMA*/

    if(EstadoSistema==off){
        GpioDigitalWrite(LED8,GPIO_LOW);
        bandera=1;
        mensaje_temperatura=off;                        /*si la bandera estado de sistema es igual off ponemos todo por defecto para cuando se*/
        estado_alarma=off;                              /*encienda el sistema no se guarden los datos anteriores */
        EstadoPuerta=off;
        mensaje_no_hay_espacio=off;
        espacio_ahora_personas=0;
        estado_boton_auto=AUTO;
        estado_boton_cool= COOL;
        GpioDigitalWrite(LED1,GPIO_LOW);
        GpioDigitalWrite(LED4,GPIO_LOW);
    }

    /*****************************************************************************************/



    if(EstadoSistema==on){ /*TODO LO QUE VIENE A CONTINUACION SE HARA SOLO SI EL SISTEMA ESTA ENCENDIDO */


/********************************************************************************************/

    /*CONVERSION DE VALORES LEIDOS POR LOS ADC PARA PONERLOS EN GRADOS CELSIUS*/

        tv = 3.3 * adc_val1/4095.0;
        tr = tv * 10000.0/(3.3-tv);               /*Esta formula es para el sensor de la placa el NTC*/
         y = log(tr/10000.0);
         y = (1.0/298.15)+(y * (1.0/4050.0));
        temp = 1.0/y;
        temp = temp -273.15;                      /*guardamos el valor en grados celsius en la variable temp*/


        temp_corporal = adc_val2/divisor;         /*temperatura corpora va de 0 a 102 grados celsius lo ajsutamos con un pot*/
/*********************************************************************************************/

/*********************************************************************************************/

            /*CONTROL DE LA PUERTA RELE*/
        if(EstadoPuerta==on)
            GpioDigitalWrite(LED4,GPIO_HIGH);        /*si la bandera estado puerta esta on se activa de lo contrario se apaga */
        else
            GpioDigitalWrite(LED4,GPIO_LOW);
/*********************************************************************************************/

        /*CONTROL PARA LA VARIABLE QUE GUARDA el texto a imprimir del modo del fan por oled*/

        if(estado_boton_auto==AUTO)    /*si el estado del boton auto es == auto guardamos "Auto"en la variable a imprimir por oled*/
            sprintf(s_auto,"Auto");
        else
            sprintf(s_auto,"On");      /*de lo contrario guardamos"on"*/

        if(estado_boton_cool==COOL)    /*si el estado del boton cool es == cool guardamos "Cool"en la variable a imprimir por oled*/
            sprintf(s_cool,"Cool");
        else
            sprintf(s_cool,"Heat");    /*de lo contrario guardamos"Heat"*/

/********************************************************************************************/
        
    }


}

void imprimir_oled(lv_obj_t *label){

    static uint32_t bandera =1;
    if(EstadoSistema==off){
        sprintf(state,"  Ctrl De Acceso  \n\n  ->Sistema: OFF");/*Esto solo se imprime cuando el estado del sistema es == off*/
        lv_label_set_text(label, state);                        
        bandera=1;
    }
    
    if(EstadoSistema==on){/*cuan el estado del sistema == on se imprimen todos los demas datos por la oled*/

        if(EstadoPuerta==on){          /*Esto deberia estar en actualizar salidas */
            sprintf(s_puerta,"Open");  /*si la bandera estado puerta = on  se edita el arreglo de caracteres y se coloca dentro open */
        }
        else
            sprintf(s_puerta,"Closed");/*en caso contrario se escribe closed, este arreglo de pasa a las pantallas, terminal para su impresion */
        /************************************************************************/
        if(mensaje_temperatura==on){
            lv_label_set_text(label, "\n\nTemp_Out_Of_Range");/*si la bandera mensaje_temperatura == on esta activa se imprime esto y se omite lo otro*/
        }                                                  /*el tiempo de la impresion del texto esta establecido en la funcion imprimirTerminal*/

        else if(mensaje_no_hay_espacio==on){
                lv_label_set_text(label,"\n\nWe are Full, wait");/*el tiempo de impresion se controla en la funcion imprimir Terminal */
                                                                /*si se imprime esto lo demas se omite */
        }
        /*si no se cumplio lo de arriba significa que todo esta en orden asi que se imprimen todos los datos normalmente*/
        else{
            sprintf(state," Tc:%ld°c   Ta:%.1f°c\n Door:%s\n Fan:%s/%s\n        %ld de %ld",temp_corporal,temp,s_puerta,s_auto,s_cool,espacio_ahora_personas,espacio_total_personas);
            lv_label_set_text(label, state);
        }

    }

}
void imprimir_terminal(){
    static uint32_t bandera =1;
/**************************************************************************************************************************/
    if(EstadoSistema==off && bandera ==1){
       /*printf("\n\nSISTEMA: OFF\n\n");*/
       sprintf(state,"\nSISTEMA:OFF\n");
       uart_write_bytes(UART_NUM_0,state,strlen(state));  /*Esto solo se imprime si el esto del sisteam ==off*/
                                                          /*usamos una bandera interna para que nomas se imprima una vez */
        bandera=0;
    }
/**************************************************************************************************************************/        

    if(EstadoSistema==on){/*TODO LO QUE SIGUIE DESPUES DE AQUI SOLO SE PODRA HACER SI EL SISTEMA ESTA ENCENDIDO*/
        bandera=1;
/****************************************************************************************************************************/
        if(EstadoPuerta==on){
            sprintf(s_puerta,"OPEN");/*si estado de puerta ==on guarda la cadena "OPEN "en la variable para que se imprima por terminal*/
        }
/*****************************************************************************************************************************/
        
/*******************************************************************************************************************************/

                            /*FORMATO DE IMPRESION POR TERMINAL*/   

                            /*usamos variables para dependiendo el estado que este guarde se imprima el valor correcto en termianl*/
        sprintf(state,"Modo:%s/%s  |  Door: %s  |  Temp:%.2f C  | Personas: %ld de %ld  | TempCorp: %ld C\n",s_auto,s_cool,s_puerta,temp,espacio_ahora_personas,espacio_total_personas,temp_corporal);
        /*printf("%s", state);*/
        uart_write_bytes(UART_NUM_0,state,strlen(state));   /*envia por la uart*/

/******************************************************************************************************************************************/

    /*impresion de door: open por 5 segundos cuando se detecta a una persona*/

        if(EstadoPuerta==on){
            /*printf("\n\nDOOR:OPEN\n\n");*/
            sprintf(state,"\r\rDORR:OPEN\r\r");
            uart_write_bytes(UART_NUM_0,state,strlen(state));
            vTaskDelay(pdMS_TO_TICKS(5000));                     /*despues de 5 segundos continuamos */
            sprintf(s_puerta,"CLOSED");
            EstadoPuerta=off;                                    /*ponemos la bandera en off para que se cierre */
            
        }

 /***********************************************************************************************************************************/

 /*************************************************************************************************************************************/ 

                /*SI LA BANDERA MENSAJE_TEMPERATURA ==ON IMPRIMIMOS EL MENSAJE POR 5 SEGUNDOS */
                    /*solo se imrimira el mensaje en terminal y oled despues se imprimira el formato normal*/

        if(mensaje_temperatura==on){
            /*printf("\n\nTemp_Out_Of_Range\n\n");*/
            sprintf(state,"\n\nTemp_Out_Of_Range\n\n");
            uart_write_bytes(UART_NUM_0,state,strlen(state));
            vTaskDelay(pdMS_TO_TICKS(5000));
            mensaje_temperatura=off;            /*despues de los 5 segundos desctivamos la bandera */
            estado_alarma=off;                  /*tambien la bandera_estado_alarma la apagamos, esta bandera tiene efecto */
                                                /*en la funcion  Alarma que ocurre en otra tarea  */
        }
/*************************************************************************************************************************************/

/**************************************************************************************************************************************/
            
            /*  MENSAJE NO HAY ESPACIO SE IMPRIME SI LA BANDERA ESTA EN ON*/
            /*solo se imrimira el mensaje en terminal y oled despues se imprimira el formato normal*/
            /*lo imprimimos por 1 segundo ya que no espesifica tiempo*/

        else if(mensaje_no_hay_espacio==on){
                /*printf("\n\nWe are Full, wait\n\n");*/
                sprintf(state,"\n\nWe are Full, wait\n\n");
                uart_write_bytes(UART_NUM_0,state,strlen(state));
                vTaskDelay(pdMS_TO_TICKS(1000));
                mensaje_no_hay_espacio=off;     /*despues del tiempo lo desactivamos para que se imprima de forma normal */

        }

/***************************************************************************************************************************************/

    }



}

/*ESTA FUNCION SE EJECUTA EN OTRA TAREA */

void alarma(void){

    if(estado_alarma==on){
        GpioDigitalWrite(LEDR,GPIO_LOW);      /*Lo unico que hacemos aqui es dependiendo del valor de la bandera estado_alarma */
        GpioDigitalWrite(LEDB,GPIO_HIGH);     /*se activara o se desactivara */
        vTaskDelay(pdMS_TO_TICKS(100));
        GpioDigitalWrite(LEDB,GPIO_LOW);
        GpioDigitalWrite(LEDR,GPIO_HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    else
    {
        GpioDigitalWrite(LEDR,GPIO_HIGH);    /*si la bandera estan en off apagamos los leds */
        GpioDigitalWrite(LEDB,GPIO_HIGH);
    }
    
    

}

/*ESTA FUNCION SE EJECUTA EN OTRA TAREA */

void fan(void)
{
    if(EstadoSistema==on){               /*SOlo entra si el estado del sistema esta encendido*/
     
        if(estado_boton_auto==AUTO && estado_boton_cool==COOL){     /* BASICAMENTE 4 OPCIONES PARA CONTROLAR EL FAN*/
            if(temp>set_point)                                      /*#1*/
                GpioDigitalWrite(LED1,GPIO_HIGH);                   /*La primera dice que si el modo es =auto y cool*/
            else                                                    /*el fan se encendera si la temperatura ambiente es mayor al setpoint*/
                GpioDigitalWrite(LED1,GPIO_LOW);                    /*de lo contrario se apaga*/
        }
        else if(estado_boton_auto==AUTO && estado_boton_cool==HEAT){/*#2*/
            if(temp<set_point)                                      /*Si no se cumple lo primero y tenemos AUTO y HEAT en modo*/
                GpioDigitalWrite(LED1,GPIO_HIGH);                   /*el fan se encendera si la temperatura ambiente es menor al setpoint*/   
            else
                GpioDigitalWrite(LED1,GPIO_LOW);                    /*de lo contrario se apaga*/
        }

        else if(estado_boton_auto==ON && estado_boton_cool==COOL){/*#3*/
            GpioDigitalWrite(LED1,GPIO_HIGH);                     /*Si no se cumple las opciones de arriba y tenemos On y cool*/
        }                                                         /*El fan se encendera y asi permanecera independientemente del setpoint*/

        else
            GpioDigitalWrite(LED1,GPIO_LOW);                      /*#4*/
                                                                  /*Si no se cumple nada de lo de arriba es porque estamos en on y heat*/
                                                                  /*aqui el fan permanecera apagado siempre*/
    }

}
void init_uart(void){
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,                  /*9600 de velocidad de trasmision*/
        .data_bits = UART_DATA_8_BITS,           /*8 bits*/
        .parity = UART_PARITY_DISABLE,           /*paridad desabilitada*/
        .stop_bits = UART_STOP_BITS_1,           /*un bit para paro */
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_0, &uart_config); /*el puerto de la uart a usar el el 0 le pasamos la estructura anterior */
    uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);  /*configuramos los pines */
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);   /*le decimos el tamaño del buffer en eeste caso 1024*/
}