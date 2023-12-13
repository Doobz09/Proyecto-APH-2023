
#include "funciones.h"

char state[100];
char s_puerta[10];
char s_auto[10];
char s_cool[10];
uint32_t divisor = 4095/100;
uint32_t temp_corporal;
uint32_t set_point=28;


uint32_t BotonEncState =off;
uint32_t EstadoSistema =off;
uint32_t EstadoPuerta=off;
uint32_t mensaje_no_hay_espacio = off;
uint32_t adc_val1;
uint32_t adc_val2;

uint32_t mensaje_temperatura=off;
uint32_t tem_min=8;
uint32_t tem_max=40;

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



/*GPIO5 OCUPADO POR EL LED 8*/

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

    GpioDigitalWrite(LED8,GPIO_LOW);


}
void init_adc(void){

    /*CONFIGURAMOS LOS ADC USAMOS EL ADC1 CANAL 0 DONDE ESTA EL SENSOR Y EL CANAL 4 DONDE SIMULAMOS EL OTRO */
    /*RESOLUCION DE 12 BITS MAXIMO VALOR 4095 ATENUACION 11 PARA TENER UN VALOR DE 3.3 EN VOLTAJE*/
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);/*PIN32*/
    adc1_config_width(ADC_WIDTH_BIT_12);

}

void actualizar_entradas(void){

    if(!GpioDigitalRead(BTN1) && BotonEncState==off){
        BotonEncState=on;
        EstadoSistema=on;
    }

    if(EstadoSistema==on){
        /*LEEMOS LA TEMPERATURA DEL SALON */
        adc_val1 = adc1_get_raw(ADC1_CHANNEL_0);

        /*LEEMOS TEMPERATURA CORPORAL*/
        adc_val2 = adc1_get_raw(ADC1_CHANNEL_4);

        /*SENSOR S_IN*/

        if(!GpioDigitalRead(S_IN)){
            while(!GpioDigitalRead(S_IN));

            if(temp_corporal<tem_min || temp_corporal>tem_max){
                mensaje_temperatura=on;
                estado_alarma=on;
            }

            else if(espacio_ahora_personas<espacio_total_personas){
                    espacio_ahora_personas++;   
                    EstadoPuerta=on; 
            }
            else{
                mensaje_no_hay_espacio=on;
            }
  
        }

        if(!GpioDigitalRead(GPIO15)){
            while(!GpioDigitalRead(GPIO15));
            if(espacio_ahora_personas>0){
                espacio_ahora_personas--;    
            }
        }

        if(!GpioDigitalRead(GPIO23)){
            while(!GpioDigitalRead(GPIO23));
            estado_boton_auto=!estado_boton_auto;
        }

        if(!GpioDigitalRead(GPIO27)){
            while(!GpioDigitalRead(GPIO27));
            estado_boton_cool=!estado_boton_cool;
        }

        

    }
    



}

void actualizar_salidas(void){
    static uint32_t bandera = 1;
    if(BotonEncState==on && bandera==1){
        GpioDigitalWrite(LED8,GPIO_HIGH);
        bandera=0;
    }

    if(EstadoSistema==on){
        tv = ((3.3)*(adc_val1))/4095.0;
        tr = ((tv)*(10000.0))/(3.3-tv);
         y = log(tr/10000.0);
         y = (1.0/298.15)+(y*(1.0/4050.0));
        temp = 1.0/y;
        temp = temp -273.15;


        temp_corporal = adc_val2/divisor;

        if(EstadoPuerta==on)
            GpioDigitalWrite(LED4,GPIO_HIGH);
        else
            GpioDigitalWrite(LED4,GPIO_LOW);

        if(estado_boton_auto==AUTO)
            sprintf(s_auto,"AUTO");
        else
            sprintf(s_auto,"ON");

        if(estado_boton_cool==COOL)
            sprintf(s_cool,"COOL");
        else
            sprintf(s_cool,"HEAT");


        
    }

    
    

}

void imprimir_oled(lv_obj_t *label){

    static uint32_t bandera =1;
    
    if(BotonEncState==on && bandera==1){
    sprintf(state,"Sistema: ON");
    lv_label_set_text(label, state);
    bandera=0;
    }

    if(BotonEncState==on){

        if(EstadoPuerta==on){
            sprintf(s_puerta,"OPEN");
        }
        else
            sprintf(s_puerta,"CLOSED");

        if(mensaje_temperatura==on){
            lv_label_set_text(label, "Temp_Out_Of_Range");
        }

        else if(mensaje_no_hay_espacio==on){
                lv_label_set_text(label, "We are Full, wait");

        }
        else{
            sprintf(state,"Sistema: ON\nDoor: %s\nTemp:%.2f°C\nFan:%s/%s",s_puerta,temp,s_auto,s_cool);
            lv_label_set_text(label, state);
        }

        

    }

}
void imprimir_terminal(){
    static uint32_t bandera =1;
    
    if(BotonEncState==on && bandera==1){
    sprintf(state,"Sistema: ON");
    printf("%s", state);
    bandera=0;
    }

    if(BotonEncState==on){

        if(EstadoPuerta==on){
            sprintf(s_puerta,"OPEN");
        }

        

        sprintf(state,"Sistema: ON  |  Door: %s  |  Temp:%.2f°C  | Personas: %ld de %ld  | TempCorp: %ld°C\n",s_puerta,temp,espacio_ahora_personas,espacio_total_personas,temp_corporal);
        printf("%s", state);


        if(EstadoPuerta==on){
            printf("\n\nDOOR:OPEN\n\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
            sprintf(s_puerta,"CLOSED");
            EstadoPuerta=off;
            
        }
        if(mensaje_temperatura==on){
            printf("\n\nTemp_Out_Of_Range\n\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
            mensaje_temperatura=off;
            estado_alarma=off;

        }

        else if(mensaje_no_hay_espacio==on){
                printf("\n\nWe are Full, wait\n\n");
                vTaskDelay(pdMS_TO_TICKS(1000));
                mensaje_no_hay_espacio=off;

        }

    }



}

void alarma(void){
    if(estado_alarma==on){
        GpioDigitalWrite(LEDR,GPIO_LOW);
        GpioDigitalWrite(LEDB,GPIO_HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        GpioDigitalWrite(LEDB,GPIO_LOW);
        GpioDigitalWrite(LEDR,GPIO_HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    else
    {
        GpioDigitalWrite(LEDR,GPIO_HIGH);
        GpioDigitalWrite(LEDB,GPIO_HIGH);
    }
    
    

}
