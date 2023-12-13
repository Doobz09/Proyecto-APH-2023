
#include "funciones.h"


lv_disp_t * disp;
#define STACK_SIZE 1024

void vTaskEntradas(void* pvParameters);
void vTasKAlarma(void* pvParameters);
esp_err_t create_tasks(void);

void app_main(void)
{


    init_gpios();
    init_adc();
    disp = init_oled();
    create_tasks();

    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); 

    lv_label_set_text(label, "SISTEMA: OFF");
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);

    printf("Sistema: OFF\n");
    sprintf(s_puerta,"CLOSED");

    /*HOLA*/

 
    
    while(1){
        actualizar_salidas();
        imprimir_oled(label);
        imprimir_terminal();
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    
}



esp_err_t create_tasks(void){
    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;

    xTaskCreate(vTaskEntradas,
                "vTaskEntradas",
                STACK_SIZE,
                 &ucParameterToPass,
                 1,
                 &xHandle);

    xTaskCreate(vTasKAlarma,
                "vTasKAlarma",
                STACK_SIZE,
                 &ucParameterToPass,
                 1,
                 &xHandle);

    return ESP_OK;
}

void vTaskEntradas(void* pvParameters){

    while(1){
        actualizar_entradas();
        vTaskDelay(pdMS_TO_TICKS(50));

    }
    


}

void vTasKAlarma(void* pvParameters){
    while(1){
        alarma();
        vTaskDelay(pdMS_TO_TICKS(50));  
    }

}



