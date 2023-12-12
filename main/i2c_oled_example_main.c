
#include "funciones.h"


lv_disp_t * disp;



void app_main(void)
{

    init_gpios();
    disp = init_oled();

    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); 

    lv_label_set_text(label, "SISTEMA: OFF");
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);

    vTaskDelay(pdMS_TO_TICKS(2000));
    /*HOLA*/

 
    
    while(1){
        actualizar_entradas();
        imprimir_oled(label);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    
}

