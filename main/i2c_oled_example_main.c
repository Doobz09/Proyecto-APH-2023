
#include "funciones.h"


lv_disp_t * disp;

char text[100];

void app_main(void)
{

    disp = init_oled();

    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */

    lv_label_set_text(label, "GUADALUPE\n6391613564\n10/01/2002\nING.Electronico");
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);

    vTaskDelay(pdMS_TO_TICKS(5000));

    sprintf(text,"GUADALUPE\n%s\n10/01/2002\nING.Electronico","sin Numero");
    lv_label_set_text(label,text);
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(5000));
    int i=10;
    while(1){
        sprintf(text,"GUADALUPE\n%s%d\n10/01/2002\nING.Electronico","Cuenta:",i++);
        lv_label_set_text(label,text);
        lv_obj_set_width(label, disp->driver->hor_res);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    
}

