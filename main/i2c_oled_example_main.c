
#include "funciones.h"


lv_disp_t * disp;
#define STACK_SIZE 1024

void vTaskEntradas(void* pvParameters);         /*DECLARACION DE LA FUNCIONES QUE SE EJECUTARAN EN "PARALELO" CON EL MAIN */
void vTasKAlarma(void* pvParameters);
void vTasKFan(void* pvParameters);
esp_err_t create_tasks(void);                   /*DECLARACION DE LA FUNCION QUE CREARA LAS 3 TAREAS QUE INVOCARAN LAS FUNCIONES DE ARRIBA DECLARADAS*/

void app_main(void)
{

/************************************************************/

    /*INICIALIZACION DE LOS COMPONENTES DEL SISTEMA */

    init_gpios();           /*Inicializamos los gpio declaramos entradas,salidas,pullup,etc.*/
    init_uart();            /*inicializamos la uart: velocidad de trasmision , paridad, bit de paro etc */
    init_adc();             /*iniciamos el adc */
    disp = init_oled();     /*iniciamos el i2c para la oled y sus configuraciones */
    create_tasks();         /*creamos las tareas*/
/***************************************************************/
    lv_obj_t *scr = lv_disp_get_scr_act(disp);  /*le damos como parametro lo que retorno el llamada a init_oled*/
    lv_obj_t *label = lv_label_create(scr);     /*creamso estos punteros a estructuras que son importantes para escribir en la oled*/


    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);  /*una vez creado nuestra "variable" label podemos configurarla"*/
    lv_label_set_text(label, "Sistema: Off");       /*este sera el formato para escribir en la oled, se pasa el label y el texto*/
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);             

    /*sprintf(s_puerta,"Closed");*/

    /*HOLA*/

 
    
    while(1){
        actualizar_salidas();
        imprimir_oled(label);/*una vez configurado el objeto label lo pasamos a la funcion imprimir_oled para dentro de la funcion poder escribir en la oled*/
        imprimir_terminal();
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    
}



/*En esta funcion se crean las tareas que en este caso seran 3 para que el programa tenga un mejor fucionamiento*/
esp_err_t create_tasks(void){
    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;

    xTaskCreate(vTaskEntradas,                           /*Funcion que va a llamar*/
                "vTaskEntradas",                         /*Nombre de la funcion que va a llamar*/
                STACK_SIZE,                              /*Memoria que asignaremos*/
                 &ucParameterToPass,
                 1,                                      /*prioridad*/
                 &xHandle);

    xTaskCreate(vTasKAlarma,                             /*Funcion que va a llamar*/
                "vTasKAlarma",                           /*Nombre de la funcion que va a llamar*/
                STACK_SIZE,                              /*Memoria que asignaremos*/           
                 &ucParameterToPass,
                 1,                                      /*prioridad*/
                 &xHandle);

    xTaskCreate(vTasKFan,                                /*Funcion que va a llamar*/
                "vTasKFan",                              /*Nombre de la funcion que va a llamar*/
                STACK_SIZE,                              /*Memoria que asignaremos*/
                 &ucParameterToPass,
                 1,                                      /*prioridad*/
                 &xHandle);

    return ESP_OK;
}

/*tarea #1*/

void vTaskEntradas(void* pvParameters){

    while(1){
        actualizar_entradas();
        vTaskDelay(pdMS_TO_TICKS(50));
    }

}

/*tarea #2*/
void vTasKAlarma(void* pvParameters){
    while(1){
        alarma();
        vTaskDelay(pdMS_TO_TICKS(50));  
    }

}

/*tarea #3*/
void vTasKFan(void* pvParameters){
    while(1){
        fan();
        vTaskDelay(pdMS_TO_TICKS(50));  
    }

}



