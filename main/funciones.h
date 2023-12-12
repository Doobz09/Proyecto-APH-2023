#ifndef FUNCIONES_H
#define FUNCIONES_H
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "driver/adc.h"
#include <math.h>
#include<string.h>
#include"GPIO.h"

enum STATE{
    off,
    on,

};
#define S_IN BTN2

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#include "esp_lcd_sh1107.h"
#else
#include "esp_lcd_panel_vendor.h"
#endif

static const char *TAG = "example";

#define I2C_HOST  0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           21
#define EXAMPLE_PIN_NUM_SCL           22
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#define EXAMPLE_LCD_H_RES              64
#define EXAMPLE_LCD_V_RES              128
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

extern void example_lvgl_demo_ui(lv_disp_t *disp);

/* The LVGL port component calls esp_lcd_panel_draw_bitmap API for send data to the screen. There must be called
lvgl_port_flush_ready(disp) after each transaction to display. The best way is to use on_color_trans_done
callback from esp_lcd IO config structure. In IDF 5.1 and higher, it is solved inside LVGL port component. */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t * disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}


extern uint32_t BotonEncState;
extern uint32_t EstadoSistema;
extern uint32_t adc_val1;
extern double tv;
extern double tr;
extern double y;
extern double temp;

extern uint32_t espacio_personas;
extern uint32_t espacio_ahora_personas;


extern lv_disp_t * disp;
extern char state[100];

extern lv_disp_t * init_oled(void);
extern void init_gpios(void);
extern void actualizar_entradas(void);
extern void imprimir_oled(lv_obj_t *label);
extern void actualizar_salidas(void);
extern void init_adc(void);
extern void imprimir_terminal(void);

#endif