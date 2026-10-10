/**
 * @file lv_port_indev.c
 * @brief LVGL 输入设备移植层。本项目只使用 XPT2046 电阻触摸,
 *        官方模板中的 mouse/keypad/encoder/button 示例代码已裁剪。
 */

/*Copy this file as "lv_port_indev.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_indev.h"
#include "lvgl.h"
#include "bsp_XPT2046.h"
#include "PowerMgr.h"

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void touchpad_init(void);
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
static bool touchpad_is_pressed(void);
static void touchpad_get_xy(lv_coord_t * x, lv_coord_t * y);

/**********************
 *  STATIC VARIABLES
 **********************/
lv_indev_t * indev_touchpad;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv;

    /*Initialize your touchpad if you have*/
    touchpad_init();

    /*Register a touchpad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    indev_touchpad = lv_indev_drv_register(&indev_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*Initialize your touchpad*/
static void touchpad_init(void)
{
    /*Your code comes here*/
}

/*Will be called by the library to read the touchpad*/
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    static lv_coord_t last_x = 0;
    static lv_coord_t last_y = 0;
    static uint8_t rel_cnt = 0;
    static uint8_t pressing = 0;

    /*Save the pressed coordinates and the state*/
    if(touchpad_is_pressed()) {
        rel_cnt = 0;
        PowerMgr_Feed();                          /* 有触摸操作,刷新息屏计时 */
        lv_coord_t nx, ny;
        touchpad_get_xy(&nx, &ny);
        if(pressing == 0U) {
            /* 按下沿:直接采用首坐标,保证点哪就是哪 */
            last_x = nx;
            last_y = ny;
            pressing = 1U;
        }
        else {
            /* 按住期间坐标死区 8px:电阻屏按住时坐标会持续微抖,
             * 抖动超过 LVGL 的滚动阈值会把点击误判成拖拽(点不中按钮)。
             * 位移小于 8px 时保持原坐标;真实滑动(>8px)仍能滚动列表 */
            lv_coord_t dx = (nx > last_x) ? (nx - last_x) : (last_x - nx);
            lv_coord_t dy = (ny > last_y) ? (ny - last_y) : (last_y - ny);
            if((dx >= 8) || (dy >= 8)) {
                last_x = nx;
                last_y = ny;
            }
        }
        data->state = LV_INDEV_STATE_PR;
    }
    else if(rel_cnt < 3U) {
        /* 释放滞回:电阻屏在按下沿/轻按时 IsPressed 会抖动返回"未按下",
         * 若直接上报会把一次点击拆成 press+release 闪烁导致点击丢失。
         * 连续 3 个采样周期(indev 默认 30ms 一读,约 90ms)都未按下
         * 才认为真正松开;期间保持按下状态与最后坐标 */
        rel_cnt++;
        data->state = LV_INDEV_STATE_PR;
    }
    else {
        data->state = LV_INDEV_STATE_REL;
        pressing = 0U;
    }

    /*Set the last pressed coordinates*/
    data->point.x = last_x;
    data->point.y = last_y;
}

/*Return true is the touchpad is pressed*/
static bool touchpad_is_pressed(void)
{
    /*Your code comes here*/
    return XPT2046_IsPressed();  /* XPT2046 中断脚;低:0-未按下,1-按下 */
}

/*Get the x and y coordinates if the touchpad is pressed*/
static void touchpad_get_xy(lv_coord_t * x, lv_coord_t * y)
{
    /*Your code comes here*/

    (*x) = XPT2046_GetX();
    (*y) = XPT2046_GetY();
    /* 参考例程此处有 LCD_DrawPoint 调试画点,已删除(避免在界面上留黑点) */
}

#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif
