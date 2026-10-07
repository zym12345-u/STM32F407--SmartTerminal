/**
  ******************************************************************************
  * @file    SensorData.h
  * @brief   传感器采集任务与显示任务之间的共享数据结构
  *
  *          架构说明:
  *            SensorTask 负责周期采集 BH1750(光照) 与 DHT11(温湿度),
  *            结果写入本文件定义的 g_sensorData;
  *            GuiTask 通过 LVGL 定时器周期读取 g_sensorData 并刷新屏幕标签。
  *
  *          数据一致性:
  *            float 在 Cortex-M4 上单次读写并非严格原子,但显示场景对
  *            撕裂读容忍度高(最多显示一帧半旧半新),无需加互斥锁;
  *            is_valid 为 uint8_t,读写原子。
  ******************************************************************************
  */
#ifndef __SENSOR_DATA_H
#define __SENSOR_DATA_H

#include <stdint.h>


typedef struct
{
    volatile float    temperature;   /* 温度, °C      */
    volatile float    humidity;      /* 湿度, %RH     */
    volatile float    light;         /* 光照, lux     */
    volatile uint8_t  temp_valid;    /* 温湿度数据有效标志 */
    volatile uint8_t  light_valid;   /* 光照数据有效标志 */
} SensorData_t;


/* 全局共享传感器数据,由 SensorTask 写入,GuiTask 读取 */
extern SensorData_t g_sensorData;


#endif /* __SENSOR_DATA_H */
