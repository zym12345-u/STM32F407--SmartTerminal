/* SensorData.h
 * 传感器数据结构：SensorTask 写，GUI/Modbus/CAN 读，环境值以 float 存物理量。
 */
/*float 单次读写非严格原子，显示容忍一帧半新半旧，不加锁；temp_valid 同时代表温湿度，断开时保留旧值只清有效位*/
#ifndef __SENSOR_DATA_H
#define __SENSOR_DATA_H

#include <stdint.h>


/*传感器快照：温度 °C，湿度 %RH，光照 lux；SensorTask 写，GUI/Modbus/CAN 读*/
typedef struct
{
    volatile float    temperature;   /*温度 °C，DHT11 量程 0~50*/
    volatile float    humidity;      /*湿度 %RH，0~100*/
    volatile float    light;         /*光照 lux，0~65535*/
    volatile uint8_t  temp_valid;    /*温湿度有效：0=无效 1=有效*/
    volatile uint8_t  light_valid;   /*光照有效：0=无效 1=有效*/
} SensorData_t;


/*全局共享数据，SensorTask 写，其余读*/
extern SensorData_t g_sensorData;


#endif /* __SENSOR_DATA_H */
