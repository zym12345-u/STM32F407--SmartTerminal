/* SdCardTask.c
 * TF/SD 卡延后初始化任务。
 *
 * 为什么单独开任务：HAL_SD_Init 在无卡或卡异常时会长时间轮询等待，
 * 若放在 main() 调度器启动之前，会直接卡住 GUI（黑屏）。
 * 这里用最低优先级 + 延时启动，保证 GUI/WiFi 先跑起来；
 * 初始化只做一次，结果打印串口，遥测里也带 sd/sdkb 字段。
 */
#include "cmsis_os2.h"

#include "bsp_SDCard.h"
#include "sdio.h"
#include "usart.h"
#include "ff.h"
#include <string.h>

#define SDCARD_INIT_DELAY_MS   3000U   /*先让 GUI/WiFi 稳定，再动 SDIO*/
#define SDCARD_TASK_STACK      1024U   /*字节；SD 轮询不用大缓冲，留足调用链*/

/**
  * 功    能：挂载失败时读扇区 0 做诊断：区分"读卡失败"与"卡里没有 FAT 文件系统"
  * 参    数：无
  * 返 回 值：无
  * 补    充：只打印关键字段，512B 缓冲放静态区，不占任务栈
  */
static void SdCard_Diag(void)
{
    static uint8_t buf[512];
    HAL_StatusTypeDef st;
    uint32_t lba;
    uint32_t i;

    st = HAL_SD_ReadBlocks(&hsd, buf, 0U, 1U, 5000U);
    UART_Printf("[SD] read0=%d\r\n", (int)st);
    if (st != HAL_OK)
    {
        return;
    }

    lba = (uint32_t)buf[446 + 8] |
          ((uint32_t)buf[446 + 9] << 8) |
          ((uint32_t)buf[446 + 10] << 16) |
          ((uint32_t)buf[446 + 11] << 24);

    UART_Printf("[SD] mbr=%02X%02X%02X%02X sig=%02X%02X ptype=0x%02X plba=%lu\r\n",
                buf[0], buf[1], buf[2], buf[3],
                buf[510], buf[511], buf[446 + 4], (unsigned long)lba);

    /*有 MBR 时再读分区首扇区，看 BPB 里有没有 FAT 标志*/
    if ((buf[510] == 0x55U) && (buf[511] == 0xAAU) && (buf[446 + 4] != 0U))
    {
        st = HAL_SD_ReadBlocks(&hsd, buf, lba, 1U, 5000U);
        UART_Printf("[SD] readP=%d sig=%02X%02X fat82=%c%c%c\r\n",
                    (int)st, buf[510], buf[511], buf[82], buf[83], buf[84]);
    }
    else
    {
        /*无 MBR：可能是 superfloppy，直接按引导扇区解释*/
        UART_Printf("[SD] no-mbr fat54=");
        for (i = 54U; i < 62U; i++)
        {
            UART_Printf("%c", (buf[i] >= 0x20U && buf[i] < 0x7FU) ? (char)buf[i] : '.');
        }
        UART_Printf(" fat82=");
        for (i = 82U; i < 90U; i++)
        {
            UART_Printf("%c", (buf[i] >= 0x20U && buf[i] < 0x7FU) ? (char)buf[i] : '.');
        }
        UART_Printf("\r\n");
    }
}

/**
  * 功    能：最小文件读写自检：写一个文件再读回比对
  * 参    数：无
  * 返 回 值：无
  * 补    充：文件名固定 8.3 格式，避免依赖长文件名；结果打印 rw=1 为通过
  */
static void SdCard_RwTest(void)
{
    static FIL  f;
    static char wbuf[24] = "stm32 fatfs rw test";
    char        rbuf[24];
    UINT        bw = 0U;
    UINT        br = 0U;
    UINT        len;
    FRESULT     fr;
    uint8_t     ok = 0U;

    (void)memset(rbuf, 0, sizeof(rbuf));
    (void)memset(&f, 0, sizeof(f));
    len = (UINT)strlen(wbuf);

    fr = f_open(&f, "0:/sdt.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (fr == FR_OK)
    {
        fr = f_write(&f, wbuf, len, &bw);
        (void)f_close(&f);

        if ((fr == FR_OK) && (bw == len))
        {
            fr = f_open(&f, "0:/sdt.txt", FA_READ);
            if (fr == FR_OK)
            {
                fr = f_read(&f, rbuf, len, &br);
                (void)f_close(&f);

                if ((fr == FR_OK) && (br == len) && (memcmp(wbuf, rbuf, len) == 0))
                {
                    ok = 1U;
                }
            }
        }
    }

    UART_Printf("[SD] rw=%u wr=%u rd=%u fres=%u\r\n",
                (unsigned)ok, (unsigned)bw, (unsigned)br, (unsigned)fr);
}

/**
  * 功    能：延后初始化 TF 卡并打印结果，之后常驻空转
  * 参    数：argument 未使用
  * 返 回 值：无
  */
static void StartSdCardTask(void *argument)
{
    uint8_t ret;

    (void)argument;

    (void)osDelay(SDCARD_INIT_DELAY_MS);

    ret = SDCard_Init();
    UART_Printf("[SD] init=%u fres=%u ready=%u bus=%u cap=%luKB\r\n",
                (unsigned)ret,
                (unsigned)SDCard_GetMountResult(),
                (unsigned)SDCard_IsReady(),
                (unsigned)SDCard_GetBusWidth(),
                (unsigned long)SDCard_GetCapacityKB());

    if (ret == SDCARD_OK)
    {
        SdCard_RwTest();
    }
    else
    {
        UART_Printf("[SD] type=0x%08lX err=0x%08lX clkdiv=%lu\r\n",
                    (unsigned long)hsd.SdCard.CardType,
                    (unsigned long)hsd.ErrorCode,
                    (unsigned long)hsd.Init.ClockDiv);
        SdCard_Diag();
    }

    for (;;)
    {
        (void)osDelay(10000U);
    }
}

/**
  * 功    能：创建 TF 卡初始化任务，由 freertos.c 启动时调一次
  * 参    数：无
  * 返 回 值：无
  */
void SDCardTask_Create(void)
{
    static const osThreadAttr_t attr = {
        .name       = "SdCardTask",
        .stack_size = SDCARD_TASK_STACK,
        .priority   = (osPriority_t)osPriorityLow,
    };
    (void)osThreadNew(StartSdCardTask, NULL, &attr);
}
