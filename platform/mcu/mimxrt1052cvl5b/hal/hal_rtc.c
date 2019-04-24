/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <k_api.h>
#include <stdio.h>
#include <stdlib.h>
#include "aos/hal/rtc.h"

#include "MIMXRT1052.h"
#include "fsl_snvs_hp.h"
#include "fsl_common.h"
#include "board.h"


//#define RTC_START_YEAR (1970)
/**@brief This function will initialize the on board CPU real time clock
 *
 * @note  This function should be called by MICO system when initializing clocks, so
 *        It is not needed to be called by application
 * @param     rtc   : rtc device
 * @return    0     : on success.
 * @return    EIO   : if an error occurred with any step
 */
int32_t hal_rtc_init(rtc_dev_t *rtc)
{
    snvs_hp_rtc_config_t snvsRtcConfig;
    snvs_hp_rtc_datetime_t rtcDate;
   
	/* Init SNVS */
    /*
     * snvsConfig->rtccalenable = false;
     * snvsConfig->rtccalvalue = 0U;
     * snvsConfig->srtccalenable = false;
     * snvsConfig->srtccalvalue = 0U;
     * snvsConfig->PIFreq = 0U;
     */
    SNVS_HP_RTC_GetDefaultConfig(&snvsRtcConfig); /* 不启用RTC校准 */
	//snvsRtcConfig.periodicInterruptFreq = 15;
    SNVS_HP_RTC_Init(SNVS, &snvsRtcConfig);


    /* Set a start date time and start RT */
    rtcDate.year = 2018U; /* 注意需要转换范围 */
    rtcDate.month = 9U;
    rtcDate.day = 20U;
    rtcDate.hour = 19U;
    rtcDate.minute = 40;
    rtcDate.second = 0;

    /* 配置 RTC 时间，启动 RTC */
    //RHINO_CRITICAL_ENTER();
    SNVS_HP_RTC_SetDatetime(SNVS, &rtcDate);
    SNVS_HP_RTC_StartTimer(SNVS);
    //RHINO_CRITICAL_EXIT();
}

/**@brief This function will return the value of time read from the on board CPU real time clock. Time value must be given in the format of
 * the structure hal_rtc_time_t
 * @param     rtc  : rtc device
 * @param     time : pointer to a time structure
 *
 * @return    0    : on success.
 * @return    EIO  : if an error occurred with any step
 */
int32_t hal_rtc_get_time(rtc_dev_t *rtc, rtc_time_t *time)
{
    snvs_hp_rtc_datetime_t datetime;
    if (time != NULL) {
        //RHINO_CRITICAL_ENTER();
        SNVS_HP_RTC_GetDatetime(SNVS, &datetime);
        //RHINO_CRITICAL_EXIT();
        if (rtc->config.format == HAL_RTC_FORMAT_DEC) {
            /* 转换为十进制 */
            time->year  = datetime.year % 100; /* DEC format:value range from 0 to 99 */
            time->month = datetime.month;      /* DEC format:value range from 1 to 12 */
            time->date  = datetime.day;        /* DEC format:value range from 1 to 31 */
            time->hr    = datetime.hour;       /* DEC format:value range from 0 to 23 */
            time->min   = datetime.minute;     /* DEC format:value range from 0 to 59 */
            time->sec   = datetime.second;     /* DEC format:value range from 0 to 59 */
                    

        } else {
            /* 转换为BCD  */
            time->year  = /* BCD format:value range from 0x0 to 0x99 */
                          (datetime.year % 100 % 10) + ((datetime.year % 100 / 10) << 4);
            time->month = /* BCD format:value range from 0x1 to 0x12 */
                          (datetime.month % 10) + ((datetime.month / 10) << 4);
            time->date  = /* BCD format:value range from 0x1 to 0x31 */
                          (datetime.day % 10) + ((datetime.day / 10) << 4);
            time->hr    = /* BCD format:value range from 0x0 to 0x23 */
                          (datetime.hour % 10) + ((datetime.hour / 10) << 4);
            time->min   = /* BCD format:value range from 0x0 to 0x59 */
                          (datetime.minute % 10) + ((datetime.minute / 10) << 4); 
            time->sec   = /* BCD format:value range from 0x0 to 0x59 */
                          (datetime.second % 10) + ((datetime.second / 10) << 4);    
        }
        return 0; 
    
    } else {
        return -1;
    }
}

/**@brief This function will set MCU RTC time to a new value. Time value must be given in the format of
 * the structure hal_rtc_time_t
 * @param     rtc    : rtc device
 * @param     time   : pointer to a time structure
 *
 * @return    0      : on success.
 * @return    EIO    : if an error occurred with any step
 */
int32_t hal_rtc_set_time(rtc_dev_t *rtc, const rtc_time_t *time)
{  
    snvs_hp_rtc_datetime_t datetime; /* 需要和time转换一下 */
    if (time != NULL ) {
        if (rtc->config.format == HAL_RTC_FORMAT_DEC) {
            /* 十进制设置 */
            datetime.year   =                /* DEC format:value range from 0 to 99 */
                              2000 + time->year;//(datetime.year / 100 * 100) + time->year; 
            datetime.month  = time->month;   /* DEC format:value range from 1 to 12 */
            datetime.day    = time->date;    /* DEC format:value range from 1 to 31 */
            datetime.hour   = time->hr;      /* DEC format:value range from 0 to 23 */
            datetime.minute = time->min;     /* DEC format:value range from 0 to 59 */
            datetime.second = time->sec;     /* DEC format:value range from 0 to 59 */
                    

        } else {
            /* BCD设置  */
            datetime.year   = /* BCD format:value range from 0x0 to 0x99 */
                              2000 + (time->year % 16) + (time->year / 16) * 10; 
            datetime.month  = /* BCD format:value range from 0x1 to 0x12 */
                              (time->month % 16) + (time->month / 16) * 10;
            datetime.day    = /* BCD format:value range from 0x1 to 0x31 */
                              (time->date % 16) + (time->date / 16) * 10;
            datetime.hour   = /* BCD format:value range from 0x0 to 0x23 */
                              (time->hr % 16) + (time->hr / 16) * 10;
            datetime.minute = /* BCD format:value range from 0x0 to 0x59 */
                              (time->min % 16) + (time->min / 16) * 10; 
            datetime.second = /* BCD format:value range from 0x0 to 0x59 */
                              (time->sec % 16) + (time->sec / 16) * 10;    
        }
        SNVS_HP_RTC_SetDatetime(SNVS, &datetime);
        return 0; 
    
    } else {
        return -1;
    }
}