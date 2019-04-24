/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */


#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <k_api.h>
#include <ulog/ulog.h>
#include <aos/hal/timer.h>
#include <aos/hal/uart.h>
#include <network/hal/wifi.h>
#include "aos/init.h"
#include "aos/kernel.h"

#include "board.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_lpi2c.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
 
#include "fsl_spinor.h"

#if (__WICED__ == 1)
//zw6201 include
#include "wifi_spi.h"
#endif

#ifdef TINY_ENGINE
#define AOS_START_STACK 1536 + 1024*14
#else
#define AOS_START_STACK 1536
#endif

ktask_t *g_aos_init;
cpu_stack_t g_aos_init_stack[AOS_START_STACK];
ktask_t *g_aos_app = NULL;

static kinit_t kinit;
extern int application_start(int argc, char **argv);

extern uart_dev_t uart_1;

static void var_init()
{
    kinit.argc = 0;
    kinit.argv = NULL;
    kinit.cli_enable = 1;
}


#if (__WICED__ == 1)
static int init_wifi_zw6201(void)
{
   int ret;

   extern hal_wifi_module_t aos_wifi_module_zw6201;
   hal_wifi_register_module(&aos_wifi_module_zw6201);    

   ret = hal_wifi_init();
   PRINTF("hal_wifi_init return %d\r\n", ret);
}
#endif


static void sys_init(void)
{
    zw6201_init_task(NULL);

    init_wifi_zw6201();

    var_init();    
    aos_components_init(&kinit);
    application_start(kinit.argc, kinit.argv);  /* jump to app/example entry */
}


static void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 80000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

static void BOARD_InitModuleClock(void)
{
    const clock_enet_pll_config_t config = {true, false, 1};
    CLOCK_InitEnetPll(&config);
}

static void platform_init(void)
{
    /* Define the init structure for the output phy reset pin*/
    gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
    /* Board pin, clock, debug console init */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitModuleClock();

    /* LPI2C1 Pins init */
    BOARD_InitSensorPins();

    BOARD_InitUart56Pins();

    /* SPI Nor Pins init */
    BOARD_FlexspiInitPins();
    flexspi_nor_init();
    
#if   0  //for enet1
    /* Enable ENET MCLK OUT 50MHz */
    IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true);

    GPIO_PinInit(GPIO1, 9, &gpio_config);
    GPIO_PinInit(GPIO1, 10, &gpio_config);
    /* pull up the ENET_INT before RESET. */
    GPIO_WritePinOutput(GPIO1, 10, 0);
    GPIO_WritePinOutput(GPIO1, 9, 0);
    delay();
    GPIO_WritePinOutput(GPIO1, 9, 1);
#endif
    //Board led init
    board_led_init();

}


#define us2tick(us) \
    ((us * RHINO_CONFIG_TICKS_PER_SECOND + 999999) / 1000000)


void hal_reboot(void)
{
    NVIC_SystemReset();
}


static void _timer_cb(void *timer, void *arg)
{
    timer_dev_t *tmr = arg;
    tmr->config.cb(tmr->config.arg);
}

int32_t hal_timer_init(timer_dev_t *tim)
{
    if (tim->config.reload_mode == TIMER_RELOAD_AUTO) {
        krhino_timer_dyn_create((ktimer_t **)&tim->priv, "hwtmr", _timer_cb,
                                us2tick(tim->config.period), us2tick(tim->config.period), tim, 0);
    }
    else {
        krhino_timer_dyn_create((ktimer_t **)&tim->priv, "hwtmr", _timer_cb,
                                us2tick(tim->config.period), 0, tim, 0);
    }
}

int32_t hal_timer_start(timer_dev_t *tmr)
{
    return krhino_timer_start(tmr->priv);
}


void hal_timer_stop(timer_dev_t *tmr)
{
    krhino_timer_stop(tmr->priv);
    krhino_timer_dyn_del(tmr->priv);
    tmr->priv = NULL;
}


void hw_start_hal(void)
{
    PRINTF("start-----------hal\n");
}

int main(void)
{

    platform_init();
   
    aos_init();

    /* 使能Fault中断 */
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | /* 使能usage fault */
                 SCB_SHCSR_BUSFAULTENA_Msk | /* 使能Bus Fault */
                 SCB_SHCSR_MEMFAULTENA_Msk;  /* 使能Mem Manage Fault */

    hal_uart_init(&uart_1);

    //krhino_task_dyn_create(&g_aos_app, "aos-init", 0, AOS_DEFAULT_APP_PRI, 0, AOS_START_STACK, (task_entry_t)sys_init, 1);
    krhino_task_create(&g_aos_init, "aos-init", 0,AOS_DEFAULT_APP_PRI, 
        0, g_aos_init_stack, AOS_START_STACK, (task_entry_t)sys_init, 1);

    SysTick_Config(SystemCoreClock / RHINO_CONFIG_TICKS_PER_SECOND);

    aos_start();

   return 0;
}

void HardFault_Handler(void)
{
    LOG("HardFault_Handler\r\n");    
    while (1)
    {

    }
}

void MemManage_Handler(void)
{
    LOG("MemManage_Fault_Handler\r\n");
    while(1){

    }
}

void BusFault_Handler(void)
{
    LOG("BusFault_Handler\r\n");
    while(1){

    }
}

void UsageFault_Handler(void)
{
    LOG("UsageFault_Handler");
    while(1){

    }
}

