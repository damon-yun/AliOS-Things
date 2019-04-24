/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include <k_api.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include <aos/hal/gpio.h>
#include "fsl_common.h"
#include "fsl_gpio.h"

int32_t hal_gpio_enable_irq(gpio_dev_t *gpio, gpio_irq_trigger_t trigger,
                            gpio_irq_handler_t handler, void *arg)
{
    int32_t ret = -1;
    
    return ret;
}


int32_t hal_gpio_init(gpio_dev_t *gpio)
{
    int32_t ret = 0;
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Init output LED GPIO. */
    GPIO_PinInit(GPIO1, 19, &led_config);

    return ret;
}

int32_t hal_gpio_output_high(gpio_dev_t *gpio)
{
    int32_t ret = 0;

    GPIO_PinWrite(GPIO1, 19, 1);

    return ret;
}

int32_t hal_gpio_output_low(gpio_dev_t *gpio)
{
    int32_t ret = 0;

    GPIO_PinWrite(GPIO1, 19, 0);

    return ret;
}

int32_t hal_gpio_output_toggle(gpio_dev_t *gpio)
{
    int32_t ret = 0;

    GPIO_PortToggle(GPIO1, 1u << 19);

    return ret;
}

int32_t hal_gpio_input_get(gpio_dev_t *gpio, uint32_t *value)
{

    int32_t ret = 0;


    return ret;
}




