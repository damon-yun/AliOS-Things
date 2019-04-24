/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 * Defines WWD SPI functions for STM32F4xx MCU
 */
#include "string.h" /* for memcpy */
#include "wifi_nvram_image.h"
#include "wwd_bus_protocol.h"
#include "wwd_assert.h"
//#include "wwd_rtos_isr.h"
//#include "wwd_platform_common.h"
#include "network/wwd_buffer_interface.h"
#include "platform/wwd_platform_interface.h"
#include "platform/wwd_bus_interface.h"
#include "platform/wwd_spi_interface.h"
#include "wwd_rtos.h"
#include "fsl_lpspi.h"
#include "e_sim_spi.h"
#include "MIMXRT1052.h"


//#include "platform_cmsis.h"
//#include "platform_config.h"
//#include "platform_peripheral.h"

/******************************************************
 *             Constants
 ******************************************************/

//#define DMA_TIMEOUT_LOOPS     (10000000)
//#define SPI_BUS_TX_DMA_STREAM (DMA1_Stream3_IRQn)

/******************************************************
 *             Structures
 ******************************************************/

/******************************************************
 *             Variables
 ******************************************************/

// static host_semaphore_type_t spi_transfer_finished_semaphore;

/******************************************************
 *             Static Function Declarations
 ******************************************************/

// static void spi_irq_handler( void* arg );
// void dma_irq( void );

/******************************************************
 *             Function definitions
 ******************************************************/

wwd_result_t host_platform_bus_init( void )
{
    wwd_result_t result = WWD_SUCCESS;
    
    return result;
}

wwd_result_t host_platform_bus_deinit( void )
{
    wwd_result_t result = WWD_SUCCESS;
    
    return result;
}
// static uint8_t null_buffer[1700]
wwd_result_t host_platform_spi_transfer( wwd_bus_transfer_direction_t dir, uint8_t* buffer, uint16_t buffer_length )
{
    wwd_result_t result = WWD_SUCCESS;
//    uint32_t junk;
//    uint32_t loop_count;
    e_sim_spi_trans(buffer, NULL, 4, false);
    if(BUS_READ == dir) {
        e_sim_spi_trans(NULL, buffer+4, buffer_length-4, true);
    } else if(BUS_WRITE == dir) {
        e_sim_spi_trans(buffer+4, NULL, buffer_length-4, true);
    }
    return result;
}

wwd_result_t host_platform_bus_enable_interrupt( void )
{
    wwd_result_t result = WWD_SUCCESS;
    
    EnableIRQ(GPIO3_Combined_0_15_IRQn);
    
    return  result;
}

wwd_result_t host_platform_bus_disable_interrupt( void )
{
    wwd_result_t result = WWD_SUCCESS;
    
    DisableIRQ(GPIO3_Combined_0_15_IRQn);
    
    return  result;
}

void host_platform_bus_buffer_freed( wwd_buffer_dir_t direction )
{
    UNUSED_PARAMETER( direction );
}

/******************************************************
 *             IRQ Handler definitions
 ******************************************************/

#define spi_irq_handler  GPIO3_Combined_0_15_IRQHandler
void spi_irq_handler( void )
{
    GPIO_PortClearInterruptFlags(WL_IRQ_PORT, 1 << WL_IRQ_PIN);
    wwd_thread_notify_irq();
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}



/******************************************************
 *             IRQ Handler Mapping
 ******************************************************/

//WWD_RTOS_MAP_ISR( dma_irq, DMA1_Stream3_irq )
