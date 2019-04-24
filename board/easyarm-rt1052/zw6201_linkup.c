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
 *
 * Scan Application
 *
 * Features demonstrated
 *  - WICED scan API
 *
 * This application snippet regularly scans for nearby Wi-Fi access points
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 *   Each time the application scans, a list of Wi-Fi access points in
 *   range is printed to the UART
 *
 */

#include <stdlib.h>
#include <stdint.h>
#include "rtos.h"
#include "wiced_wifi.h"
#include "wwd_assert.h"
#include "wiced_rtos_common.h"
#include <string.h>
#include "lwip/ip4_addr.h"
#include "wwd_network.h"
#include "netif/ethernet.h"
#include "lwip/tcpip.h"
#include "wiced_time.h"
#include "wwd_structures.h"
#include "lwip/netif.h"
#include "lwip/igmp.h"
//lwip dhcp include
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"
//alios things include
#include <aos/kernel.h>
#include <aos/yloop.h>
#include <ulog/ulog.h>
#include <k_api.h>

#include "e_sim_spi.h"

#define TAG "zw6201"
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
 
#define AP_SSID  "Bo_2301"                 // 待连接wifi名称（SSID）
#define AP_SEC    WICED_SECURITY_WPA2_AES_PSK   // 加密方式
#define AP_PASS   "Yun201167."                  // wifi密码

/* 本机IP地址 */
#define configIP_ADDR0 192
#define configIP_ADDR1 168
#define configIP_ADDR2 0
#define configIP_ADDR3 230

/* 子网掩码  */
#define configNET_MASK0 255
#define configNET_MASK1 255
#define configNET_MASK2 255
#define configNET_MASK3 0

/* 网关地址 (一般为路由器地址) */
#define configGW_ADDR0 192
#define configGW_ADDR1 168
#define configGW_ADDR2 0
#define configGW_ADDR3 1

/* 被ping主机地址 */
#define configTarget_ADDR0  192
#define configTarget_ADDR1  168
#define configTarget_ADDR2  0
#define configTarget_ADDR3  1

extern wiced_result_t wiced_join_ap_specific( wiced_ap_info_t* details, uint8_t security_key_length, const char security_key[ 64 ] );

extern struct netif* wiced_ip_handle[ 2 ];


static void tcpip_dhcpc_cb(struct netif *pstnetif) {
    struct dhcp *dhcp = (struct dhcp *)netif_get_client_data(pstnetif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);
    if (dhcp->state == DHCP_STATE_BOUND)
    {
        LOGI(TAG," IPv4 Address     : %s\r\n", ipaddr_ntoa(&pstnetif->ip_addr));
        LOGI(TAG," IPv4 Subnet mask : %s\r\n", ipaddr_ntoa(&pstnetif->netmask));
        LOGI(TAG," IPv4 Gateway     : %s\r\n", ipaddr_ntoa(&pstnetif->gw));

        aos_post_event(EV_WIFI, CODE_WIFI_ON_GOT_IP, 0xdeaddead);
    }
    // aos_msleep(2000);
    return;
}

err_t tcpip_dhcpc_start(struct netif *pstnetif)
{
    if (NULL == pstnetif){
        LOG("%s input netif is NULL \r\n");
        return -1;
    }

    if (netif_is_up(pstnetif)) {
        if (dhcp_start(pstnetif) != ERR_OK) {
            LOG("dhcp client start failed");
            return -1;
        }
    }

    netif_set_status_callback(pstnetif, tcpip_dhcpc_cb);
}

static void zw6201_service_event(input_event_t *event, void *priv_data)
{
    ip4_addr_t ipaddr, netmask, gw, target_ip;

    if (event->type != EV_LAN) {
        return;
    }


    if (event->value == VALUE_LAN_IN) {

#if 0
        IP4_ADDR(&ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
        IP4_ADDR(&netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
        IP4_ADDR(&gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);
        IP4_ADDR(&target_ip, configTarget_ADDR0,configTarget_ADDR1,configTarget_ADDR2,configTarget_ADDR3);
#else
        IP4_ADDR(&ipaddr, 0u, 0u, 0u, 0u);
        IP4_ADDR(&netmask, 0u, 0u, 0u, 0u);
        IP4_ADDR(&gw, 0u, 0u, 0u, 0u);
#endif
        netif_add(wiced_ip_handle[0],&ipaddr, &netmask, &gw, (void*) WWD_STA_INTERFACE, ethernetif_init, tcpip_input);
        netif_set_status_callback(wiced_ip_handle[0], NULL);
        netif_set_default(wiced_ip_handle[0]);
        netif_set_up(wiced_ip_handle[0]);
        //start dhcp
        tcpip_dhcpc_start(wiced_ip_handle[0]);

        igmp_start(wiced_ip_handle[0]);

    } else {

    }

    return;
}


static void __delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 80000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

static void __zw6201_pin_init (void)
{
    /* Define the init structure for the output phy reset pin*/
    gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    BOARD_InitWifiPins();
    
    GPIO_PinInit(WL_REG_ON_PORT, WL_REG_ON_PIN, &gpio_config);
    GPIO_WritePinOutput(WL_REG_ON_PORT, WL_REG_ON_PIN, 1);

    e_sim_spi_init();
    for(int i = 0; i < 100; i++) {
        __delay();
    }
    GPIO_WritePinOutput(WL_REG_ON_PORT, WL_REG_ON_PIN, 1);
    gpio_config.direction = kGPIO_DigitalInput;
    gpio_config.outputLogic = 0;
    gpio_config.interruptMode = kGPIO_IntRisingEdge;

    //NVIC_SetPriority(GPIO3_Combined_0_15_IRQn, 3);
    DisableIRQ(GPIO3_Combined_0_15_IRQn);
    GPIO_PinInit(WL_IRQ_PORT, WL_IRQ_PIN,&gpio_config);
    /* Enable GPIO pin interrupt */
    GPIO_PortEnableInterrupts(WL_IRQ_PORT, 1U << WL_IRQ_PIN);
}

void zw6201_init_task(void *arg)
{
    UNUSED_PARAMETER( arg );
    ip4_addr_t ipaddr, netmask, gw, target_ip;

    wiced_result_t result;
    wiced_mac_t  mac;
    wiced_ap_info_t ap;

    __zw6201_pin_init();
    
    wiced_rtos_init();
    wiced_wlan_connectivity_init();

    // get mac addr
    wwd_wifi_get_mac_address(&mac, WWD_STA_INTERFACE);
    WPRINT_WICED_INFO(("MAC %x:%x:%x:%x:%x:%x\r\n", mac.octet[0],
                       mac.octet[1],
                       mac.octet[2],
                       mac.octet[3],
                       mac.octet[4],
                       mac.octet[5]));

    aos_register_event_filter(EV_LAN, zw6201_service_event, NULL);
#if 0
    memset(&ap, 0, sizeof(ap));
    memcpy(ap.SSID.value, AP_SSID, sizeof(AP_SSID)-1);
    ap.SSID.length = sizeof(AP_SSID)-1;
    ap.security = AP_SEC;
    ap.bss_type = WICED_BSS_TYPE_INFRASTRUCTURE;
    do {
        result = wiced_join_ap_specific(&ap, sizeof(AP_PASS)-1, AP_PASS);
    } while(result != WWD_SUCCESS);

#if 0
    IP4_ADDR(&ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
    IP4_ADDR(&netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
    IP4_ADDR(&gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);
    IP4_ADDR(&target_ip, configTarget_ADDR0,configTarget_ADDR1,configTarget_ADDR2,configTarget_ADDR3);
#else
    IP4_ADDR(&ipaddr, 0u, 0u, 0u, 0u);
    IP4_ADDR(&netmask, 0u, 0u, 0u, 0u);
    IP4_ADDR(&gw, 0u, 0u, 0u, 0u);
#endif
    netif_add(wiced_ip_handle[0],&ipaddr, &netmask, &gw, (void*) WWD_STA_INTERFACE, ethernetif_init, tcpip_input);
    netif_set_status_callback(wiced_ip_handle[0], NULL);
    netif_set_default(wiced_ip_handle[0]);
    netif_set_up(wiced_ip_handle[0]);
    //start dhcp
    tcpip_dhcpc_start(wiced_ip_handle[0]);

    igmp_start(wiced_ip_handle[0]);
#endif
    aos_task_exit(NULL);
}

