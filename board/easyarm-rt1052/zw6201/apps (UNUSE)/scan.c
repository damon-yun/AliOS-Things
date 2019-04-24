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
//#include "wiced.h"


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define DELAY_BETWEEN_SCANS       ( 5000 )

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct {
    wiced_semaphore_t   semaphore;      /* Semaphore used for signaling scan complete */
    uint32_t            result_count;   /* Count to measure the total scan results    */
} app_scan_data_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

wiced_result_t scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
//typedef uint32_t wiced_time_t;

#define AP_SSID  "TP-LINK_D82A"
#define AP_SEC    WICED_SECURITY_WPA2_AES_PSK
#define AP_PASS   "1234567890"

/* IP address configuration. */
#define configIP_ADDR0 192
#define configIP_ADDR1 168
#define configIP_ADDR2 1
#define configIP_ADDR3 230

/* Netmask configuration. */
#define configNET_MASK0 255
#define configNET_MASK1 255
#define configNET_MASK2 255
#define configNET_MASK3 0

/* Gateway address configuration. */
#define configGW_ADDR0 192
#define configGW_ADDR1 168
#define configGW_ADDR2 1
#define configGW_ADDR3 1
extern wiced_result_t wiced_join_ap_specific( wiced_ap_info_t* details, uint8_t security_key_length, const char security_key[ 64 ] );
extern void ping_init(const ip_addr_t* ping_addr);

static const wiced_ssid_t ap_ssid = {
    .length = sizeof(AP_SSID)-1,
    .value  = AP_SSID,
};



extern struct netif* wiced_ip_handle[ 2 ];
void application_start(void *arg )
{
    UNUSED_PARAMETER( arg );
    ip4_addr_t ipaddr, netmask, gw;

    wwd_result_t result;
    wiced_mac_t  mac;
    wiced_ap_info_t ap;



    IP4_ADDR(&ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
    IP4_ADDR(&netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
    IP4_ADDR(&gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);

    wiced_rtos_init( );
    wiced_wlan_connectivity_init();

    // 获取MAC地址
    wwd_wifi_get_mac_address(&mac, WWD_STA_INTERFACE);
    WPRINT_WICED_INFO(("MAC %x:%x:%x:%x:%x:%x\r\n", mac.octet[0],
                       mac.octet[1],
                       mac.octet[2],
                       mac.octet[3],
                       mac.octet[4],
                       mac.octet[5]));

    memset(&ap, 0, sizeof(ap));
    memcpy(ap.SSID.value, AP_SSID, sizeof(AP_SSID)-1);
    ap.SSID.length = sizeof(AP_SSID)-1;
    ap.security = AP_SEC;
    ap.bss_type = WICED_BSS_TYPE_INFRASTRUCTURE;
    do {
        result = wiced_join_ap_specific(&ap, sizeof(AP_PASS)-1, AP_PASS);
    } while(result != WWD_SUCCESS);
    netif_add(wiced_ip_handle[0],&ipaddr, &netmask, &gw, (void*) WWD_STA_INTERFACE, ethernetif_init, tcpip_input);
    netif_set_status_callback(&wiced_ip_handle[0], NULL);
    netif_set_up(wiced_ip_handle[0]);
    netif_set_default(wiced_ip_handle[0]);
    wiced_ip_handle[0]->hostname = "wiced";
    igmp_start(wiced_ip_handle[0]);
    ping_init(&gw);
    vTaskDelete(NULL);
}

