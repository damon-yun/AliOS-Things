/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */
//alios things include
#include <aos/kernel.h>
#include <aos/yloop.h>
#include <k_api.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ulog/ulog.h>
#include <network/hal/base.h>
#include <network/hal/wifi.h>

//ze6201 include
#include "wiced_wifi.h"
#include "wwd_wifi.h"
#include "wwd_structures.h" //wiced_mac_t
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "wwd_buffer_interface.h"
#include "wwd_bus_protocol_interface.h"

#define TAG "wifi_port"

extern struct netif* wiced_ip_handle[ 2 ];

static int wifi_init(hal_wifi_module_t *m)
{
    LOGI(TAG, "wifi init success!!\n");
    return 0;
};

// mac - hex[6]
static void wifi_get_mac_addr(hal_wifi_module_t *m, uint8_t *mac)
{
    wiced_mac_t  wiced_mac;
    int i = 0;
    // 获取MAC地址
    wwd_wifi_get_mac_address(&wiced_mac, WWD_STA_INTERFACE);
    
    for (i = 0; i < 6; i++) {
        mac[i] = wiced_mac.octet[i];
    }
    // LOGI(TAG, "mac in hex: %02x%02x%02x%02x%02x%02x",
    //   mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static int get_ip_stat(hal_wifi_module_t *m, hal_wifi_ip_stat_t *out_net_para, hal_wifi_type_t wifi_type)
{
    (void)wifi_type;
    (void)m;
    ip_addr_t* dns;

    LOGI(TAG, "[WIFI] Get IP");

    dns = dns_getserver(0);

#if LWIP_IPV4 && LWIP_IPV6
    sprintf(out_net_para->ip, "%u.%u.%u.%u", \
            ((u8_t *)&(wiced_ip_handle[0])->ip_addr.u_addr.ip4)[0],\
            ((u8_t *)&(wiced_ip_handle[0])->ip_addr.u_addr.ip4)[1],\
            ((u8_t *)&(wiced_ip_handle[0])->ip_addr.u_addr.ip4)[2],\
            ((u8_t *)&(wiced_ip_handle[0])->ip_addr.u_addr.ip4)[3]);
    sprintf(out_net_para->mask, "%u.%u.%u.%u", \
            ((u8_t *)&(wiced_ip_handle[0])->netmask.u_addr.ip4)[0],\
            ((u8_t *)&(wiced_ip_handle[0])->netmask.u_addr.ip4)[1],\
            ((u8_t *)&(wiced_ip_handle[0])->netmask.u_addr.ip4)[2],\
            ((u8_t *)&(wiced_ip_handle[0])->netmask.u_addr.ip4)[3]);
    sprintf(out_net_para->gate, "%u.%u.%u.%u", \
            ((u8_t *)&(wiced_ip_handle[0])->gw.u_addr.ip4)[0],\
            ((u8_t *)&(wiced_ip_handle[0])->gw.u_addr.ip4)[1],\
            ((u8_t *)&(wiced_ip_handle[0])->gw.u_addr.ip4)[2],\
            ((u8_t *)&(wiced_ip_handle[0])->gw.u_addr.ip4)[3]);
    sprintf(out_net_para->dns, "%u.%u.%u.%u", \
            ((u8_t *)&dns->u_addr.ip4)[0],\
            ((u8_t *)&dns->u_addr.ip4)[1],\
            ((u8_t *)&dns->u_addr.ip4)[2],\
            ((u8_t *)&dns->u_addr.ip4)[3]);
#else
#if LWIP_IPV4
    sprintf(out_net_para->ip, "%u.%u.%u.%u", \
            ((u8_t *)&(wiced_ip_handle[0])->ip_addr.addr)[0],\
            ((u8_t *)&(wiced_ip_handle[0])->ip_addr.addr)[1],\
            ((u8_t *)&(wiced_ip_handle[0])->ip_addr.addr)[2],\
            ((u8_t *)&(wiced_ip_handle[0])->ip_addr.addr)[3]);
    sprintf(out_net_para->mask, "%u.%u.%u.%u", \
            ((u8_t *)&(wiced_ip_handle[0])->netmask.addr)[0],\
            ((u8_t *)&(wiced_ip_handle[0])->netmask.addr)[1],\
            ((u8_t *)&(wiced_ip_handle[0])->netmask.addr)[2],\
            ((u8_t *)&(wiced_ip_handle[0])->netmask.addr)[3]);
    sprintf(out_net_para->gate, "%u.%u.%u.%u", \
            ((u8_t *)&(wiced_ip_handle[0])->gw.addr)[0],\
            ((u8_t *)&(wiced_ip_handle[0])->gw.addr)[1],\
            ((u8_t *)&(wiced_ip_handle[0])->gw.addr)[2],\
            ((u8_t *)&(wiced_ip_handle[0])->gw.addr)[3]);
    sprintf(out_net_para->dns, "%u.%u.%u.%u", \
            ((u8_t *)&dns->addr)[0],\
            ((u8_t *)&dns->addr)[1],\
            ((u8_t *)&dns->addr)[2],\
            ((u8_t *)&dns->addr)[3]);
#endif //LWIP_IPV4
#endif //LWIP_IPV4 && LWIP_IPV6

    LOGD(TAG, "result: %s %s %s %s\r\n",
      out_net_para->ip, out_net_para->mask, out_net_para->gate, out_net_para->dns);
    return 0;
}

//TODO:
void at_wevent_handler(void *arg, char *buf, int buflen)
{
    hal_wifi_module_t *m;
    
    if (NULL == arg){
        m = hal_wifi_get_default_module();
    } else {
        m = (hal_wifi_module_t *)arg;
    }

    if (NULL == m) {
        return;
    }
    
    if (m->ev_cb->stat_chg != NULL) {
        m->ev_cb->stat_chg(m, NOTIFY_STATION_UP, NULL);
    }

    fetch_ip_stat(m);
}

static int wifi_start(hal_wifi_module_t *m, hal_wifi_init_type_t *init_para)
{
    (void)init_para;
    wiced_result_t result;
    wiced_ap_info_t ap;

    LOGI(TAG, "[WIFI] wifi start");

    memset(&ap, 0, sizeof(ap));
    ap.SSID.length = sprintf(ap.SSID.value, "%s", init_para->wifi_ssid);

    if (strcmp(init_para->wifi_key, "open") == 0) {
        ap.security = WICED_SECURITY_OPEN;
    } else {
        ap.security = WICED_SECURITY_WPA2_AES_PSK;
    }
    ap.bss_type = WICED_BSS_TYPE_INFRASTRUCTURE;

    result = wiced_join_ap_specific(&ap, strlen(init_para->wifi_key), init_para->wifi_key);

    if (result != WWD_SUCCESS) {
        return -1;
    }

    aos_post_event(EV_LAN,1,VALUE_LAN_IN);

    return 0;
}

static int wifi_start_adv(hal_wifi_module_t *m, hal_wifi_init_type_adv_t *init_para_adv)
{
    (void)init_para_adv;
    LOGI(TAG, "[WIFI] wifi start adv");
    return 0;
}



static int get_link_stat(hal_wifi_module_t *m, hal_wifi_link_stat_t *out_stat)
{
    LOGI(TAG, "[WIFI] wifi get link stat");
    return 0;
}

hal_wifi_scan_result_t result;
uint32_t wiced_scan_index = 0;

wiced_result_t wiced_scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result )
{
    if (wiced_scan_index <= 9) {
        memset(result.ap_list[wiced_scan_index].ssid, 0, sizeof(result.ap_list[wiced_scan_index].ssid));

        memcpy(result.ap_list[wiced_scan_index].ssid, malloced_scan_result->ap_details.SSID.value,
                                        malloced_scan_result->ap_details.SSID.length);
        result.ap_list[wiced_scan_index].ap_power = (0 - malloced_scan_result->ap_details.signal_strength);
        LOGI("#%-2d: ssid: %-32.32s ap_power: %d\n", wiced_scan_index+1,
                                    result.ap_list[wiced_scan_index].ssid, result.ap_list[wiced_scan_index].ap_power);
        wiced_scan_index++;
    }

    if (malloced_scan_result->status == WICED_SCAN_INCOMPLETE) {
        hal_wifi_module_t *m = hal_wifi_get_default_module();

        if (m->ev_cb != NULL && m->ev_cb->scan_compeleted != NULL) {
            m->ev_cb->scan_compeleted(m, &result, NULL);
        }
        free(result.ap_list);
        wiced_scan_index = 0;
    }

}

static void start_scan(hal_wifi_module_t *m)
{
    LOGI(TAG, "[WIFI] start_scan");

    wiced_wifi_scan_enable();
    wiced_wifi_scan_networks(wiced_scan_result_handler,NULL);
}

static void start_scan_adv(hal_wifi_module_t *m)
{
    LOGI(TAG, "[WIFI] start_scan_adv");

    wiced_wifi_scan_enable();
    wiced_wifi_scan_networks(wiced_scan_result_handler,NULL);
    result.ap_num = 10;
    result.ap_list = malloc(10 * sizeof(*(result.ap_list)));
    wiced_scan_index = 0;
}

static int power_off(hal_wifi_module_t *m)
{
    return 0;
}

static int power_on(hal_wifi_module_t *m)
{
    return 0;
}

static int suspend(hal_wifi_module_t *m)
{
    return 0;
}

static int suspend_station(hal_wifi_module_t *m)
{
    LOGI(TAG, "[WIFI] suspend_station");
    return 0;
}

static int suspend_soft_ap(hal_wifi_module_t *m)
{

    return 0;
}

static int set_channel(hal_wifi_module_t *m, int ch)
{
    (void)m;

    wwd_wifi_set_channel(WWD_STA_INTERFACE,ch);
    return 0;
}

static void start_monitor(hal_wifi_module_t *m)
{
    (void)m;
    LOGI(TAG, "[WIFI] start_monitor");
    wwd_wifi_enable_monitor_mode();
}

static void stop_monitor(hal_wifi_module_t *m)
{
    (void)m;
    LOGI(TAG, "[WIFI] stop_monitor");
    wwd_wifi_disable_monitor_mode();
}

monitor_data_cb_t aos_monitro_cb = NULL;
static void wwd_wifi_raw_packet_processor( wiced_buffer_t buffer, wwd_interface_t interface )
{
    hal_wifi_link_info_t info;
    info.rssi = 0;
    uint8_t *buf = ((uint8_t *)buffer->payload);

    if (aos_monitro_cb) {
        aos_monitro_cb(buffer->payload, buffer->len - 4, &info);
    }
#if 0
	if (1)
	{
		uint32_t i = 0;
		printf("raw frame data:\n");
		for (i = 0; i < buffer->len - 4; i++) {
			printf("%02x ", buf[i]);
			if ((i % 32) == 31)
				printf("\n");
		}
		printf("\n");
    }
#endif
    host_buffer_release( buffer, WWD_NETWORK_RX );
}


static void register_monitor_cb(hal_wifi_module_t *m, monitor_data_cb_t fn)
{
    (void)m;
    aos_monitro_cb = fn;
    wwd_wifi_set_raw_packet_processor(wwd_wifi_raw_packet_processor);
}

static void register_wlan_mgnt_monitor_cb(hal_wifi_module_t *m, monitor_data_cb_t fn)
{

}


static int wlan_send_80211_raw_frame(hal_wifi_module_t *m, uint8_t *buf, int len)
{
    return 0;
}

hal_wifi_module_t aos_wifi_module_zw6201 = {
    .base.name           = "aos_wifi_module_zw6201",
    .init                =  wifi_init,
    .get_mac_addr        =  wifi_get_mac_addr,
    .start               =  wifi_start,
    .start_adv           =  wifi_start_adv,
    .get_ip_stat         =  get_ip_stat,
    .get_link_stat       =  get_link_stat,
    .start_scan          =  start_scan,
    .start_scan_adv      =  start_scan_adv,
    .power_off           =  power_off,
    .power_on            =  power_on,
    .suspend             =  suspend,
    .suspend_station     =  suspend_station,
    .suspend_soft_ap     =  suspend_soft_ap,
    .set_channel         =  set_channel,
    .start_monitor       =  start_monitor,
    .stop_monitor        =  stop_monitor,
    .register_monitor_cb =  register_monitor_cb,
    .register_wlan_mgnt_monitor_cb = register_wlan_mgnt_monitor_cb,
    .wlan_send_80211_raw_frame = wlan_send_80211_raw_frame
};

