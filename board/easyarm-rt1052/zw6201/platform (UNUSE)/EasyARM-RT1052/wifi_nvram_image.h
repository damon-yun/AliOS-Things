/*
 * Copyright 2017, Cypress Semiconductor Corporation or a subsidiary of 
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

#ifndef INCLUDED_NVRAM_IMAGE_H_
#define INCLUDED_NVRAM_IMAGE_H_

#include <string.h>
#include <stdint.h>
#include "../generated_mac_address.txt"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Character array of NVRAM image
 * Generated from bcm94334wlagb.txt
 */

static const char wifi_nvram_image[] =
        "manfid=0x2d0"                                                       "\x00"
        "prodid=0x05de"                                                      "\x00"
        "vendid=0x14e4"                                                      "\x00"
        "devid=0x4380"                                                       "\x00"
        "boardtype=0x05de"                                                   "\x00"
        "boardrev=0x1206"                                                    "\x00"
        "boardnum=22"                                                        "\x00"
        NVRAM_GENERATED_MAC_ADDRESS                                          "\x00"
        "sromrev=3"                                                          "\x00"
        "boardflags=0x10081800"                                              "\x00"
        "xtalfreq=37400"                                                     "\x00"
        "nocrc=1"                                                            "\x00"
        "ag0=255"                                                            "\x00"
        "aa2g=1"                                                             "\x00"
        "ccode=ALL"                                                          "\x00"
        "pa0itssit=0x20"                                                     "\x00"
        "pa0b0=6850"                                                         "\x00"
        "pa0b1=-849"                                                         "\x00"
        "pa0b2=-214"                                                         "\x00"
        "tssifloor2g=63"                                                     "\x00"
        "extpagain2g=2"                                                      "\x00"
        "extpagain5g=2"                                                      "\x00"
        "rssismf2g=0xf"                                                      "\x00"
        "rssismc2g=0x8"                                                      "\x00"
        "rssisav2g=0x1"                                                      "\x00"
        "cckPwrOffset=3"                                                     "\x00"
        "rssismf5g=0xf"                                                      "\x00"
        "rssismc5g=0x7"                                                      "\x00"
        "rssisav5g=0x1"                                                      "\x00"
        "pa1lob0=6629"                                                       "\x00"
        "pa1lob1=-831"                                                       "\x00"
        "pa1lob2=-218"                                                       "\x00"
        "tssifloor5gl=46"                                                    "\x00"
        "pa1b0=6553"                                                         "\x00"
        "pa1b1=-824"                                                         "\x00"
        "pa1b2=-219"                                                         "\x00"
        "tssifloor5gm=51"                                                    "\x00"
        "pa1hib0=6606"                                                       "\x00"
        "pa1hib1=-829"                                                       "\x00"
        "pa1hib2=-217"                                                       "\x00"
        "tssifloor5gh=51"                                                    "\x00"
        "rxpo5g=0"                                                           "\x00"
        "maxp2ga0=0x4c"                                                      "\x00"
        "maxp5ga0=0x4c"                                                      "\x00"
        "maxp5gla0=0x4c"                                                     "\x00"
        "maxp5gha0=0x4c"                                                     "\x00"
        "swctrlmap_2g=0x00040004,0x00020002,0x00000000,0x10200,0x1ff"        "\x00"
        "swctrlmap_5g=0x00180018,0x00200020,0x00000000,0x10200,0x2f8"        "\x00"
        "gain=32"                                                            "\x00"
        "triso2g=8"                                                          "\x00"
        "triso5g=8"                                                          "\x00"
        "loflag=1"                                                           "\x00"
        "iqlocalidx5g=40"                                                    "\x00"
        "dlocalidx5g=70"                                                     "\x00"
        "iqcalidx5g=50"                                                      "\x00"
        "lpbckmode5g=1 "                                                     "\x00"
        "txiqlopapu5g=0"                                                     "\x00"
        "txiqlopapu2g=0"                                                     "\x00"
        "dlorange_lowlimit=5"                                                "\x00"
        "aci_detect_en_2g=1"                                                 "\x00"
        "gain_settle_dly_2g=4"                                               "\x00"
        "gain_settle_dly_5g=4"                                               "\x00"
        "noise_cal_po_2g=-1"                                                 "\x00"
        "noise_cal_po_40_2g=-1"                                              "\x00"
        "noise_cal_high_gain_2g=73"                                          "\x00"
        "noise_cal_nf_substract_val_2g=346"                                  "\x00"
        "noise_cal_po_5g=-1"                                                 "\x00"
        "noise_cal_po_40_5g=-1"                                              "\x00"
        "noise_cal_high_gain_5g=73"                                          "\x00"
        "noise_cal_nf_substract_val_5g=300"                                  "\x00"
        "noise_cal_high_gain_2g.fab.4=76"                                    "\x00"
        "\x00\x00";
#ifdef __cplusplus
} /*extern "C" */
#endif

#else /* ifndef INCLUDED_NVRAM_IMAGE_H_ */

#error Wi-Fi NVRAM image included twice

#endif /* ifndef INCLUDED_NVRAM_IMAGE_H_ */
