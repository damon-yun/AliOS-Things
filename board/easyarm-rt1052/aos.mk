NAME := board_easyarm-rt1052

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION    := 1.0.0
$(NAME)_SUMMARY    := configuration for board easyarm-rt1052
MODULE             := 1052
HOST_ARCH          := Cortex-M7
HOST_MCU_FAMILY    := mcu_mimxrt1052cvl5bimpl

$(NAME)_COMPONENTS += $(HOST_MCU_FAMILY) kernel_init lwip yloop netmgr kv

CONFIG_SYSINFO_PRODUCT_MODEL := ALI_AOS_MIMXRT1052CVL5B
CONFIG_SYSINFO_DEVICE_NAME   := EasyARM-RT1052

GLOBAL_CFLAGS += -DSYSINFO_PRODUCT_MODEL=\"$(CONFIG_SYSINFO_PRODUCT_MODEL)\"
GLOBAL_CFLAGS += -DSYSINFO_DEVICE_NAME=\"$(CONFIG_SYSINFO_DEVICE_NAME)\"
GLOBAL_CFLAGS += -DSYSINFO_ARCH=\"$(HOST_ARCH)\"
GLOBAL_CFLAGS += -DSYSINFO_MCU=\"$(HOST_MCU_FAMILY)\"
GLOBAL_CFLAGS += -DCPU_MIMXRT1052CVL5B
# GLOBAL_CFLAGS += -DCONFIG_NO_TCPIP   #注释之后会导致配网获取密码失败,但是定义了CONFIG_NO_TCPIP之后会导致ota建立socket connect函数为空

GLOBAL_CFLAGS += -DEASYARM_RT1052
GLOBAL_CFLAGS += -D__USE_CMSIS  -DDISABLE_WDOG -DXIP_EXTERNAL_FLASH=1 -DXIP_BOOT_HEADER_ENABLE=1 -DXIP_BOOT_HEADER_DCD_ENABLE=1
GLOBAL_CFLAGS += -DFSL_FEATURE_PHYKSZ8081_USE_RMII50M_MODE -DFSL_SDK_ENABLE_DRIVER_CACHE_CONTROL=1
GLOBAL_CFLAGS += -D__NEWLIB__ 

#user cofig
GLOBAL_DEFINES += CONFIG_AOS_INIT_WELCOME
GLOBAL_DEFINES += SYNC_LOG_DETAILS
GLOBAL_DEFINES += SYNC_DETAIL_COLOR

GLOBAL_DEFINES += AOS_OTA_BANK_SINGLE
# GLOBAL_DEFINES += AOS_OTA_2BOOT_UPDATE_SUPPORT
# GLOBAL_DEFINES += AOS_OTA_RECOVERY_TYPE
#end
GLOBAL_DEFINES += SKIP_SYSCLK_INIT
GLOBAL_DEFINES += XIP_EXTERNAL_FLASH
GLOBAL_DEFINES += XIP_BOOT_HEADER_ENABLE
GLOBAL_DEFINES += CLI_CONFIG_STACK_SIZE=0x4000
## KV Config
GLOBAL_DEFINES += KV_CONFIG_TOTAL_SIZE=0x10000  #64KB
GLOBAL_DEFINES += KV_CONFIG_BLOCK_SIZE_BITS=12  #4KB



GLOBAL_INCLUDES += ./
GLOBAL_INCLUDES += ../../platform/mcu/mimxrt1052cvl5b/
GLOBAL_INCLUDES += ../../platform/mcu/mimxrt1052cvl5b/CMSIS/Include
GLOBAL_INCLUDES += ../../platform/mcu/mimxrt1052cvl5b/drivers
GLOBAL_INCLUDES += ../../platform/mcu/mimxrt1052cvl5b/gcc
GLOBAL_INCLUDES += ../../platform/mcu/mimxrt1052cvl5b/utilities
GLOBAL_INCLUDES += ../../platform/mcu/mimxrt1052cvl5b/utilities/str
GLOBAL_INCLUDES += ../../platform/mcu/mimxrt1052cvl5b/utilities/log
GLOBAL_INCLUDES += ../../platform/mcu/mimxrt1052cvl5b/utilities/io
GLOBAL_INCLUDES += ../../platform/mcu/mimxrt1052cvl5b/xip


$(NAME)_SOURCES     :=
$(NAME)_SOURCES     += ./board.c
$(NAME)_SOURCES     += ./clock_config.c
$(NAME)_SOURCES     += ./pin_mux.c
$(NAME)_SOURCES     += ./fsl_phy.c
$(NAME)_SOURCES     += ./fsl_spinor.c

#ZW6201 code & include
GLOBAL_CFLAGS += -DWITH_LWIP
GLOBAL_CFLAGS += -D__WICED__=1
GLOBAL_CFLAGS += -DHAVE_NOT_ADVANCED_FORMATE
#GLOBAL_CFLAGS += -DCONFIG_NO_TCPIP
GLOBAL_CFLAGS += -DOSAL_RHINO
GLOBAL_CFLAGS += -DA_LITTLE_ENDIAN
GLOBAL_CFLAGS += -DFSL_RTOS_AOS
GLOBAL_CFLAGS += -DUSE_RTOS=1
GLOBAL_CFLAGS += -DALTERNATE_MAIN
GLOBAL_CFLAGS += -Dwifi_firmware_image=resources_firmware_DIR_43362_DIR_43362A2_bin
GLOBAL_CFLAGS += -DWPRINT_ENABLE_WWD_ERROR
#GLOBAL_CFLAGS += -DPRINTF_FLOAT_ENABLE=1
GLOBAL_CFLAGS += -DWPRINT_ENABLE_WWD_INFO=1
GLOBAL_CFLAGS += -DWWD_LOGGING_STDOUT_ENABLE
GLOBAL_CFLAGS += -DWPRINT_ENABLE_WWD_DEBUG
GLOBAL_CFLAGS += -DWPRINT_ENABLE_NETWORK_DEBUG
GLOBAL_CFLAGS += -DBUS=SPI
GLOBAL_CFLAGS += -DPING_USE_SOCKETS=1
GLOBAL_CFLAGS += -DDEV_WIFI_ZW6201

$(NAME)_SOURCES += ./hal_wifi_zw6201.c
$(NAME)_SOURCES += ./zw6201_linkup.c
$(NAME)_SOURCES += ./alios-zw6201-port/e_sim_spi.c

GLOBAL_INCLUDES += ./alios-zw6201-port
#ZW6201 Code use zw6201 need to remove enet ethernetif.c
$(NAME)_SOURCES     += ./zw6201/libraries/tlv/tlv.c
$(NAME)_SOURCES     += ./zw6201/resources/firmware/43362/43362A2_bin.c
$(NAME)_SOURCES     += ./zw6201/WICED/internal/wiced_core.c
$(NAME)_SOURCES     += ./zw6201/WICED/internal/wifi.c
$(NAME)_SOURCES     += ./zw6201/WICED/network/LwIP/WWD/wwd_buffer.c
$(NAME)_SOURCES     += ./zw6201/WICED/network/LwIP/WWD/wwd_network.c
$(NAME)_SOURCES     += ./zw6201/WICED/platform/ARM_CM7/host_cm7.c
$(NAME)_SOURCES     += ./zw6201/WICED/platform/MCU/MIMXRT1052/wwd_sim_SPI.c
$(NAME)_SOURCES     += ./zw6201/WICED/platform/platform_resource.c
$(NAME)_SOURCES     += ./zw6201/WICED/platform/wwd_platform_separate_mcu.c
$(NAME)_SOURCES     += ./zw6201/WICED/platform/wwd_resources.c
$(NAME)_SOURCES     += ./zw6201/WICED/RTOS/AliOS/WICED/wiced_rtos.c
$(NAME)_SOURCES     += ./zw6201/WICED/RTOS/AliOS/WWD/wwd_rtos.c
$(NAME)_SOURCES     += ./zw6201/WICED/RTOS/wiced_rtos_common.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/bus_protocols/wwd_bus_common.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/bus_protocols/SPI/wwd_bus_protocol.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/chips/43362/wwd_ap.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/chips/43362/wwd_chip_specific_functions.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/wwd_ap_common.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/wwd_debug.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/wwd_internal.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/wwd_logging.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/wwd_management.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/wwd_rtos_interface.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/wwd_sdpcm.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/wwd_thread_internal.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/wwd_thread.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/wwd_wifi_chip_common.c
$(NAME)_SOURCES     += ./zw6201/WICED/WWD/internal/wwd_wifi.c

GLOBAL_INCLUDES += ./zw6201
GLOBAL_INCLUDES += ./zw6201/include
GLOBAL_INCLUDES += ./zw6201/libraries/tlv
GLOBAL_INCLUDES += ./zw6201/WICED
GLOBAL_INCLUDES += ./zw6201/WICED/network/LwIP/WWD
GLOBAL_INCLUDES += ./zw6201/WICED/platform
GLOBAL_INCLUDES += ./zw6201/WICED/RTOS
GLOBAL_INCLUDES += ./zw6201/WICED/RTOS/AliOS
GLOBAL_INCLUDES += ./zw6201/WICED/RTOS/AliOS/WWD
GLOBAL_INCLUDES += ./zw6201/WICED/WWD
GLOBAL_INCLUDES += ./zw6201/WICED/WWD/include
GLOBAL_INCLUDES += ./zw6201/WICED/WWD/include/network
GLOBAL_INCLUDES += ./zw6201/WICED/WWD/include/platform
GLOBAL_INCLUDES += ./zw6201/WICED/WWD/include/RTOS
GLOBAL_INCLUDES += ./zw6201/WICED/WWD/internal
GLOBAL_INCLUDES += ./zw6201/WICED/WWD/internal/bus_protocols
GLOBAL_INCLUDES += ./zw6201/WICED/WWD/internal/bus_protocols/SPI
GLOBAL_INCLUDES += ./zw6201/WICED/WWD/internal/chips/43362
#end zw6201

TEST_COMPONENTS += certificate_test

