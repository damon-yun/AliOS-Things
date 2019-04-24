HOST_OPENOCD := MIMXRT1052CVL5B
NAME         := mcu_mimxrt1052cvl5bimpl

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION    := 1.0.0
$(NAME)_SUMMARY    := driver & sdk for platform/mcu mimxrt1052cvl5bimpl

$(NAME)_COMPONENTS += arch_armv7m
$(NAME)_COMPONENTS += rhino newlib_stub cli kv

GLOBAL_CFLAGS += -DA_LITTLE_ENDIAN

GLOBAL_CFLAGS += -DCPU_MIMXRT1052CVL5B


GLOBAL_CFLAGS += -DCORE_M7

GLOBAL_DEFINES += CONFIG_AOS_CLI_STACK_SIZE=8192

GLOBAL_CFLAGS += -DRHINO_CONFIG_TASK_STACK_CUR_CHECK=1

GLOBAL_CFLAGS += -fmessage-length=0
GLOBAL_CFLAGS += -fno-builtin -ffunction-sections -fdata-sections -fno-common -std=gnu99 -nostdlib -DSDK_DEBUGCONSOLE=1 -DSDK_DEBUGCONSOLE_UART

GLOBAL_CFLAGS += -ffreestanding -mapcs -MMD -MP -Wno-format -Wno-incompatible-pointer-types

GLOBAL_CFLAGS += -mcpu=cortex-m7 \
                 -march=armv7-m  \
                 -mlittle-endian \
                 -mthumb -mthumb-interwork \
                 -w \
                 -mfloat-abi=hard -mfpu=fpv5-d16


GLOBAL_ASMFLAGS += -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs -std=gnu99
GLOBAL_ASMFLAGS += -D__STARTUP_CLEAR_BSS
GLOBAL_ASMFLAGS += -D__STARTUP_INITIALIZE_NONCACHEDATA
GLOBAL_ASMFLAGS += -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16

GLOBAL_LDFLAGS += -mcpu=cortex-m7  \
                  -mlittle-endian  \
                  -mthumb -mthumb-interwork \
                  --specs=nosys.specs \
                  -mfloat-abi=hard -mfpu=fpv5-d16 \
                  $(CLIB_LDFLAGS_NANO_FLOAT)


# Common source
$(NAME)_SOURCES     += ./drivers/fsl_adc.c
$(NAME)_SOURCES     += ./drivers/fsl_adc_etc.c
$(NAME)_SOURCES     += ./drivers/fsl_aipstz.c
$(NAME)_SOURCES     += ./drivers/fsl_aoi.c
$(NAME)_SOURCES     += ./drivers/fsl_bee.c
$(NAME)_SOURCES     += ./drivers/fsl_cache.c
$(NAME)_SOURCES     += ./drivers/fsl_clock.c
$(NAME)_SOURCES     += ./drivers/fsl_cmp.c
$(NAME)_SOURCES     += ./drivers/fsl_common.c
$(NAME)_SOURCES     += ./drivers/fsl_csi.c
$(NAME)_SOURCES     += ./drivers/fsl_dcdc.c
$(NAME)_SOURCES     += ./drivers/fsl_dcp.c
$(NAME)_SOURCES     += ./drivers/fsl_dmamux.c
$(NAME)_SOURCES     += ./drivers/fsl_edma.c
$(NAME)_SOURCES     += ./drivers/fsl_elcdif.c
$(NAME)_SOURCES     += ./drivers/fsl_enc.c
$(NAME)_SOURCES     += ./drivers/fsl_enet.c
$(NAME)_SOURCES     += ./drivers/fsl_ewm.c
$(NAME)_SOURCES     += ./drivers/fsl_flexcan.c
$(NAME)_SOURCES     += ./drivers/fsl_flexio.c
$(NAME)_SOURCES     += ./drivers/fsl_flexio_i2c_master.c
$(NAME)_SOURCES     += ./drivers/fsl_flexio_i2s.c
$(NAME)_SOURCES     += ./drivers/fsl_flexio_i2s_edma.c
$(NAME)_SOURCES     += ./drivers/fsl_flexio_spi.c
$(NAME)_SOURCES     += ./drivers/fsl_flexio_spi_edma.c
$(NAME)_SOURCES     += ./drivers/fsl_flexio_uart.c
$(NAME)_SOURCES     += ./drivers/fsl_flexio_uart_edma.c
$(NAME)_SOURCES     += ./drivers/fsl_flexram.c
$(NAME)_SOURCES     += ./drivers/fsl_flexspi.c
$(NAME)_SOURCES     += ./drivers/fsl_gpc.c
$(NAME)_SOURCES     += ./drivers/fsl_gpio.c
$(NAME)_SOURCES     += ./drivers/fsl_gpt.c
$(NAME)_SOURCES     += ./drivers/fsl_kpp.c
$(NAME)_SOURCES     += ./drivers/fsl_lpi2c.c
$(NAME)_SOURCES     += ./drivers/fsl_lpi2c_edma.c
$(NAME)_SOURCES     += ./drivers/fsl_lpspi.c
$(NAME)_SOURCES     += ./drivers/fsl_lpspi_edma.c
$(NAME)_SOURCES     += ./drivers/fsl_lpuart.c
$(NAME)_SOURCES     += ./drivers/fsl_lpuart_edma.c
$(NAME)_SOURCES     += ./drivers/fsl_pit.c
$(NAME)_SOURCES     += ./drivers/fsl_pmu.c
$(NAME)_SOURCES     += ./drivers/fsl_pwm.c
$(NAME)_SOURCES     += ./drivers/fsl_pxp.c
$(NAME)_SOURCES     += ./drivers/fsl_qtmr.c
$(NAME)_SOURCES     += ./drivers/fsl_rtwdog.c
$(NAME)_SOURCES     += ./drivers/fsl_sai.c
$(NAME)_SOURCES     += ./drivers/fsl_sai_edma.c
$(NAME)_SOURCES     += ./drivers/fsl_semc.c
$(NAME)_SOURCES     += ./drivers/fsl_snvs_hp.c
$(NAME)_SOURCES     += ./drivers/fsl_snvs_lp.c
$(NAME)_SOURCES     += ./drivers/fsl_spdif.c
$(NAME)_SOURCES     += ./drivers/fsl_spdif_edma.c
$(NAME)_SOURCES     += ./drivers/fsl_src.c
$(NAME)_SOURCES     += ./drivers/fsl_trng.c
$(NAME)_SOURCES     += ./drivers/fsl_tsc.c
$(NAME)_SOURCES     += ./drivers/fsl_usdhc.c
$(NAME)_SOURCES     += ./drivers/fsl_wdog.c
$(NAME)_SOURCES     += ./drivers/fsl_xbara.c
$(NAME)_SOURCES     += ./drivers/fsl_xbarb.c

$(NAME)_SOURCES     += ./system_MIMXRT1052.c

$(NAME)_SOURCES     += ./utilities/log/fsl_log.c
$(NAME)_SOURCES     += ./utilities/str/fsl_str.c
$(NAME)_SOURCES     += ./utilities/io/fsl_io.c
$(NAME)_SOURCES     += ./utilities/fsl_debug_console.c

$(NAME)_SOURCES     += ./gcc/startup_MIMXRT1052.S

$(NAME)_SOURCES     += ./hal/hal_uart.c
$(NAME)_SOURCES     += ./hal/csp_log.c
# $(NAME)_SOURCES     += ./hal/hal_ota.c
$(NAME)_SOURCES     += ./hal/hal_gpio.c
$(NAME)_SOURCES     += ./hal/hook_impl.c
$(NAME)_SOURCES     += ./hal/hal_flash.c
$(NAME)_SOURCES     += ./hal/hal_lpi2c.c
$(NAME)_SOURCES     += ./hal/hal_lpspi.c
$(NAME)_SOURCES     += ./hal/hal_rtc.c

$(NAME)_SOURCES     += ./xip/evkbimxrt1050_flexspi_nor_config.c
$(NAME)_SOURCES     += ./xip/evkbimxrt1050_sdram_ini_dcd.c
$(NAME)_SOURCES     += ./xip/fsl_flexspi_nor_boot.c

ifneq (1,$(sal))

GLOBAL_INCLUDES += ./lwip-2.0.2/port/
#$(NAME)_SOURCES     += ./lwip-2.0.2/port/ethernetif.c

endif

$(NAME)_SOURCES     += ./aos/aos.c
$(NAME)_SOURCES     += ./aos/soc_impl.c

GLOBAL_INCLUDES += ./      \
                   drivers \
                   CMSIS/Include

GLOBAL_INCLUDES += ./CMSIS/include/

#GLOBAL_CFLAGS += -DSDRAM_IS_SHAREABLE

ifndef OVERRIDE_LD_FILE
GLOBAL_LDFLAGS += -T platform/mcu/mimxrt1052cvl5b/gcc/MIMXRT1052xxxxx_flexspi_nor_sdram.ld
#GLOBAL_LDFLAGS += -T platform/mcu/mimxrt1052cvl5b/gcc/MIMXRT1052xxxxx_sdram_txt.ld
#GLOBAL_LDFLAGS += -T platform/mcu/mimxrt1052cvl5b/gcc/MIMXRT1052xxxxx_flexspi_nor.ld
endif
