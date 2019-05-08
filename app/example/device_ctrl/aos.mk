NAME := device_ctrl

$(NAME)_MBINS_TYPE := app
$(NAME)_VERSION := 1.0.0
$(NAME)_SUMMARY := device ctrl examples

$(NAME)_SOURCES := device_ctrl.c
$(NAME)_COMPONENTS := linkkit_sdk_c netmgr cjson cli

# for developerkit board
ifeq ($(CONFIG_SYSINFO_DEVICE_NAME),developerkit)
GLOBAL_DEFINES += CLD_CMD_LED_REMOTE_CTRL_SUPPORT
endif

# for stm32l476rg-nucleo, stm32f412-nucleo, NXP lpc54102, TI msp432p4111launchpad
ifeq ($(strip $(CONFIG_SYSINFO_DEVICE_NAME)),$(filter $(CONFIG_SYSINFO_DEVICE_NAME),476-nucleo lpc54102 msp432p4111launchpad f412-nucleo))
GLOBAL_DEFINES += CLD_CMD_LED_REMOTE_CTRL_SUPPORT
endif

# for developerkit board
ifeq ($(CONFIG_SYSINFO_DEVICE_NAME),EasyARM-RT1052)
GLOBAL_DEFINES += CLD_CMD_LED_REMOTE_CTRL_SUPPORT
endif

GLOBAL_INCLUDES += ./

