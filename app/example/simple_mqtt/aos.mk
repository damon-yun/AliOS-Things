NAME := simple_mqtt

$(NAME)_MBINS_TYPE := app
$(NAME)_VERSION := 1.0.0
$(NAME)_SUMMARY := simple mqtt examples

$(NAME)_SOURCES := simple_mqtt.c
$(NAME)_COMPONENTS := linkkit_sdk_c netmgr cjson cli

GLOBAL_INCLUDES += ./

