/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include <stdio.h>
#include "aos/hal/i2c.h"
#include "k_types.h"
#include "errno.h"

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_lpi2c.h"

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define LPI2C_MASTER_CLOCK_FREQUENCY LPI2C_CLOCK_FREQUENCY
#define LPI2C_SLAVE_CLOCK_FREQUENCY LPI2C_CLOCK_FREQUENCY

#define LOG_TAG		"lpi2c"

static LPI2C_Type *hal_get_i2c_base(uint8_t port)
{
	switch (port) {
	case 0:
		return NULL;
	case 1:
		return LPI2C1;
	case 2:
		return LPI2C2;
	case 3:
		return LPI2C3;
	case 4:
		return LPI2C4;
	default:
		return NULL;
	}
	return NULL;
}


int32_t hal_i2c_mem_write(i2c_dev_t *i2c, uint16_t dev_addr, uint16_t mem_addr,
	uint16_t mem_addr_size, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    lpi2c_master_transfer_t masterXfer = {0};
    status_t reVal = kStatus_Fail;

	if (!i2c || !data) {
		LOGE(LOG_TAG, "%s: invalid arguments !", __func__);
		return -1;
	}

	LPI2C_Type *base = hal_get_i2c_base(i2c->port);
	if (!base) {
		LOGE(LOG_TAG, "%s: I2C%u base not found !", __func__, i2c->port);
		return -2;
	}

    masterXfer.slaveAddress =  dev_addr >> 1;
    masterXfer.direction = kLPI2C_Write;
    masterXfer.subaddress = mem_addr;
    masterXfer.subaddressSize = mem_addr_size;
    masterXfer.data = (uint8_t *)data;
    masterXfer.dataSize = size;
    masterXfer.flags = kLPI2C_TransferDefaultFlag;

	reVal = LPI2C_MasterTransferBlocking(base, &masterXfer);
    if (reVal != kStatus_Success)
    {
		LOGE(LOG_TAG, "%s: LPI2C%u Send fail !", __func__, i2c->port);
		return -3;
    }

	return 0;
};

int32_t hal_i2c_mem_read(i2c_dev_t *i2c, uint16_t dev_addr, uint16_t mem_addr,
	uint16_t mem_addr_size, uint8_t *data, uint16_t size, uint32_t timeout)
{
    lpi2c_master_transfer_t masterXfer = {0};
    status_t reVal = kStatus_Fail;

	if (!i2c || !data || size <= 0) {
		LOGE(LOG_TAG, "%s: invalid arguments !", __func__);
		return -1;
	}

	LPI2C_Type *base = hal_get_i2c_base(i2c->port);
	if (!base) {
		LOGE(LOG_TAG, "%s: I2C%u base not found !", __func__, i2c->port);
		return -2;
	}

    masterXfer.slaveAddress = dev_addr >> 1;
    masterXfer.direction = kLPI2C_Read;
    masterXfer.subaddress = mem_addr;
    masterXfer.subaddressSize = mem_addr_size;
    masterXfer.data = data;
    masterXfer.dataSize = size;
    masterXfer.flags = kLPI2C_TransferDefaultFlag;

	reVal = LPI2C_MasterTransferBlocking(base, &masterXfer);
    if (reVal != kStatus_Success)
    {
		LOGE(LOG_TAG, "%s: LPI2C%u Send fail !", __func__, i2c->port);
		return -3;
    }

	return 0;
}


int32_t hal_i2c_master_send(i2c_dev_t *i2c, uint16_t dev_addr, const uint8_t *data,
	uint16_t size, uint32_t timeout)
{
    lpi2c_master_transfer_t masterXfer = {0};
    status_t reVal = kStatus_Fail;

	if (!i2c || !data) {
		LOGE(LOG_TAG, "%s: invalid arguments !", __func__);
		return -1;
	}

	LPI2C_Type *base = hal_get_i2c_base(i2c->port);
	if (!base) {
		LOGE(LOG_TAG, "%s: LPI2C%u base not found !", __func__, i2c->port);
		return -2;
	}

    masterXfer.slaveAddress =  dev_addr >> 1;
    masterXfer.direction = kLPI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = (uint8_t *)data;
    masterXfer.dataSize = size;
    masterXfer.flags = kLPI2C_TransferDefaultFlag;

	reVal = LPI2C_MasterTransferBlocking(base, &masterXfer);
    if (reVal != kStatus_Success)
    {
		LOGE(LOG_TAG, "%s: LPI2C%u Send fail !", __func__, i2c->port);
		return -3;
    }

	return 0;
}

int32_t hal_i2c_master_recv(i2c_dev_t *i2c, uint16_t dev_addr, uint8_t *data,
	uint16_t size, uint32_t timeout)
{
    lpi2c_master_transfer_t masterXfer = {0};
    status_t reVal = kStatus_Fail;

	if (!i2c || !data) {
		LOGE(LOG_TAG, "%s: invalid arguments !", __func__);
		return -1;
	}

	LPI2C_Type *base = hal_get_i2c_base(i2c->port);
	if (!base) {
		LOGE(LOG_TAG, "%s: LPI2C%u base not found !",
			__func__, i2c->port);
		return -2;
	}
    masterXfer.slaveAddress =  dev_addr >> 1;
    masterXfer.direction = kLPI2C_Read;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = data;
    masterXfer.dataSize = size;
    masterXfer.flags = kLPI2C_TransferDefaultFlag;

	reVal = LPI2C_MasterTransferBlocking(base, &masterXfer);
    if (reVal != kStatus_Success)
    {
		LOGE(LOG_TAG, "%s: LPI2C%u Recv fail !", __func__, i2c->port);
		return -3;
    }

    return 0;
}


int32_t hal_i2c_init(i2c_dev_t *i2c)
{
    lpi2c_master_config_t masterConfig = {0};

    /*Clock setting for LPI2C*/
    CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);

    LPI2C_Type *base = hal_get_i2c_base(i2c->port);
	if (!base) {
		LOGE(LOG_TAG, "%s: I2C%u base not found !", __func__, i2c->port);
		return -1;
	}

	/* I2C clock refer to LPC5410x User manual Rev2.5 Page332 */

    /* Setup lpi2c master */
    /*
     * masterConfig.debugEnable = false;
     * masterConfig.ignoreAck = false;
     * masterConfig.pinConfig = kLPI2C_2PinOpenDrain;
     * masterConfig.baudRate_Hz = 100000U;
     * masterConfig.busIdleTimeout_ns = 0;
     * masterConfig.pinLowTimeout_ns = 0;
     * masterConfig.sdaGlitchFilterWidth_ns = 0;
     * masterConfig.sclGlitchFilterWidth_ns = 0;
     */
    LPI2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz = 100000U;

    LPI2C_MasterInit(base, &masterConfig, LPI2C_MASTER_CLOCK_FREQUENCY);

	LOG("%s: init i2c%u success", __func__, i2c->port);
	return 0;
}


/**
 * Deinitialises an I2C device
 *
 * @param[in]  i2c  the i2c device
 *
 * @return  0 : on success, EIO : if an error occurred during deinitialisation
 */
int32_t hal_i2c_finalize(i2c_dev_t *i2c)
{
    LPI2C_Type *base = hal_get_i2c_base(i2c->port);

	if (!base) {
		LOGE(LOG_TAG, "%s: I2C%u base not found !", __func__, i2c->port);
		return -1;
	}

	LPI2C_MasterDeinit(base);

}
