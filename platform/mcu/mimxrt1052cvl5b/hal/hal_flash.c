/*
 * Copyright 2019 ZLG
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdarg.h>
#include <stdio.h>

#include "fsl_device_registers.h"
#include "fsl_common.h"
#include "fsl_clock.h"
#include "aos/hal/flash.h"
#include "aos/kernel.h"
#include "aos/errno.h"

#include "board.h"
#include "fsl_spinor.h"


typedef status_t (*flash_device_erase)(uint32_t address, uint32_t size);
typedef status_t (*flash_device_write)(uint32_t address, const void *data, uint32_t size);
typedef status_t (*flash_device_read)(uint32_t address, void *data, uint32_t size);

typedef struct 
{
    flash_device_erase erase;
    flash_device_write write;
    flash_device_read  read;
    aos_mutex_t lock;
    bool isInited;
} flash_device_t;

static flash_device_t flash_device[HAL_FLASH_MAX];

/* Logic partition on flash devices */
extern const hal_logic_partition_t hal_logic_partition[HAL_PARTITION_MAX];     /* define in board.c */

static int32_t hal_flash_device_init(flash_device_t *device)
{
    int32_t ret;

    ret = flexspi_nor_init();

    return 0;
}

/**
 * Get the infomation of the specified flash area
 *
 * @param[in]  in_partition  The target flash logical partition which should be erased
 *
 * @return     HAL_logi_partition struct
 */
/* FIXME: Return value should be a CONST */
hal_logic_partition_t *hal_flash_get_info(hal_partition_t in_partition)
{
    if (in_partition >= HAL_PARTITION_MAX) {
        return NULL;
    }

    return (hal_logic_partition_t *)&hal_logic_partition[in_partition];
}

/**
 * Erase an area on a Flash logical partition
 *
 * @note  Erase on an address will erase all data on a sector that the
 *        address is belonged to, this function does not save data that
 *        beyond the address area but in the affected sector, the data
 *        will be lost.
 *
 * @param[in]  in_partition  The target flash logical partition which should be erased
 * @param[in]  off_set       Start address of the erased flash area
 * @param[in]  size          Size of the erased flash area
 *
 * @return  0 : On success, -EIO : If an error occurred with any step
 */
int32_t hal_flash_erase(hal_partition_t in_partition, uint32_t off_set, uint32_t size)
{
    status_t status = kStatus_Success;
    int32_t ret = 0;
    hal_logic_partition_t *p_logic_partition = NULL;

    p_logic_partition = hal_flash_get_info(in_partition);
    if ((p_logic_partition == NULL) || (p_logic_partition->partition_owner == HAL_FLASH_NONE))
    {
        return EIO;
    }

    if ((off_set + size) > (p_logic_partition->partition_length)) {
        return -EIO;
    }
    if (size == 0) {
        return -EIO;
    }
    __disable_irq();
    SCB_DisableDCache();

    /* ceil method for erase sector size */
    for (int i = 0; i < ((size + SECTOR_SIZE - 1) / SECTOR_SIZE); i++ ) {
        status = flexspi_nor_flash_erase_sector(off_set + (p_logic_partition->partition_start_addr) + SECTOR_SIZE * i);
        if (status != kStatus_Success)
        {
            SCB_EnableDCache();
            __enable_irq();
            return -EIO;
        }
    }

    SCB_EnableDCache();
    __enable_irq();

    return ret;
}

/**
 * Write data to an area on a flash logical partition without erase
 *
 * @param[in]  in_partition    The target flash logical partition which should be read which should be written
 * @param[in]  off_set         Point to the start address that the data is written to, and
 *                             point to the last unwritten address after this function is
 *                             returned, so you can call this function serval times without
 *                             update this start address.
 * @param[in]  inBuffer        point to the data buffer that will be written to flash
 * @param[in]  inBufferLength  The length of the buffer
 *
 * @return  0 : On success, -EIO : If an error occurred with any step
 */
int32_t hal_flash_write(hal_partition_t in_partition, uint32_t *off_set,
                        const void *in_buf, uint32_t in_buf_len)
{

    int32_t ret = 0;
    int i = 0;
    status_t status;
    hal_logic_partition_t *p_logic_partition = NULL;

    p_logic_partition = hal_flash_get_info(in_partition);
    if ((p_logic_partition == NULL) || (p_logic_partition->partition_owner == HAL_FLASH_NONE))
    {
        return EIO;
    }

    if ((*off_set + in_buf_len) > (p_logic_partition->partition_length) ) {
        return -EIO;
    }


    __disable_irq();
    SCB_DisableDCache();

    status = flexspi_nor_flash_program(*off_set + (p_logic_partition->partition_start_addr), in_buf, in_buf_len);
    if (status != kStatus_Success)
    {
        SCB_EnableDCache();
        __enable_irq();
        return -EIO;
    }
    *off_set = *off_set + in_buf_len;

    SCB_EnableDCache();
    __enable_irq();

    return ret;
}

/**
 * Write data to an area on a flash logical partition with erase first
 *
 * @param[in]  in_partition    The target flash logical partition which should be read which should be written
 * @param[in]  off_set         Point to the start address that the data is written to, and
 *                             point to the last unwritten address after this function is
 *                             returned, so you can call this function serval times without
 *                             update this start address.
 * @param[in]  inBuffer        point to the data buffer that will be written to flash
 * @param[in]  inBufferLength  The length of the buffer
 *
 * @return  0 : On success, -EIO : If an error occurred with any step
 */
int32_t hal_flash_erase_write(hal_partition_t in_partition, uint32_t *off_set,
                              const void *in_buf, uint32_t in_buf_len)
{

    int32_t ret = 0;
    int i = 0;
    status_t status;
    hal_logic_partition_t *p_logic_partition = NULL;

    p_logic_partition = hal_flash_get_info(in_partition);
    if ((p_logic_partition == NULL) || (p_logic_partition->partition_owner == HAL_FLASH_NONE))
    {
        return EIO;
    }

    if ((*off_set + in_buf_len) > (p_logic_partition->partition_length) ) {
        return -EIO;
    }

    __disable_irq();
    SCB_DisableDCache();

    for (int i = 0; i < (in_buf_len / SECTOR_SIZE + 1); i++ ) {
        status = flexspi_nor_flash_erase_sector(*off_set + (p_logic_partition->partition_start_addr) + SECTOR_SIZE * i);
        if (status != kStatus_Success)
        {
            SCB_EnableDCache();
            __enable_irq();
            return -EIO;
        }
    }

    status = flexspi_nor_flash_program(*off_set + (p_logic_partition->partition_start_addr), in_buf, in_buf_len);
    if (status != kStatus_Success)
    {
        SCB_EnableDCache();
        __enable_irq();
        return -EIO;
    }
    *off_set = *off_set + in_buf_len;

    SCB_EnableDCache();
    __enable_irq();

    return ret;
}

/**
 * Read data from an area on a Flash to data buffer in RAM
 *
 * @param[in]  in_partition    The target flash logical partition which should be read
 * @param[in]  off_set         Point to the start address that the data is read, and
 *                             point to the last unread address after this function is
 *                             returned, so you can call this function serval times without
 *                             update this start address.
 * @param[in]  outBuffer       Point to the data buffer that stores the data read from flash
 * @param[in]  out_buf_len     The length of the buffer
 *
 * @return  0 : On success, -EIO : If an error occurred with any step
 */
int32_t hal_flash_read(hal_partition_t in_partition, uint32_t *off_set,
                       void *out_buf, uint32_t out_buf_len)
{

    int32_t ret = 0;
    int i = 0;
    hal_logic_partition_t *p_logic_partition = NULL;

    p_logic_partition = hal_flash_get_info(in_partition);
    if ((p_logic_partition == NULL) || (p_logic_partition->partition_owner == HAL_FLASH_NONE))
    {
        return EIO;
    }

    if ((*off_set + out_buf_len) > (p_logic_partition->partition_length) ) {
        return -EIO;
    }



    memcpy(out_buf, (uint8_t *)(*off_set + (p_logic_partition->partition_start_addr) ), out_buf_len); //p_logic_partition->partition_start_addr
    *off_set = *off_set + out_buf_len;

    return ret;
}

/**
 * Set security options on a logical partition
 *
 * @param[in]  partition  The target flash logical partition
 * @param[in]  offset     Point to the start address that the data is read, and
 *                        point to the last unread address after this function is
 *                        returned, so you can call this function serval times without
 *                        update this start address.
 * @param[in]  size       Size of enabled flash area
 *
 * @return  0 : On success, -EIO : If an error occurred with any step
 */
int32_t hal_flash_enable_secure(hal_partition_t partition, uint32_t off_set, uint32_t size)
{
    return 0;
}

/**
 * Disable security options on a logical partition
 *
 * @param[in]  partition  The target flash logical partition
 * @param[in]  offset     Point to the start address that the data is read, and
 *                        point to the last unread address after this function is
 *                        returned, so you can call this function serval times without
 *                        update this start address.
 * @param[in]  size       Size of disabled flash area
 *
 * @return  0 : On success, -EIO : If an error occurred with any step
 */
int32_t hal_flash_dis_secure(hal_partition_t partition, uint32_t off_set, uint32_t size)
{
    return 0;
}
