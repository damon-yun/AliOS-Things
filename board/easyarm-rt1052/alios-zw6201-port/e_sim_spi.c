/*******************************************************************************
* -----------------------------------------------------------------------------
*
* i2c.h - definitions for the i2c-bus interface
*
* -----------------------------------------------------------------------------
* Copyright (C) Damon Zhang
* All rights reserved.
*
* Author : Damon Zhang
* Website: https://damon-yun.github.io/blog.github.io/
* E-mail : damoncheung@foxmail.com
*
* -----------------------------------------------------------------------------
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
* -----------------------------------------------------------------------------
*
* @file
* Dynamic memory manager
*
* -----------------------------------------------------------------------------
*******************************************************************************/

/**
 * \file
 * \brief sim spi driver
 *
 * \internal
 * \par Modification History
 * - 1.00 16-10-13  damon.zhang, first implementation.
 * \endinternal
 */


#include "e_sim_spi.h"
#include <stddef.h>
#include <stdbool.h>

/*******************************************************************************
 * Private define region: constant & MACRO defined here
 ******************************************************************************/
#if (SPI_MODE ==0)
#define E_SIM_SPI_CHOL    0     /** \brief spi clk default level          */
#define E_SIM_SPI_CPHA    0     /** \brief spi data sample at Edge 1/2    */
#elif (SPI_MODE == 1)
#define E_SIM_SPI_CHOL    0
#define E_SIM_SPI_CPHA    1
#elif (SPI_MODE == 2)
#define E_SIM_SPI_CHOL    1
#define E_SIM_SPI_CPHA    0
#elif (SPI_MODE == 3)
#define E_SIM_SPI_CHOL    1
#define E_SIM_SPI_CPHA    1
#else
#error "Unsupport SPI Mode define!"
#endif

/*******************************************************************************
 * extern region: extern global variable & function prototype
 ******************************************************************************/

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/


/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/


/*****************************************************************************
 * Private functions
 ****************************************************************************/


/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * \brief
 */
void e_sim_spi_init(void)
{

    gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
    /*** WIFI PIN INIT ***********/
    /** OUT */
    GPIO_PinInit(WL_CS_PORT,     WL_CS_PIN,     &gpio_config);
    GPIO_PinInit(WL_MOSI_PORT,   WL_MOSI_PIN,   &gpio_config);
    GPIO_PinInit(WL_REG_ON_PORT, WL_REG_ON_PIN, &gpio_config);
    GPIO_PinInit(WL_CLK_PORT,    WL_CLK_PIN,    &gpio_config);

    /* in **/
    gpio_config.direction = kGPIO_DigitalInput;
    GPIO_PinInit(WL_MISO_PORT,  WL_MISO_PIN, &gpio_config);
    
#if (E_SIM_SPI_CHOL == 0)
    SIM_SPI_SCK_LOW();
#else
    SIM_SPI_SCK_HIGHT();
#endif

    SIM_SPI_MOSI_HIGHT();
    SIM_SPI_CS_HIGHT();


}

/**
 * \brief
 */
uint8_t e_sim_spi_byte_send_recv(uint8_t dat)
{
    int     i     = 0;
    uint8_t temp = 0xAA;

#if (E_SIM_SPI_CPHA == 0)

#if (E_SIM_SPI_CHOL == 0)
    SIM_SPI_SCK_LOW();
#else
    SIM_SPI_SCK_HIGHT();
#endif

    for (i = 0; i < 8; i ++) {
        if (dat & 0x80) {
            SIM_SPI_MOSI_HIGHT();
        } else {
            SIM_SPI_MOSI_LOW();
        }

        dat <<= 1;
#if (E_SIM_SPI_CHOL == 0)
        SIM_SPI_SCK_HIGHT();
#else
        SIM_SPI_SCK_LOW();
#endif
        BIT_DELAY();
        temp <<= 1;

        if (SIM_SPI_MISO_READ()) {
            temp ++;
        }

#if (E_SIM_SPI_CHOL == 0)
        SIM_SPI_SCK_LOW();
#else
        SIM_SPI_SCK_HIGHT();
#endif
        BIT_DELAY();
    }

#else  //E_SIM_SPI_CPHA == 1

    BIT_DELAY();

    for (i = 0; i < 8; i ++) {
#if (E_SIM_SPI_CHOL == 0)
        SIM_SPI_SCK_HIGHT();
#else
        SIM_SPI_SCK_LOW();
#endif
        BIT_DELAY();

        if (dat & 0x80) {
            SIM_SPI_MOSI_HIGHT();
        } else {
            SIM_SPI_MOSI_LOW();
        }

        dat <<= 1;

#if (E_SIM_SPI_CHOL == 0)
        SIM_SPI_SCK_LOW();
#else
        SIM_SPI_SCK_HIGHT();
#endif

        BIT_DELAY();
        temp <<= 1;

        if (SIM_SPI_MISO_READ()) {
            temp ++;
        }
    }

#endif  //end E_SIM_SPI_CPHA == 1

    return temp;
}


int e_sim_spi_trans(const uint8_t *p_write, uint8_t *p_read, int nbytes, bool end)
{
    int i, mode = 0;
    uint8_t w_dummy = 0xff, r_null;

    if ((NULL == p_write) && (NULL == p_read)) {
        return -1;
    }

    if (p_write) {
        mode += 1;
    }

    if (p_read) {
        mode += 2;
    }

    switch (mode) {
    case 3:
        e_sim_spi_cs_low();
        BIT_DELAY();
        for (i = 0; i < nbytes; i++) {
            *p_read++ = e_sim_spi_byte_send_recv(*p_write++);
        }

        break;

    /* give up read data */
    case 1:
        e_sim_spi_cs_low();
        BIT_DELAY();
        for (i = 0; i < nbytes; i++) {
            r_null = e_sim_spi_byte_send_recv(*p_write++);
        }
        (void)r_null;

        break;

    /* only want to read data */
    case 2:
        e_sim_spi_cs_low();
        BIT_DELAY();
        for (i = 0; i < nbytes; i++) {
            *p_read++ = e_sim_spi_byte_send_recv(w_dummy);
        }

        break;

    /* error */
    default:
        break;
    }

    if (end) {
        e_sim_spi_cs_hight();
        BIT_DELAY();
    }

    if ((1 == mode) | (2 == mode) | (3 == mode)) {
        return i;
    }

    return -1;
}


/*******************************************************************************
 * main code region: function implement
 ******************************************************************************/


/* end of file */


