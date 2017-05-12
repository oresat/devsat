/*! \file ltc2990.c
 *
 * API for ltc2990.c
 */

/*!
 * \defgroup ltc2990 Voltage, current  and Temperature Monitor
 *
 * @{
 */

#include <stdint.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "ltc2990.h"

extern const  uint8_t    LTC2990_I2C_ADDR;

static        uint8_t    i2c_txbuf[LTC2990_I2C_TX_BUFSIZE];
static        uint8_t    i2c_rxbuf[LTC2990_I2C_RX_BUFSIZE];

uint8_t ltc2990_readreg(uint8_t reg, i2cflags_t * i2c_errors)
{
	msg_t status   = MSG_OK;
	systime_t tmo  = MS2ST(4);

	i2c_txbuf[0] = reg;
	i2c_rxbuf[0] = 0xff;
	i2cAcquireBus(&I2CD1);
	status       = i2cMasterTransmitTimeout(&I2CD1, LTC2990_I2C_ADDR, i2c_txbuf, 1, i2c_rxbuf, 1, tmo);
	i2cReleaseBus(&I2CD1);

	if (status != MSG_OK)
	{
		*i2c_errors = i2cGetErrors(&I2CD1);
	}

	return(i2c_rxbuf[0]);
}

void ltc2990_writereg(uint8_t reg, uint8_t val, i2cflags_t * i2c_errors)
{
	msg_t status   = MSG_OK;
	systime_t tmo  = MS2ST(4);

	i2c_txbuf[0] = reg;
	i2c_txbuf[1] = val;
	i2cAcquireBus(&I2CD1);
	status       = i2cMasterTransmitTimeout(&I2CD1, LTC2990_I2C_ADDR, i2c_txbuf, 2, i2c_rxbuf, 0, tmo);
	i2cReleaseBus(&I2CD1);

	if (status != MSG_OK)
	{
		*i2c_errors = i2cGetErrors(&I2CD1);
	}
}

void ltc2990_read_all(ltc2990_data * d, i2cflags_t * i2c_errors)
{
	msg_t status   = MSG_OK;
	systime_t tmo  = MS2ST(4);

	i2c_txbuf[0]   = LTC2990_STATUS;
	i2cAcquireBus(&I2CD1);
	status         = i2cMasterTransmitTimeout(&I2CD1, LTC2990_I2C_ADDR, i2c_txbuf, 1, i2c_rxbuf, 0xf, tmo);
	i2cReleaseBus(&I2CD1);

	if (status != MSG_OK)
	{
		*i2c_errors = i2cGetErrors(&I2CD1);
	}
	else
	{
		d->STATUS    = i2c_rxbuf[0];
		d->CONTROL   = i2c_rxbuf[1];
		d->TRIGGER   = i2c_rxbuf[2];
		d->NA        = i2c_rxbuf[3];
		d->T_INT_MSB = i2c_rxbuf[4];
		d->T_INT_LSB = i2c_rxbuf[5];
		d->V1_MSB    = i2c_rxbuf[6];
		d->V1_LSB    = i2c_rxbuf[7];
		d->V2_MSB    = i2c_rxbuf[8];
		d->V2_LSB    = i2c_rxbuf[9];
		d->V3_MSB    = i2c_rxbuf[10];
		d->V3_LSB    = i2c_rxbuf[11];
		d->V4_MSB    = i2c_rxbuf[12];
		d->V4_LSB    = i2c_rxbuf[13];
		d->VCC_MSB   = i2c_rxbuf[14];
		d->VCC_LSB   = i2c_rxbuf[15];
	}
}

//! @}
