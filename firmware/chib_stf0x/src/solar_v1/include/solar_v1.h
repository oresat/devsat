/*! \file solar_v1.h */

/*!
 * \addtogroup solar_v1
 *  * @{
 *
 */
#ifndef _SOLAR_V1_H_
#define _SOLAR_V1_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ch.h"
#include "hal.h"

#include "ltc2990.h"

signed int solar_v1_calc_temp(ltc2990_data * d, ltc2990_error * error);
signed int solar_v1_calc_current(ltc2990_data * d, ltc2990_error * error);

#ifdef __cplusplus
}
#endif

#endif
//! @}
