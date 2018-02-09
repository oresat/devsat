/*! \file dio_ext.h
 *
 */

#ifndef _DIO_EXT_H
#define _DIO_EXT_H

/*!
 * \addtogroup dioext
 * @{
 */
#include "ch.h"
#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern           event_source_t     dio0_event;
extern           event_source_t     dio1_event;
extern           event_source_t     dio2_event;
extern           event_source_t     dio3_event;
extern           event_source_t     dio4_event;
extern           event_source_t     dio5_event;

extern           event_source_t     dio0_event_sx2;
extern           event_source_t     dio1_event_sx2;
extern           event_source_t     dio2_event_sx2;
extern           event_source_t     dio3_event_sx2;
extern           event_source_t     dio4_event_sx2;
extern           event_source_t     dio5_event_sx2;

extern const     EXTConfig       extcfg;

void dio_init(void) ;

void dio0_handler(EXTDriver *extp, expchannel_t channel) ;
void dio1_handler(EXTDriver *extp, expchannel_t channel) ;
void dio2_handler(EXTDriver *extp, expchannel_t channel) ;
void dio3_handler(EXTDriver *extp, expchannel_t channel) ;
void dio4_handler(EXTDriver *extp, expchannel_t channel) ;
void dio5_handler(EXTDriver *extp, expchannel_t channel) ;

void dio0_sx2_handler(EXTDriver *extp, expchannel_t channel) ;
void dio1_sx2_handler(EXTDriver *extp, expchannel_t channel) ;
void dio2_sx2_handler(EXTDriver *extp, expchannel_t channel) ;
void dio3_sx2_handler(EXTDriver *extp, expchannel_t channel) ;
void dio4_sx2_handler(EXTDriver *extp, expchannel_t channel) ;
void dio5_sx2_handler(EXTDriver *extp, expchannel_t channel) ;

#ifdef __cplusplus
}
#endif

//! @}


#endif

