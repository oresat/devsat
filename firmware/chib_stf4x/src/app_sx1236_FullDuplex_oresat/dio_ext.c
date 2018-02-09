/*! \file dio_ext.c
 */

/*!
 * \defgroup dioext DIO PIN Event Utilities
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "dio_ext.h"

event_source_t     dio0_event;
event_source_t     dio1_event;
event_source_t     dio2_event;
event_source_t     dio3_event;
event_source_t     dio4_event;
event_source_t     dio5_event;  // in continuous mode clockout. This is too fast to for interrupts?

event_source_t     dio0_event_sx2;
event_source_t     dio1_event_sx2;
event_source_t     dio2_event_sx2;
event_source_t     dio3_event_sx2;
event_source_t     dio4_event_sx2;
event_source_t     dio5_event_sx2;  // in continuous mode clockout. This is too fast to for interrupts?


/*! \sa HAL_USE_EXT in hal_conf.h
 */
const EXTConfig extcfg =
{
    {
        {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, dio0_handler},
        {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, dio1_handler},
        {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, dio2_handler},
        {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, dio3_handler},
        {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, dio4_handler},
        // {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, dio5_handler},   // disable this interrupt because tooo fassssst!
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, dio0_sx2_handler},
        {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, dio1_sx2_handler},
        {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, dio2_sx2_handler},
        {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, dio3_sx2_handler},
        {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, dio4_sx2_handler},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL}
    }
};

void dio_init(void)
{
    // extInit();
    chEvtObjectInit(&dio0_event);
    chEvtObjectInit(&dio1_event);
    chEvtObjectInit(&dio2_event);
    chEvtObjectInit(&dio3_event);
    chEvtObjectInit(&dio4_event);
    chEvtObjectInit(&dio5_event);

    chEvtObjectInit(&dio0_event_sx2);
    chEvtObjectInit(&dio1_event_sx2);
    chEvtObjectInit(&dio2_event_sx2);
    chEvtObjectInit(&dio3_event_sx2);
    chEvtObjectInit(&dio4_event_sx2);
    chEvtObjectInit(&dio5_event_sx2);

}

void dio0_handler(EXTDriver * extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    chSysLockFromISR();
    chEvtBroadcastI(&dio0_event);
    // palTogglePad(GPIOA, GPIOA_SX_TESTOUT);

    chSysUnlockFromISR();

};

void dio1_handler(EXTDriver * extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    chSysLockFromISR();
    chEvtBroadcastI(&dio1_event);

    chSysUnlockFromISR();

};

void dio2_handler(EXTDriver * extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    chSysLockFromISR();
    chEvtBroadcastI(&dio2_event);

    chSysUnlockFromISR();

};

void dio3_handler(EXTDriver * extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    chSysLockFromISR();
    chEvtBroadcastI(&dio3_event);

    chSysUnlockFromISR();

};

void dio4_handler(EXTDriver * extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    chSysLockFromISR();
    chEvtBroadcastI(&dio4_event);

    chSysUnlockFromISR();

};

void dio5_handler(EXTDriver * extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    chSysLockFromISR();
    chEvtBroadcastI(&dio5_event);

    chSysUnlockFromISR();

};

void dio0_sx2_handler(EXTDriver * extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    chSysLockFromISR();
    chEvtBroadcastI(&dio0_event_sx2);
    // palTogglePad(GPIOA, GPIOA_SX_TESTOUT);

    chSysUnlockFromISR();

};

void dio1_sx2_handler(EXTDriver * extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    chSysLockFromISR();
    chEvtBroadcastI(&dio1_event_sx2);

    chSysUnlockFromISR();

};

void dio2_sx2_handler(EXTDriver * extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    chSysLockFromISR();
    chEvtBroadcastI(&dio2_event_sx2);

    chSysUnlockFromISR();

};

void dio3_sx2_handler(EXTDriver * extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    chSysLockFromISR();
    chEvtBroadcastI(&dio3_event_sx2);

    chSysUnlockFromISR();

};

void dio4_sx2_handler(EXTDriver * extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    chSysLockFromISR();
    chEvtBroadcastI(&dio4_event_sx2);

    chSysUnlockFromISR();

};

void dio5_sx2_handler(EXTDriver * extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    chSysLockFromISR();
    chEvtBroadcastI(&dio5_event_sx2);

    chSysUnlockFromISR();

};

//! @}

