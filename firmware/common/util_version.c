/*! \file util_version.c
 * 
 * Version utilities
 *
 * @defgroup util_version  Version Utilities
 * @{
 */

#include <string.h>

#include "util_version.h"

VERSIONData version_info;

/*! \brief Firmware GIT Hash
 * GIT_COMMIT_VERSION is inserted by the build system
 *     generated in common/marionette.mk
 */
void set_util_fwversion(VERSIONData * ver_data)
{
#ifndef GIT_COMMIT_VERSION
#define GIT_COMMIT_VERSION "Unknown"
#endif
    strncpy(ver_data->firmware, GIT_COMMIT_VERSION, MAX_FW_VERSION_LENGTH);
}

void set_util_hwversion(VERSIONData * ver_data)
{
        ver_data->hardware.id_low    = *((uint32_t*)UID_BASE);
        ver_data->hardware.id_center = *((uint32_t*)UID_BASE + 0x4);
        ver_data->hardware.id_high   = *((uint32_t*)UID_BASE + 0x8);
}

//! @}
