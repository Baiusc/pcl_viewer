#ifndef SOC_SDK_INTERNAL_H
#define SOC_SDK_INTERNAL_H

#include "fs_sdk_api.h"

#ifdef __cplusplus
extern "C" {
#endif

fs_sdk_work_mode_e fs_sdk_get_work_mode(void);

int32_t fs_sdk_setup_mode_impl(fs_sdk_work_mode_e work_mode);

#ifdef __cplusplus
}
#endif

#endif /* SOC_SDK_INTERNAL_H */
