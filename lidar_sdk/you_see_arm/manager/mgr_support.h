#ifndef MGR_SUPPORT_H
#define MGR_SUPPORT_H
#include "../third_party/cJSON/cJSON.h"
#include <stddef.h>
#include <stdint.h>
int64_t mgr_now(void);
cJSON *mgr_read_json(const char *path);
int mgr_write_json(const char *directory, const char *name, cJSON *value, const char *flash_kind);
const char *mgr_string(cJSON *object, const char *name);
int mgr_int(cJSON *object, const char *name, int fallback);
int mgr_verify_package(const char *root, const char *key, const char *openssl, int simulation, cJSON **manifest);
enum mgr_runtime_flags { MGR_RUNTIME_NO_FILE_LOG = 1 };
int mgr_runtime_config(const char *base, const char *directory, int mode, const char *user_settings,
		       unsigned int flags);
#endif
