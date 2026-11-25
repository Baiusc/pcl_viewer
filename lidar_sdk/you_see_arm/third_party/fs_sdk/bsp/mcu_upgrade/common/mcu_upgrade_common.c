#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "fs_err.h"
#include "mcu_upgrade.h"
#include "utils/fs_util.h"

#undef LOG_TAG
#define LOG_TAG "mcu_upgrade"

int mcu_upgrade_compare_version(const mcu_version_t *current, const mcu_version_t *target)
{
    if ((current == NULL) || (target == NULL)) {
        return 0;
    }

    return strncmp(current->hrd.version_string,
                   target->hrd.version_string,
                   sizeof(current->hrd.version_string));
}

int32_t mcu_upgrade_parse_version_string(const char *version_string, mcu_version_t *version)
{
    const char *version_part;
    char *underscore_ptr;
    char *dash_ptr;
    int parts_read;
    int git_commit_len;

    if ((version_string == NULL) || (version == NULL)) {
        return FS_ERR_BAD_PARAMS;
    }

    strncpy(version->hrd.version_string,
            version_string,
            sizeof(version->hrd.version_string) - 1u);
    version->hrd.version_string[sizeof(version->hrd.version_string) - 1u] = '\0';

    if (version_string[0] != 'v') {
        FS_LOGE("Invalid version string format: missing 'v' prefix. version:%s", version_string);
        return FS_ERR_BAD_PARAMS;
    }

    underscore_ptr = strchr(version_string, '_');
    if (underscore_ptr == NULL) {
        FS_LOGE("Invalid version string format: missing '_' separator");
        return FS_ERR_BAD_PARAMS;
    }

    dash_ptr = strchr(underscore_ptr, '-');
    if (dash_ptr == NULL) {
        FS_LOGE("Invalid version string format: missing '-' separator");
        return FS_ERR_BAD_PARAMS;
    }

    version_part = version_string + 1;
    parts_read = sscanf(version_part, "%u.%u.%u",
                        &version->major,
                        &version->minor,
                        &version->patch);
    if (parts_read != 3) {
        FS_LOGE("Failed to parse version numbers from: %s", version_string);
        return FS_ERR_BAD_PARAMS;
    }

    git_commit_len = (int)(dash_ptr - (underscore_ptr + 1));
    if (git_commit_len != 8) {
        FS_LOGE("Invalid git commit length: expected 8, got %d", git_commit_len);
        return FS_ERR_BAD_PARAMS;
    }

    strncpy(version->git_commit, underscore_ptr + 1, 8u);
    version->git_commit[8] = '\0';

    strncpy(version->pipeline_id, dash_ptr + 1, sizeof(version->pipeline_id) - 1u);
    version->pipeline_id[sizeof(version->pipeline_id) - 1u] = '\0';

    return FS_SUCCESS;
}

int32_t mcu_upgrade_load_firmware(const char *firmware_path,
                                  mcu_version_t *version,
                                  uint8_t **fw_buf_out,
                                  uint32_t *fw_len_out)
{
    mcu_version_t tmp_ver = {0};
    mcu_version_t *ver_ptr = version ? version : &tmp_ver;
    mcu_version_hrd_t hrd = {0};
    int fd;
    ssize_t read_size;

    if (firmware_path == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    fd = open(firmware_path, O_RDONLY);
    if (fd < 0) {
        FS_LOGE("Failed to open firmware file: %s", firmware_path);
        return FS_ERR_IO;
    }

    read_size = read(fd, &hrd, sizeof(hrd));
    if (read_size < (ssize_t)sizeof(hrd)) {
        FS_LOGE("Firmware header too short: %zd", read_size);
        close(fd);
        return FS_ERR_IO;
    }

    if (hrd.hrd_ver != 1u) {
        FS_LOGE("Unsupported firmware header version: %u", hrd.hrd_ver);
        close(fd);
        return FS_ERR_UNSUPPORT;
    }

    if ((hrd.hrd_len == 0u) || (hrd.hrd_len > sizeof(hrd))) {
        FS_LOGE("Invalid firmware header length: %u", hrd.hrd_len);
        close(fd);
        return FS_ERR_BAD_PARAMS;
    }

    ver_ptr->hrd = hrd;
    if (mcu_upgrade_parse_version_string(hrd.version_string, ver_ptr) != FS_SUCCESS) {
        close(fd);
        return FS_ERR_BAD_PARAMS;
    }

    if ((fw_buf_out != NULL) || (fw_len_out != NULL)) {
        struct stat st;
        uint8_t *buf = NULL;
        uint16_t calc_crc = 0u;
        uint32_t remaining = hrd.fw_len;
        uint8_t tmp[4096];
        uint8_t *p = NULL;

        if ((fstat(fd, &st) == 0) &&
            (st.st_size < (off_t)((uint32_t)hrd.fw_offset + hrd.fw_len))) {
            FS_LOGE("Firmware file size %ld smaller than header declared %u",
                    (long)st.st_size,
                    (unsigned)((uint32_t)hrd.fw_offset + hrd.fw_len));
            close(fd);
            return FS_ERR_IO;
        }

        if (lseek(fd, hrd.fw_offset, SEEK_SET) < 0) {
            FS_LOGE("Failed to seek payload offset: %u", hrd.fw_offset);
            close(fd);
            return FS_ERR_IO;
        }

        if (fw_buf_out != NULL) {
            buf = (uint8_t *)malloc(hrd.fw_len);
            if (buf == NULL) {
                close(fd);
                return FS_ERR_NOMEM;
            }
            p = buf;
        }

        while (remaining > 0u) {
            size_t chunk = remaining > sizeof(tmp) ? sizeof(tmp) : (size_t)remaining;
            ssize_t r = read(fd, buf ? p : tmp, chunk);
            if (r <= 0) {
                FS_LOGE("Failed to read firmware payload (read=%zd)", r);
                free(buf);
                close(fd);
                return FS_ERR_IO;
            }
            calc_crc = fs_util_crc_16(buf ? p : tmp, (uint32_t)r, calc_crc);
            remaining -= (uint32_t)r;
            if (buf != NULL) {
                p += r;
            }
        }

        if (calc_crc != hrd.fw_crc) {
            FS_LOGE("Firmware CRC mismatch: header=0x%04X, calc=0x%04X",
                    hrd.fw_crc,
                    calc_crc);
            free(buf);
            close(fd);
            return FS_ERR_CHECK_CRC;
        }

        if (fw_buf_out != NULL) {
            *fw_buf_out = buf;
        } else {
            free(buf);
        }
        if (fw_len_out != NULL) {
            *fw_len_out = hrd.fw_len;
        }
    }

    close(fd);
    return FS_SUCCESS;
}
