#include <stdio.h>
#include "utils/fs_err.h"

#undef LOG_TAG
#define LOG_TAG "com.err"

const char* fs_err_strerror(int err)
{
    static char errstr[32] = {
        '\0',
    };

    switch (err) {
    /* Generic error. */
    case FS_ERR_INTERNAL:
        return "Internal error";

    /* Base on unix errno. */
    case FS_ERR_NOT_FOUND:
        return "No such file or directory";

    case FS_ERR_IO:
        return "I/O error";

    case FS_ERR_AGAIN:
        return "Try again";

    case FS_ERR_NOMEM:
        return "Out of memory";

    case FS_ERR_BUSY:
        return "Device or resource busy";

    case FS_ERR_BAD_PARAMS:
        return "Invalid argument";

    /* Common error. */
    case FS_ERR_NULL_PTR:
        return "Null pointer";

    case FS_ERR_BUF_OVERFLOW:
        return "Buffer overflow";

    case FS_ERR_BAD_PROTOCOL:
        return "Bad protocol";

    case FS_ERR_SPI_READ:
        return "Read SPI data failed";

    case FS_ERR_SPI_WRITE:
        return "Write SPI data failed";

    case FS_ERR_SPI_CRC:
        return "RW SPI data crc err";

    case FS_ERR_NO_DEVICE:
        return "Maybe no device";

    case FS_SUCCESS:
        return "Success";

    default:
        break;
    }

    snprintf(errstr, sizeof(errstr), "%d", err);
    return (const char*)errstr;
}