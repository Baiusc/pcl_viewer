#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>

#include "fs_err.h"
#include "fs_hw.h"
#include "board_cfg.h"
#include "mcu_protocol.h"
#include "fs_sdk_api.h"
#include "utils/fs_util.h"
#include "utils/fs_version.h"
#include "drivers/i2c/i2c_master.h"
#include "drivers/uart/uart.h"
#include "mcu_log_reader.h"
#include "pinctrl.h"

#include "mcu_upgrade.h"

#undef  LOG_TAG
#define LOG_TAG "mcu_upgrade"

#define FS_MCU_UART_SPEED_HANDSHAKE_NO_BOOT 500000          // 无 boot 引脚握手时的波特率
#define FS_MCU_UART_SPEED_HANDSHAKE         115200          // 握手时的波特率
#define FS_MCU_UART_SPEED_UPDATE            115200          // 烧录固件时的波特率
#define FS_MCU_RESET_ACTIVE_MS                   5          // MCU 复位信号持续时间
#define FS_MCU_HANDSHAKE_TIMEOUT_MS             20          // MCU 握手超时时间
#define FS_MCU_INIT_MS                         300          // MCU 初始化时间

#define MCU_CMD_HEADER                        0x65          // MCU 升级协议命令头
#define MCU_INFO_BASE_ADDR             (0x00100000)         // MCU 信息存储基地址
#define MCU_INFO_OFFSET_MODEL_NAME     (0x00001B40)         // MCU 型号名称偏移地址
#define MCU_MODEL_NAME                 "HC32L121JAUB"       // 预期的 MCU 型号名称, 用于校验 MCU 是否为预期型号
#ifndef MCU_ROM_ISP_PROGRAM_BASE
#define MCU_ROM_ISP_PROGRAM_BASE        0x00000000u         // ROM ISP program base (0x0 standalone/bootloader, APP slot for recovery)
#endif
#define MCU_FW_START_ADDRESS            MCU_ROM_ISP_PROGRAM_BASE
#define MCU_FLASH_MAX_SZIE             (128 * 1024)         // MCU Flash 尺寸
#define MCU_FLASH_SECTOR_SIZE                   512
#define MCU_MAX_WRITE_LEN                     (248)         // MCU 协议中单次最大写入数据长度, 需要预留 Offset 参数
#define MCU_MAX_READ_LEN                      (255)         // MCU 协议中单次最大读取数据长度, 需要预留 Offset 参数
#define MCU_MAX_DATA_LEN               (sizeof(uint32_t) + (MCU_MAX_READ_LEN > MCU_MAX_WRITE_LEN ? MCU_MAX_READ_LEN : MCU_MAX_WRITE_LEN)) // MCU 协议中单次最大数据长度

typedef enum {
    MCU_CMD_QUERY             = 0x10, // Level0~2: 编程器查询目标 MCU 是否能正常通信
    MCU_CMD_PPS               = 0x11, // Level0~2: 编程器与目标 MCU 协商通信速率, 需要传 BRR
    MCU_CMD_FLASH_CHIPERASE   = 0x20, // Level0~2: 擦除目标 MCU 用户闪存区内所有数据
    MCU_CMD_FLASH_SECTORERASE = 0x21, // Level0~2: 擦除地址“<BaseAddr> + <Offset>”所在页面, 需要传 Offset
    MCU_CMD_FLASH_BLANK_CHECK = 0x22, // Level0~2: 检查目标 MCU 闪存数据是否全空（全为 0xFF）
    MCU_CMD_VERIFY_DATA       = 0x23, // Level0  : 返回从<BaseAddr>+<Offset>开始<ByteCnt>字节的 CRC 值, 需要传 Offset 和 ByteCnt
    MCU_CMD_CHECKSUM          = 0x24, // Level0~2: 返回从<BaseAddr>+<Offset>开始<ByteCnt>字节的 CRC 值。注意：最小校验单位为页, 需要传 Offset 和 ByteCnt
    MCU_CMD_SET_BASEADDR      = 0x27, // Level0~2: 设置后续读、写、擦操作基地址为<BaseAddr>, 需要传 BaseAddr
    MCU_CMD_WRITE_DATA        = 0x28, // Level0  : 将 N 字节数据写入<BaseAddr>+ <Offset>处，并读回验证是否与写入数据相同, 需要传 Offset 和 D1~Dn
    MCU_CMD_READ_DATA         = 0x29, // Level0  : 从<BaseAddr > + <Offset>处，读出N个字节。N = 1 ~ 255, 需要传 Offset 和 N
    MCU_CMD_SET_PROTECT_LEVEL = 0x2C, // Level0~2: 设置芯片的读保护等级，支持 Level0~Level3 共四种保护等级, 需要传 RdLevel
    MCU_CMD_JUMP              = 0x30, // Level0  : 跳转到 Addr 所指定的地址开始执行程序, 需要传 Addr
} mcu_cmd_e;

typedef enum {
    MCU_SUCCESS               = 0x00, // 代表执行成功
    MCU_ERR_CHECK_CRC         = 0x10, // 代表通信 CRC 检验错误，需要重发该命令
    MCU_ERR_UNKNOWN_CMD       = 0x20, // 代表指令不支持
    MCU_ERR_PERMISSION_RD     = 0x30, // 代表没有读权限
    MCU_ERR_PERMISSION_WR     = 0x31, // 代表没有写权限
    MCU_ERR_PERMISSION_JUMP   = 0x32, // 代表没有跳转权限
    MCU_ERR_PERMISSION_ERASE  = 0x33, // 代表没有擦权限
    MCU_ERR_PERMISSION_VERIFY = 0x34, // 代表 Verify 权限
    MCU_ERR_WRITE_DATA        = 0x40, // 代表 Write Data 失败
    MCU_ERR_BLANK_CHECK       = 0x41, // 代表 Blank Check 失败
} mcu_error_no_e;

// 注意：加密等级更新时，只要从其他等级降级到 Level0，就执行全片擦除
typedef enum {
    MCU_RD_LEVEL_0            = 0x00, // SWD 可读写芯片，bootloader 可读写芯片
    MCU_RD_LEVEL_1            = 0x01, // SWD 可降级芯片，bootloader 可降级芯片，数据不可读出
    MCU_RD_LEVEL_2            = 0x02, // SWD 端口物理断开，bootloader 可降级芯片，数据不可读出
    MCU_RD_LEVEL_3            = 0x03, // SWD 端口物理断开，bootloader 端口物理断开，数据不可读出。注意：Level3 保护等级生效后芯片不能再次进行编程，请慎用
    MCU_RD_LEVEL_GET          = 0x55, // 不操作当前读保护等级，只返回当前保护等级
} mcu_rd_level_e;

#pragma pack(push)
#pragma pack(1)

typedef struct {
    uint16_t  crc; // 从 cmd 加到 data[dlen -1]
} mcuCmdTail_t;

typedef struct {
    uint8_t  head;      // --> 0x65
    uint8_t  len;       // --> 包的数据长度
    uint8_t  cmd;       // --> 命令码 mcu_cmd_e
    uint16_t data[0];
} mcuBaseReq_t;

typedef struct {
    uint8_t  head;      // --> 0x65
    uint8_t  len;       // --> 包的数据长度
    uint8_t  err;       // --> 错误码: mcu_error_no_e
    uint16_t data[0];
} mcuBaseRsq_t;

#pragma pack(pop)

// 全局变量
static int32_t mFd = -1;

static uint32_t mcu_cmd_timeout_floor_ms(mcu_cmd_e cmd, uint32_t timeout_ms)
{
    uint32_t min_timeout_ms = 100;

    if (cmd == MCU_CMD_FLASH_SECTORERASE) {
        min_timeout_ms = 1000;
    }
    else if (cmd == MCU_CMD_WRITE_DATA) {
        min_timeout_ms = 600;
    }
    else if ((cmd == MCU_CMD_VERIFY_DATA) || (cmd == MCU_CMD_CHECKSUM) ||
             (cmd == MCU_CMD_FLASH_BLANK_CHECK)) {
        min_timeout_ms = 1000;
    }

    return timeout_ms < min_timeout_ms ? min_timeout_ms : timeout_ms;
}

int32_t hwResetMcu(uint32_t duration_us)
{
    int32_t err = FS_SUCCESS;
    CHECK_ERR(pinctrl_set_gpio(PIN_MCU_HW_RST, 0));
    fs_hw_usleep(duration_us > 0 ? duration_us : 1);
    CHECK_ERR(pinctrl_set_gpio(PIN_MCU_HW_RST, 1));
    return err;
}

int32_t mcu_power_on(void)
{
    CHECK_ERR(pinctrl_set_gpio(PIN_MCU_MODE, 0));
    CHECK_ERR(hwResetMcu(FS_MCU_RESET_ACTIVE_MS * 1000));
    fs_hw_msleep(FS_MCU_INIT_MS);
    return FS_SUCCESS;
}

/**
 * @brief 初始化MCU升级模块
 * @return 0表示成功，非0表示失败
 */
int32_t mcu_upgrade_init(void)
{
    FS_LOGI("MCU upgrade module initialized");
    return 0;
}

/**
 * @brief 清理MCU升级模块
 */
void mcu_upgrade_cleanup(void)
{
    // 关闭串口
    if (mFd >= 0) {
        fs_uart_close(mFd);
        mFd = -1;
    }

    FS_LOGI("MCU upgrade module cleaned up");
}

/**
 * @brief 获取当前MCU固件版本
 * @param version 版本信息结构体指针
 * @return 0表示成功，非0表示失败
 */
int32_t mcu_upgrade_get_hw_version(mcu_version_t *version)
{
    int32_t err = FS_SUCCESS;
    int32_t retry = 5;
    char version_data[sizeof(version->hrd.version_string)] = {0};

    if (!version) {
        return FS_ERR_BAD_PARAMS;
    }

    do {
        memset(version, 0, sizeof(*version));

        // 通过 McuProt UART 读取 MCU 版本
        err = mcu_prot_get_mcu_version((uint8_t *)version_data, sizeof(version_data));

        if (err) {
            FS_LOGE("Failed to read MCU version via McuProt: %d, retrying(%d)...", err, retry);
            fs_hw_msleep(100);
            continue;
        }

        break;
    }
    while(retry-- > 0);

    if (err) {
        FS_LOGE("Failed to read MCU version via McuProt: %d", err);
        return err;
    }

    snprintf(g_version.app_mcu, sizeof(g_version.app_mcu), "%s", version_data);
    // 解析版本字符串
    err = mcu_upgrade_parse_version_string(version_data, version);

    if (err) {
        FS_LOGE("Failed to parse MCU version string: %s", version_data);
        return err;
    }

    return err;
}


/*
* fuction:read data from Device
* para: buf for data and buf length
*
* return : if success read data length ,fail -1
*/
static int32_t readFromDevice(void *rcv_buf, int32_t data_len, int32_t sec, int32_t msec)
{
    int32_t len = 0;
    int32_t ret = FS_SUCCESS;
    int32_t timeout_total_ms;
    int64_t deadline_ms = 0;
    const int32_t inter_byte_wait_ms = 20;
    fd_set read_set;
    struct timeval tv;

    if (mFd < 0) {
        FS_LOGE("mFd not init!");
        return FS_ERR_BAD_PARAMS;
    }

    timeout_total_ms = sec * 1000 + msec;
    if (timeout_total_ms < 0) {
        timeout_total_ms = 0;
    }

    if (timeout_total_ms > 0) {
        struct timeval now;
        gettimeofday(&now, NULL);
        deadline_ms = (int64_t)now.tv_sec * 1000LL + (int64_t)now.tv_usec / 1000LL + timeout_total_ms;
    }

    // Phase 1: wait up to the caller timeout for the first byte.
    while (len == 0) {
        int32_t wait_ms = timeout_total_ms;

        if (timeout_total_ms > 0) {
            struct timeval now;
            int64_t now_ms;
            int64_t remain_ms;

            gettimeofday(&now, NULL);
            now_ms = (int64_t)now.tv_sec * 1000LL + (int64_t)now.tv_usec / 1000LL;
            remain_ms = deadline_ms - now_ms;
            if (remain_ms <= 0) {
                break;
            }
            wait_ms = (int32_t)remain_ms;
        }

        FD_ZERO(&read_set);
        FD_SET(mFd, &read_set);
        tv.tv_sec = wait_ms / 1000;
        tv.tv_usec = (wait_ms % 1000) * 1000;

        ret = select(mFd + 1, &read_set, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            return ret;
        }
        if (ret == 0) {
            break;
        }
        if (!FD_ISSET(mFd, &read_set)) {
            continue;
        }

        ret = read(mFd, &((uint8_t *)rcv_buf)[len], data_len - len);
        if (ret > 0) {
            len += ret;
            break;
        }

        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            continue;
        }

        if (ret < 0) {
            return ret;
        }
    }

    // Phase 2: after first byte, keep reading only while bytes continue arriving.
    while (len > 0 && len < data_len) {
        FD_ZERO(&read_set);
        FD_SET(mFd, &read_set);
        tv.tv_sec = inter_byte_wait_ms / 1000;
        tv.tv_usec = (inter_byte_wait_ms % 1000) * 1000;

        ret = select(mFd + 1, &read_set, NULL, NULL, &tv);
        if (ret <= 0 || !FD_ISSET(mFd, &read_set)) {
            break;
        }

        ret = read(mFd, &((uint8_t *)rcv_buf)[len], data_len - len);
        if (ret > 0) {
            len += ret;
            continue;
        }

        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            continue;
        }

        break;
    }

    return len;
}

/*
    * write data to Device
    * para: send_buf,and data_len
    *
    * return: if success datalen,fail -1
    */
static int32_t writeToDevice(const void *send_buf, int32_t data_len, int32_t sec, int32_t msec, bool clear)
{
    int32_t ret;
    fd_set write_set;
    struct timeval tv;

    if (mFd < 0) {
        FS_LOGE("mFd not init!");
        return FS_ERR_BAD_PARAMS;
    }

    if (clear) {
        int32_t value;
        char    temp[8];

        // 清空接收缓冲区中的数据,确保在发送数据之前不会有未读取的数据残留
        do {
            value = readFromDevice(temp, sizeof(temp), 0, 0);
        }
        while (value > 0);
    }

    FD_ZERO(&write_set);
    FD_SET(mFd, &write_set);
    tv.tv_sec = sec;
    tv.tv_usec = msec * 1000;
    ret = select(mFd + 1, 0, &write_set, 0, &tv);

    if (ret <= 0) {
        return ret;
    }

    if (FD_ISSET(mFd, &write_set)) {
        ret = write(mFd, send_buf, data_len);
    }
    else {
        return FS_ERR_TIME_OUT;
    }

    if (data_len != ret) {
        FS_LOGE("write bytes(%d) != expected(%d)", ret, data_len);
        tcflush(mFd, TCOFLUSH);
        return FS_ERR_INTERNAL;
    }

    return FS_SUCCESS;
}

/**
 * @brief 计算与 MCU 通信数据的 CRC16 校验值
 * @param data 数据指针
 * @param dlen 数据长度
 * @param initCrc 初始 CRC 值（目前未使用，固定为0）
 * @return 计算得到的 CRC16 校验值
 */
static uint16_t mcu_calc_crc(const void *data, uint32_t dlen, uint16_t initCrc)
{
    const uint8_t *ptr = (const uint8_t *)data;
    uint32_t i;
    uint16_t tmp16;
    uint16_t CRCValue;
    CRCValue = 0xFFFF;

    for (i = 0; i < dlen; i++) {
        tmp16 = ptr[i];
        tmp16 = tmp16 ^ CRCValue;
        tmp16 = tmp16 ^ (tmp16 << 4);
        tmp16 &= 0xFF;
        CRCValue = ((CRCValue >> 8) ^ (tmp16 << 8) ^ (tmp16 << 3) ^ (tmp16 >> 4));
    }

    initCrc = CRCValue ^ 0xFFFF;
    return initCrc;
}

/**
 * @brief 与 MCU 进行一次完整的命令通信流程
 * @param cmd MCU 命令码
 * @param w_data 写数据指针（可为 NULL）
 * @param w_len 写数据长度（如果 w_data 不为 NULL，则必须大于 0）
 * @param r_data 读数据指针（可为 NULL）
 * @param rlen 读数据长度（如果 r_data 不为 NULL，则必须大于 0）
 * @param timeout_ms 整个通信流程的超时时间（单位为毫秒）
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_base_communicate(mcu_cmd_e cmd, const void *w_data, uint16_t w_len, void *r_data, uint16_t rlen, uint32_t timeout_ms)
{
    int32_t err = MCU_SUCCESS;
    int32_t timeoutS, timeoutMs, count;
    mcuBaseReq_t *req;
    mcuBaseRsq_t *rsp;
    mcuCmdTail_t *reqTail, *rspTail;
    uint8_t cmdBuf[sizeof(*req) + MCU_MAX_DATA_LEN + sizeof(*reqTail)] = {0,};
    uint8_t ackBuf[sizeof(*rsp) + MCU_MAX_DATA_LEN + sizeof(*rspTail)] = {0,};
    uint16_t calc_crc;

    if (((w_data == NULL) && (w_len > 0)) || ((w_data != NULL) && (w_len == 0))
     || ((r_data == NULL) && (rlen  > 0)) || ((r_data != NULL) && (rlen  == 0))
     || w_len > MCU_MAX_DATA_LEN || rlen > MCU_MAX_DATA_LEN) {
        FS_LOGE("Invalid parameters for mcu_base_communicate: w_data=%p, w_len=%u, r_data=%p, rlen=%u",
            w_data, w_len, r_data, rlen);
        return FS_ERR_BAD_PARAMS;
    }

    timeout_ms = mcu_cmd_timeout_floor_ms(cmd, timeout_ms);
    timeoutS  = timeout_ms / 1000;
    timeoutMs = timeout_ms % 1000;
    req = (mcuBaseReq_t *)&cmdBuf[0];
    rsp = (mcuBaseRsq_t *)&ackBuf[0];
    req->head    = MCU_CMD_HEADER;
    req->len     = w_len + 1;
    req->cmd     = (uint8_t)cmd;
    if (w_len > 0) {
        memcpy(req->data, w_data, w_len);
    }
    reqTail      = (mcuCmdTail_t *)&cmdBuf[sizeof(*req) - 1 + req->len];
    reqTail->crc = mcu_calc_crc(req, sizeof(*req) - 1 + req->len, 0);
    // fs_dump_hex((uint8_t *)req, sizeof(*req) - 1 + req->len + sizeof(*reqTail), "MCU CMD:", 0);
    err = writeToDevice(req, sizeof(*req) - 1 + req->len + sizeof(*reqTail), 0, 0, true);

    if (err) {
        FS_LOGE("Failed to write command 0x%02X to device: %d", cmd, err);
        return err;
    }

    count = readFromDevice(rsp, sizeof(*rsp) + rlen + sizeof(*rspTail), timeoutS, timeoutMs);

    if (count == 0) {
        FS_LOGE("Timeout waiting for response to command 0x%02X", cmd);
        return FS_ERR_TIME_OUT;
    }
    else if (count < 0) {
        FS_LOGE("Error reading response to command 0x%02X: %d", cmd, count);
        return count;
    }
    else if (count != (int32_t)(sizeof(*rsp) + rlen + sizeof(*rspTail))) {
        FS_LOGE("Incomplete response to command 0x%02X: expected %d, got %d", cmd, (int32_t)(sizeof(*rsp) + rlen + sizeof(*rspTail)), count);
        return FS_ERR_BAD_PROTOCOL;
    }

    if ((rsp->head != MCU_CMD_HEADER) || (rsp->len != (uint8_t)(rlen + 1))) {
        FS_LOGE("Invalid response header for command 0x%02X: head=0x%02X len=%u expected_len=%u",
                cmd, rsp->head, rsp->len, (uint8_t)(rlen + 1));
        return FS_ERR_BAD_PROTOCOL;
    }


    rspTail = (mcuCmdTail_t *)&ackBuf[sizeof(*rsp) + rlen];
    calc_crc = mcu_calc_crc(rsp, sizeof(*rsp) + rlen, 0);

    if (calc_crc != rspTail->crc) {
        FS_LOGE("CRC mismatch in response to command 0x%02X: expected 0x%04X, got 0x%04X", cmd, rspTail->crc, calc_crc);
        return FS_ERR_CHECK_CRC;
    }

    if ((rsp->err != MCU_SUCCESS) && (rsp->err != MCU_ERR_BLANK_CHECK)) {
        FS_LOGE("Device returned error for command 0x%02X: 0x%02X", cmd, rsp->err);
        return rsp->err;
    }

    if (r_data && (rlen > 0)) {
        memcpy(r_data, rsp->data, rlen);
    }

    return err;
}

/**
 * @brief 查询 MCU 是否进入原厂 bootloader 模式
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_query(void)
{
    int32_t err = FS_SUCCESS;
    uint8_t boot_ver[4];
    err = mcu_base_communicate(MCU_CMD_QUERY, NULL, 0, boot_ver, sizeof(boot_ver), 2);

    if (err < 0) {
        FS_LOGE("Failed to query MCU: %d", err);
        return err;
    }
    else if (err > 0) {
        FS_LOGE("query MCU returned error code: 0x%02X", err);
    }
    else {
        FS_LOGD("MCU query successful, bootloader version: %c%c%02X", boot_ver[1], boot_ver[2], boot_ver[3]);
    }

    return err;
}

/**
 * @brief 设置 MCU 的通信波特率
 * @param baud_rate 期望的波特率，支持以下值：19200、115200、250000、500000、1000000、1500000、2000000
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_set_baud_rate(uint32_t baud_rate)
{
    int32_t err = FS_SUCCESS;
    uint16_t brr;
    uint32_t actual;

    switch (baud_rate) {
        case 19200: {
            brr = 0xCF00; // 实际波特率: 19231
            actual = 19231;
            break;
        }

        case 115200: {
            brr = 0x21FB; // 实际波特率: 115349
            actual = 115349;
            break;
        }

        case 250000: {
            brr = 0x0F00; // 实际波特率: 250000
            actual = 250000;
            break;
        }

        case 500000: {
            brr = 0x0700; // 实际波特率: 500000
            actual = 500000;
            break;
        }

        case 1000000: {
            brr = 0x0300; // 实际波特率: 1000000
            actual = 1000000;
            break;
        }

        case 1500000: {
            brr = 0x01C0; // 实际波特率: 1500000
            actual = 1500000;
            break;
        }

        case 2000000: {
            brr = 0x0100; // 实际波特率: 2000000
            actual = 2000000;
            break;
        }

        default: {
            FS_LOGE("Unsupported baud rate: %u", baud_rate);
            return FS_ERR_BAD_PARAMS;
        }
    }

    err = mcu_base_communicate(MCU_CMD_PPS, &brr, sizeof(brr), NULL, 0, 2);

    if (err < 0) {
        FS_LOGE("Failed to set baud rate(%u): %d", baud_rate, err);
        return err;
    }
    else if (err > 0) {
        FS_LOGE("set baud rate(%u) returned error: 0x%02X", baud_rate, err);
    }
    else {
        FS_LOGD("MCU set baud rate successful: %u -> actual(%u)", baud_rate, actual);
    }

    return err;
}

/**
 * @brief 设置 MCU 的基地址（固件存储起始地址或者读取信息、设置配置的基地址）
 * @param base_addr 基地址
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_set_base_addr(uint32_t base_addr)
{
    int32_t err = FS_SUCCESS;
    err = mcu_base_communicate(MCU_CMD_SET_BASEADDR, &base_addr, sizeof(base_addr), NULL, 0, 2);

    if (err < 0) {
        FS_LOGE("Failed to set base address(0x%08X): %d", base_addr, err);
        return err;
    }
    else if (err > 0) {
        FS_LOGE("set base address(0x%08X) returned error: 0x%02X", base_addr, err);
    }
    else {
        FS_LOGD("MCU set base address successful: 0x%08X", base_addr);
    }

    return err;
}

/**
 * @brief 设置或读取 MCU 的读保据状态，支持 Level0 ~ Level3 共四种保护等级
 * @param protect_level 保护等级，取值范围: 0 ~ 3
 *                      protect_level 为 0x00~0x03 分别代表 Level0 ~ Level3，保护等级限制如下：
 *                        protect_level 为 0x55 则不操作当前读保护等级，只返回当前保护等级。
 *                        Level0：SWD 可读写芯片，bootloader 可读写芯片。
 *                        Level1：SWD 可降级芯片，bootloader 可降级芯片，数据不可读出。
 *                        Level2：SWD 端口物理断开，bootloader 可降级芯片，数据不可读出。
 *                        Level3：SWD 端口物理断开，bootloader 端口物理断开，数据不可读出。
 *                        注意：Level3 保护等级生效后芯片不能再次进行编程，请慎用。
 *                        注意：加密等级更新时，只要从其他等级降级到 Level0，就执行全片擦除。
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_set_protect_level(mcu_rd_level_e protect_level, mcu_rd_level_e *current_level_out)
{
    int32_t err = FS_SUCCESS;
    uint8_t level = (uint8_t)protect_level;
    uint8_t current_level = 0;

    if ((protect_level > MCU_RD_LEVEL_3) && (protect_level != MCU_RD_LEVEL_GET)) {
        FS_LOGE("Invalid protect level: 0x%02X", protect_level);
        return FS_ERR_BAD_PARAMS;
    }
    // MCU 访问等级防护: 可酌情按需放开 level3 的检查, 确需放开的, 需要清楚其后果: 存储在内置 Flash 的校准数据将会丢失, 并且 MCU 固件不再支持升级
    else if (protect_level == MCU_RD_LEVEL_3) {
        FS_LOGE("Protect level 3 is very dangerous as it will permanently disable SWD and bootloader access. "
                "Make sure you understand the consequences before setting this level.");
        return FS_ERR_BAD_PARAMS;
    }
    // MCU 访问等级防护: 可酌情按需放开 level1~2 的检查, 确需放开的, 需要清楚其后果: 存储在内置 Flash 的校准数据将会丢失
    else if ((protect_level > MCU_RD_LEVEL_0) && (protect_level != MCU_RD_LEVEL_GET)) {
        FS_LOGE("Protect level 0x%02X will disable SWD read access and may require a full chip erase to recover. "
                "Make sure you understand the consequences before setting this level.", protect_level);
        return FS_ERR_BAD_PARAMS;
    }

    err = mcu_base_communicate(MCU_CMD_SET_PROTECT_LEVEL, &level, sizeof(level), &current_level, sizeof(current_level), 2);

    if (err < 0) {
        FS_LOGE("Failed to set protect level(0x%02X): %d", protect_level, err);
        return err;
    }
    else if (err > 0) {
        FS_LOGE("set protect level(0x%02X) returned error: 0x%02X", protect_level, err);
    }
    else {
        FS_LOGD("MCU set protect level successful: 0x%02X", protect_level);
    }

    if (current_level_out) {
        *current_level_out = (mcu_rd_level_e)current_level;
    }

    return err;
}

/**
 * @brief 擦除 MCU 内置 Flash 的全部内容
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_flash_chip_erase(void)
{
    int32_t err = FS_SUCCESS;
    err = mcu_base_communicate(MCU_CMD_FLASH_CHIPERASE, NULL, 0, NULL, 0, 2);

    if (err < 0) {
        FS_LOGE("Failed to erase MCU flash chip: %d", err);
        return err;
    }
    else if (err > 0) {
        FS_LOGE("erase MCU flash chip returned error code: 0x%02X", err);
    }
    else {
        FS_LOGD("MCU flash chip erase successful");
    }

    return err;
}

/**
 * @brief 擦除 MCU 内置 Flash 的一个扇区，扇区大小因 MCU 型号的不同而不同, HC32L121JAUB 为 512bytes(MCU_FLASH_SECTOR_SIZE)
 * @param offset 需要擦除的扇区的偏移地址，偏移地址的基准由 mcu_set_base_addr 设置的地址决定，必须是扇区大小的整数倍
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_flash_sector_erase(uint32_t offset)
{
    int32_t err = FS_SUCCESS;

    if (offset % MCU_FLASH_SECTOR_SIZE != 0) {
        FS_LOGE("Invalid sector offset: 0x%08X", offset);
        return FS_ERR_BAD_PARAMS;
    }

    err = mcu_base_communicate(MCU_CMD_FLASH_SECTORERASE, &offset, sizeof(offset), NULL, 0, 10);

    if (err < 0) {
        FS_LOGE("Failed to erase MCU flash sector(0x%08X): %d", offset, err);
        return err;
    }
    else if (err > 0) {
        FS_LOGE("erase MCU flash sector(0x%08X) returned error code: 0x%02X", offset, err);
    }
    // else {
    //     FS_LOGD("MCU flash sector(0x%08X) erase successful", offset);
    // }

    return err;
}

/**
 * @brief 检查 MCU 内置 Flash 是否为空
 * @return 0表示非空，MCU_ERR_BLANK_CHECK 表示为空, 非0表示失败
 */
static int32_t mcu_flash_blank_check(void)
{
    int32_t err = FS_SUCCESS;
    err = mcu_base_communicate(MCU_CMD_FLASH_BLANK_CHECK, NULL, 0, NULL, 0, 100);

    if (err < 0) {
        FS_LOGE("Failed to check MCU flash blank: %d", err);
        return err;
    }
    else if (err > 0 && err != MCU_ERR_BLANK_CHECK) {
        FS_LOGE("check MCU flash blank returned error code: 0x%02X", err);
    }

    return err;
}

/**
 * @brief 向 MCU 写入数据，写入的数据长度不超过 MCU_MAX_WRITE_LEN
 * @param offset 写入数据的偏移地址，偏移地址的基准由 mcu_set_base_addr 设置的地址决定
 * @param data 写入数据的指针，调用者需要保证该指针至少有 len 字节的可用数据
 * @param len 需要写入的数据长度，单位为字节，必须大于 0 且不超过 MCU_MAX_WRITE_LEN
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_write_data(uint32_t offset, const void *data, uint8_t len)
{
    int32_t err = FS_SUCCESS;
    uint8_t cmd_data[sizeof(offset) + MCU_MAX_WRITE_LEN] = {
        ((offset >>  0) & 0xFF),
        ((offset >>  8) & 0xFF),
        ((offset >> 16) & 0xFF),
        ((offset >> 24) & 0xFF),
        0,
    };

    if (len > MCU_MAX_WRITE_LEN) {
        FS_LOGE("Data length(%u) exceeds maximum allowed length(%u)", len, MCU_MAX_WRITE_LEN);
        return FS_ERR_BAD_PARAMS;
    }
    else if (len % 2 != 0) {
        FS_LOGW("Data length (%u) is not a multiple of 2, which may cause slower write performance", len);
    }

    memcpy(&cmd_data[4], data, len);
    err = mcu_base_communicate(MCU_CMD_WRITE_DATA, cmd_data, sizeof(offset) + len, NULL, 0, 100);

    if (err < 0) {
        FS_LOGE("Failed to write data(0x%04X, %u): %d", offset, len, err);
        return err;
    }
    else if (err > 0) {
        FS_LOGE("write data(0x%04X, %u) returned error: 0x%02X", offset, len, err);
    }
    // else {
    //     FS_LOGD("MCU write data(0x%04X, %3u) successful: 0x%02X", offset, len, ((uint8_t*)data)[0]);
    // }

    return err;
}

/**
 * @brief 从 MCU 读取数据，读取的数据长度不超过 MCU_MAX_READ_LEN
 * @param offset 读取数据的偏移地址，偏移地址的基准由 mcu_set_base_addr 设置的地址决定
 * @param data 读取数据的缓冲区指针，调用者需要保证该缓冲区至少有 len 字节的可用空间
 * @param len 需要读取的数据长度，单位为字节，必须大于 0 且不超过 MCU_MAX_READ_LEN
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_read_data(uint32_t offset, void *data, uint8_t len)
{
    int32_t err = FS_SUCCESS;
    uint8_t cmd_data[] = {
        ((offset >>  0) & 0xFF),
        ((offset >>  8) & 0xFF),
        ((offset >> 16) & 0xFF),
        ((offset >> 24) & 0xFF),
        len
    };
    CHECK_PTR(data);
    err = mcu_base_communicate(MCU_CMD_READ_DATA, cmd_data, sizeof(cmd_data), data, len, 100);

    if (err < 0) {
        FS_LOGE("Failed to read data(0x%04X, %u): %d", offset, len, err);
        return err;
    }
    else if (err > 0) {
        FS_LOGE("read data(0x%04X, %u) returned error: 0x%02X", offset, len, err);
    }
    // else {
    //     FS_LOGD("MCU read  data(0x%04X, %3u) successful: 0x%02X", offset, len, ((uint8_t*)data)[0]);
    // }

    return err;
}

/**
 * @brief 获取 MCU 内置 Flash 中指定地址范围数据的 CRC16 校验值(只能在 level0 保护等级下使用)
 * @param offset 需要校验的数据的偏移地址，偏移地址的基准由 mcu_set_base_addr 设置的地址决定
 * @param len 需要校验的数据长度，单位为字节，必须大于 0 且不超过 MCU_FLASH_MAX_SZIE
 * @param crc 输出参数，返回计算得到的 CRC16 校验值
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_get_verify_crc(uint32_t offset, uint32_t len, uint16_t *crc)
{
    int32_t err = FS_SUCCESS;
    uint8_t cmd_data[] = {
        ((offset >>  0) & 0xFF),
        ((offset >>  8) & 0xFF),
        ((offset >> 16) & 0xFF),
        ((offset >> 24) & 0xFF),
        ((len    >>  0) & 0xFF),
        ((len    >>  8) & 0xFF),
        ((len    >> 16) & 0xFF),
        ((len    >> 24) & 0xFF),
    };
    CHECK_PTR(crc);

    if (len == 0 || len > MCU_FLASH_MAX_SZIE) {
        FS_LOGE("Length (0x%04X) is out of bounds (0x%04X)", len, MCU_FLASH_MAX_SZIE);
        return FS_ERR_BAD_PARAMS;
    }

    err = mcu_base_communicate(MCU_CMD_VERIFY_DATA, cmd_data, sizeof(cmd_data), crc, sizeof(*crc), 500);

    if (err < 0) {
        FS_LOGE("Failed to get verify CRC(0x%04X, %u): %d", offset, len, err);
        return err;
    }
    else if (err > 0) {
        FS_LOGE("get verify CRC(0x%04X, %u) returned error: 0x%02X", offset, len, err);
    }
    // else {
    //     FS_LOGD("MCU get verify CRC(0x%04X, %u) successful: 0x%04X", offset, len, *crc);
    // }

    return err;
}

/**
 * @brief 获取 MCU 内置 Flash 中指定地址范围数据的 CRC16 校验值(只能在 level0 ~ level2 保护等级下使用)
 * @param offset 需要校验的数据的偏移地址，偏移地址的基准由 mcu_set_base_addr 设置的地址决定，必须是扇区大小的整数倍
 * @param len 需要校验的数据长度，单位为字节，必须是扇区大小的整数倍且不超过 MCU_FLASH_MAX_SZIE
 * @param crc 输出参数，返回计算得到的 CRC16 校验值
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_get_verify_crc_protected(uint32_t offset, uint32_t len, uint16_t *crc)
{
    int32_t err = FS_SUCCESS;
    uint8_t cmd_data[] = {
        ((offset >>  0) & 0xFF),
        ((offset >>  8) & 0xFF),
        ((offset >> 16) & 0xFF),
        ((offset >> 24) & 0xFF),
        ((len    >>  0) & 0xFF),
        ((len    >>  8) & 0xFF),
        ((len    >> 16) & 0xFF),
        ((len    >> 24) & 0xFF),
    };
    CHECK_PTR(crc);

    if (offset % MCU_FLASH_SECTOR_SIZE != 0) {
        FS_LOGE("Offset (0x%04X) is not aligned to sector boundary (0x%04X)", offset, MCU_FLASH_SECTOR_SIZE);
        return FS_ERR_BAD_PARAMS;
    }
    else if (len % MCU_FLASH_SECTOR_SIZE != 0) {
        FS_LOGE("Length (0x%04X) is not aligned to sector boundary (0x%04X)", len, MCU_FLASH_SECTOR_SIZE);
        return FS_ERR_BAD_PARAMS;
    }
    else if (len == 0 || len > MCU_FLASH_MAX_SZIE) {
        FS_LOGE("Length (0x%04X) is out of bounds (0x%04X)", len, MCU_FLASH_MAX_SZIE);
        return FS_ERR_BAD_PARAMS;
    }

    err = mcu_base_communicate(MCU_CMD_CHECKSUM, cmd_data, sizeof(cmd_data), crc, sizeof(*crc), 24);

    if (err < 0) {
        FS_LOGE("Failed to get verify CRC(0x%04X, %u): %d", offset, len, err);
        return err;
    }
    else if (err > 0) {
        FS_LOGE("get verify CRC(0x%04X, %u) returned error: 0x%02X", offset, len, err);
    }
    else {
        FS_LOGD("MCU get verify CRC(0x%04X, %u) successful: 0x%04X", offset, len, *crc);
    }

    return err;
}

static int32_t mcu_jump_app(uint32_t app_addr)
{
    int32_t err = FS_SUCCESS;
    err = mcu_base_communicate(MCU_CMD_JUMP, &app_addr, sizeof(app_addr), NULL, 0, 2);

    if (err < 0) {
        FS_LOGE("Failed to jump to application(0x%08X): %d", app_addr, err);
        return err;
    }
    else if (err > 0) {
        FS_LOGE("jump to application(0x%08X) returned error code: 0x%02X", app_addr, err);
    }
    else {
        FS_LOGD("MCU jump to application(0x%08X) successful", app_addr);
    }

    return err;
}

/**
 * @brief 进入 bootloader 模式（不依赖 boot 引脚）
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_handshake_no_boot(void)
{
    int32_t err = FS_SUCCESS;
    struct timeval tv_start = {0};
    struct timeval tv_now   = {0};
    uint64_t elapsed_us = 0;
    uint8_t handshake_cmd[] = { 0xFF, 0x56, 0xFF, 0x56, }; // 特殊握手命令，需要在波特率 500000、8/N/1 配置下持续发送至少 15ms

    // 将波特率更改为 500000
    if (mFd != -1) {
        fs_uart_close(mFd);
        mFd = -1;
    }

    mFd = fs_uart_open_init(FS_MCU_UART_DEV_NAME, FS_MCU_UART_SPEED_HANDSHAKE_NO_BOOT);

    if (mFd < 0) {
        FS_LOGE("Failed to open UART(%s, %u) for handshake: %d", FS_MCU_UART_DEV_NAME, FS_MCU_UART_SPEED_HANDSHAKE_NO_BOOT, mFd);
        return FS_ERR_IO;
    }

    // 硬件复位 MCU 并在至少 15ms 内持续发送握手命令，等待 MCU 进入 bootloader 模式
    CHECK_ERR(hwResetMcu(FS_MCU_RESET_ACTIVE_MS * 1000));
    gettimeofday(&tv_start, NULL);

    do {
        writeToDevice(handshake_cmd, sizeof(handshake_cmd), 0, 0, false);
        gettimeofday(&tv_now, NULL);
        elapsed_us = (uint64_t)(tv_now.tv_sec - tv_start.tv_sec) * 1000000ULL
                    + (uint64_t)(tv_now.tv_usec - tv_start.tv_usec);
    }
    while (elapsed_us < (FS_MCU_HANDSHAKE_TIMEOUT_MS * 1000ULL));

    // 切换到 bootloader 模式的默认通信波特率: 115200
    fs_uart_close(mFd);
    mFd = fs_uart_open_init(FS_MCU_UART_DEV_NAME, FS_MCU_UART_SPEED_UPDATE);

    if (mFd < 0) {
        FS_LOGE("Failed to open UART(%s, %u) for query: %d", FS_MCU_UART_DEV_NAME, FS_MCU_UART_SPEED_UPDATE, mFd);
        return FS_ERR_IO;
    }

    // 查询 MCU 是否能正常通信
    CHECK_ERR(mcu_query());
    return err;
}

/**
 * @brief 与 MCU 进行握手通信(依赖 boot 引脚)
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_handshake(void)
{
    int32_t err = FS_SUCCESS;

    if (mFd != -1) {
        fs_uart_close(mFd);
        mFd = -1;
    }

    mFd = fs_uart_open_init(FS_MCU_UART_DEV_NAME, FS_MCU_UART_SPEED_UPDATE);

    if (mFd < 0) {
        FS_LOGE("Failed to open UART(%s, %u) for query: %d", FS_MCU_UART_DEV_NAME, FS_MCU_UART_SPEED_UPDATE, mFd);
        return FS_ERR_IO;
    }

    CHECK_ERR(hwResetMcu(FS_MCU_RESET_ACTIVE_MS * 1000));
    fs_hw_msleep(10);
    // 查询 MCU 是否能正常通信
    CHECK_ERR(mcu_query());
    return err;
}

/**
 * @brief 检查 MCU 是否满足升级要求，并进行必要的设置
 * @param fw_base_addr 固件存储的基地址，通常为 MCU 内置 Flash 的起始地址
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_check_and_setup(uint32_t fw_base_addr)
{
    int32_t err = FS_SUCCESS;
    char mcu_model[sizeof(MCU_MODEL_NAME)] = {0};
    mcu_rd_level_e mcu_rd_level;
    CHECK_ERR(mcu_set_baud_rate(FS_MCU_UART_SPEED_UPDATE));
    CHECK_ERR(mcu_set_base_addr(MCU_INFO_BASE_ADDR));
    CHECK_ERR(mcu_read_data(MCU_INFO_OFFSET_MODEL_NAME, mcu_model, sizeof(mcu_model) - 1));
    mcu_model[sizeof(mcu_model) - 1] = '\0';

    if (memcmp(mcu_model, MCU_MODEL_NAME, sizeof(mcu_model) - 1) != 0) {
        FS_LOGE("MCU model mismatch: expected '%s', got '%s'", MCU_MODEL_NAME, mcu_model);
        return FS_ERR_NO_DEVICE;
    }

    CHECK_ERR(mcu_set_protect_level(MCU_RD_LEVEL_GET, &mcu_rd_level));

    if (mcu_rd_level == MCU_RD_LEVEL_3) {
        // level3 无法二次烧录
        FS_LOGE("MCU protect level3 not supported fw upgrade!");
        return FS_ERR_UNSUPPORT;
    }
    else if ((mcu_rd_level == MCU_RD_LEVEL_1) || (mcu_rd_level == MCU_RD_LEVEL_2)) {
        // level1~2 需要降级到 level0 才能进行升级，降级会导致 MCU 执行全片擦除
        FS_LOGW("MCU is currently at protect level %d, which may require a full chip erase to recover. Attempting to set to level 0...", mcu_rd_level);
        CHECK_ERR(mcu_set_protect_level(MCU_RD_LEVEL_0, NULL));
    }
    else if (mcu_rd_level != MCU_RD_LEVEL_0) {
        FS_LOGE("Unsupported MCU protect level: %d", mcu_rd_level);
        return FS_ERR_UNSUPPORT;
    }

    FS_LOGD("MCU model: %s, protect level: %d", mcu_model, mcu_rd_level);
    CHECK_ERR(mcu_set_base_addr(fw_base_addr)); // MCU_FW_START_ADDRESS
    return err;
}

/**
 * @brief 将固件数据写入 MCU 内置 Flash，并校验写入结果
 * @param fwBuf 固件数据缓冲区指针，调用者需要保证该指针至少有 fwLen 字节的可用数据
 * @param fwLen 固件数据长度，单位为字节，必须大于 0 且不超过 flashSize
 * @param flashSize MCU 内置 Flash 的总大小，单位为字节
 * @param block_size 写入时的块大小，单位为字节，必须是 MCU 内置 Flash 的扇区大小的整数倍，HC32L121JAUB 的扇区大小为 512bytes(MCU_FLASH_SECTOR_SIZE)
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_upload_firmware(const uint8_t *fwBuf, uint32_t fwLen,
                                   uint32_t flashSize, uint16_t block_size)
{
    int32_t err = FS_SUCCESS;
    int32_t retry;
    uint32_t offset, count;
    uint16_t fw_crc, read_crc;

    if ((fwBuf == NULL) || (fwLen == 0)) {
        FS_LOGE("Invalid firmware buffer or length: fwBuf=%p, fwLen=%u", fwBuf, fwLen);
        return FS_ERR_BAD_PARAMS;
    }
    else if (fwLen > flashSize) {
        FS_LOGE("fw size(%u) > flash size(%u).", fwLen, flashSize);
        return FS_ERR_BUF_OVERFLOW;
    }

    // fs_dump_hex(fwBuf, fwLen, "Firmware to upload:", 0);
    fw_crc = mcu_calc_crc(fwBuf, fwLen, 0);
    FS_LOGD("(1/3)Erase sector...");

    for (offset = 0; offset < fwLen; offset += block_size) {
        if ((offset % (MCU_FLASH_SECTOR_SIZE * 20) == 0) || (offset + block_size >= fwLen)) {
            FS_LOGD("Erased : 0x%05X/0x%05X(%3d%%)", offset, fwLen, (offset * 100) / fwLen);
        }

        retry = 3;

        do {
            CONTINUE_ON_ERR(err = mcu_flash_sector_erase(offset));
            break;
        }
        while (--retry > 0);

        if (err) {
            FS_LOGE("Failed to erase flash sector at offset 0x%08X", offset);
            return FS_ERR_IO;
        }

        if (offset + block_size >= fwLen) {
            FS_LOGD("Erased : 0x%05X/0x%05X(%3d%%)", fwLen, fwLen, 100);
        }
    }

    FS_LOGD("(2/3)Writing firmware data...");
    for (offset = 0; offset < fwLen;) {
        retry = 3;
        count = (fwLen - offset) > MCU_MAX_WRITE_LEN ? MCU_MAX_WRITE_LEN : (fwLen - offset);

        if ((offset % (MCU_MAX_WRITE_LEN * 20) == 0) || (offset + count >= fwLen)) {
            FS_LOGD("Written: 0x%05X/0x%05X(%3d%%)", offset, fwLen, (offset * 100) / fwLen);
        }

        err = FS_SUCCESS;

        do {
            if (err) {
                fs_hw_msleep(100); // 等待 100ms 后重试
            }

            CONTINUE_ON_ERR(err = mcu_write_data(offset, fwBuf + offset, count));
            break;
        }
        while (--retry > 0);

        if (err) {
            FS_LOGE("Failed to write firmware data at offset 0x%08X", offset);
            return FS_ERR_IO;
        }

        offset += count;

        if (offset >= fwLen) {
            FS_LOGD("Written: 0x%05X/0x%05X(%3d%%)", offset, fwLen, 100);
        }
    }

    FS_LOGD("(3/3)Verifying firmware CRC...");
    CHECK_ERR(mcu_get_verify_crc(0, fwLen, &read_crc));

    if (read_crc != (fw_crc = mcu_calc_crc(fwBuf, fwLen, 0))) {
        const char *skip_verify = getenv("MCU_ROM_ISP_SKIP_VERIFY");
        const bool allow_skip = (skip_verify != NULL) && (skip_verify[0] != '\0') &&
                                (strcmp(skip_verify, "0") != 0);

        FS_LOGE("Firmware CRC mismatch: expected 0x%04X, got 0x%04X", fw_crc, read_crc);
        if (!allow_skip) {
            return FS_ERR_CHECK_CRC;
        }

        FS_LOGW("MCU_ROM_ISP_SKIP_VERIFY set; accepting programmed image at base 0x%08X",
                (uint32_t)MCU_FW_START_ADDRESS);
    }

    FS_LOGI("Firmware upload successful, CRC: 0x%04X", fw_crc);
    return err;
}

/**
 * @brief 执行升级 MCU 固件的动作
 * @param fw_buf 已加载并校验通过的固件数据指针
 * @param fw_len 固件长度
 * @return 0表示成功，非0表示失败
 */
static int32_t mcu_upgrade_run(const uint8_t *fw_buf, uint32_t fw_len)
{
    int32_t err = FS_SUCCESS;
    int32_t mode_err = FS_SUCCESS;
    int32_t prot_err = FS_SUCCESS;
    int32_t retry = 3;
    int32_t attempt = 0;

    // 参数合法性检查
    if (!fw_buf) {
        FS_LOGE("Invalid parameter: fw_buf is NULL");
        return FS_ERR_BAD_PARAMS;
    }
    else if (fw_len == 0) {
        FS_LOGE("Invalid parameter: fw_len is 0");
        return FS_ERR_BAD_PARAMS;
    }
    else if (MCU_FW_START_ADDRESS >= MCU_FLASH_MAX_SZIE) {
        FS_LOGE("Invalid ROM ISP program base: 0x%08X exceeds MCU flash size %u",
                (uint32_t)MCU_FW_START_ADDRESS, MCU_FLASH_MAX_SZIE);
        return FS_ERR_BAD_PARAMS;
    }
    else if (fw_len > (MCU_FLASH_MAX_SZIE - MCU_FW_START_ADDRESS)) {
        FS_LOGE("Invalid parameter: fw_len (%u) exceeds MCU flash window [%08X, %08X)",
                fw_len, (uint32_t)MCU_FW_START_ADDRESS, MCU_FLASH_MAX_SZIE);
        return FS_ERR_BAD_PARAMS;
    }

    CHECK_ERR(pinctrl_set_gpio(PIN_MCU_MODE, 1));
    mcu_prot_deinit();

    // 挂起 MCU 日志读取，释放串口
    mcu_log_reader_suspend();

    do {
        attempt = 4 - retry;
        FS_LOGI("Starting MCU ROM ISP firmware upgrade (attempt %d/%d, base=0x%08X)...",
                attempt, 3, (uint32_t)MCU_FW_START_ADDRESS);

        err = mcu_handshake();

        if (err) {
            FS_LOGE("Handshake failed, retry %d...", retry);
            continue;
        }

        err = mcu_check_and_setup(MCU_FW_START_ADDRESS);

        if (err) {
            FS_LOGE("MCU check and setup failed, retry %d...", retry);
            continue;
        }

        err = mcu_upload_firmware(fw_buf, fw_len, MCU_FLASH_MAX_SZIE - MCU_FW_START_ADDRESS,
                                  MCU_FLASH_SECTOR_SIZE);

        if (err) {
            FS_LOGE("Firmware upload failed, retry %d...", retry);
            continue;
        }

        break;
    }
    while (--retry > 0);

    // 升级完成，关闭串口
    if (mFd >= 0) {
        fs_uart_close(mFd);
        mFd = -1;
    }

    mode_err = pinctrl_set_gpio(PIN_MCU_MODE, 0);
    if (mode_err) {
        FS_LOGE("failed to leave MCU ROM ISP mode: %d", mode_err);
        err = mode_err;
    }

    // Resume log reader before reset to capture early boot logs.
    mcu_log_reader_resume();

    FS_LOGI("Resetting MCU to apply new firmware...");
    hwResetMcu(FS_MCU_RESET_ACTIVE_MS * 1000);
    fs_hw_msleep(FS_MCU_INIT_MS); // 等待 MCU 加载固件并初始化完毕
    prot_err = mcu_prot_init();
    if (prot_err) {
        FS_LOGE("restore MCU protocol failed after ROM ISP upgrade: %d", prot_err);
    }

    if (err) {
        FS_LOGE("MCU firmware upgrade FAILED after %d attempts!", 3);
        return err;
    }

    FS_LOGI("MCU firmware upgrade completed SUCCESSFULLY!");
    return prot_err;
}

/**
 * @brief 使用指定路径的固件文件, 升级 MCU 固件
 * @param firmware_path 固件文件路径
 * @return 0表示成功，非0表示失败
 */
int32_t mcu_upgrade_fw(const char *firmware_path)
{
    int32_t err = FS_SUCCESS;
    mcu_version_t current_version = {0};
    mcu_version_t target_version  = {0};
    uint8_t *fw_buf = NULL;
    uint32_t fw_len = 0;

    if (!firmware_path) {
        return FS_ERR_BAD_PARAMS;
    }

    err = mcu_upgrade_load_firmware(firmware_path, &target_version, &fw_buf, &fw_len);
    if (err) {
        FS_LOGE("Failed to load firmware for upgrade: %d", err);
        return err;
    }

    // 执行升级
    err = mcu_upgrade_run(fw_buf, fw_len);

    if(fw_buf) {
        FS_LOGI("free fw buff");
        free(fw_buf);
        fw_buf = NULL;
    }

    if (err) {
        FS_LOGE("MCU firmware upgrade failed: %d", err);
        return err;
    }

    CHECK_ERR(mcu_upgrade_get_hw_version(&current_version));
    FS_LOGI("MCU firmware upgraded to version: %s", current_version.hrd.version_string);
    snprintf(g_version.app_mcu, sizeof(g_version.app_mcu), "%s", current_version.hrd.version_string);

    if (mcu_upgrade_compare_version(&current_version, &target_version) != 0) {
        FS_LOGE("MCU version mismatch after upgrade! Expected: %s", target_version.hrd.version_string);
        return FS_ERR_INTERNAL;
    }

    return err;
}
