#!/usr/bin/env python3
"""编译真实驱动，用内存模拟统一内核的共享参数和 DMA。"""
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]
SOURCE = r'''
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

static int test_open(const char *path, int flags, ...);
static int test_ioctl(int fd, unsigned long request, ...);
static ssize_t test_write(int fd, const void *buffer, size_t length);
static int test_close(int fd);
static void *test_mmap(void *address, size_t length, int prot, int flags, int fd, off_t offset);
static int test_munmap(void *address, size_t length);

#define open test_open
#define ioctl test_ioctl
#define write test_write
#define close test_close
#define mmap test_mmap
#define munmap test_munmap
#include "driver/src/yousee_lidar_api.c"

static lidarParam shared;
static _Alignas(uint64_t) unsigned char dma[122912 * LIDAR_PCL_RING_BATCHES];
static int fail_dma_map;
static int mapped_params, mapped_dma, open_count;

static int test_open(const char *path, int flags, ...)
{
    return ++open_count;
}

static int test_ioctl(int fd, unsigned long request, ...)
{
    return 0;
}

static ssize_t test_write(int fd, const void *buffer, size_t length)
{
    return length;
}

static int test_close(int fd)
{
    --open_count;
    return 0;
}

static void *test_mmap(void *address, size_t length, int prot, int flags, int fd, off_t offset)
{
    assert(flags == MAP_PRIVATE);
    if (length == LIDAR_PARAM_SIZE) {
        assert(prot == (PROT_READ | PROT_WRITE));
        ++mapped_params;
        return &shared;
    }
    assert(length == LIDAR_DDR_PHYSIZE && prot == PROT_READ);
    if (fail_dma_map) {
        errno = EIO;
        return MAP_FAILED;
    }
    ++mapped_dma;
    return dma;
}

static int test_munmap(void *address, size_t length)
{
    if (address == &shared) {
        assert(length == LIDAR_PARAM_SIZE);
        --mapped_params;
    } else {
        assert(address == dma && length == LIDAR_DDR_PHYSIZE);
        --mapped_dma;
    }
    return 0;
}

static void reset_kernel_layout(void)
{
    shared = (lidarParam){
        .mode = 0, .bram1 = 0x4000, .tran_once = 10, .tran_total = 800,
        .tranSize = 1552, .tranSizeOnce = 15520, .tranSizeTotal = 1241600,
        .recvCntTotal = 80, .recvCnt = 2, .woffset = 15520,
    };
}

static void assert_closed(void)
{
    assert(!lidar_get_bufaddr());
    assert(!open_count && !mapped_params && !mapped_dma);
}

int main(void)
{
    static const unsigned int expected_bytes[] = {15520, 19360, 9680, 2312, 98312, 122912};
    static const unsigned int expected_bram1[] = {0x4000, 0x4e20, 0x2710, 0, 0, 0};
    static const unsigned int expected_batches[] = {80, 80, 2, 168, 56, 56};
    static const unsigned int legacy_mode[] = {4, 0, 5, 1, 2, 3};
    assert(lidar_api_init() == -1);
    assert(lidar_api_init_profile(NULL) == -1);
    assert(lidar_api_init_mode(6) == -1 && errno == EINVAL);
    lidar_switch_mode(MODE_PCL);
    assert(errno == EINVAL);
    assert_closed();

    /* 旧内核的参数页缺少统一布局字段，初始化失败时保持原样。 */
    shared = (lidarParam){ .roffset = 8, .woffset = 16 };
    assert(lidar_api_init_mode(MODE_GRY) == -3 && errno == EPROTO);
    assert(shared.roffset == 8 && shared.woffset == 16 && shared.tranSizeOnce == 0);
    assert_closed();
    reset_kernel_layout();
    fail_dma_map = 1;
    assert(lidar_api_init_mode(MODE_GRY) == -2);
    assert(shared.mode == MODE_PCL && shared.woffset == 15520);
    assert_closed();
    fail_dma_map = 0;

    for (unsigned mode = 0; mode < 6; ++mode) {
        unsigned int bytes = expected_bytes[mode];
        reset_kernel_layout();
        assert(lidar_api_init_mode(mode) == 0);
        assert(lidar_api_init_mode(mode) == -1 && errno == EINVAL);
        assert(shared.mode == mode && shared.bram1 == expected_bram1[mode]);
        assert(shared.tranSizeOnce == bytes && shared.recvCntTotal == expected_batches[mode]);
        assert(shared.tranSizeTotal == bytes * expected_batches[mode]);
        if (mode == MODE_PCL) {
            assert(shared.roffset == bytes && shared.woffset == bytes && shared.recvCnt == 2);
        } else {
            assert(shared.roffset == 0 && shared.woffset == 0 && shared.recvCnt == 0);
        }
        assert(!lidar_get_bufaddr());

        memset(dma, 0x12, bytes);
        memset(dma + bytes, 0x34, bytes);
        shared.roffset = 0;
        shared.woffset = bytes;
        const unsigned char *block = lidar_get_bufaddr();
        assert(block && block[0] == 0x12 && block[bytes - 1] == 0x12 && shared.roffset == bytes);
        dma[0] = 0;
        assert(block[0] == 0x12);
        shared.woffset = 0;
        block = lidar_get_bufaddr();
        assert(block && block[0] == 0x34 && block[bytes - 1] == 0x34 && shared.roffset == 0);
        assert(!lidar_get_bufaddr());
        shared.roffset = 1;
        errno = 0;
        assert(!lidar_get_bufaddr() && errno == EPROTO && shared.roffset == 1);
        shared.roffset = 0;
        shared.woffset = bytes * 2;
        assert(!lidar_get_bufaddr() && errno == EPROTO && shared.roffset == 0);
        lidar_switch_mode(UINT32_MAX);
        assert(errno == EINVAL && shared.mode == mode && shared.woffset == bytes * 2);
        assert(lidar_api_exit() == 0);
        assert_closed();
    }

    reset_kernel_layout();
    shared.pcl_ring_magic = LIDAR_PCL_RING_MAGIC;
    shared.pcl_write_seq = UINT32_MAX;
    assert(lidar_api_init_mode(MODE_PCL) == 0);
    assert(shared.pcl_ring_enabled && shared.pcl_read_seq == UINT32_MAX);
    assert(!lidar_get_bufaddr());
    unsigned int bytes = expected_bytes[MODE_PCL];
    memset(dma + (LIDAR_PCL_RING_BATCHES - 1) * bytes, 0x46, bytes);
    memset(dma, 0x57, bytes);
    shared.pcl_write_seq = 1;
    const unsigned char *block = lidar_get_bufaddr();
    assert(block && block[0] == 0x46 && block[bytes - 1] == 0x46 && shared.pcl_read_seq == 0);
    block = lidar_get_bufaddr();
    assert(block && block[0] == 0x57 && block[bytes - 1] == 0x57 && shared.pcl_read_seq == 1);
    assert(!lidar_get_bufaddr());
    shared.pcl_write_seq += LIDAR_PCL_RING_BATCHES + 1;
    assert(!lidar_get_bufaddr() && errno == EPROTO);
    assert(lidar_api_exit() == 0 && !shared.pcl_ring_enabled);
    assert_closed();

    reset_kernel_layout();
    shared.pcl_ring_magic = LIDAR_PCL_RING_MAGIC;
    assert(lidar_api_init_mode(MODE_GRY) == 0 && !shared.pcl_ring_enabled);
    bytes = expected_bytes[MODE_GRY];
    shared.pcl_write_seq = 1;
    assert(!lidar_get_bufaddr());
    memset(dma, 0x68, bytes);
    shared.woffset = bytes;
    block = lidar_get_bufaddr();
    assert(block && block[0] == 0x68 && block[bytes - 1] == 0x68 && shared.roffset == bytes);
    assert(shared.pcl_read_seq == 0);
    assert(lidar_api_exit() == 0);
    assert_closed();

    reset_kernel_layout();
    for (unsigned id = 0; id < 6; ++id) {
        const ModeProfile *profile = lidar_mode_profile_get(id);
        ModeProfile malformed = *profile;
        ++malformed.abi_version;
        assert(lidar_api_init_profile(&malformed) == -1 && errno == EINVAL);
        assert(lidar_api_init_profile(profile) == 0 && shared.mode == legacy_mode[id]);
        assert(lidar_api_exit() == 0);
        assert_closed();
    }
}
'''
with tempfile.TemporaryDirectory(prefix='ys-runtime-driver-') as temp:
    subprocess.run(['sh', str(ROOT/'driver/src/gen_version.sh'),
                    str(Path(temp)/'yousee_lidar_ver.h')], cwd=ROOT/'driver', check=True,
                   capture_output=True)
    source = Path(temp) / 'test.c'
    source.write_text(SOURCE)
    target = Path(temp) / 'test'
    subprocess.run(['gcc', '-std=gnu11', '-ffunction-sections', '-fdata-sections',
                    '-I'+str(ROOT), '-I'+str(ROOT/'driver/inc'), '-I'+temp, str(source),
                    '-Wl,--gc-sections', '-o', str(target)], check=True)
    subprocess.run([str(target)], check=True)
print('统一驱动六种布局、双缓冲回绕、旧接口映射及失败清理检查通过')
