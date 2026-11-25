
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <getopt.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <assert.h>
#include "fs_log.h"

#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
#include <sys/mman.h>
#include <pthread.h>
#include <dirent.h>

#include "csi_interface.h"

#define V4L2_MODE_VIDEO         0x0002
#define V4L2_MODE_IMAGE         0x0003
#define V4L2_MODE_PREVIEW       0x0004

#define ALIGN_16B(x)            (((x) + (15)) & ~(15))

#define USE_V4L2_MEMORY_TYPE    V4L2_MEMORY_MMAP

#define USE_MEMORY_MALLOC_TYPE  MEM_TYPE_CDX_NEW

#define CSI_SUBDEV_PATH_ENV     "CSI_SUBDEV_PATH"
#define CSI_SUBDEV_DIR          "/dev"
#define CSI_SUBDEV_PREFIX       "v4l-subdev"

static bool csi_is_sensor_like_subdev(int fd)
{
    struct v4l2_queryctrl queryctrl;

    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = V4L2_CID_LINK_FREQ;
    if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) == 0) {
        return true;
    }

    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = V4L2_CID_PIXEL_RATE;
    if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) == 0) {
        return true;
    }

    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = V4L2_CID_EXPOSURE;
    if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) == 0) {
        return true;
    }

    return false;
}

static int csi_auto_discover_subdev(csi_info_t *csi)
{
    DIR *dir;
    struct dirent *entry;
    int sensor_fd = -1;
    char sensor_path[64] = {0};
    int sensor_like_count = 0;

    dir = opendir(CSI_SUBDEV_DIR);
    if (dir == NULL) {
        FS_LOGE("Open %s failed: %d", CSI_SUBDEV_DIR, errno);
        return -1;
    }

    while ((entry = readdir(dir)) != NULL) {
        char candidate_path[64];
        int candidate_fd;

        if (strncmp(entry->d_name, CSI_SUBDEV_PREFIX, strlen(CSI_SUBDEV_PREFIX)) != 0) {
            continue;
        }

        if (snprintf(candidate_path, sizeof(candidate_path), "%s/%s", CSI_SUBDEV_DIR, entry->d_name) >=
            (int)sizeof(candidate_path)) {
            continue;
        }

        candidate_fd = open(candidate_path, O_RDWR | O_NONBLOCK, 0);
        if (candidate_fd < 0) {
            continue;
        }

        if (csi_is_sensor_like_subdev(candidate_fd)) {
            sensor_like_count++;
            if (sensor_like_count == 1) {
                sensor_fd = candidate_fd;
                if (snprintf(sensor_path, sizeof(sensor_path), "%s", candidate_path) >= (int)sizeof(sensor_path)) {
                    close(sensor_fd);
                    sensor_fd = -1;
                    sensor_like_count = 0;
                    sensor_path[0] = '\0';
                }
            } else {
                close(candidate_fd);
            }
            continue;
        }

        close(candidate_fd);
    }

    closedir(dir);

    if (sensor_like_count == 1 && sensor_fd >= 0) {
        if (snprintf(csi->subdev_path, sizeof(csi->subdev_path), "%s", sensor_path) < (int)sizeof(csi->subdev_path)) {
            csi->subdev_fd = sensor_fd;
            FS_LOGI("Auto discovered CSI sensor subdev %s", csi->subdev_path);
            return 0;
        }
        close(sensor_fd);
    }

    if (sensor_fd >= 0) {
        close(sensor_fd);
    }

    if (sensor_like_count > 1) {
        FS_LOGW("Multiple sensor-like subdevs detected (%d); set %s explicitly", sensor_like_count,
                CSI_SUBDEV_PATH_ENV);
    }

    csi->subdev_path[0] = '\0';
    return -1;
}

static int csi_open_subdev(csi_info_t *csi)
{
    const char *subdev_path;

    if (csi->subdev_fd >= 0) {
        return 0;
    }

    subdev_path = getenv(CSI_SUBDEV_PATH_ENV);
    if (subdev_path == NULL || subdev_path[0] == '\0') {
        return csi_auto_discover_subdev(csi);
    }

    if (snprintf(csi->subdev_path, sizeof(csi->subdev_path), "%s", subdev_path) >= (int)sizeof(csi->subdev_path)) {
        FS_LOGE("CSI subdev path is too long: %s", subdev_path);
        csi->subdev_path[0] = '\0';
        return -1;
    }

    csi->subdev_fd = open(csi->subdev_path, O_RDWR | O_NONBLOCK, 0);
    if (csi->subdev_fd < 0) {
        FS_LOGE("Open CSI subdev %s failed: %d", csi->subdev_path, errno);
        csi->subdev_path[0] = '\0';
        return -1;
    }

    FS_LOGI("Opened CSI subdev %s", csi->subdev_path);
    return 0;
}

static void csi_close_subdev(csi_info_t *csi)
{
    if (csi->subdev_fd >= 0) {
        close(csi->subdev_fd);
        csi->subdev_fd = -1;
    }

    csi->subdev_path[0] = '\0';
}

static int csi_configure_subdev_format(csi_info_t *csi, uint32_t width, uint32_t height)
{
    struct v4l2_subdev_format sub_format;

    if (csi_open_subdev(csi) < 0) {
        FS_LOGW("CSI subdev is not configured; set %s to a valid /dev/v4l-subdevX if auto discovery is not correct",
                CSI_SUBDEV_PATH_ENV);
        return 0;
    }

    memset(&sub_format, 0, sizeof(sub_format));
    sub_format.pad = 0;
    sub_format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    sub_format.format.width = width;
    sub_format.format.height = height;
    sub_format.format.code = MEDIA_BUS_FMT_SBGGR12_1X12;
    sub_format.format.field = V4L2_FIELD_NONE;

    if (ioctl(csi->subdev_fd, VIDIOC_SUBDEV_S_FMT, &sub_format) < 0) {
        FS_LOGE("Set subdev format failed for %s: %d", csi->subdev_path, errno);
        return -1;
    }

    FS_LOGI("Configured CSI subdev %s to %ux%u code 0x%x", csi->subdev_path,
            sub_format.format.width, sub_format.format.height, sub_format.format.code);
    return 0;
}

static void csi_try_set_capture_crop(csi_info_t *csi, uint32_t width, uint32_t height)
{
    struct v4l2_selection sel;

    memset(&sel, 0, sizeof(sel));
    sel.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    sel.target = V4L2_SEL_TGT_CROP;
    sel.r.left = 0;
    sel.r.top = 0;
    sel.r.width = width;
    sel.r.height = height;

    if (ioctl(csi->fd, VIDIOC_S_SELECTION, &sel) < 0) {
        FS_LOGW("Video%d set capture crop to %ux%u failed: errno=%d (%s)",
                csi->vi, width, height, errno, strerror(errno));
        return;
    }

    FS_LOGI("Video%d capture crop set to %ux%u", csi->vi, width, height);
}


static int csi_disable_mipi_lvds_compact(void)
{
    char compact_path[256];
    sprintf(compact_path, "%s", "/sys/devices/platform/rkcif-mipi-lvds/compact_test");

    char value[4];
    int res = 0;
    FS_LOGI("disable rkcif mipi lvds compact function");
    int fd = open(compact_path, O_WRONLY);

    if (fd < 0) {
        FS_LOGE("Error:open %s", compact_path);
    }
    sprintf(value, "%d", 0);
    res = write(fd, value, 1);

    if (res < 0) {
        res = -errno;

        if (res != -EBUSY) {
            FS_LOGE("disable rkcif mipi lvds compact function failed!");
        }
    }
    close(fd);

    return res;
}

int csi_interface_init(csi_info_t *csi)
{
    // int res = 0;
    // struct v4l2_input inp;
    // struct v4l2_streamparm parms;

    csi_disable_mipi_lvds_compact();
    csi->subdev_fd = -1;
    csi->subdev_path[0] = '\0';

    char devname[128];

    // res = allocOpen(USE_MEMORY_MALLOC_TYPE, &csi->mem, NULL);
    // if (res < 0) {
    //     printf("allocOpen failed.\n");
    //     return -1;
    // }

    memset(devname, 0, sizeof(devname));
    snprintf(devname, 128, "/dev/video%d", csi->vi);

    if (access(devname, F_OK) != 0) {
        printf("No device: %s\n", devname);
        return -1;
    }

    csi->fd = open(devname, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (csi->fd < 0) {
        printf("Error:open %s\n", devname);
        return -1;
    }

    printf("Opened %s\n", devname);

    // memset(&inp, 0, sizeof(inp));
    // memset(&parms, 0, sizeof(parms));
    // inp.index = 0;

    // if ((res = ioctl(csi->fd, VIDIOC_S_INPUT, &inp)) < 0) {
    //     close(csi->fd);
    //     csi->fd = -1;
    //     printf("Set input error res = %d err = %d\n", res, errno);
    //     return -1;
    // }

    // parms.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    // parms.parm.capture.timeperframe.numerator = 1;
    // parms.parm.capture.timeperframe.denominator = CAPTURE_RATE_FPS;  // fps 25
    // parms.parm.capture.capturemode = V4L2_MODE_VIDEO;
    // parms.parm.capture.reserved[0] = 0;
    // parms.parm.capture.reserved[1] = 0; /*2:command, 1: wdr, 0: normal*/

    // if (ioctl(csi->fd, VIDIOC_S_PARM, &parms) < 0) {
    //     close(csi->fd);
    //     csi->fd = -1;
    //     printf("Set parms error, err = %d\n", errno);
    //     return -1;
    // }

    // 初始化 buffers 互斥锁
    pthread_mutex_init(&csi->buffers_lock, NULL);
    return 0;
}

int csi_interface_deinit(csi_info_t *csi)
{
    csi_interface_stream_enable(csi, false);
    csi_interface_free_buffers(csi);
    csi_close_subdev(csi);
    if (csi->fd >= 0) {
        close(csi->fd);
        csi->fd = -1;
    }
    // 销毁互斥锁
    pthread_mutex_destroy(&csi->buffers_lock);
    return 0;
}

int csi_interface_fmt_set(csi_info_t *csi, uint32_t width, uint32_t height, uint32_t pix_fmt)
{
    struct v4l2_format fmt;
    struct v4l2_pix_format_mplane *pixm;

    csi->width = width;
    csi->height = height;

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    pixm = &fmt.fmt.pix_mp;
    pixm->width = width;
    pixm->height = height;
    pixm->pixelformat = pix_fmt;
    pixm->field = V4L2_FIELD_NONE;

    // if (ioctl(csi->fd, VIDIOC_SUBDEV_S_FMT, &fmt) < 0) {
    //     FS_LOGI("Video%d Set format error\n", csi->vi);
    //     return -1;
    // }

    if (csi_configure_subdev_format(csi, width, height) < 0) {
        return -1;
    }

    if (ioctl(csi->fd, VIDIOC_S_FMT, &fmt) < 0) {
        printf("Video%d Set format error\n", csi->vi);
        return -1;
    }

    csi_try_set_capture_crop(csi, width, height);

    // if (csi_configure_subdev_format(csi, width, height) < 0) {
    //     return -1;
    // }

    if (ioctl(csi->fd, VIDIOC_G_FMT, &fmt) < 0) {
        FS_LOGE("Video%d get format error: %d", csi->vi, errno);
        return -1;
    }

    csi->width = fmt.fmt.pix_mp.width;
    csi->height = fmt.fmt.pix_mp.height;
    csi->nplanes = fmt.fmt.pix_mp.num_planes;
    FS_LOGI("Video%d configured to %ux%u, pixelformat 0x%x, nplanes %u", csi->vi,
            csi->width, csi->height, fmt.fmt.pix_mp.pixelformat, csi->nplanes);
    return 0;
}

int csi_interface_request_buffers(csi_info_t* csi)
{
    struct v4l2_requestbuffers reqbuf;
    unsigned int i, j;
    int rc = 0;
    bool locked = false;

    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.count = CAPTURE_FRAME_NUM;
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    reqbuf.memory = USE_V4L2_MEMORY_TYPE;

    if (ioctl(csi->fd, VIDIOC_REQBUFS, &reqbuf) < 0) {
        printf("Video%d Request buffer error\n", csi->vi);
        return -1;
    }
    csi->nbuffers = reqbuf.count;

    // 进入临界区，保护 buffers 的分配与填充
    pthread_mutex_lock(&csi->buffers_lock);
    locked = true;
    csi->buffers = (struct csi_buffers*)calloc(reqbuf.count, sizeof(struct csi_buffers));
    if (!csi->buffers) {
        printf("Video%d No memory malloc\n", csi->vi);
        rc = -1;
        goto out;
    }

    for (i = 0; i < reqbuf.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = USE_V4L2_MEMORY_TYPE;
        buf.index = i;  // 与sample code不同
        buf.length = csi->nplanes;
        buf.m.planes = (struct v4l2_plane*)calloc(csi->nplanes, sizeof(struct v4l2_plane));

        if (buf.m.planes == NULL) {
            free((void*)csi->buffers);
            csi->buffers = NULL;
            printf("Video%d No memory malloc\n", csi->vi);
            rc = -1;
            goto out;
        }

        if (ioctl(csi->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            free((void*)csi->buffers);
            free(buf.m.planes);
            buf.m.planes = NULL;
            csi->buffers = NULL;
            printf("Video%d QUERYBUF error\n", csi->vi);
            rc = -1;
            goto out;
        }

        for (j = 0; j < csi->nplanes; j++) {

            /* Save the kernel-reported DMA buffer size from VIDIOC_QUERYBUF.
             * This must be used for both mmap() and munmap() to avoid
             * unmapping pages beyond the actual buffer region. */
            uint32_t mmap_len = buf.m.planes[j].length;

            // #if SOC_T527
            // csi->mem.size = mmap_len;
            // int ret = allocAlloc(USE_MEMORY_MALLOC_TYPE, &csi->mem, NULL);
            // if (ret < 0) {
            //     printf("allocAlloc failed.\n");
            //     rc = -1;
            //     goto out;
            // }
            // #else
            // csi->mem.size = mmap_len;
            csi->buffers[i].start[j]  = mmap(NULL, mmap_len,
                                                    PROT_READ | PROT_WRITE, MAP_SHARED, csi->fd, buf.m.planes[j].m.mem_offset);
            if (csi->buffers[i].start[j] == MAP_FAILED) {
                free((void*)csi->buffers);
                free(buf.m.planes);
                buf.m.planes = NULL;
                csi->buffers = NULL;
                printf("Camera v4l2QueryBuf buffer allocate ERROR\n");
                rc = -1;
                goto out;
            }
            /* Store the actual mmap'd size so munmap() in csi_interface_free_buffers()
             * uses the same length that was passed to mmap(). */
            csi->buffers[i].length[j] = mmap_len;
            // #endif

#if SOC_T527
            // for (int32_t k = 0; k < (int32_t)(csi->buffers[i].length[j] / sizeof(uint16_t)); k++) {
            //     ((uint16_t *)csi->buffers[i].start[j])[k] = 0xF000;
            // }

            // csi->buffers[i].phy_addr[j] = (void*)csi->mem.phy;
            // csi->buffers[i].start[j] = (void*)csi->mem.vir;
            // if (csi->buffers[i].start[j] == NULL) {
            //     printf("Camera v4l2QueryBuf buffer allocate ERROR\n");
            //     rc = -1;
            //     goto out;
            // }
            // csi->buffers[i].fd[j] = csi->mem.ion_buffer.fd_data.aw_fd;

            // csi->buffers[i].vbuf.size = csi->mem.size;
            // csi->buffers[i].vbuf.phy = csi->mem.phy;
            // csi->buffers[i].vbuf.vir = csi->mem.vir;
            // csi->buffers[i].vbuf.ops = csi->mem.ops;
            // csi->buffers[i].vbuf.ion_buffer.fd_data.aw_fd = csi->mem.ion_buffer.fd_data.aw_fd;
#endif
            buf.m.planes[j].m.userptr = (unsigned long)csi->buffers[i].start[j];
            buf.m.planes[j].length = csi->buffers[i].length[j];
        }

        csi->buffers[i].nplanes = csi->nplanes;
        free(buf.m.planes);
    }

    for (i = 0; i < csi->nbuffers; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = USE_V4L2_MEMORY_TYPE;
        buf.index = i;
        buf.length = csi->nplanes;
        buf.m.planes = (struct v4l2_plane*)calloc(csi->nplanes, sizeof(struct v4l2_plane));

        if (buf.m.planes == NULL) {
            free((void*)csi->buffers);
            csi->buffers = NULL;
            printf("Video%d No memory malloc\n", csi->vi);
            rc = -1;
            goto out;
        }
        for (j = 0; j < csi->nplanes; j++) {
            buf.m.planes[j].m.userptr = (unsigned long)csi->buffers[i].start[j];

// #if SOC_T527
//             buf.m.planes[j].m.fd = csi->buffers[i].fd[j];
// #endif
            buf.m.planes[j].length = csi->buffers[i].length[j];
        }

        if (ioctl(csi->fd, VIDIOC_QBUF, &buf) < 0) {

// #if SOC_T527
//             allocFree(USE_MEMORY_MALLOC_TYPE, (dma_mem_des_t*)csi->buffers[i].phy_addr[0], NULL);
//             #endif
            free((void*)csi->buffers);
            free(buf.m.planes);

            buf.m.planes = NULL;
            csi->buffers = NULL;
            printf("Video%d QBUF error\n", csi->vi);
            rc = -1;
            goto out;
        }

        free(buf.m.planes);
    }

    rc = 0;
out:
    if (locked) {
        pthread_mutex_unlock(&csi->buffers_lock);
    }
    return rc;
}

int csi_interface_free_buffers(csi_info_t *csi)
{
    int ret = 0;
    uint32_t i, j;
    bool locked = false;

    // Check if buffers were actually allocated
    if (!csi) {
        return 0;
    }

    pthread_mutex_lock(&csi->buffers_lock);
    locked = true;

    if (!csi->buffers) {
        pthread_mutex_unlock(&csi->buffers_lock);
        return 0;
    }

    for (i = 0; i < csi->nbuffers; i++) {
        for (j = 0; j < csi->nplanes; j++) {

// #if SOC_T527
//             allocFree(USE_MEMORY_MALLOC_TYPE, (dma_mem_des_t*)&csi->buffers[i].vbuf, NULL);
// #else
            ret += munmap(csi->buffers[i].start[j], csi->buffers[i].length[j]);
// #endif
        }
    }

    free((void*)csi->buffers);
    csi->buffers = NULL;

#if SOC_T527
    // ret = allocClose(USE_MEMORY_MALLOC_TYPE, &csi->mem, NULL);
    // if (ret < 0) {
    //     printf("Video%d allocClose failed.\n", csi->vi);
    //     if (locked) {
    //         pthread_mutex_unlock(&csi->buffers_lock);
    //     }
    //     return -1;
    // }
#endif
    if (locked) {
        pthread_mutex_unlock(&csi->buffers_lock);
    }
    return ret;
}

int csi_interface_stream_enable(csi_info_t* csi, bool en)
{
    int res = 0;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    if (en) {
        res = ioctl(csi->fd, VIDIOC_STREAMON, &type);
    } else {
        res = ioctl(csi->fd, VIDIOC_STREAMOFF, &type);
    }

    if (res < 0) {
        FS_LOGE("Video%d StreamEnable %s failed: errno=%d (%s)", csi->vi,
                en ? "ON" : "OFF", errno, strerror(errno));
        return res;
    }

    return res;
}

int csi_interface_stream_monitor(csi_info_t* csi, uint32_t timeout)
{
    fd_set fds;
    struct timeval tv;
    int r = 0;
    int max = 0;

    FD_ZERO(&fds);
    FD_SET(csi->fd, &fds);
    max = csi->fd;

    tv.tv_sec = (int)(timeout / 1000);
    tv.tv_usec = (timeout % 1000) * 1000;
    r = select(max + 1, &fds, NULL, NULL, &tv);

    if (r < 0) {
        return -1;
    } else if (r == 0) {
        return 0;
    }

    if (!FD_ISSET(csi->fd, &fds)) {
        return 0;
    }

    return 1;
}

int csi_interface_read_frame(csi_info_t* csi, csi_buffers_t* f)
{
    memset(f, 0, sizeof(csi_buffers_t));
    f->v4l_buf = (struct v4l2_buffer*)calloc(1, sizeof(struct v4l2_buffer));
    f->v4l_buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    f->v4l_buf->memory = USE_V4L2_MEMORY_TYPE;
    f->v4l_buf->length = csi->nplanes;
    f->v4l_buf->m.planes = (struct v4l2_plane*)calloc(csi->nplanes, sizeof(struct v4l2_plane));
    f->video_fd = csi->fd;

// #if SOC_T527
//     f->buffers = csi->buffers;
// #endif

    if (f->v4l_buf->m.planes == NULL) {
        free(f->v4l_buf);
        printf("Video%d No memory malloc\n", csi->vi);
        return -1;
    }

    if (ioctl(csi->fd, VIDIOC_DQBUF, f->v4l_buf) < 0) {
        free(f->v4l_buf->m.planes);
        free(f->v4l_buf);
        printf("Video%d DQBUF error\n", csi->vi);
        return -1;
    }

    // 读取 buffers 指针时加锁，避免并发释放
    pthread_mutex_lock(&csi->buffers_lock);
    for (uint32_t i = 0; i < csi->nplanes; i++) {
        f->start[i] = csi->buffers[f->v4l_buf->index].start[i];
        f->length[i] = csi->buffers[f->v4l_buf->index].length[i];
        #if SOC_T527
        f->fd[i] = csi->buffers[f->v4l_buf->index].fd[i];
        #endif
    }
    pthread_mutex_unlock(&csi->buffers_lock);
    // static int file_save_count = 0;
    // file_save_count++;
    // printf(" recv data from mipi. frame count = %d lenght = %d \n", file_save_count, f->length[0]);
#if 0
        static int file_save_count = 0;

        if (file_save_count < 10) {
            file_save_count++;
            char name[128];
            snprintf(name, sizeof(name), "/fortsense/vc%d_raw_%d.dat", 0, file_save_count);
            printf("write file '%s'", name);
            FILE *file = fopen(name, "wb+");
            uint32_t res = fwrite(f->start[0], 1, f->length[0], file);

            if (res != f->length[0]) {
                printf("Write file");
            }

            fclose(file);
        }

#endif

    f->nplanes = csi->nplanes;
    return 0;
}

int csi_interface_release_frame(csi_buffers_t* f)
{
    int res;

    // 使用从 read_frame 中复制的 fd/length，避免依赖共享 buffers 指针
// #if SOC_T527
//     f->v4l_buf->m.planes[0].m.fd = f->fd[0];
//     f->v4l_buf->m.planes[0].length = f->length[0];
// #endif

    res = ioctl(f->video_fd, VIDIOC_QBUF, f->v4l_buf);
    free(f->v4l_buf->m.planes);
    free(f->v4l_buf);

    return res;
}
