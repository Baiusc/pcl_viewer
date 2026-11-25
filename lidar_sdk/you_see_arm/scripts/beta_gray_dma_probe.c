/* 直接只读映射 /dev/lidar，不依赖 liblidar.so，不修改共享读写指针。
 * 交叉编译后运行：beta_gray_dma_probe /tmp/beta_gray_dma.bin
 * 输出完整 32 MiB DMA 快照，并观察约 6 秒的写指针变化。
 */
#define _POSIX_C_SOURCE 200809L
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define DMA_SIZE (32U * 1024U * 1024U)

int main(int argc, char **argv)
{
	if (argc != 2) {
		fprintf(stderr, "用法：%s 输出文件\n", argv[0]);
		return 1;
	}
	int fd = open("/dev/lidar", O_RDWR);
	if (fd < 0) {
		perror("open");
		return 1;
	}
	volatile uint32_t *param = mmap(NULL, 4096, PROT_READ, MAP_PRIVATE, fd, 0);
	if (param == MAP_FAILED) {
		perror("mmap 参数");
		close(fd);
		return 1;
	}
	const void *data = mmap(NULL, DMA_SIZE, PROT_READ, MAP_PRIVATE, fd, 0);
	if (data == MAP_FAILED) {
		perror("mmap 数据");
		munmap((void *)param, 4096);
		close(fd);
		return 1;
	}
	FILE *out = fopen(argv[1], "wb");
	int failed = 0;
	if (!out) {
		perror("fopen");
		failed = 1;
	} else {
		printf("快照前 roffset=%u woffset=%u\n", param[0], param[1]);
		if (fwrite(data, 1, DMA_SIZE, out) != DMA_SIZE) {
			perror("fwrite");
			failed = 1;
		}
		if (fclose(out)) {
			perror("fclose");
			failed = 1;
		}
		printf("快照后 roffset=%u woffset=%u\n", param[0], param[1]);
	}
	uint32_t last = param[1];
	unsigned changes = 0;
	const struct timespec delay = { .tv_nsec = 10000000 };
	for (unsigned i = 0; i < 600; ++i) {
		nanosleep(&delay, NULL);
		uint32_t current = param[1];
		if (current != last) {
			printf("采样=%u roffset=%u woffset=%u 上次=%u\n", i, param[0], current, last);
			last = current;
			++changes;
		}
	}
	printf("写指针变化次数=%u\n", changes);
	munmap((void *)data, DMA_SIZE);
	munmap((void *)param, 4096);
	close(fd);
	return failed;
}
