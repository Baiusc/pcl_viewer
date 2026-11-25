#define _GNU_SOURCE // NOLINT(bugprone-reserved-identifier)：需要 Linux 扩展接口。
#include "mgr_support.h"
#include "fpga_spi.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>

/* 本工具和验签工具、公钥由系统镜像预置，先验证包，再执行包内程序。 */
static int trusted(const char *path, int directory)
{
	struct stat st;
	return path && path[0] == '/' && !lstat(path, &st) && (directory ? S_ISDIR(st.st_mode) : S_ISREG(st.st_mode)) &&
	       (st.st_uid == 0 || st.st_uid == geteuid()) && !(st.st_mode & 0022);
}

static int absolute_path(const char *path)
{
	if (!path || path[0] != '/' || strlen(path) >= PATH_MAX || strstr(path, "//"))
		return 0;
	for (const char *p = path; (p = strchr(p, '/')); ++p)
		if ((p[1] == '.' && (!p[2] || p[2] == '/')) || (p[1] == '.' && p[2] == '.' && (!p[3] || p[3] == '/')))
			return 0;
	return 1;
}

static int below(const char *path, const char *parent)
{
	size_t size = strlen(parent);
	return !strncmp(path, parent, size) && path[size] == '/' && path[size + 1];
}

static int join(char out[PATH_MAX], const char *parent, const char *name)
{
	return snprintf(out, PATH_MAX, "%s/%s", parent, name) >= PATH_MAX ? -1 : 0;
}

static int directory(const char *path)
{
	return !path || (mkdir(path, 0700) && errno != EEXIST) || !trusted(path, 1) ? -1 : 0;
}

static int trusted_tree(const char *path)
{
	struct stat st;
	if (lstat(path, &st) || (st.st_uid != 0 && st.st_uid != geteuid()) || (st.st_mode & 0022))
		return -1;
	if (S_ISREG(st.st_mode))
		return 0;
	if (!S_ISDIR(st.st_mode))
		return -1;
	DIR *dir = opendir(path);
	if (!dir)
		return -1;
	int ret = 0;
	struct dirent *entry;
	while ((entry = readdir(dir))) {
		if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, ".."))
			continue;
		char child[PATH_MAX];
		if (join(child, path, entry->d_name) || trusted_tree(child)) {
			ret = -1;
			break;
		}
	}
	closedir(dir);
	return ret;
}

static int copy_config(const char *source, const char *destination, int replace)
{
	struct stat st;
	if (!lstat(destination, &st)) {
		if (!trusted(destination, 0))
			return -1;
		if (!replace)
			return 0;
	} else if (errno != ENOENT)
		return -1;
	int input = open(source, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
	if (input < 0 || fstat(input, &st) || !S_ISREG(st.st_mode) || st.st_size <= 0 || st.st_size > 1048576 ||
	    (replace && (st.st_uid != geteuid() || (st.st_mode & 0022)))) {
		if (input >= 0)
			close(input);
		return -1;
	}
	char temporary[PATH_MAX];
	if (snprintf(temporary, sizeof(temporary), "%s%s", destination, replace ? ".save.tmp" : ".tmp.XXXXXX") >=
	    (int)sizeof(temporary)) {
		close(input);
		return -1;
	}
	/* 挂载锁已串行化提交，清理前一次断电可能遗留的候选文件。 */
	if (replace && unlink(temporary) && errno != ENOENT) {
		close(input);
		return -1;
	}
	int output = replace ? open(temporary, O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600) : mkstemp(temporary);
	int ret = -1;
	if (output < 0) {
		close(input);
		return -1;
	}
	char buffer[4096];
	ssize_t length;
	while ((length = read(input, buffer, sizeof(buffer))) > 0)
		if (write(output, buffer, (size_t)length) != length)
			goto end;
	if (length || fsync(output))
		goto end;
	if (close(output)) {
		output = -1;
		goto end;
	}
	output = -1;
	if (replace ? rename(temporary, destination) : (link(temporary, destination) && errno != EEXIST))
		goto end;
	char parent[PATH_MAX];
	snprintf(parent, sizeof(parent), "%s", destination);
	*strrchr(parent, '/') = 0;
	int dirfd = open(parent, O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
	if (dirfd >= 0) {
		ret = fsync(dirfd);
		close(dirfd);
	}
end:
	close(input);
	if (output >= 0)
		close(output);
	unlink(temporary);
	return ret;
}

/* 只初始化缺失文件，已有配置与本机标定永不被软件默认值覆盖。 */
static int seed(const char *source, const char *destination)
{
	return copy_config(source, destination, 0);
}

static int prepare_capture_config(cJSON *cfg, const char *data, const char *package, const char *base,
				  const char *calibration)
{
	char config_dir[PATH_MAX], source[PATH_MAX], target[PATH_MAX];
	if (join(config_dir, data, "config") || directory(config_dir) || directory(calibration))
		return -1;
	const char *legacy = mgr_string(cfg, "legacy_config_dir");
	if (join(source, legacy ? legacy : package, legacy ? "ys_conf.ini" : "conf/ys_conf.ini") || seed(source, base))
		return -1;
	const char *cal_files[] = { "ys_calibration.ini", "ys_encoder_angle.csv" };
	for (size_t i = 0; i < sizeof(cal_files) / sizeof(cal_files[0]); ++i) {
		if (join(target, calibration, cal_files[i]))
			return -1;
		if (legacy && (join(source, legacy, cal_files[i]) || seed(source, target)))
			return -1;
		if (!trusted(target, 0)) {
			fprintf(stderr, "缺少本机标定文件：%s；拒绝使用通用默认标定\n", target);
			return -1;
		}
	}
	return 0;
}

static int snapshot(cJSON *cfg, int print)
{
	const char *path = mgr_string(cfg, "socket_path"), *model = mgr_string(cfg, "model");
	if (!path || strlen(path) >= sizeof(((struct sockaddr_un *)0)->sun_path) || !model)
		return -1;
	int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
	if (fd < 0)
		return -1;
	struct sockaddr_un address = { .sun_family = AF_UNIX };
	strcpy(address.sun_path, path);
	int ret = -1;
	if (connect(fd, (struct sockaddr *)&address, sizeof(address)))
		goto end;
	struct ucred peer;
	socklen_t size = sizeof(peer);
	if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &peer, &size) || peer.uid != geteuid())
		goto end;
	const char request[] = "{\"op\":\"status\"}";
	if (send(fd, request, sizeof(request) - 1, MSG_NOSIGNAL) != sizeof(request) - 1)
		goto end;
	struct pollfd event = { .fd = fd, .events = POLLIN };
	if (poll(&event, 1, 1000) <= 0)
		goto end;
	char response[8192];
	ssize_t n = recv(fd, response, sizeof(response) - 1, MSG_TRUNC);
	if (n <= 0 || n >= (ssize_t)sizeof(response))
		goto end;
	response[n] = 0;
	cJSON *value = cJSON_ParseWithOpts(response, NULL, 1);
	const char *actual_model = mgr_string(value, "model");
	if (mgr_int(value, "code", 0) == 200 && actual_model && !strcmp(actual_model, model) &&
	    cJSON_IsFalse(cJSON_GetObjectItemCaseSensitive(value, "simulation")) && mgr_string(value, "phase")) {
		if (print)
			puts(response);
		ret = 0;
	}
	cJSON_Delete(value);
end:
	close(fd);
	return ret;
}

typedef enum { BOOT_VERIFY, BOOT_PACKAGE_PATH, BOOT_PREPARE, BOOT_RUN } boot_action_t;

static int wait_bounded(pid_t child, int seconds)
{
	if (child < 0)
		return 1;
	int status;
	int64_t deadline = mgr_now() + seconds * 1000;
	for (;;) {
		pid_t result = waitpid(child, &status, WNOHANG);
		if (result == child)
			return WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
		if (result < 0 && errno != EINTR)
			return 1;
		if (mgr_now() >= deadline) {
			kill(child, SIGKILL);
			while (waitpid(child, NULL, 0) < 0 && errno == EINTR)
				;
			return 124;
		}
		usleep(10000);
	}
}

static int detect_fpga_mode(const char *adapter)
{
	int output[2];
	if (pipe2(output, O_CLOEXEC | O_NONBLOCK))
		return -1;
	pid_t parent = getpid(), child = fork();
	if (!child) {
		if (prctl(PR_SET_PDEATHSIG, SIGKILL) || getppid() != parent || dup2(output[1], STDOUT_FILENO) < 0)
			_exit(126);
		execl(adapter, adapter, "fpga-version", (char *)NULL);
		_exit(127);
	}
	close(output[1]);
	int result = wait_bounded(child, 5);
	char value[12] = { 0 };
	ssize_t length = read(output[0], value, sizeof(value));
	close(output[0]);
	if (result || length != 11 || strncmp(value, "0x", 2) || strspn(value + 2, "0123456789abcdefABCDEF") != 8 ||
	    value[10] != '\n') {
		fprintf(stderr, "无法读取当前 FPGA 的 version_num\n");
		return -1;
	}
	uint32_t version = (uint32_t)strtoul(value, NULL, 16);
	unsigned int product = version >> 24;
	unsigned int function = (version >> 16) & 0xff;
	unsigned int revision = (version >> 8) & 0xff;
	fprintf(stderr, "FPGA version_num=0x%08" PRIx32 "，B1=0x%02x，B2=0x%02x，B3=0x%02x\n", version, product,
		function, revision);
	if (product != 0x11 || revision != 0) {
		fprintf(stderr, "当前 Beta 应用仅支持 FPGA 产品号 0x11、布局版本 0x00\n");
		return -1;
	}
	switch (function) {
	case 0x00:
		return 2;
	case 0x01:
		return 0;
	case 0x02:
		return LIDAR_BETA_SPOT_MODE;
	case 0x03:
		return 3;
	default:
		fprintf(stderr, "当前 FPGA 功能版本 0x%02x 无受支持的采集布局\n", function);
		return -1;
	}
}

/* 待烧录状态阻止监督器启动采集，恢复后仅使用运行期调试状态。 */
static int jtag_session(const char *runtime)
{
	char path[PATH_MAX];
	struct stat st;
	if (join(path, runtime, "fpga-jtag"))
		return -1;
	if (lstat(path, &st))
		return errno == ENOENT ? 0 : -1;
	if (!trusted(path, 0) || st.st_size != 7)
		return -1;
	FILE *file = fopen(path, "r");
	if (!file)
		return -1;
	char value[8] = { 0 };
	int active = fgets(value, sizeof(value), file) && !strcmp(value, "active\n");
	fclose(file);
	return active ? 1 : -1;
}

static const char *fpga_format(int mode)
{
	switch (mode) {
	case 0:
		return "histogram";
	case 2:
		return "pointcloud";
	case 3:
		return "gray";
	case LIDAR_BETA_SPOT_MODE:
		return "spot";
	default:
		return NULL;
	}
}

/* 返回 -1 表示没有旧版模式记录，-2 表示烧录未完成或记录损坏。 */
static int installed_fpga_mode(const char *state)
{
	char path[PATH_MAX];
	struct stat st;
	if (join(path, state, "fpga-profile"))
		return -2;
	if (lstat(path, &st))
		return errno == ENOENT ? -1 : -2;
	if (!trusted(path, 0) || (st.st_size != 67 && st.st_size != 75))
		return -2;
	FILE *file = fopen(path, "r");
	if (!file)
		return -2;
	char record[76] = { 0 };
	int ok = fgets(record, sizeof(record), file) != NULL;
	fclose(file);
	if (!ok || strlen(record) != (size_t)st.st_size)
		return -2;
	const char *hex = "0123456789abcdef";
	if (st.st_size == 75)
		return !strncmp(record, "installed ", 10) && strspn(record + 10, hex) == 64 && record[74] == '\n' ? -1 :
														    -2;
	if (!strchr("023", record[0]) || record[1] != ' ' || strspn(record + 2, hex) != 64 || record[66] != '\n')
		return -2;
	return record[0] - '0';
}

static int prepare(cJSON *cfg, boot_action_t action)
{
	const char *root = mgr_string(cfg, "data_root"), *release = mgr_string(cfg, "release_root");
	const char *active = mgr_string(cfg, "package_dir"), *key = mgr_string(cfg, "public_key");
	const char *crypto = mgr_string(cfg, "openssl"), *adapter = mgr_string(cfg, "adapter");
	const char *model = mgr_string(cfg, "model"), *runtime = mgr_string(cfg, "boot_runtime");
	const char *state = mgr_string(cfg, "state_dir"), *manager_runtime = mgr_string(cfg, "runtime_dir");
	const char *base = mgr_string(cfg, "base_config"), *user = mgr_string(cfg, "user_settings");
	const char *calibration = mgr_string(cfg, "calibration_dir"), *socket = mgr_string(cfg, "socket_path");
	const char *paths[] = { root,  release,		active, key,  crypto,	   adapter, runtime,
				state, manager_runtime, base,	user, calibration, socket };
	for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); ++i)
		if (!absolute_path(paths[i]))
			return -1;
	char resolved[PATH_MAX], releases[PATH_MAX], data[PATH_MAX], verifier[PATH_MAX], public_key[PATH_MAX];
	if (!root || !release || !active || !model || !runtime || !state || !manager_runtime || !base || !user ||
	    !calibration || !socket || !realpath(root, data) || strcmp(root, data) != 0 || !trusted(data, 1) ||
	    !realpath(release, releases) || !below(releases, data) || !trusted(releases, 1) ||
	    !realpath(active, resolved) || !below(resolved, releases) || trusted_tree(resolved) || !trusted(key, 0) ||
	    !trusted(crypto, 0) || !trusted(adapter, 0) || access(crypto, X_OK) || access(adapter, X_OK) ||
	    !realpath(crypto, verifier) || !realpath(key, public_key) || below(verifier, releases) ||
	    below(public_key, releases) || !below(base, data) || !below(user, data) || !below(state, data) ||
	    !below(calibration, data) || !below(manager_runtime, runtime) || !below(socket, runtime) ||
	    strlen(socket) >= 108 || !strcmp(base, user)) {
		fprintf(stderr, "启动路径、权限、外置信任工具或持久目录约束不满足\n");
		return -1;
	}
	cJSON *manifest = NULL;
	if (mgr_verify_package(resolved, key, crypto, 0, &manifest)) {
		fprintf(stderr, "发布包签名或文件摘要验证失败\n");
		return -1;
	}
	const char *package_model = mgr_string(manifest, "model");
	if (!package_model || strcmp(model, package_model))
		goto bad_manifest;
	int unified = mgr_int(manifest, "driver_abi", 0) == 2;
	int probe = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(cfg, "fpga_probe"));
	int boot_capture = action == BOOT_RUN || action == BOOT_PREPARE;
	int fpga_mode = installed_fpga_mode(state);
	int jtag = 0;
	if (probe && boot_capture && fpga_mode != -2) {
		if (!unified || strcmp(model, "CS96-beta")) {
			fprintf(stderr, "FPGA 自动识别需要与 Beta 统一驱动配套的应用\n");
			goto bad_manifest;
		}
		jtag = jtag_session(runtime);
		if (jtag < 0) {
			fprintf(stderr, "JTAG 烧录尚未恢复采集，或调试记录损坏\n");
			goto bad_manifest;
		}
		jtag |= cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(cfg, "fpga_jtag_boot"));
		fpga_mode = detect_fpga_mode(adapter);
		if (fpga_mode < 0)
			goto bad_manifest;
	}
	int matched = 0;
	const char *format = probe && !boot_capture ? NULL : fpga_mode >= 0 ? fpga_format(fpga_mode) : "pointcloud";
	cJSON *capability;
	cJSON_ArrayForEach(capability, cJSON_GetObjectItemCaseSensitive(manifest, "supported_modes"))
	{
		if (!cJSON_IsString(capability))
			continue;
		const char *name = capability->valuestring;
		matched |= format ? !strcmp(name, format) :
				    !strcmp(name, "pointcloud") || !strcmp(name, "histogram") ||
					    !strcmp(name, "gray") || !strcmp(name, "spot");
	}
	cJSON_Delete(manifest);
	/* 烧录中仍可加载 Web，由 SWUpdate 或 SSH 提供恢复入口。 */
	if (action == BOOT_PACKAGE_PATH) {
		if (fpga_mode != -2 && !matched)
			return -1;
		puts(resolved);
		return 0;
	}
	if (fpga_mode == -2) {
		fprintf(stderr, "FPGA 更新未完成或状态记录损坏，采集保持停止，请重新上传签名 SWU\n");
		return -1;
	}
	if (!matched || (fpga_mode >= 0 && (!unified || strcmp(model, "CS96-beta"))))
		return -1;
	if (fpga_mode >= 0) {
		const int pointcloud_modes[] = { 1, 2 };
		cJSON *modes = fpga_mode == 2 ? cJSON_CreateIntArray(pointcloud_modes, 2) :
						cJSON_CreateIntArray(&fpga_mode, 1);
		if (!modes)
			return -1;
		if (!cJSON_ReplaceItemInObjectCaseSensitive(cfg, "validated_work_modes", modes)) {
			cJSON_Delete(modes);
			return -1;
		}
		if (!cJSON_ReplaceItemInObjectCaseSensitive(cfg, "initial_work_mode", cJSON_CreateNumber(fpga_mode)))
			return -1;
	}
	cJSON *modes = cJSON_GetObjectItemCaseSensitive(cfg, "validated_work_modes"), *entry;
	int initial = mgr_int(cfg, "initial_work_mode", -1), included = 0;
	if (!cJSON_IsArray(modes) || cJSON_GetArraySize(modes) < 1)
		return -1;
	cJSON_ArrayForEach(entry, modes)
	{
		if (!cJSON_IsNumber(entry) || entry->valuedouble != entry->valueint ||
		    (fpga_mode < 0 && entry->valueint != 1 && entry->valueint != 2))
			return -1;
		included |= entry->valueint == initial;
	}
	if (!included)
		return -1;
	char manager[PATH_MAX], target[PATH_MAX];
	if (join(manager, resolved, "bin/lidar_mgrd") || !trusted(manager, 0) || access(manager, X_OK))
		return -1;
	if (action == BOOT_VERIFY)
		return 0;
	if (directory(state) || directory(runtime) || directory(manager_runtime))
		return -1;
	if (fpga_mode != LIDAR_BETA_SPOT_MODE && prepare_capture_config(cfg, data, resolved, base, calibration))
		return -1;
	if (!cJSON_ReplaceItemInObjectCaseSensitive(cfg, "package_dir", cJSON_CreateString(resolved)))
		return -1;
	if (jtag) {
		char jtag_state[PATH_MAX];
		if (join(jtag_state, runtime, "fpga-jtag-state") || directory(jtag_state) ||
		    !cJSON_ReplaceItemInObjectCaseSensitive(cfg, "state_dir", cJSON_CreateString(jtag_state)))
			return -1;
		unsetenv("LIDAR_PERSIST_COMMAND");
	}
	cJSON_DeleteItemFromObjectCaseSensitive(cfg, "fpga_detected");
	if (probe) {
		char mode[] = { '0' + fpga_mode, 0 };
		if (setenv("LIDAR_FPGA_ACTIVE_MODE", mode, 1) || !cJSON_AddBoolToObject(cfg, "fpga_detected", 1))
			return -1;
	} else {
		unsetenv("LIDAR_FPGA_ACTIVE_MODE");
	}
	if (mgr_write_json(runtime, "manager.json", cfg, NULL) || join(target, runtime, "manager.json"))
		return -1;
	if (action == BOOT_RUN) {
		unsetenv("LD_PRELOAD");
		unsetenv("LD_LIBRARY_PATH");
		execl(manager, manager, target, (char *)NULL);
		return -1;
	}
	return 0;

bad_manifest:
	cJSON_Delete(manifest);
	return -1;
}

int main(int argc, char **argv)
{
	umask(0077);
	if (argc >= 4 && !strcmp(argv[1], "storage-lock")) {
		/* 锁随脚本和子进程持有。调用方退出后仍完成卸载。 */
		struct stat st;
		int lock = open(argv[2], O_RDWR | O_CREAT | O_NOFOLLOW, 0600);
		if (lock < 0 || fstat(lock, &st) || !S_ISREG(st.st_mode) || st.st_uid != geteuid() ||
		    (st.st_mode & 0077) || flock(lock, LOCK_EX))
			return 1;
		execv(argv[3], argv + 3);
		return 1;
	}
	if (argc == 4 && !strcmp(argv[1], "storage-replace"))
		return !absolute_path(argv[2]) || !absolute_path(argv[3]) || copy_config(argv[2], argv[3], 1) != 0;

	/* 脚本适配器被管理服务终止时，不遗留正在执行的板级初始化子进程。 */
	pid_t parent = getppid();
	if (prctl(PR_SET_PDEATHSIG, SIGKILL) || getppid() != parent)
		return 1;
	if (argc == 2 && !strcmp(argv[1], "fpga-version"))
		return fpga_print_version();
	if (argc >= 4 && !strcmp(argv[1], "bounded")) {
		char *end;
		long seconds = strtol(argv[2], &end, 10);
		if (!*argv[2] || *end || seconds < 1 || seconds > 30 || argv[3][0] != '/')
			return 2;
		pid_t own = getpid(), child = fork();
		if (!child) {
			if (prctl(PR_SET_PDEATHSIG, SIGKILL) || getppid() != own)
				_exit(126);
			execv(argv[3], argv + 3);
			_exit(127);
		}
		return wait_bounded(child, seconds);
	}
	if (argc != 3 ||
	    (strcmp(argv[1], "run") != 0 && strcmp(argv[1], "prepare") != 0 && strcmp(argv[1], "verify") != 0 &&
	     strcmp(argv[1], "package-path") != 0 && strcmp(argv[1], "ready") != 0 && strcmp(argv[1], "status") != 0)) {
		fprintf(stderr, "用法：lidar_boot run|prepare|verify|package-path|ready|status 可信启动配置\n");
		return 2;
	}
	if (!trusted(argv[2], 0)) {
		fprintf(stderr, "启动配置不是受信任的普通文件\n");
		return 1;
	}
	cJSON *cfg = mgr_read_json(argv[2]);
	if (!cfg)
		return 1;
	int ret;
	if (!strcmp(argv[1], "ready") || !strcmp(argv[1], "status"))
		ret = snapshot(cfg, !strcmp(argv[1], "status"));
	else
		ret = prepare(cfg, !strcmp(argv[1], "run")	    ? BOOT_RUN :
				   !strcmp(argv[1], "verify")	    ? BOOT_VERIFY :
				   !strcmp(argv[1], "package-path") ? BOOT_PACKAGE_PATH :
								      BOOT_PREPARE);
	if (ret)
		fprintf(stderr, "启动校验或准备失败，保留发布包、配置与恢复状态供诊断\n");
	cJSON_Delete(cfg);
	return ret ? 1 : 0;
}
