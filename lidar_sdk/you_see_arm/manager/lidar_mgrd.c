#define _GNU_SOURCE
#include "mgr_support.h"
#include "fpga_spi.h"
#include "../network/network.h"
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/prctl.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

/* 状态只由主循环修改；UDS 请求不能提供程序路径或任意命令。 */
typedef struct {
	char package[PATH_MAX], state_dir[PATH_MAX], runtime_dir[PATH_MAX];
	char socket_path[108], adapter[PATH_MAX], base_config[PATH_MAX], key[PATH_MAX], openssl[PATH_MAX];
	char app[PATH_MAX], config_path[PATH_MAX];
	char user_settings[PATH_MAX];
	char network_socket[108];
	const char *model;
	const char *fault_code;
	cJSON *settings, *manifest;
	int simulation, fpga_detected, initial_mode, good, desired, current, rollback, recoveries, revalidating_timeout;
	int server, lock, health, initialized, data_seen, killed;
	int verify_ms, stop_ms, adapter_ms, settle_ms;
	pid_t app_pid, adapter_pid;
	int64_t deadline, ready_at;
	char phase[24], error[128], token[33], operation_id[33];
	char boot_id[37];
} Manager;
static const char verification_timeout_code[] = "verification_timeout";
static const char *verification_timeout_error(int mode)
{
	return mode == LIDAR_BETA_SPOT_MODE ? "FPGA 版本寄存器验证超时" : "数据验证超时";
}
static volatile sig_atomic_t stopping;
static void signal_stop(int signo)
{
	(void)signo;
	stopping = 1;
}
static int copy(char *out, size_t size, const char *in)
{
	if (!in || *in != '/' || strlen(in) >= size)
		return -1;
	strcpy(out, in);
	return 0;
}
static const char *mode_name(int mode)
{
	switch (mode) {
	case 0:
		return "histogram";
	case 1:
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
static int supported(Manager *m, int mode)
{
	const char *name = mode_name(mode);
	if (!name || (mode == LIDAR_BETA_SPOT_MODE && (!m->fpga_detected || m->initial_mode != mode)))
		return 0;
	cJSON *entry;
	int listed = 0, validated = 0;
	cJSON_ArrayForEach(entry, cJSON_GetObjectItemCaseSensitive(m->manifest, "supported_modes"))
	{
		if (cJSON_IsString(entry) && !strcmp(entry->valuestring, name))
			listed = 1;
	}
	cJSON_ArrayForEach(entry, cJSON_GetObjectItemCaseSensitive(m->settings, "validated_work_modes"))
	{
		if (cJSON_IsNumber(entry) && entry->valuedouble == mode)
			validated = 1;
	}
	return listed && validated;
}
static int persist(Manager *m, const char *phase, int save_to_flash)
{
	cJSON *state = cJSON_CreateObject();
	cJSON_AddNumberToObject(state, "schema_version", 1);
	cJSON_AddStringToObject(state, "model", m->model);
	cJSON_AddStringToObject(state, "operation_id", m->operation_id);
	cJSON_AddNumberToObject(state, "confirmed_mode", m->good);
	cJSON_AddNumberToObject(state, "desired_mode", m->desired);
	cJSON_AddStringToObject(state, "phase", phase);
	cJSON_AddNumberToObject(state, "recovery_count", m->recoveries);
	cJSON_AddStringToObject(state, "boot_id", m->boot_id);
	if (m->fault_code)
		cJSON_AddStringToObject(state, "fault_code", m->fault_code);
	int ret = mgr_write_json(m->state_dir, "state.json", state, save_to_flash ? "state" : NULL);
	cJSON_Delete(state);
	return ret;
}
static void kill_child(pid_t pid, int signo)
{
	if (pid > 0)
		kill(pid, signo);
}
static void fault(Manager *m, const char *reason, int save_to_flash)
{
	snprintf(m->error, sizeof(m->error), "%s", reason);
	strcpy(m->phase, "FAULT");
	m->current = -1;
	m->initialized = m->data_seen = 0;
	kill_child(m->app_pid, SIGKILL);
	kill_child(m->adapter_pid, SIGKILL);
	if (persist(m, "FAULT", save_to_flash)) {
		persist(m, "FAULT", 0);
		fprintf(stderr, "状态落盘失败，保持故障状态\n");
	}
}
static int random_token(char token[33])
{
	unsigned char bytes[16];
	if (getrandom(bytes, sizeof(bytes), 0) != sizeof(bytes))
		return -1;
	for (size_t i = 0; i < sizeof(bytes); ++i)
		snprintf(token + i * 2, 3, "%02x", bytes[i]);
	return 0;
}
static pid_t spawn_adapter(Manager *m)
{
	char target[16], previous[16];
	snprintf(target, sizeof(target), "%d", m->desired);
	snprintf(previous, sizeof(previous), "%d", m->good);
	pid_t parent = getpid();
	pid_t pid = fork();
	if (!pid) {
		prctl(PR_SET_PDEATHSIG, SIGKILL);
		if (getppid() != parent)
			_exit(126);
		execl(m->adapter, m->adapter, m->rollback || m->killed ? "recover" : "prepare", target, previous,
		      (char *)NULL);
		_exit(127);
	}
	return pid;
}
static int start_app(Manager *m)
{
	int pair[2];
	int spot = m->desired == LIDAR_BETA_SPOT_MODE;
	unsigned int flags = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(m->settings, "disable_file_log")) ?
				     MGR_RUNTIME_NO_FILE_LOG :
				     0;
	if ((!spot && mgr_runtime_config(m->base_config, m->runtime_dir, m->desired,
					 *m->user_settings ? m->user_settings : NULL, flags)) ||
	    random_token(m->token) || socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, pair))
		return -1;
	if (m->health >= 0)
		close(m->health);
	m->health = pair[0];
	m->initialized = m->data_seen = 0;
	m->ready_at = 0;
	pid_t parent = getpid();
	m->app_pid = fork();
	if (m->app_pid == 0) {
		/* 管理进程异常消失后不允许采集进程继续独占硬件。下次启动必须 recover。 */
		prctl(PR_SET_PDEATHSIG, SIGKILL);
		if (getppid() != parent)
			_exit(126);
		close(pair[0]);
		fcntl(pair[1], F_SETFD, 0);
		char fd[24];
		snprintf(fd, sizeof(fd), "%d", pair[1]);
		setenv("LIDAR_MGR_FD", fd, 1);
		setenv("LIDAR_MGR_TOKEN", m->token, 1);
		setenv("YS_CONFIG_PATH", m->config_path, 1);
		if (*m->network_socket) {
			setenv("LIDAR_NETWORK_SOCKET", m->network_socket, 1);
			setenv("LIDAR_MANAGER_SOCKET", m->socket_path, 1);
		} else {
			unsetenv("LIDAR_NETWORK_SOCKET");
			unsetenv("LIDAR_MANAGER_SOCKET");
		}
		if (*m->user_settings)
			setenv("YS_USER_SETTINGS_PATH", m->user_settings, 1);
		else
			unsetenv("YS_USER_SETTINGS_PATH");
		char mode[16];
		snprintf(mode, sizeof(mode), "%d", m->desired);
		setenv("LIDAR_WORK_MODE", mode, 1);
		const char *argument = spot ? "--fpga-spot" : NULL;
		execl(m->app, m->app, argument, (char *)NULL);
		_exit(127);
	}
	close(pair[1]);
	if (m->app_pid < 0)
		return -1;
	strcpy(m->phase, "VERIFYING");
	m->deadline = mgr_now() + m->verify_ms;
	return 0;
}
static void prepare(Manager *m)
{
	m->adapter_pid = spawn_adapter(m);
	if (m->adapter_pid < 0) {
		fault(m, "无法启动平台适配程序", 1);
		return;
	}
	strcpy(m->phase, "PREPARING");
	m->deadline = mgr_now() + m->adapter_ms;
}
static void stop_app(Manager *m)
{
	m->current = -1;
	m->initialized = m->data_seen = 0;
	m->ready_at = 0;
	if (m->app_pid > 0) {
		kill_child(m->app_pid, SIGTERM);
		strcpy(m->phase, "STOPPING");
		m->deadline = mgr_now() + m->stop_ms;
	} else
		prepare(m);
}
static void recover(Manager *m, const char *reason, const char *fault_code)
{
	snprintf(m->error, sizeof(m->error), "%s", reason);
	if (m->rollback || m->good < 0 || m->recoveries >= 1 || !supported(m, m->good)) {
		m->fault_code = fault_code;
		fault(m, reason, 1);
		return;
	}
	m->rollback = 1;
	++m->recoveries;
	m->desired = m->good;
	if (persist(m, "RECOVERING", 0)) {
		fault(m, "恢复意图无法落盘", 0);
		return;
	}
	stop_app(m);
}
static int begin(Manager *m, int mode)
{
	if (random_token(m->operation_id))
		return -1;
	int previous = m->desired;
	int previous_recoveries = m->recoveries;
	m->desired = mode;
	m->rollback = m->killed = 0;
	m->recoveries = 0;
	m->error[0] = 0;
	if (persist(m, "PENDING", 0)) {
		m->desired = previous;
		m->recoveries = previous_recoveries;
		snprintf(m->error, sizeof(m->error), "切换意图无法落盘，保留现有采集");
		return -1;
	}
	stop_app(m);
	return 0;
}
static void health_message(Manager *m)
{
	char buffer[1024];
	ssize_t n = recv(m->health, buffer, sizeof(buffer) - 1, MSG_DONTWAIT | MSG_TRUNC);
	if (n <= 0 || n >= (ssize_t)sizeof(buffer))
		return;
	buffer[n] = 0;
	cJSON *message = cJSON_Parse(buffer);
	const char *token = mgr_string(message, "token");
	const char *event = mgr_string(message, "event");
	if (token && event && !strcmp(token, m->token) && mgr_int(message, "work_mode", -1) == m->desired &&
	    !strcmp(m->phase, "VERIFYING")) {
		if (!strcmp(event, "INITIALIZED"))
			m->initialized = 1;
		int spot = m->desired == LIDAR_BETA_SPOT_MODE;
		if (!spot && !strcmp(event, "PIPELINE_PROGRESS") && mgr_int(message, "valid_packets", 0) >= 10)
			m->data_seen = 1;
		if (m->initialized && (spot || m->data_seen) && !m->ready_at)
			m->ready_at = mgr_now();
	}
	cJSON_Delete(message);
}
static void tick(Manager *m)
{
	int status;
	if (m->app_pid > 0 && waitpid(m->app_pid, &status, WNOHANG) == m->app_pid) {
		m->app_pid = 0;
		if (!strcmp(m->phase, "STOPPING")) {
			if (!WIFEXITED(status) || WEXITSTATUS(status))
				m->killed = 1;
			prepare(m);
		} else if (strcmp(m->phase, "FAULT"))
			recover(m, "采集进程退出", NULL);
	}
	if (m->adapter_pid > 0 && waitpid(m->adapter_pid, &status, WNOHANG) == m->adapter_pid) {
		m->adapter_pid = 0;
		if (!strcmp(m->phase, "PREPARING")) {
			if (!WIFEXITED(status) || WEXITSTATUS(status))
				recover(m, "平台重配失败", NULL);
			else if (start_app(m))
				recover(m, "运行配置或采集启动失败", NULL);
		}
	}
	int64_t now = mgr_now();
	if (!strcmp(m->phase, "VERIFYING")) {
		if (m->ready_at && now - m->ready_at >= m->settle_ms) {
			int previous = m->good;
			m->good = m->desired;
			/* 超时再验证成功后，以确认状态替换 Flash 中的故障记录。 */
			if (persist(m, "COMMITTED", m->revalidating_timeout || (!m->rollback && previous != m->good))) {
				m->good = previous;
				fault(m, "提交状态无法落盘", 0);
			} else {
				m->rollback = m->revalidating_timeout = 0;
				m->current = m->good;
				strcpy(m->phase, "COMMITTED");
			}
		} else if (now >= m->deadline)
			recover(m, verification_timeout_error(m->desired), verification_timeout_code);
	} else if (!strcmp(m->phase, "STOPPING") && now >= m->deadline) {
		if (m->killed)
			fault(m, "采集进程强制终止后仍未退出", 1);
		else {
			m->killed = 1;
			kill_child(m->app_pid, SIGKILL);
			m->deadline = now + 1000;
		}
	} else if (!strcmp(m->phase, "PREPARING") && now >= m->deadline) {
		/* 辅助程序未退出前不能并发执行下一次硬件重配。 */
		fault(m, "平台适配程序超时，需要人工检查硬件", 1);
	}
}
static void reply(Manager *m, int fd, int code, const char *message)
{
	cJSON *value = cJSON_CreateObject();
	cJSON_AddNumberToObject(value, "code", code);
	cJSON_AddStringToObject(value, "message", message);
	cJSON_AddStringToObject(value, "model", m->model);
	cJSON_AddStringToObject(value, "operation_id", m->operation_id);
	cJSON *version = cJSON_GetObjectItemCaseSensitive(m->manifest, "version");
	cJSON_AddItemToObject(value, "version", version ? cJSON_Duplicate(version, 1) : cJSON_CreateNull());
	cJSON_AddBoolToObject(value, "simulation", m->simulation);
	cJSON_AddStringToObject(value, "phase", m->phase);
	cJSON_AddNumberToObject(value, "confirmed_mode", m->good);
	cJSON_AddNumberToObject(value, "actual_mode", m->current);
	cJSON_AddNumberToObject(value, "desired_mode", m->desired);
	cJSON_AddStringToObject(value, "error", m->error);
	cJSON_AddNumberToObject(value, "pid", m->app_pid);
	cJSON_AddBoolToObject(value, "initialized", m->initialized);
	cJSON_AddBoolToObject(value, "pipeline_verified", m->data_seen);
	cJSON_AddItemToObject(value, "supported_modes",
			      cJSON_Duplicate(cJSON_GetObjectItemCaseSensitive(m->manifest, "supported_modes"), 1));
	cJSON_AddItemToObject(value, "validated_work_modes",
			      cJSON_Duplicate(cJSON_GetObjectItemCaseSensitive(m->settings, "validated_work_modes"),
					      1));
	char *text = cJSON_PrintUnformatted(value);
	if (text)
		send(fd, text, strlen(text), MSG_NOSIGNAL | MSG_DONTWAIT);
	free(text);
	cJSON_Delete(value);
}
static cJSON *network_status(Manager *m)
{
	cJSON *request = cJSON_CreateObject();
	cJSON_AddStringToObject(request, "op", "network_get");
	cJSON *response = net_rpc(m->network_socket, request, 400);
	cJSON_Delete(request);
	return response;
}
static void network_request(Manager *m, int fd, const cJSON *request, const char *op)
{
	if (!*m->network_socket) {
		reply(m, fd, 501, "此系统尚未配置独立网络服务");
		return;
	}
	int changing = !strcmp(op, "network_set") || !strcmp(op, "network_reset");
	if (changing && strcmp(m->phase, "COMMITTED") && strcmp(m->phase, "FAULT")) {
		reply(m, fd, 409, "模式切换或恢复正在进行，暂不能修改网络");
		return;
	}
	cJSON *response = net_rpc(m->network_socket, request, !strcmp(op, "network_confirm") ? 5000 : 400);
	if (!response)
		reply(m, fd, 503, "网络服务不可用；超时后请查询任务状态");
	else {
		char *text = cJSON_PrintUnformatted(response);
		if (text)
			send(fd, text, strlen(text), MSG_NOSIGNAL | MSG_DONTWAIT);
		free(text);
		cJSON_Delete(response);
	}
}
static void request(Manager *m)
{
	int fd = accept4(m->server, NULL, NULL, SOCK_CLOEXEC | SOCK_NONBLOCK);
	if (fd < 0)
		return;
	struct ucred cred;
	socklen_t length = sizeof(cred);
	if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &cred, &length) || cred.uid != geteuid()) {
		close(fd);
		return;
	}
	struct pollfd pollfd = { .fd = fd, .events = POLLIN };
	char buffer[1024];
	ssize_t n = poll(&pollfd, 1, 50) > 0 ? recv(fd, buffer, sizeof(buffer) - 1, MSG_TRUNC) : -1;
	cJSON *value = NULL;
	if (n > 0 && n < (ssize_t)sizeof(buffer)) {
		buffer[n] = 0;
		value = cJSON_ParseWithOpts(buffer, NULL, 1);
	}
	const char *op = mgr_string(value, "op");
	if (!op)
		reply(m, fd, 400, "请求格式错误");
	else if (!strcmp(op, "status"))
		reply(m, fd, 200, "状态");
	else if (!strcmp(op, "network_get") || !strcmp(op, "network_set") || !strcmp(op, "network_apply") ||
		 !strcmp(op, "network_confirm") || !strcmp(op, "network_reset"))
		network_request(m, fd, value, op);
	else if (!strcmp(op, "switch")) {
		int mode = mgr_int(value, "work_mode", -1);
		cJSON *network = *m->network_socket ? network_status(m) : NULL;
		int network_unavailable =
			*m->network_socket &&
			(!network || !net_integer(cJSON_GetObjectItemCaseSensitive(network, "code"), 200, 200));
		int network_pending = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(network, "pending"));
		const char *network_phase = net_string(network, "phase");
		network_unavailable |= network && ((network_phase && !strcmp(network_phase, "FAULT")) ||
						   cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(network, "recovery")));
		cJSON_Delete(network);
		if (network_unavailable)
			reply(m, fd, 503, "无法确认网络事务状态，暂不能切换模式");
		else if (network_pending)
			reply(m, fd, 409, "网络设置等待完成或确认，暂不能切换模式");
		else if (strcmp(m->phase, "COMMITTED"))
			reply(m, fd, !strcmp(m->phase, "FAULT") ? 503 : 409, "当前不能切换");
		else if (!supported(m, mode))
			reply(m, fd, 422, "模式未包含或尚未通过平台验证");
		else if (mode == m->good)
			reply(m, fd, 200, "已在目标模式");
		else {
			cJSON *verified = NULL;
			if (mgr_verify_package(m->package, m->key, m->openssl, m->simulation, &verified))
				reply(m, fd, 422, "包校验失败，保留当前采集");
			else {
				cJSON_Delete(verified);
				int ret = begin(m, mode);
				reply(m, fd, ret ? 500 : 202, ret ? "状态落盘失败" : "切换已受理");
			}
		}
	} else
		reply(m, fd, 501, "此阶段仅支持 status 和 switch");
	cJSON_Delete(value);
	close(fd);
}
static int setup(Manager *m, const char *settings)
{
	int boot_fd = open("/proc/sys/kernel/random/boot_id", O_RDONLY | O_CLOEXEC);
	if (boot_fd < 0)
		return -1;
	ssize_t boot_size = read(boot_fd, m->boot_id, sizeof(m->boot_id) - 1);
	close(boot_fd);
	if (boot_size != sizeof(m->boot_id) - 1)
		return -1;
	m->settings = mgr_read_json(settings);
	if (!m->settings)
		return -1;
#define SET(field, key)                                                     \
	if (copy(m->field, sizeof(m->field), mgr_string(m->settings, key))) \
	return -1
	SET(package, "package_dir");
	SET(state_dir, "state_dir");
	SET(runtime_dir, "runtime_dir");
	SET(socket_path, "socket_path");
	SET(adapter, "adapter");
	SET(base_config, "base_config");
	SET(openssl, "openssl");
	if (cJSON_HasObjectItem(m->settings, "user_settings")) {
		SET(user_settings, "user_settings");
	}
	if (cJSON_HasObjectItem(m->settings, "network_socket")) {
		SET(network_socket, "network_socket");
	}
	if (!m->simulation) {
		SET(key, "public_key");
	}
#undef SET
	char resolved[PATH_MAX];
	if (!realpath(m->package, resolved))
		return -1;
	strcpy(m->package, resolved);
	if (mgr_verify_package(m->package, m->key, m->openssl, m->simulation, &m->manifest))
		return -1;
	m->model = mgr_string(m->manifest, "model");
	const char *expected = mgr_string(m->settings, "model");
	if (!m->model || !expected || strcmp(m->model, expected))
		return -1;
	m->fpga_detected = !m->simulation && !strcmp(m->model, "CS96-beta") &&
			   cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(m->settings, "fpga_detected"));
	if (snprintf(m->app, sizeof(m->app), "%s/bin/you_see", m->package) >= (int)sizeof(m->app) ||
	    snprintf(m->config_path, sizeof(m->config_path), "%s/ys_conf.ini", m->runtime_dir) >=
		    (int)sizeof(m->config_path))
		return -1;
	if (access(m->app, X_OK) || access(m->adapter, X_OK))
		return -1;
	m->initial_mode = mgr_int(m->settings, "initial_work_mode", 2);
	if (!supported(m, m->initial_mode))
		return -1;
	m->verify_ms = mgr_int(m->settings, "verify_timeout_ms", 30000);
	m->stop_ms = mgr_int(m->settings, "stop_timeout_ms", 5000);
	m->adapter_ms = mgr_int(m->settings, "adapter_timeout_ms", 10000);
	m->settle_ms = mgr_int(m->settings, "settle_ms", 1000);
	if (m->verify_ms < 100 || m->stop_ms < 100 || m->adapter_ms < 100 || m->settle_ms < 0 ||
	    m->settle_ms >= m->verify_ms)
		return -1;
	if (mkdir(m->state_dir, 0700) && errno != EEXIST)
		return -1;
	if (mkdir(m->runtime_dir, 0700) && errno != EEXIST)
		return -1;
	struct stat directory_stat;
	if (lstat(m->state_dir, &directory_stat) || !S_ISDIR(directory_stat.st_mode) ||
	    directory_stat.st_uid != geteuid() || (directory_stat.st_mode & 0022))
		return -1;
	if (lstat(m->runtime_dir, &directory_stat) || !S_ISDIR(directory_stat.st_mode) ||
	    directory_stat.st_uid != geteuid() || (directory_stat.st_mode & 0022))
		return -1;
	char lockfile[PATH_MAX];
	if (snprintf(lockfile, sizeof(lockfile), "%s/manager.lock", m->state_dir) >= (int)sizeof(lockfile))
		return -1;
	m->lock = open(lockfile, O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
	if (m->lock < 0 || flock(m->lock, LOCK_EX | LOCK_NB))
		return -1;
	m->server = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
	if (m->server < 0)
		return -1;
	struct sockaddr_un address = { .sun_family = AF_UNIX };
	strcpy(address.sun_path, m->socket_path);
	struct stat st;
	if (!lstat(m->socket_path, &st)) {
		if (!S_ISSOCK(st.st_mode) || st.st_uid != geteuid())
			return -1;
		int probe = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
		int connected = connect(probe, (struct sockaddr *)&address, sizeof(address));
		close(probe);
		if (!connected)
			return -1;
		if (unlink(m->socket_path))
			return -1;
	}
	if (bind(m->server, (struct sockaddr *)&address, sizeof(address)) || chmod(m->socket_path, 0600) ||
	    listen(m->server, 8))
		return -1;
	return 0;
}
static int valid_boot_id(const char *value)
{
	if (!value || strlen(value) != 36)
		return 0;
	for (int i = 0; i < 36; ++i) {
		if (i == 8 || i == 13 || i == 18 || i == 23) {
			if (value[i] != '-')
				return 0;
		} else if (!strchr("0123456789abcdef", value[i]))
			return 0;
	}
	return 1;
}

static void start_initial(Manager *m)
{
	m->good = -1;
	m->desired = m->initial_mode;
	m->recoveries = 0;
	m->rollback = !m->fpga_detected;
	m->killed = 1;
	if (persist(m, "PENDING", 0))
		fault(m, "初始意图无法落盘", 0);
	else
		prepare(m);
}

static void boot(Manager *m)
{
	if (random_token(m->operation_id)) {
		fault(m, "无法生成任务标识", 0);
		return;
	}
	char path[PATH_MAX];
	if (snprintf(path, sizeof(path), "%s/state.json", m->state_dir) >= (int)sizeof(path)) {
		fault(m, "状态路径过长", 0);
		return;
	}
	struct stat st;
	if (lstat(path, &st) && errno == ENOENT) {
		start_initial(m);
		return;
	}
	cJSON *saved = mgr_read_json(path);
	const char *phase = mgr_string(saved, "phase"), *model = mgr_string(saved, "model");
	const char *last_boot = mgr_string(saved, "boot_id");
	const char *fault_code = mgr_string(saved, "fault_code");
	m->good = mgr_int(saved, "confirmed_mode", -1);
	m->recoveries = mgr_int(saved, "recovery_count", 1);
	int boot_id_valid = valid_boot_id(last_boot);
	int cold_boot = boot_id_valid && strcmp(last_boot, m->boot_id);
	int stopped = phase && !strcmp(phase, "STOPPED");
	int committed = phase && !strcmp(phase, "COMMITTED");
	int pending = phase && !strcmp(phase, "PENDING");
	int faulted = phase && !strcmp(phase, "FAULT");
	int confirmed = mgr_int(saved, "confirmed_mode", -2);
	int saved_mode = confirmed == -1 ? mgr_int(saved, "desired_mode", -1) : confirmed;
	int valid = mgr_int(saved, "schema_version", 0) == 1 && phase && model && !strcmp(model, m->model) &&
		    (!cJSON_HasObjectItem(saved, "boot_id") || boot_id_valid) && m->recoveries >= 0 &&
		    m->recoveries <= 1;
	/* 固件切换输出布局后重新验证。原布局的状态不能作为当前布局的回滚目标。 */
	if (m->fpga_detected && valid && (stopped || committed || pending || faulted || !strcmp(phase, "RECOVERING")) &&
	    mode_name(saved_mode) && !supported(m, saved_mode)) {
		cJSON_Delete(saved);
		start_initial(m);
		return;
	}
	int cold_stable = cold_boot && (committed || stopped);
	int confirmed_supported = supported(m, m->good);
	/* 只有明确记录的就绪验证超时允许在新一轮系统启动时重试。 */
	int timeout_fault = faulted && boot_id_valid && fault_code && !strcmp(fault_code, verification_timeout_code) &&
			    (confirmed_supported || confirmed == -1);
	if (cold_stable && m->recoveries >= 0 && m->recoveries <= 1)
		m->recoveries = 0;
	int resumable = confirmed_supported && (stopped || ((committed || pending) && m->recoveries < 1));
	if (!valid || (!resumable && !timeout_fault)) {
		cJSON_Delete(saved);
		fault(m, "上次恢复未完成、状态损坏或恢复次数耗尽", 0);
		return;
	}
	m->desired = m->good == -1 ? m->initial_mode : m->good;
	if (timeout_fault && !cold_boot) {
		m->desired = mgr_int(saved, "desired_mode", m->desired);
		m->fault_code = verification_timeout_code;
		cJSON_Delete(saved);
		fault(m, verification_timeout_error(m->desired), 0);
		return;
	}
	m->revalidating_timeout = timeout_fault;
	m->rollback = m->killed = 1;
	if (timeout_fault)
		m->recoveries = 1;
	else if (!stopped && !cold_stable)
		++m->recoveries;
	if (persist(m, "RECOVERING", 0))
		fault(m, "启动恢复状态无法落盘", 0);
	else
		prepare(m);
	cJSON_Delete(saved);
}
int main(int argc, char **argv)
{
	Manager manager = { .good = -1, .current = -1, .health = -1, .lock = -1, .server = -1 };
	if (argc < 2 || argc > 3 || (argc == 3 && strcmp(argv[2], "--simulation"))) {
		fprintf(stderr, "用法：lidar_mgrd 配置路径 [--simulation]\n");
		return 2;
	}
	manager.simulation = argc == 3;
	umask(0077);
	signal(SIGTERM, signal_stop);
	signal(SIGINT, signal_stop);
	signal(SIGPIPE, SIG_IGN);
	if (setup(&manager, argv[1])) {
		fprintf(stderr, "管理配置、软件包校验或独占锁检查失败\n");
		return 1;
	}
	boot(&manager);
	while (!stopping) {
		struct pollfd fds[2] = { { .fd = manager.server, .events = POLLIN },
					 { .fd = manager.health, .events = POLLIN } };
		if (poll(fds, 2, 20) > 0) {
			if (fds[0].revents & POLLIN)
				request(&manager);
			if (fds[1].revents & POLLIN)
				health_message(&manager);
			if (fds[1].revents & (POLLHUP | POLLERR | POLLNVAL)) {
				close(manager.health);
				manager.health = -1;
			}
		}
		tick(&manager);
	}
	kill_child(manager.adapter_pid, SIGKILL);
	kill_child(manager.app_pid, SIGTERM);
	int64_t deadline = mgr_now() + manager.stop_ms;
	int status = 0, clean = manager.app_pid <= 0;
	while (manager.app_pid > 0 && mgr_now() < deadline) {
		if (waitpid(manager.app_pid, &status, WNOHANG) == manager.app_pid) {
			clean = WIFEXITED(status) && !WEXITSTATUS(status);
			manager.app_pid = 0;
			break;
		}
		usleep(10000);
	}
	if (manager.app_pid > 0) {
		kill_child(manager.app_pid, SIGKILL);
		waitpid(manager.app_pid, NULL, 0);
	}
	if (manager.adapter_pid > 0)
		waitpid(manager.adapter_pid, NULL, 0);
	if (clean && !strcmp(manager.phase, "COMMITTED") && persist(&manager, "STOPPED", 0))
		clean = 0;
	unlink(manager.socket_path);
	cJSON_Delete(manager.settings);
	cJSON_Delete(manager.manifest);
	return clean ? 0 : 1;
}
