#define _GNU_SOURCE
#include "network_discovery.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

static int directory(const char *path)
{
	struct stat st;
	return (mkdir(path, 0755) && errno != EEXIST) || lstat(path, &st) || !S_ISDIR(st.st_mode) ||
			       st.st_uid != geteuid() || (st.st_mode & 0022) ?
		       -1 :
		       0;
}
static int write_text(const char *path, const char *text)
{
	int fd = open(path, O_CREAT | O_WRONLY | O_TRUNC | O_CLOEXEC | O_NOFOLLOW, 0644);
	if (fd < 0)
		return -1;
	size_t size = strlen(text), done = 0;
	while (done < size) {
		ssize_t n = write(fd, text + done, size - done);
		if (n < 0 && errno == EINTR)
			continue;
		if (n <= 0)
			break;
		done += (size_t)n;
	}
	int result = done == size && !fchmod(fd, 0644) ? 0 : -1;
	close(fd);
	return result;
}
int net_discovery_open(NetDiscovery *discovery, const NetInterface *interface, const char *path,
		       const char *services_path, const char *identity, const char *sn, const char *model,
		       unsigned port, int avahi_dbus_supported)
{
	*discovery = (NetDiscovery){ .socket = -1 };
	if (interface->simulation)
		return 0;
	/* 两条发现通道独立初始化，mDNS 文件或组件缺失不应关闭广播兜底。 */
	discovery->socket = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
	int on = 1;
	int broadcast_error = discovery->socket < 0 ||
			      setsockopt(discovery->socket, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) ||
			      setsockopt(discovery->socket, SOL_SOCKET, SO_BINDTODEVICE, interface->interface,
					 strlen(interface->interface) + 1);
	if (broadcast_error && discovery->socket >= 0) {
		close(discovery->socket);
		discovery->socket = -1;
	}
	/* 由 Avahi 处理名称探测、冲突改名、记录更新及接口变化；不自行编解码 DNS。 */
	char services[4096], file[4096], text[2048];
	if (snprintf(services, sizeof(services), "%s", services_path) >= (int)sizeof(services) ||
	    snprintf(file, sizeof(file), "%s/lidar.service", services) >= (int)sizeof(file) ||
	    snprintf(discovery->config_path, sizeof(discovery->config_path), "%s/avahi-daemon.conf", path) >=
		    (int)sizeof(discovery->config_path) ||
	    directory(path) || directory(services))
		return -1;
	/* identity、SN 和 model 均先经过仅允许字母、数字、点、横线、下划线的校验。 */
	snprintf(
		text, sizeof(text),
		"<?xml version=\"1.0\" standalone='no'?>\n<!DOCTYPE service-group SYSTEM \"avahi-service.dtd\">\n"
		"<service-group><name replace-wildcards=\"yes\">%%h</name><service protocol=\"ipv4\">"
		"<type>_lidar._tcp</type><port>%u</port><txt-record>proto=1</txt-record>"
		"<txt-record>identity=%s</txt-record><txt-record>sn=%s</txt-record><txt-record>model=%s</txt-record>"
		"<txt-record>path=/cgi-bin/lidar_network.cgi</txt-record><txt-record>modes=auto,static,pool</txt-record>"
		"</service></service-group>\n",
		port, identity, sn, model);
	if (write_text(file, text))
		return -1;
	snprintf(text, sizeof(text),
		 "[server]\nhost-name=lidar-%s\nuse-ipv4=yes\nuse-ipv6=no\nallow-interfaces=%s\n"
		 "use-iff-running=yes\n%sdisallow-other-stacks=yes\n"
		 "[wide-area]\nenable-wide-area=no\n[publish]\npublish-addresses=yes\npublish-hinfo=no\n"
		 "publish-workstation=no\npublish-domain=no\n[reflector]\nenable-reflector=no\n",
		 identity, interface->interface, avahi_dbus_supported ? "enable-dbus=no\n" : "");
	if (write_text(discovery->config_path, text))
		return -1;
	discovery->enabled = 1;
	return broadcast_error ? -1 : 0;
}
void net_discovery_tick(NetDiscovery *discovery, const NetInterface *interface, const char *binary,
			const cJSON *snapshot, unsigned port, int64_t now)
{
	if (interface->simulation)
		return;
	int status;
	if (discovery->pid > 0 && waitpid(discovery->pid, &status, WNOHANG) == discovery->pid) {
		discovery->pid = 0;
		++discovery->failures;
		discovery->retry_at = now + 60000;
		fprintf(stderr, "mDNS 服务退出，广播发现继续可用；下次重试在 60 秒后\n");
	}
	if (discovery->enabled && !discovery->pid && now >= discovery->retry_at && access(binary, X_OK) == 0) {
		pid_t parent = getpid();
		discovery->pid = fork();
		if (!discovery->pid) {
			if (prctl(PR_SET_PDEATHSIG, SIGTERM) || getppid() != parent)
				_exit(126);
			execl(binary, binary, "--no-drop-root", "--no-rlimits", "-f", discovery->config_path,
			      (char *)NULL);
			_exit(127);
		}
		if (discovery->pid < 0) {
			discovery->pid = 0;
			discovery->retry_at = now + 60000;
		}
	}
	if (discovery->socket < 0 || !interface->address || now < discovery->broadcast_at)
		return;
	discovery->broadcast_at = now + 5000;
	cJSON *message = cJSON_CreateObject();
	const char *keys[] = { "schema_version", "identity", "identity_source", "sn",	 "mac",	     "model",
			       "address",	 "source",   "phase",		"ready", "recovery", "generation" };
	for (unsigned i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
		cJSON *item = cJSON_GetObjectItemCaseSensitive(snapshot, keys[i]);
		if (item)
			cJSON_AddItemToObject(message, keys[i], cJSON_Duplicate(item, 1));
	}
	cJSON_AddStringToObject(message, "protocol", "lidar-network");
	cJSON_AddStringToObject(message, "service", "http");
	cJSON_AddStringToObject(message, "path", "/cgi-bin/lidar_network.cgi");
	cJSON *management = cJSON_GetObjectItemCaseSensitive(snapshot, "management_port");
	if (management)
		cJSON_AddItemToObject(message, "port", cJSON_Duplicate(management, 1));
	char *text = cJSON_PrintUnformatted(message);
	if (text) {
		struct sockaddr_in destination = { .sin_family = AF_INET,
						   .sin_port = htons((uint16_t)port),
						   .sin_addr.s_addr = INADDR_BROADCAST };
		sendto(discovery->socket, text, strlen(text), MSG_NOSIGNAL, (struct sockaddr *)&destination,
		       sizeof(destination));
	}
	free(text);
	cJSON_Delete(message);
}
void net_discovery_close(NetDiscovery *discovery)
{
	if (discovery->pid > 0) {
		kill(discovery->pid, SIGTERM);
		int64_t deadline = net_now() + 2000;
		int status;
		while (waitpid(discovery->pid, &status, WNOHANG) == 0 && net_now() < deadline)
			usleep(10000);
		if (waitpid(discovery->pid, &status, WNOHANG) == 0) {
			kill(discovery->pid, SIGKILL);
			waitpid(discovery->pid, &status, 0);
		}
	}
	if (discovery->socket >= 0)
		close(discovery->socket);
}
