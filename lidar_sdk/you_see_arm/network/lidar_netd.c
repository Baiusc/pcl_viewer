#define _GNU_SOURCE
#include "network_address.h"
#include "network_discovery.h"
#include <arpa/inet.h>
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
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

typedef struct {
	NetInterface interface;
	NetSaved saved;
	NetConfig desired, lease;
	NetProbe probe;
	NetDiscovery discovery;
	char state_path[PATH_MAX], runtime[PATH_MAX], socket_path[108];
	char dhcp_binary[PATH_MAX], dhcp_script[PATH_MAX], avahi_binary[PATH_MAX], mdns_directory[PATH_MAX];
	char avahi_services[PATH_MAX];
	char sn[NET_SN_CAPACITY], identity[40], model[64], dhcp_token[33], boot_token[33];
	char phase[24], transaction[24], source[16], target_source[16], error[160], conflict_mac[18];
	uint32_t seed, attempt, target, target_gateway;
	uint32_t rejected_lease;
	unsigned target_prefix, generation, conflicts, management_port, broadcast_port;
	int server, lock, simulation, link_up, lease_valid, changed, legacy, operation_code, recovery,
		configuration_fault;
	int avahi_dbus_supported;
	int confirm_ms, dhcp_ms, simulated_link_up;
	int64_t next_link, next_attempt, apply_at, confirm_at, dhcp_at, dhcp_retry_at, lease_until, defended_at;
	int64_t rejected_until;
	pid_t dhcp_pid;
} Network;
static volatile sig_atomic_t stopping;
static void stop_signal(int signo)
{
	(void)signo;
	stopping = 1;
}
static int64_t now_ms(const Network *n)
{
	return net_now() * (n->simulation ? 20 : 1);
}
static uint32_t random_value(void)
{
	uint32_t value = 0;
	if (net_random(&value, sizeof(value)))
		stopping = 1;
	return value;
}
static void state(Network *n, const char *phase)
{
	if (strcmp(n->phase, phase)) {
		snprintf(n->phase, sizeof(n->phase), "%s", phase);
		++n->generation;
	}
}
static cJSON *snapshot(const Network *n, int code, const char *message)
{
	char text[32];
	cJSON *value = cJSON_CreateObject();
	cJSON_AddNumberToObject(value, "schema_version", NET_SCHEMA);
	cJSON_AddNumberToObject(value, "code", code);
	cJSON_AddStringToObject(value, "message", message);
	cJSON_AddBoolToObject(value, "simulation", n->simulation);
	cJSON_AddBoolToObject(value, "ready",
			      n->link_up && n->interface.address && !n->recovery && !n->configuration_fault &&
				      !strcmp(n->phase, "READY"));
	cJSON_AddBoolToObject(value, "recovery", n->recovery);
	cJSON_AddBoolToObject(value, "configuration_valid", !n->configuration_fault);
	cJSON_AddStringToObject(value, "phase", n->configuration_fault ? "FAULT" : n->phase);
	cJSON_AddStringToObject(value, "transaction_phase", n->transaction);
	cJSON_AddBoolToObject(value, "pending", n->saved.pending);
	cJSON_AddBoolToObject(value, "legacy", n->legacy);
	cJSON_AddStringToObject(value, "operation_id", n->saved.operation);
	cJSON_AddNumberToObject(value, "operation_code", n->operation_code);
	cJSON_AddStringToObject(value, "error", n->error);
	cJSON_AddStringToObject(value, "interface", n->interface.interface);
	cJSON_AddStringToObject(value, "address", n->interface.address ? net_ip_text(n->interface.address, text) : "");
	cJSON_AddStringToObject(value, "netmask",
				n->interface.address ? net_ip_text(net_mask(n->interface.prefix), text) : "");
	cJSON_AddStringToObject(value, "gateway", n->interface.gateway ? net_ip_text(n->interface.gateway, text) : "");
	cJSON_AddStringToObject(value, "candidate_address", n->target ? net_ip_text(n->target, text) : "");
	cJSON_AddStringToObject(value, "source", n->source);
	cJSON_AddStringToObject(value, "identity", n->identity);
	cJSON_AddStringToObject(value, "identity_source", *n->sn ? "sn" : "mac");
	cJSON_AddStringToObject(value, "sn", n->sn);
	cJSON_AddStringToObject(value, "model", n->model);
	snprintf(text, sizeof(text), "%02x:%02x:%02x:%02x:%02x:%02x", n->interface.mac[0], n->interface.mac[1],
		 n->interface.mac[2], n->interface.mac[3], n->interface.mac[4], n->interface.mac[5]);
	cJSON_AddStringToObject(value, "mac", text);
	cJSON_AddStringToObject(value, "conflict_mac", n->conflict_mac);
	cJSON_AddNumberToObject(value, "attempt", n->attempt);
	cJSON_AddNumberToObject(value, "generation", n->generation);
	cJSON_AddStringToObject(value, "epoch", n->boot_token);
	cJSON_AddBoolToObject(value, "mdns_running", n->discovery.pid > 0);
	cJSON_AddNumberToObject(value, "management_port", n->management_port);
	cJSON_AddNumberToObject(value, "confirm_remaining_ms",
				n->saved.pending && !strcmp(n->transaction, "WAIT_CONFIRM") &&
						n->confirm_at > now_ms(n) ?
					n->confirm_at - now_ms(n) :
					0);
	cJSON_AddItemToObject(value, "confirmed_config", net_config_json(&n->saved.confirmed));
	cJSON_AddItemToObject(value, "desired_config", net_config_json(&n->desired));
	return value;
}
static void stop_dhcp(Network *n)
{
	if (n->dhcp_pid > 0) {
		/* udhcpc 及其事件脚本共享进程组，不能留下旧代回调。 */
		kill(-n->dhcp_pid, SIGKILL);
		waitpid(n->dhcp_pid, NULL, 0);
		n->dhcp_pid = 0;
	}
	n->dhcp_token[0] = 0;
}
static int start_dhcp(Network *n)
{
	if (n->simulation || n->dhcp_pid > 0)
		return 0;
	if (access(n->dhcp_binary, X_OK) || access(n->dhcp_script, X_OK) || net_token(n->dhcp_token))
		return -1;
	pid_t parent = getpid();
	n->dhcp_pid = fork();
	if (!n->dhcp_pid) {
		if (setpgid(0, 0) || prctl(PR_SET_PDEATHSIG, SIGKILL) || getppid() != parent)
			_exit(126);
		setenv("LIDAR_NET_SOCKET", n->socket_path, 1);
		setenv("LIDAR_DHCP_TOKEN", n->dhcp_token, 1);
		execl(n->dhcp_binary, n->dhcp_binary, "-f", "-i", n->interface.interface, "-s", n->dhcp_script, "-t",
		      "3", "-T", "2", "-A", "5", (char *)NULL);
		_exit(127);
	}
	if (n->dhcp_pid < 0) {
		n->dhcp_pid = 0;
		return -1;
	}
	setpgid(n->dhcp_pid, n->dhcp_pid);
	return 0;
}
static void start_policy(Network *n);
static void rollback(Network *n, int code, const char *reason)
{
	snprintf(n->error, sizeof(n->error), "%s", reason);
	n->operation_code = code;
	snprintf(n->transaction, sizeof(n->transaction), "FAILED");
	NetSaved restored = n->saved;
	restored.pending = 0;
	restored.candidate = restored.confirmed;
	if (net_saved_store(n->state_path, &restored, 0)) {
		n->configuration_fault = 1;
		n->saved.pending = 0;
		n->legacy = 0;
		/* 磁盘上保留 PENDING，重启仍恢复旧配置；当前不能无限保留未确认地址。 */
		snprintf(n->error, sizeof(n->error), "回滚状态无法持久化，保留原事务并进入临时恢复网络");
		if (net_interface_apply(&n->interface, 0, 0, 0))
			state(n, "FAULT");
		else
			start_policy(n);
		return;
	}
	n->saved = restored;
	if (n->changed && net_interface_apply(&n->interface, 0, 0, 0)) {
		state(n, "FAULT");
		return;
	}
	n->changed = 0;
	n->legacy = 0;
	start_policy(n);
}
static void choose(Network *n)
{
	int64_t now = now_ms(n);
	if (n->desired.mode == NET_AUTO && n->lease_valid && now < n->lease_until) {
		n->target = n->lease.address;
		n->target_prefix = n->lease.prefix;
		n->target_gateway = n->lease.gateway;
		strcpy(n->target_source, "dhcp");
	} else {
		NetConfig automatic = { .mode = NET_AUTO };
		const NetConfig *policy = n->recovery ? &automatic : &n->desired;
		n->target = net_candidate(policy, n->seed, n->attempt, n->saved.confirmed_address);
		n->target_prefix = policy->mode == NET_AUTO ? 16 : policy->prefix;
		n->target_gateway = policy->mode == NET_AUTO ? 0 : policy->gateway;
		strcpy(n->target_source, n->recovery		     ? "recovery" :
					 n->desired.mode == NET_AUTO ? "linklocal" :
					 n->desired.mode == NET_POOL ? "pool" :
								       "static");
		while (n->target && n->target == n->target_gateway) {
			++n->attempt;
			n->target = net_candidate(policy, n->seed, n->attempt, n->saved.confirmed_address);
		}
	}
	if (!n->target) {
		if (n->saved.pending)
			rollback(n, 409, "指定地址范围已探测完毕，没有可用地址");
		else {
			if (!n->recovery && n->desired.mode != NET_AUTO) {
				n->recovery = 1;
				n->attempt = 0;
				snprintf(n->error, sizeof(n->error),
					 "业务地址范围已耗尽，启用仅用于管理的 Link-Local 恢复地址");
				choose(n);
			} else {
				state(n, "NO_ADDRESS");
				n->next_attempt = now + 60000;
			}
		}
		return;
	}
	net_probe_start(&n->probe, n->target, now, random_value());
	state(n, "PROBING");
	if (n->saved.pending)
		strcpy(n->transaction, "PROBING");
}
static void start_policy(Network *n)
{
	if (n->configuration_fault) {
		/* 原文件保持不动，只提供临时的链路本地诊断地址。 */
		stop_dhcp(n);
		n->lease_valid = 0;
		n->desired = (NetConfig){ .mode = NET_AUTO };
		n->recovery = 1;
		n->probe.address = 0;
		n->attempt = 0;
		if (n->link_up)
			choose(n);
		else
			state(n, "LINK_DOWN");
		return;
	}
	n->desired = n->saved.pending ? n->saved.candidate : n->saved.confirmed;
	n->probe.address = 0;
	n->attempt = n->conflicts = 0;
	n->recovery = 0;
	n->next_attempt = 0;
	if (!n->link_up) {
		state(n, "LINK_DOWN");
		return;
	}
	if (n->desired.mode == NET_AUTO) {
		int dhcp_ok = !start_dhcp(n);
		if (!n->lease_valid) {
			state(n, "WAIT_DHCP");
			n->dhcp_at = now_ms(n) + (dhcp_ok ? n->dhcp_ms : 0);
			return;
		}
	} else
		stop_dhcp(n);
	choose(n);
}
static void candidate_conflict(Network *n, const unsigned char *packet)
{
	int64_t now = now_ms(n);
	snprintf(n->conflict_mac, sizeof(n->conflict_mac), "%02x:%02x:%02x:%02x:%02x:%02x", packet[8], packet[9],
		 packet[10], packet[11], packet[12], packet[13]);
	++n->conflicts;
	n->probe.address = 0;
	if (n->interface.address == n->target && net_interface_apply(&n->interface, 0, 0, 0)) {
		state(n, "FAULT");
		return;
	}
	if (!strcmp(n->target_source, "dhcp")) {
		n->rejected_lease = n->target;
		n->rejected_until = now + 60000;
		n->lease_valid = 0;
		stop_dhcp(n);
		start_dhcp(n);
	}
	if (n->desired.mode == NET_STATIC && !n->recovery) {
		if (n->saved.pending)
			rollback(n, 409, "指定固定 IP 已被其他设备占用");
		else {
			n->recovery = 1;
			n->attempt = 0;
			snprintf(n->error, sizeof(n->error), "保存的固定 IP 冲突，启用管理恢复地址；固定配置未改变");
			choose(n);
		}
		return;
	}
	if (n->desired.mode == NET_AUTO || n->recovery) {
		/* 冲突回退引入新的随机性，避免两个候选相同的设备同步逐项退让。 */
		n->seed = random_value();
		n->attempt = 0;
	} else
		++n->attempt;
	if (n->conflicts >= 10) {
		state(n, "BACKOFF");
		n->next_attempt = now + 60000;
		n->conflicts = 0;
	} else
		choose(n);
}
static void receive_arp(Network *n, const unsigned char *packet)
{
	int64_t now = now_ms(n);
	if (n->probe.address && (!n->probe.ready || (n->saved.pending && !n->changed)) &&
	    net_arp_conflict(packet, 28, n->interface.mac, n->target, 1)) {
		candidate_conflict(n, packet);
		return;
	}
	if (!net_arp_conflict(packet, 28, n->interface.mac, n->interface.address, 0))
		return;
	if (!n->defended_at || now - n->defended_at >= 10000) {
		n->defended_at = now;
		if (net_arp_send(&n->interface, n->interface.address, 0))
			state(n, "FAULT");
		return;
	}
	if (net_interface_apply(&n->interface, 0, 0, 0)) {
		state(n, "FAULT");
		return;
	}
	++n->generation;
	if (n->saved.pending) {
		rollback(n, 409, "运行中地址持续冲突，已撤销候选配置");
		return;
	}
	n->target = n->probe.address ? n->target : n->saved.confirmed_address;
	candidate_conflict(n, packet);
}
static void apply_candidate(Network *n)
{
	if (!strcmp(n->target_source, "dhcp") && (!n->lease_valid || now_ms(n) >= n->lease_until)) {
		start_policy(n);
		return;
	}
	if (net_interface_apply(&n->interface, n->target, n->target_prefix, n->target_gateway)) {
		if (n->saved.pending)
			rollback(n, 500, "应用地址或路由失败，恢复最后确认配置");
		else {
			state(n, "FAULT");
			snprintf(n->error, sizeof(n->error), "地址或路由配置失败");
		}
		return;
	}
	snprintf(n->source, sizeof(n->source), "%s", n->target_source);
	n->defended_at = 0;
	n->attempt = n->conflicts = 0;
	++n->generation;
	n->discovery.broadcast_at = 0;
	state(n, "READY");
	if (net_arp_send(&n->interface, n->target, 0)) {
		if (n->saved.pending)
			rollback(n, 500, "ARP 宣告发送失败");
		else
			state(n, "FAULT");
		return;
	}
	n->probe.ready = 1;
	n->probe.announcements = 1;
	n->probe.next = now_ms(n) + 2000;
	if (n->saved.pending) {
		n->changed = 1;
		strcpy(n->transaction, "WAIT_CONFIRM");
		n->confirm_at = now_ms(n) + n->confirm_ms;
	} else if (!n->recovery && n->saved.confirmed_address != n->target) {
		NetSaved updated = n->saved;
		updated.confirmed_address = n->target;
		if (net_saved_store(n->state_path, &updated, 0)) {
			n->configuration_fault = 1;
			n->operation_code = 500;
			snprintf(n->error, sizeof(n->error), "当前地址记录无法持久化，保留原文件并进入临时恢复网络");
			if (net_interface_apply(&n->interface, 0, 0, 0))
				state(n, "FAULT");
			else
				start_policy(n);
		} else
			n->saved = updated;
	}
}
static cJSON *begin_request(Network *n, const cJSON *request)
{
	if (n->saved.pending)
		return snapshot(n, 409, "已有网络设置等待处理或确认");
	if (n->configuration_fault || !strcmp(n->phase, "FAULT"))
		return snapshot(n, 503, "网络服务故障，请检查持久存储及系统配置");
	NetSaved candidate = n->saved;
	char error[160];
	if (net_config_parse(cJSON_GetObjectItemCaseSensitive(request, "config"), &candidate.candidate, error))
		return snapshot(n, 422, error);
	if (!n->link_up)
		return snapshot(n, 409, "链路未连接，不能验证并应用新地址");
	if (net_token(candidate.operation))
		return snapshot(n, 500, "无法生成网络任务标识");
	candidate.pending = 1;
	if (net_saved_store(n->state_path, &candidate, 0))
		return snapshot(n, 500, "网络设置意图无法持久化，保留当前配置");
	n->saved = candidate;
	n->legacy = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(request, "legacy"));
	n->changed = 0;
	n->operation_code = 202;
	n->error[0] = n->conflict_mac[0] = 0;
	strcpy(n->transaction, "PROBING");
	start_policy(n);
	return snapshot(n, 202, "网络设置已受理，探测和应用结果请查询任务状态");
}
static cJSON *lease_event(Network *n, const cJSON *request)
{
	if (n->configuration_fault)
		return snapshot(n, 503, "持久配置不可用，当前仅提供临时恢复地址");
	const char *token = net_string(request, "token"), *event = net_string(request, "event"),
		   *interface = net_string(request, "interface");
	if (!n->simulation && (!token || !*n->dhcp_token || strcmp(token, n->dhcp_token)))
		return snapshot(n, 403, "DHCP 回调已过期或凭据错误");
	if (!event || !interface || strcmp(interface, n->interface.interface) || n->desired.mode != NET_AUTO)
		return snapshot(n, 409, "忽略非当前接口或非自动模式的 DHCP 事件");
	if (!strcmp(event, "deconfig") || !strcmp(event, "nak")) {
		n->lease_valid = 0;
		if (!strcmp(n->source, "dhcp")) {
			net_interface_apply(&n->interface, 0, 0, 0);
			start_policy(n);
		}
		return snapshot(n, 200, "租约已撤销");
	}
	if (strcmp(event, "bound") && strcmp(event, "renew"))
		return snapshot(n, 200, "已忽略非地址事件");
	cJSON *configuration = cJSON_CreateObject();
	cJSON_AddStringToObject(configuration, "mode", "static");
	const char *keys[] = { "address", "netmask", "gateway" };
	for (unsigned i = 0; i < 3; ++i) {
		const char *text = net_string(request, keys[i]);
		if (text)
			cJSON_AddStringToObject(configuration, keys[i], text);
	}
	char error[160];
	NetConfig lease;
	const cJSON *seconds = cJSON_GetObjectItemCaseSensitive(request, "lease_seconds");
	int invalid = net_config_parse(configuration, &lease, error) || !net_integer(seconds, 1, INT_MAX);
	cJSON_Delete(configuration);
	if (invalid)
		return snapshot(n, 422, "DHCP 租约参数无效");
	if (lease.address == n->rejected_lease && now_ms(n) < n->rejected_until)
		return snapshot(n, 409, "DHCP 地址刚发生冲突，暂时拒绝重复候选");
	n->lease = lease;
	n->lease_valid = 1;
	n->lease_until = now_ms(n) + (int64_t)seconds->valueint * 1000;
	if (n->interface.address == lease.address && n->interface.prefix == lease.prefix &&
	    n->interface.gateway == lease.gateway && !strcmp(n->source, "dhcp"))
		return snapshot(n, 200, "租约已续期");
	choose(n);
	return snapshot(n, 202, "租约已接收，地址正在探测");
}
static cJSON *handle(Network *n, const cJSON *request)
{
	const char *op = net_string(request, "op");
	if (!net_unique(request) || !op)
		return snapshot(n, 400, "请求必须是键名唯一的 JSON 对象");
	const cJSON *schema = cJSON_GetObjectItemCaseSensitive(request, "schema_version");
	if (schema && !net_integer(schema, NET_SCHEMA, NET_SCHEMA))
		return snapshot(n, 422, "不支持该网络接口版本");
	if (!strcmp(op, "network_get"))
		return snapshot(n, 200, "网络状态");
	if (!strcmp(op, "network_set"))
		return begin_request(n, request);
	if (!strcmp(op, "network_reset")) {
		cJSON *reset = cJSON_CreateObject(), *config = cJSON_CreateObject();
		cJSON_AddStringToObject(config, "mode", "auto");
		cJSON_AddItemToObject(reset, "config", config);
		cJSON *response = begin_request(n, reset);
		cJSON_Delete(reset);
		return response;
	}
	if (!strcmp(op, "network_confirm") || !strcmp(op, "network_apply")) {
		const char *operation = net_string(request, "operation_id");
		if (!operation || !n->saved.pending || strcmp(operation, n->saved.operation))
			return snapshot(n, 409, "没有匹配的待处理网络任务");
		if (!strcmp(op, "network_apply")) {
			if (!n->legacy || strcmp(n->transaction, "PREPARED"))
				return snapshot(n, 409, "旧协议候选尚未准备完成");
			strcpy(n->transaction, "APPLY_DELAY");
			n->apply_at = now_ms(n) + 1000;
			return snapshot(n, 202, "旧协议已确认回包，准备切换地址");
		}
		uint32_t via;
		if (strcmp(n->transaction, "WAIT_CONFIRM") || now_ms(n) >= n->confirm_at ||
		    net_ip(net_string(request, "via_address"), &via) || via != n->interface.address)
			return snapshot(n, 409, "须在确认期限内通过当前新地址确认");
		NetSaved committed = n->saved;
		committed.confirmed = committed.candidate;
		committed.confirmed_address = n->interface.address;
		committed.pending = 0;
		if (net_saved_store(n->state_path, &committed, 1))
			return snapshot(n, 500, "确认结果无法持久化，任务仍等待确认");
		n->saved = committed;
		n->changed = n->legacy = 0;
		n->operation_code = 200;
		strcpy(n->transaction, "COMMITTED");
		return snapshot(n, 200, "网络配置已确认并保存");
	}
	if (!strcmp(op, "dhcp_event"))
		return lease_event(n, request);
	if (n->simulation && !strcmp(op, "simulate_link")) {
		const cJSON *up = cJSON_GetObjectItemCaseSensitive(request, "up");
		if (!cJSON_IsBool(up))
			return snapshot(n, 400, "模拟链路参数须为布尔值");
		n->simulated_link_up = cJSON_IsTrue(up);
		n->next_link = 0;
		return snapshot(n, 200, "模拟链路事件已接收");
	}
	if (n->simulation && !strcmp(op, "simulate_arp")) {
		uint32_t sender, target;
		const char *mac = net_string(request, "mac");
		unsigned parts[6];
		if (net_ip(net_string(request, "sender"), &sender) || net_ip(net_string(request, "target"), &target) ||
		    !mac ||
		    sscanf(mac, "%2x:%2x:%2x:%2x:%2x:%2x", &parts[0], &parts[1], &parts[2], &parts[3], &parts[4],
			   &parts[5]) != 6)
			return snapshot(n, 400, "模拟 ARP 参数错误");
		unsigned char packet[28] = { 0, 1, 8, 0, 6, 4, 0, 1 };
		for (unsigned i = 0; i < 6; ++i)
			packet[8 + i] = (unsigned char)parts[i];
		sender = htonl(sender);
		target = htonl(target);
		memcpy(packet + 14, &sender, 4);
		memcpy(packet + 24, &target, 4);
		receive_arp(n, packet);
		return snapshot(n, 200, "模拟 ARP 已处理");
	}
	return snapshot(n, 501, "不支持的网络操作");
}
static void tick(Network *n)
{
	int64_t now = now_ms(n);
	if (now >= n->next_link) {
		n->next_link = now + 1000;
		int linked = n->simulation ? n->simulated_link_up : net_interface_link(&n->interface);
		if (linked != n->link_up) {
			n->link_up = linked;
			n->lease_valid = 0;
			stop_dhcp(n);
			if (!linked) {
				if (net_interface_apply(&n->interface, 0, 0, 0)) {
					state(n, "FAULT");
					return;
				}
				n->source[0] = 0;
				n->defended_at = 0;
			}
			if (n->saved.pending)
				rollback(n, 409, "网络链路发生变化，已取消改址任务");
			else
				start_policy(n);
		}
	}
	if (!n->link_up)
		return;
	int status;
	if (n->dhcp_pid > 0 && waitpid(n->dhcp_pid, &status, WNOHANG) == n->dhcp_pid) {
		n->dhcp_pid = 0;
		n->dhcp_retry_at = now + 5000;
	}
	if (n->desired.mode == NET_AUTO && !n->configuration_fault && !n->simulation && !n->dhcp_pid &&
	    now >= n->dhcp_retry_at) {
		start_dhcp(n);
		n->dhcp_retry_at = now + 5000;
	}
	if (n->lease_valid && now >= n->lease_until) {
		n->lease_valid = 0;
		if (!strcmp(n->source, "dhcp")) {
			net_interface_apply(&n->interface, 0, 0, 0);
			start_policy(n);
		}
	}
	if (n->saved.pending && ((!strcmp(n->transaction, "WAIT_CONFIRM") && now >= n->confirm_at) ||
				 (!strcmp(n->transaction, "PREPARED") && now >= n->apply_at))) {
		rollback(n, 408, "改址确认超时，恢复最后确认配置");
		return;
	}
	if (!strcmp(n->phase, "FAULT"))
		return;
	if (!strcmp(n->transaction, "APPLY_DELAY")) {
		if (now >= n->apply_at)
			apply_candidate(n);
		return;
	}
	if (!strcmp(n->transaction, "PREPARED"))
		return;
	if ((!strcmp(n->phase, "WAIT_DHCP") && now >= n->dhcp_at) ||
	    (!strcmp(n->phase, "BACKOFF") && now >= n->next_attempt))
		choose(n);
	else if (!strcmp(n->phase, "NO_ADDRESS") && now >= n->next_attempt) {
		n->attempt = 0;
		choose(n);
	}
	unsigned action = net_probe_tick(&n->probe, now, random_value());
	if ((action & NET_PROBE_SEND) && net_arp_send(&n->interface, n->target, 1)) {
		if (n->saved.pending)
			rollback(n, 500, "ARP 探测发送失败");
		else
			state(n, "FAULT");
		return;
	}
	if (action & NET_PROBE_READY) {
		if (n->saved.pending) {
			strcpy(n->transaction, n->legacy ? "PREPARED" : "APPLY_DELAY");
			n->apply_at = now + (n->legacy ? 30000 : 1000);
		} else
			apply_candidate(n);
	} else if ((action & NET_ANNOUNCE_SEND) && n->interface.address &&
		   net_arp_send(&n->interface, n->interface.address, 0))
		state(n, "FAULT");
}
static int path_value(char *out, size_t length, const cJSON *settings, const char *key, const char *fallback)
{
	if (cJSON_HasObjectItem(settings, key) && !cJSON_IsString(cJSON_GetObjectItemCaseSensitive(settings, key)))
		return -1;
	const char *value = net_string(settings, key);
	if (!value)
		value = fallback;
	if (!value || value[0] != '/' || strlen(value) >= length || strchr(value, '\n'))
		return -1;
	strcpy(out, value);
	return 0;
}
static int setup(Network *n, const char *path)
{
	int missing;
	cJSON *settings = net_json_read(path, &missing);
	if (!net_unique(settings)) {
		fprintf(stderr, "系统网络配置无法读取或格式错误：%s\n", path);
		cJSON_Delete(settings);
		return -1;
	}
	const char *interface = net_string(settings, "interface"), *model = net_string(settings, "model");
	char sn_path[PATH_MAX], legacy_path[PATH_MAX], legacy_flash[PATH_MAX];
	int invalid =
		!model || !*model || strlen(model) >= sizeof(n->model) ||
		strspn(model, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-") != strlen(model) ||
		path_value(n->state_path, sizeof(n->state_path), settings, "state_path",
			   "/mnt/mtdblock2/lidar/config/network.json") ||
		path_value(n->runtime, sizeof(n->runtime), settings, "runtime_dir", "/run/lidar-network") ||
		path_value(n->socket_path, sizeof(n->socket_path), settings, "socket_path", NET_SOCKET_DEFAULT) ||
		path_value(n->dhcp_binary, sizeof(n->dhcp_binary), settings, "udhcpc", "/sbin/udhcpc") ||
		path_value(n->dhcp_script, sizeof(n->dhcp_script), settings, "dhcp_script",
			   "/usr/libexec/lidar/lidar-dhcp-event") ||
		path_value(n->avahi_binary, sizeof(n->avahi_binary), settings, "avahi", "/usr/sbin/avahi-daemon") ||
		path_value(n->mdns_directory, sizeof(n->mdns_directory), settings, "mdns_directory",
			   "/run/lidar-mdns") ||
		path_value(n->avahi_services, sizeof(n->avahi_services), settings, "avahi_services",
			   "/etc/avahi/services") ||
		path_value(sn_path, sizeof(sn_path), settings, "sn_path", "/dev/mtd3") ||
		path_value(legacy_path, sizeof(legacy_path), settings, "legacy_interfaces",
			   "/etc/network/interfaces") ||
		path_value(legacy_flash, sizeof(legacy_flash), settings, "legacy_flash_interfaces",
			   "/mnt/mtdblock2/interfaces");
	const cJSON *confirm = cJSON_GetObjectItemCaseSensitive(settings, "confirm_timeout_ms"),
		    *dhcp = cJSON_GetObjectItemCaseSensitive(settings, "dhcp_timeout_ms");
	const cJSON *management_port = cJSON_GetObjectItemCaseSensitive(settings, "management_port"),
		    *broadcast_port = cJSON_GetObjectItemCaseSensitive(settings, "broadcast_port");
	const cJSON *avahi_dbus = cJSON_GetObjectItemCaseSensitive(settings, "avahi_dbus_supported");
	n->avahi_dbus_supported = avahi_dbus && cJSON_IsTrue(avahi_dbus);
	n->confirm_ms = confirm ? confirm->valueint : 60000;
	n->dhcp_ms = dhcp ? dhcp->valueint : 5000;
	n->management_port = management_port ? (unsigned)management_port->valueint : 80;
	n->broadcast_port = broadcast_port ? (unsigned)broadcast_port->valueint : 5354;
	invalid |= (confirm && !net_integer(confirm, 1000, 300000)) || (dhcp && !net_integer(dhcp, 1000, 60000)) ||
		   (management_port && !net_integer(management_port, 1, 65535)) ||
		   (broadcast_port && !net_integer(broadcast_port, 1024, 65535)) ||
		   (avahi_dbus && !cJSON_IsBool(avahi_dbus));
	if (invalid || net_interface_open(&n->interface, interface, n->simulation)) {
		fprintf(stderr, "系统网络字段或接口初始化失败：%s\n", strerror(errno));
		cJSON_Delete(settings);
		return -1;
	}
	strcpy(n->model, model);
	net_identity(sn_path, n->interface.mac, n->sn, n->identity);
	n->seed = 2166136261u;
	for (unsigned i = 0; i < 6; ++i)
		n->seed = (n->seed ^ n->interface.mac[i]) * 16777619u;
	if (net_token(n->boot_token)) {
		cJSON_Delete(settings);
		return -1;
	}
	struct stat st;
	if ((mkdir(n->runtime, 0700) && errno != EEXIST) || lstat(n->runtime, &st) || !S_ISDIR(st.st_mode) ||
	    st.st_uid != geteuid() || (st.st_mode & 0022)) {
		fprintf(stderr, "运行目录检查失败：%s\n", n->runtime);
		cJSON_Delete(settings);
		return -1;
	}
	char lockfile[PATH_MAX];
	if (snprintf(lockfile, sizeof(lockfile), "%s/network.lock", n->runtime) >= (int)sizeof(lockfile)) {
		cJSON_Delete(settings);
		return -1;
	}
	n->lock = open(lockfile, O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
	if (n->lock < 0 || flock(n->lock, LOCK_EX | LOCK_NB)) {
		fprintf(stderr, "网络配置独占锁失败：%s\n", strerror(errno));
		cJSON_Delete(settings);
		return -1;
	}
	int loaded = net_saved_load(n->state_path, &n->saved);
	if (loaded == 1) {
		const cJSON *initial = cJSON_GetObjectItemCaseSensitive(settings, "initial_config");
		char error[160];
		if (initial)
			invalid = net_config_parse(initial, &n->saved.confirmed, error);
		else {
			/* 旧固件优先从第三分区恢复每台设备的配置，镜像中的备份仅作后备。 */
			int imported = net_import_legacy(legacy_flash, n->interface.interface, &n->saved.confirmed);
			if (imported == 1)
				imported = net_import_legacy(legacy_path, n->interface.interface, &n->saved.confirmed);
			invalid = imported < 0;
		}
		n->saved.candidate = n->saved.confirmed;
		if (invalid || net_saved_store(n->state_path, &n->saved, 0))
			loaded = -1;
	}
	cJSON_Delete(settings);
	if (loaded < 0) {
		n->configuration_fault = 1;
		n->operation_code = 500;
		snprintf(n->error, sizeof(n->error), "网络配置损坏或存储不可用，保留原文件并提供临时恢复地址");
	} else if (n->saved.pending) {
		n->saved.pending = 0;
		n->saved.candidate = n->saved.confirmed;
		if (net_saved_store(n->state_path, &n->saved, 0))
			n->configuration_fault = 1;
		n->operation_code = 409;
		snprintf(n->error, sizeof(n->error), "上次改址未确认，恢复最后确认配置");
		strcpy(n->transaction, "ROLLED_BACK");
	}
	n->server = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
	struct sockaddr_un address = { .sun_family = AF_UNIX };
	strcpy(address.sun_path, n->socket_path);
	if (!lstat(n->socket_path, &st)) {
		if (!S_ISSOCK(st.st_mode) || st.st_uid != geteuid())
			return -1;
		unlink(n->socket_path);
	}
	if (n->server < 0 || bind(n->server, (struct sockaddr *)&address, sizeof(address)) ||
	    chmod(n->socket_path, 0600) || listen(n->server, 8)) {
		fprintf(stderr, "网络 UDS 监听失败：%s\n", strerror(errno));
		return -1;
	}
	n->link_up = net_interface_link(&n->interface);
	if (n->configuration_fault || strcmp(n->phase, "FAULT"))
		start_policy(n);
	if (net_discovery_open(&n->discovery, &n->interface, n->mdns_directory, n->avahi_services, n->identity, n->sn,
			       n->model, n->management_port, n->avahi_dbus_supported))
		fprintf(stderr, "发现服务资源初始化失败，请检查系统镜像配置\n");
	return 0;
}
int main(int argc, char **argv)
{
	Network network = { .server = -1,
			    .lock = -1,
			    .discovery.socket = -1,
			    .interface.arp_fd = -1,
			    .operation_code = 200,
			    .simulated_link_up = 1 };
	if (argc < 2 || argc > 3 || (argc == 3 && strcmp(argv[2], "--simulation"))) {
		fprintf(stderr, "用法：lidar_netd 系统网络配置 [--simulation]\n");
		return 2;
	}
	network.simulation = argc == 3;
	strcpy(network.transaction, "IDLE");
	umask(0077);
	struct sigaction action = { .sa_handler = stop_signal };
	sigemptyset(&action.sa_mask);
	sigaction(SIGTERM, &action, NULL);
	sigaction(SIGINT, &action, NULL);
	if (setup(&network, argv[1])) {
		fprintf(stderr, "网络服务启动失败：配置、接口、存储或独占检查未通过\n");
		return 1;
	}
	struct {
		int fd;
		int64_t deadline;
	} clients[8];
	for (unsigned i = 0; i < 8; ++i)
		clients[i].fd = -1;
	while (!stopping) {
		struct pollfd fds[10] = { { .fd = network.server, .events = POLLIN },
					  { .fd = network.interface.arp_fd, .events = POLLIN } };
		for (unsigned i = 0; i < 8; ++i)
			fds[2 + i] = (struct pollfd){ .fd = clients[i].fd, .events = POLLIN };
		int ready = poll(fds, 10, 20);
		if (ready < 0 && errno != EINTR)
			break;
		if (fds[0].revents & POLLIN) {
			int fd = accept4(network.server, NULL, NULL, SOCK_CLOEXEC | SOCK_NONBLOCK);
			if (fd >= 0) {
				struct ucred peer;
				socklen_t length = sizeof(peer);
				unsigned slot = 0;
				while (slot < 8 && clients[slot].fd >= 0)
					++slot;
				if (slot == 8 || getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &peer, &length) ||
				    peer.uid != geteuid())
					close(fd);
				else {
					clients[slot].fd = fd;
					clients[slot].deadline = net_now() + 500;
				}
			}
		}
		if (fds[1].revents & POLLIN) {
			for (unsigned count = 0; count < 64; ++count) {
				unsigned char packet[28];
				int read = net_arp_read(&network.interface, packet);
				if (read < 0)
					break;
				if (read)
					receive_arp(&network, packet);
			}
		}
		for (unsigned i = 0; i < 8; ++i) {
			if (clients[i].fd < 0)
				continue;
			if (fds[i + 2].revents & POLLIN) {
				char buffer[NET_MESSAGE_MAX + 1];
				ssize_t size = recv(clients[i].fd, buffer, NET_MESSAGE_MAX, MSG_TRUNC);
				cJSON *request = NULL;
				if (size > 0 && size <= NET_MESSAGE_MAX && !memchr(buffer, 0, (size_t)size)) {
					buffer[size] = 0;
					if (!strstr(buffer, "\\u0000"))
						request = cJSON_ParseWithLengthOpts(buffer, (size_t)size + 1, NULL, 1);
				}
				cJSON *response = handle(&network, request);
				char *text = cJSON_PrintUnformatted(response);
				if (text)
					send(clients[i].fd, text, strlen(text), MSG_NOSIGNAL | MSG_DONTWAIT);
				free(text);
				cJSON_Delete(response);
				cJSON_Delete(request);
				close(clients[i].fd);
				clients[i].fd = -1;
			} else if (net_now() >= clients[i].deadline || (fds[i + 2].revents & (POLLHUP | POLLERR))) {
				close(clients[i].fd);
				clients[i].fd = -1;
			}
		}
		tick(&network);
		cJSON *current = snapshot(&network, 200, "网络状态");
		net_discovery_tick(&network.discovery, &network.interface, network.avahi_binary, current,
				   network.broadcast_port, now_ms(&network));
		cJSON_Delete(current);
	}
	for (unsigned i = 0; i < 8; ++i)
		if (clients[i].fd >= 0)
			close(clients[i].fd);
	stop_dhcp(&network);
	net_discovery_close(&network.discovery);
	unlink(network.socket_path);
	close(network.server);
	if (network.interface.arp_fd >= 0)
		close(network.interface.arp_fd);
	close(network.lock);
	return 0;
}
