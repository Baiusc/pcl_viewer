/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 23:58:16
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-10-12 20:22:11
 * @FilePath     : /you_see_arm/core/ys_udp.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#define _GNU_SOURCE

#include "ys_base.h"
#include "ys_udp.h"
#include "ys_core_network.h"

#include "../common/common.h"

#include <uv.h>
#include <sched.h>
#include <stdatomic.h>

static atomic_uint_fast8_t g_run_flag;
cent_udp_client_t *udpc = NULL;
char *recv_buf = NULL; // 初始化的时候分配内存后就一直使用这个，减少系统开销
const int g_timer_interval_ms = 1 * 1000; // 广播定时器的超时时间

static void read_cb(uv_udp_t *handle, ssize_t nread, const uv_buf_t *buf, const struct sockaddr *addr, unsigned flags);
static void udp_client_connected(cent_udp_client_t *udpc, udp_center_t *center, udp_t *udp_instance);
static void connect_center(cent_udp_client_t *udpc, udp_center_t *center);
static void handle_hb_to_cent(uv_timer_t *timer);
static int udp_bind_local_port(cent_udp_client_t *udpc, udp_center_t *center);
static void refresh_local_address(cent_udp_client_t *udpc);
static void receive_managed_udp(uv_poll_t *poll, int status, int events);
static int init_udp_instance(udp_t *udp_instance, cent_udp_client_t *udpc);
static void init_udp_netcheck_timer(udp_t *udp_instance, cent_udp_client_t *udpc);
static void connect_udp_center(cent_udp_client_t *udpc);

/**
 * @description  : malloc buffer by uv handle
 * @param         {uv_handle_t} *handle
 * @param         {size_t} suggested_size
 * @param         {uv_buf_t} *buf
 * @return        {*}
 */
static void alloc_buffer(uv_handle_t *handle, size_t suggested_size, uv_buf_t *buf)
{
	buf->base = recv_buf;
	memset(buf->base, 0, RECV_BUF_MAX_LEN);
	buf->len = RECV_BUF_MAX_LEN;
	return;
}

int send_buf_to_center(udp_center_t *center, uint8_t *data, int len)
{
	int status = 0;
	uv_buf_t buf = uv_buf_init((char *)data, len);

	if (center->is_connected == CONNECTED) {
		status = uv_udp_try_send(&(center->uv_handle), &buf, 1, (const struct sockaddr *)&(center->addr));
		if (status < 0) {
			log_error("UDP: Fail to send data to server, error: %s", uv_strerror(status));
		}
	}

	return status;
}

/**
 * @description  : 接收 udp 数据的回调函数
 * @param         {uv_udp_t} *handle
 * @param         {ssize_t} nread
 * @param         {uv_buf_t} *buf
 * @param         {sockaddr} *addr
 * @param         {unsigned} flags
 * @return        {*}
 */
static void read_cb(uv_udp_t *handle, ssize_t nread, const uv_buf_t *buf, const struct sockaddr *addr, unsigned flags)
{
	udp_center_t *center = (udp_center_t *)handle->data;
	cent_udp_client_t *udpc = (cent_udp_client_t *)center->data;
	udp_t *udp_instance = (udp_t *)udpc->data;

	if (nread < 0) {
		log_error("UDP: read cb error: %s", uv_strerror(nread));
	} else if (nread > 0) {
		hex_to_string("recv", (uint8_t *)(buf->base), nread);
		(*udp_instance->deal_data_to_input_link)(udp_instance, (uint8_t *)(buf->base), nread);
	}

	return;
}

/**
 * @description  : udp 连接上，开始读取数据
 * @param         {cent_udp_client_t} *udpc
 * @param         {udp_center_t} *center
 * @param         {udp_t} *udp_instance
 * @return        {*}
 */
static void udp_client_connected(cent_udp_client_t *udpc, udp_center_t *center, udp_t *udp_instance)
{
	int ret = 0;

	if (udp_instance->simplex == SIMPLEX) {
		center->is_connected = CONNECTED;
		return;
	}

	// 在 alloc_cb 中就不去申请内存了，减少系统开销
	if (recv_buf == NULL) {
		recv_buf = (char *)Malloc(RECV_BUF_MAX_LEN);
	}

	// Start read
	if (getenv("LIDAR_NETWORK_SOCKET")) {
		uv_os_fd_t fd;
		ret = uv_fileno((uv_handle_t *)&center->uv_handle, &fd);
		if (!ret)
			ret = uv_poll_init_socket(&udpc->uv_loop, &udpc->receive_poll, fd);
		if (!ret) {
			udpc->receive_poll_initialized = 1;
			udpc->receive_poll.data = udpc;
			ret = uv_poll_start(&udpc->receive_poll, UV_READABLE, receive_managed_udp);
		}
	} else
		ret = uv_udp_recv_start(&center->uv_handle, alloc_buffer, read_cb);
	log_debug("udp client start read ret=%d", ret);

	if (ret < 0) {
		log_error("UDP: Start udp receive error: %s", uv_strerror(ret));
	} else {
		// start register
		center->is_connected = CONNECTED;
		log_info("network instance is connected");
	}
}

static void connect_center(cent_udp_client_t *udpc, udp_center_t *center)
{
	udp_t *udp_instance = (udp_t *)udpc->data;

	udp_client_connected(udpc, center, udp_instance);
	return;
}

/**
 * @description  : 心跳定时器的回调，负责进行广播
 * @param         {uv_timer_t} *timer
 * @return        {*}
 */
static void handle_hb_to_cent(uv_timer_t *timer)
{
	cent_udp_client_t *udpc = (cent_udp_client_t *)timer->data;
	int ret = RET_FAIL;

	if (udpc == NULL) {
		return;
	}
	udp_t *instance = udpc->data;
	if (instance->network_maintenance)
		instance->network_maintenance(instance);
	refresh_local_address(udpc);

	if (udpc->center.is_connected == DISCONNECTED) {
		return;
	}

	// 更新时间戳
	long long now_time = get_milliseconds_timestamp();
start:
	// 当没有上位机连接时，需要切换为广播模式
	if (udpc->center.is_handshaked == NO_REG) {
		if (udpc->center.broadcast_flag == OFF) {
			// 如果不是广播模式，切换为广播模式
			ret = set_broadcast_mode_on(&(udpc->center));
			if (ret != RET_OK) {
				return;
			}
		}

		// 如果是广播模式，发送广播包
		if (udpc->boradcast_buf_len <= 0 || udpc->boradcast_buf == NULL) {
			log_error("CENT: No heartbeat content, stop heartbeat timer");
			uv_timer_stop(timer);
			return;
		}

		pthread_mutex_lock(&(udpc->center.lock));
		udpc->center.timestamp = now_time;
		pthread_mutex_unlock(&(udpc->center.lock));

		log_debug("CENT: Try sending boradcast to center,port:%d, len:%d", udpc->center.port,
			  udpc->boradcast_buf_len);
		send_buf_to_center(&(udpc->center), udpc->boradcast_buf, udpc->boradcast_buf_len);
	} else if (udpc->center.is_handshaked == IS_REG) {
		if (udpc->center.heartbeat_cnt != udpc->center.pre_heartbeat) {
			// 已握手、心跳正常
			pthread_mutex_lock(&(udpc->center.lock));
			udpc->center.timestamp = now_time;
			udpc->center.pre_heartbeat = udpc->center.heartbeat_cnt;
			pthread_mutex_unlock(&(udpc->center.lock));
		} else {
			// 已握手、心跳超时
			if (now_time - udpc->center.timestamp >= udpc->center.handshake_timeout_msec) {
				log_info("heartbeat timeout");
				pthread_mutex_lock(&(udpc->center.lock));
				udpc->center.is_handshaked = NO_REG;
				pthread_mutex_unlock(&(udpc->center.lock));
				goto start;
			}
		}
	}
}

int set_broadcast_mode_on(udp_center_t *center)
{
	int ret = RET_FAIL;
	log_info("set broadcast mode on");
	ret = uv_udp_set_broadcast(&(center->uv_handle), ON);
	if (ret < 0) {
		log_error("uv_udp_set_broadcast fail,ret=%d", ret);
		return RET_FAIL;
	}

	ret = uv_ip4_addr("255.255.255.255", center->broadcast_port, &(center->addr));
	if (ret < 0) {
		log_error("uv_ip4_addr fail,ret=%d", ret);
		return RET_FAIL;
	}

	center->broadcast_flag = ON;
	return RET_OK;
}

int set_broadcast_mode_off(udp_center_t *center)
{
	int ret = RET_FAIL;
	log_info("set broadcast mode off");
	ret = uv_udp_set_broadcast(&(center->uv_handle), OFF);
	if (ret < 0) {
		log_error("uv_udp_set_broadcast fail,ret=%d", ret);
		return RET_FAIL;
	}

	ret = uv_ip4_addr(center->ip, center->port, &(center->addr));

	center->broadcast_flag = OFF;
	return RET_OK;
}

void update_udp_config(udp_t *udp_instance)
{
	cent_udp_client_t *udpc = &(udp_instance->udpc);

	snprintf(udpc->center.ip, sizeof(udpc->center.ip), "%s", udp_instance->ser_info.ip);
	udpc->center.port = udp_instance->ser_info.port;
	udpc->center.local_port = udp_instance->ser_info.local_port;

	udp_instance->socket_host = gethostbyname(udp_instance->ser_info.ip);
	udp_instance->client_addr.sin_family = AF_INET;
	udp_instance->client_addr.sin_addr = *((struct in_addr *)(udp_instance->socket_host->h_addr_list[0]));
	udp_instance->client_addr.sin_port = htons(udp_instance->ser_info.port);

	int ret = uv_ip4_addr(udpc->center.ip, udpc->center.port, &(udpc->center.addr));
	log_debug("update udp config,port:%d,ret:%d", udpc->center.port, ret);
	return;
}

/**
 * @description  : 绑定本地端口的函数
 * @param         {cent_udp_client_t} *udpc
 * @param         {udp_center_t} *center
 * @return        {*}
 */
static int udp_bind_local_port(cent_udp_client_t *udpc, udp_center_t *center)
{
	(void)udpc;
	int status = 0;
	struct sockaddr_in addr;

	if (center->local_port) {
		/* 保持接收单播和广播；用 IP_PKTINFO 确认实际到达的本机地址。 */
		uv_ip4_addr("0.0.0.0", center->local_port, &addr);
		status = uv_udp_bind(&(center->uv_handle), (const struct sockaddr *)&addr, 0);
		if (status < 0) {
			log_error("UDP: Bind udp port:%d error: %s", center->local_port, uv_strerror(status));
			return RET_FAIL;
		}
		if (getenv("LIDAR_NETWORK_SOCKET")) {
			uv_os_fd_t fd;
			int enabled = 1;
			if (uv_fileno((uv_handle_t *)&center->uv_handle, &fd) ||
			    setsockopt(fd, IPPROTO_IP, IP_PKTINFO, &enabled, sizeof(enabled)) ||
			    setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, "eth0", sizeof("eth0")))
				return RET_FAIL;
		}
	}
	return RET_OK;
}

static void refresh_local_address(cent_udp_client_t *client)
{
	if (!getenv("LIDAR_NETWORK_SOCKET"))
		return;
	uint8_t ip[4] = { 0 };
	get_ip_via_interface("eth0", ip);
	uint32_t address;
	memcpy(&address, ip, 4);
	udp_center_t *center = &client->center;
	if (address == center->local_address)
		return;
	pthread_mutex_lock(&center->lock);
	center->local_address = address;
	center->is_handshaked = NO_REG;
	pthread_mutex_unlock(&center->lock);
}
static void receive_managed_udp(uv_poll_t *poll, int status, int events)
{
	if (status < 0 || !(events & UV_READABLE))
		return;
	cent_udp_client_t *client = poll->data;
	udp_t *instance = client->data;
	uv_os_fd_t fd;
	if (uv_fileno((uv_handle_t *)&client->center.uv_handle, &fd))
		return;
	/* 保留每个报文的实际目标地址，广播握手不能误确认单播改址任务。 */
	for (unsigned count = 0; count < 16; ++count) {
		union {
			struct cmsghdr alignment;
			unsigned char bytes[CMSG_SPACE(sizeof(struct in_pktinfo))];
		} control;
		struct sockaddr_in from;
		struct iovec buffer = { .iov_base = recv_buf, .iov_len = RECV_BUF_MAX_LEN };
		struct msghdr message = { .msg_name = &from,
					  .msg_namelen = sizeof(from),
					  .msg_iov = &buffer,
					  .msg_iovlen = 1,
					  .msg_control = control.bytes,
					  .msg_controllen = sizeof(control.bytes) };
		ssize_t length = recvmsg(fd, &message, MSG_DONTWAIT | MSG_TRUNC);
		if (length < 0 && errno == EINTR)
			continue;
		if (length < 0)
			return;
		if (!length || length > RECV_BUF_MAX_LEN || (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)))
			continue;
		uint32_t destination = 0;
		for (struct cmsghdr *item = CMSG_FIRSTHDR(&message); item; item = CMSG_NXTHDR(&message, item)) {
			if (item->cmsg_level == IPPROTO_IP && item->cmsg_type == IP_PKTINFO &&
			    item->cmsg_len >= CMSG_LEN(sizeof(struct in_pktinfo))) {
				const struct in_pktinfo *info = (const void *)CMSG_DATA(item);
				destination = info->ipi_addr.s_addr;
				break;
			}
		}
		if (!destination)
			continue;
		refresh_local_address(client);
		if (destination != client->center.local_address && destination != INADDR_BROADCAST) {
			uint8_t bytes[4];
			uint32_t mask;
			if (get_mask_via_interface("eth0", bytes))
				continue;
			memcpy(&mask, bytes, 4);
			if ((destination & mask) != (client->center.local_address & mask) ||
			    (destination | mask) != UINT32_MAX)
				continue; /* 丢弃改址之前排队的旧单播包，继续允许本链路广播。 */
		}
		client->center.received_local_address = destination;
		instance->deal_data_to_input_link(instance, (uint8_t *)recv_buf, (int)length);
		client->center.received_local_address = 0;
	}
}

/**
 * @description  : initialzation udp instance
 * @param         {udp_t} *udp_instance
 * @param         {cent_udp_client_t} *udpc
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int init_udp_instance(udp_t *udp_instance, cent_udp_client_t *udpc)
{
	int status = -1;
	udpc->data = (void *)udp_instance;

	pthread_mutex_lock(&(udpc->center.lock));
	memset(&(udpc->center.addr), 0, sizeof(udpc->center.addr));

	snprintf(udpc->center.ip, sizeof(udpc->center.ip), "%s", udp_instance->ser_info.ip);
	udpc->center.is_handshaked = NO_REG;
	udpc->center.broadcast_flag = OFF;
	udpc->center.port = udp_instance->ser_info.port;
	udpc->center.broadcast_port = udp_instance->ser_info.broadcast_port;
	udpc->center.local_port = udp_instance->ser_info.local_port;
	udpc->center.data = udpc;
	udpc->boradcast_buf = udp_instance->boradcast_buf;
	udpc->boradcast_buf_len = udp_instance->boradcast_buf_len;
	udpc->center.uv_handle.data = &(udpc->center);
	udpc->center.timestamp = get_milliseconds_timestamp();
	pthread_mutex_unlock(&(udpc->center.lock));

	log_info("udp center ip:%s port:%d local port:%d", udpc->center.ip, udpc->center.port, udpc->center.local_port);
	if (strlen(udpc->center.ip) <= 0 || udpc->center.port > MAX_PORT || udpc->center.port <= MIN_PORT) {
		log_error("init udp instance fail,parameter error");
		return RET_FAIL;
	}

	status = uv_udp_init(&(udpc->uv_loop), &(udpc->center.uv_handle));
	if (status < 0) {
		log_error("UDP: init uv handle error: %s", uv_strerror(status));
		return RET_FAIL;
	}

	if (udp_bind_local_port(udpc, &(udpc->center)))
		return RET_FAIL;
	uv_udp_set_broadcast(&(udpc->center.uv_handle), OFF);
	uv_ip4_addr(udpc->center.ip, udpc->center.port, &(udpc->center.addr));

	return RET_OK;
}

/**
 * @description  : initialzation udp broadcast timer
 * @param         {udp_t} *udp_instance
 * @param         {cent_udp_client_t} *udpc
 * @return        {*}
 */
static void init_udp_netcheck_timer(udp_t *udp_instance, cent_udp_client_t *udpc)
{
	uv_timer_init(&(udpc->uv_loop), &udpc->hb_timer);
	udpc->hb_timer.data = (void *)udpc;
	uv_timer_start(&udpc->hb_timer, (uv_timer_cb)&handle_hb_to_cent, 0, g_timer_interval_ms);
	return;
}

static void connect_udp_center(cent_udp_client_t *udpc)
{
	connect_center(udpc, &(udpc->center));
	return;
}

/**
 * @description  : udp signal handler
 * @return        {int} signum
 */
static void udp_sighandler(int signum)
{
	log_debug("udp sighandler capture the signal %d", signum);
	atomic_store(&g_run_flag, STOPPED);

	// 停止定时器
	if (udpc->hb_timer.data != NULL) {
		uv_timer_stop(&(udpc->hb_timer));
		uv_close((uv_handle_t *)&(udpc->hb_timer), NULL);
	}

	// 关闭 UDP handle
	if (udpc->receive_poll_initialized && !uv_is_closing((uv_handle_t *)&udpc->receive_poll)) {
		uv_poll_stop(&udpc->receive_poll);
		uv_close((uv_handle_t *)&udpc->receive_poll, NULL);
	}
	if (!uv_is_closing((uv_handle_t *)&(udpc->center.uv_handle))) {
		uv_close((uv_handle_t *)&(udpc->center.uv_handle), NULL);
	}

	// 处理剩余事件
	uv_run(&(udpc->uv_loop), UV_RUN_DEFAULT);

	// 关闭事件循环
	uv_loop_close(&(udpc->uv_loop));
	Free((void **)&recv_buf);
	pthread_exit(0);
}

void *pthread_cent_udp_client(void *argv)
{
	log_debug("Thread is running on CPU %d", sched_getcpu());

	atomic_init(&g_run_flag, RUNNING);

	int ret = RET_FAIL;

	if (argv == NULL) {
		log_error("parameter is NULL");
		goto end;
	}
	thread_t *th_arg = (thread_t *)argv;

	if (th_arg->thread_arg == NULL) {
		log_error("thread arg is NULL");
		goto end;
	}
	udp_t *udp_instance = (udp_t *)(th_arg->thread_arg);
	udpc = &(udp_instance->udpc);

	if (th_arg->signal_enable == ON) {
		log_debug("listen signal no is %d", th_arg->signum);
		signal(th_arg->signum, udp_sighandler);
	}

	uv_loop_init(&(udpc->uv_loop));

	ret = init_udp_instance(udp_instance, udpc);
	if (ret != RET_OK) {
		goto close;
	}

	/*
	*  创建一个定时器，用来管理广播和单播状态
	*  数据线程是不需要的，因为它始终是单播 -> 单工是不需要的
	*  指令线程一开始是广播，然后接收到网络握手指令后，改成单播
	*  工作流程：
	*  1.当从设备上电后，将会从 65000 端口向局域网中的 55000 端口广播设备信息数据。
	*  2.接收到广播数据后，主设备向 65000 端口发送数据请求，请求数据中包含源端口。
	*  3.从设备接收到连接请求后向这个源端口回复 ACK 数据
	*  4.当用户接收到 ACK 数据，握手完成。可以开始发送心跳维持连接，并进行命令和数据的传输。
	*  注意：如果心跳超时，雷达将会重新回到广播状态。
	*/
	if (udp_instance->simplex == DUPLEX) {
		init_udp_netcheck_timer(udp_instance, udpc);
	}

	// connect udp center
	connect_udp_center(udpc);

	if (udp_instance->simplex == DUPLEX) {
		uv_run(&(udpc->uv_loop), UV_RUN_DEFAULT);
	} else {
		while (atomic_load(&g_run_flag) == RUNNING) {
			uv_run(&(udpc->uv_loop), UV_RUN_NOWAIT);
			usleep(UV_RUN_INTERVAL);
		}
	}

close:
	log_info("UDP: uv loop exit");
	uv_loop_close(&(udpc->uv_loop));
	Free((void **)&recv_buf);
end:
	pthread_exit(0);
}
