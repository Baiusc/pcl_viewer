#define _GNU_SOURCE
#include "../../core/ys_udp.c"
#include <assert.h>

static uint8_t configured[4] = { 127, 0, 0, 1 };
static unsigned received_count;
static uint32_t received_destination;
void *Malloc(size_t size)
{
	return malloc(size);
}
void Free(void **pointer)
{
	free(*pointer);
	*pointer = NULL;
}
void hex_to_string(char *tag, uint8_t *buffer, int length)
{
	(void)tag;
	(void)buffer;
	(void)length;
}
void log_log(int level, const char *file, int line, const char *format, ...)
{
	(void)level;
	(void)file;
	(void)line;
	(void)format;
}
int get_ip_via_interface(const char *interface, uint8_t *out)
{
	(void)interface;
	memcpy(out, configured, 4);
	return 0;
}
int get_mask_via_interface(const char *interface, uint8_t *out)
{
	(void)interface;
	const uint8_t mask[4] = { 255, 0, 0, 0 };
	memcpy(out, mask, 4);
	return 0;
}
static int receive_packet(void *argument, uint8_t *data, int length)
{
	udp_t *instance = argument;
	assert(length == 3 && !memcmp(data, "new", 3));
	received_destination = instance->udpc.center.received_local_address;
	++received_count;
	return 0;
}
static void send_packet(int fd, unsigned port, const char *destination)
{
	struct sockaddr_in to = { .sin_family = AF_INET, .sin_port = htons((uint16_t)port) };
	assert(inet_pton(AF_INET, destination, &to.sin_addr) == 1);
	assert(sendto(fd, "new", 3, 0, (struct sockaddr *)&to, sizeof(to)) == 3);
}
static void pump(uv_loop_t *loop, unsigned expected)
{
	for (unsigned i = 0; i < 300 && received_count < expected; ++i) {
		uv_run(loop, UV_RUN_NOWAIT);
		usleep(1000);
	}
	assert(received_count == expected);
}
int main(void)
{
	assert(!setenv("LIDAR_NETWORK_SOCKET", "/tmp/udp-fixture-no-service", 1));
	udp_t instance = { .simplex = DUPLEX, .deal_data_to_input_link = receive_packet };
	cent_udp_client_t *client = &instance.udpc;
	client->data = &instance;
	client->center.data = client;
	pthread_mutex_init(&client->center.lock, NULL);
	assert(!uv_loop_init(&client->uv_loop));
	assert(!uv_udp_init(&client->uv_loop, &client->center.uv_handle));
	client->center.uv_handle.data = &client->center;
	struct sockaddr_in local;
	uv_ip4_addr("0.0.0.0", 0, &local);
	assert(!uv_udp_bind(&client->center.uv_handle, (const struct sockaddr *)&local, 0));
	int size = sizeof(local), on = 1;
	assert(!uv_udp_getsockname(&client->center.uv_handle, (struct sockaddr *)&local, &size));
	uv_os_fd_t fd;
	assert(!uv_fileno((uv_handle_t *)&client->center.uv_handle, &fd));
	assert(!setsockopt(fd, IPPROTO_IP, IP_PKTINFO, &on, sizeof(on)));
	udp_client_connected(client, &client->center, &instance);
	assert(client->center.is_connected == CONNECTED);
	int sender = socket(AF_INET, SOCK_DGRAM, 0);
	assert(sender >= 0);
	assert(!setsockopt(sender, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)));
	unsigned port = ntohs(local.sin_port);
	send_packet(sender, port, "127.0.0.1");
	pump(&client->uv_loop, 1);
	assert(received_destination == htonl(0x7f000001u));
	/* 先排入旧地址报文，再模拟改址；不能把旧包误认为经新地址到达。 */
	send_packet(sender, port, "127.0.0.1");
	configured[3] = 2;
	send_packet(sender, port, "127.0.0.2");
	pump(&client->uv_loop, 2);
	assert(received_destination == htonl(0x7f000002u));
	send_packet(sender, port, "127.255.255.255");
	pump(&client->uv_loop, 3);
	assert(received_destination == htonl(0x7fffffffu));
	uv_poll_stop(&client->receive_poll);
	uv_close((uv_handle_t *)&client->receive_poll, NULL);
	uv_close((uv_handle_t *)&client->center.uv_handle, NULL);
	uv_run(&client->uv_loop, UV_RUN_DEFAULT);
	assert(!uv_loop_close(&client->uv_loop));
	close(sender);
	Free((void **)&recv_buf);
	pthread_mutex_destroy(&client->center.lock);
	puts("真实回环 UDP 验证通过：读取目标地址、丢弃旧单播、保留广播接收");
	return 0;
}
