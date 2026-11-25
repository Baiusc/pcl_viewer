/*
 * lidar_agent.c - Link-Local + mDNS + session lock, feasibility prototype
 *
 * Targets: Buildroot (busybox) on AM62x, kernel 6.1.x, rootfs ~30MB
 * Build  : see Makefile (cross compile, static link recommended)
 * Run    : ./lidar_agent CS96000C0126020001 [ifname]
 *
 * What it does
 *   P0  derive 169.254.x.y from SN (crc16), ARP probe (RFC5227), announce, defend
 *   P1  minimal mDNS responder: A / PTR / SRV / TXT for lidar-<SN>.local
 *   P2  TCP 8080 line protocol: LOCK / RENEW / UNLOCK / STATUS / IDENTIFY / INFO
 *   +   UDP broadcast announce on 255.255.255.255:5354 every 5s (IGMP-proof fallback)
 *
 * Notes
 *   - single thread, poll() driven, no locks on the data path (nothing to block)
 *   - all timers use CLOCK_MONOTONIC (board has no RTC, wall clock is 1970)
 *   - this is a TEST TOOL, do not ship as-is
 */

#define _GNU_SOURCE
#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <linux/if_packet.h>

#ifndef ETH_P_ARP
#define ETH_P_ARP 0x0806
#endif
#ifndef ETH_P_IP
#define ETH_P_IP 0x0800
#endif

#define MDNS_PORT 5353
#define MDNS_GROUP "224.0.0.251"
#define BCAST_PORT 5354
#define CTRL_PORT 8080
#define LEASE_MS 8000
#define LEASE_RENEW_HINT (LEASE_MS / 3)
#define BCAST_INTERVAL 5000
#define TICK_MS 200
#define DNS_TTL 120
#define MAX_ATTEMPTS 8
#define BUILD_TAG "v5-setip"

static volatile sig_atomic_t g_stop = 0;

static char g_ifname[IFNAMSIZ] = "eth0";
static char g_sn[32];
static char g_ip[16]; /* primary address, the one to connect to */
static char g_ip2[16]; /* secondary address (link-local), may be empty */
static char g_ll[16]; /* link-local chosen at boot */
static char g_pending_ip[16]; /* address accepted by SETIP, applied shortly */
static uint64_t g_pending_at = 0;
static char g_host[64]; /* lidar-<SN> */
static char g_hostlocal[80]; /* lidar-<SN>.local */
static char g_svcinst[96]; /* lidar-<SN>._lidar._tcp.local */
static char g_svcname[64] = "_lidar._tcp.local";
static uint8_t g_mac[6];
static int g_ifindex;
static int g_arp_fd = -1; /* raw ARP socket, shared with the control handler */

/* ---------------- session ---------------- */
static int g_busy = 0;
static char g_token[33];
static char g_owner[16];
static uint64_t g_expiry = 0; /* monotonic ms */
static uint64_t g_last_bcast = 0;
static uint64_t g_t0 = 0;
static int g_dirty = 0; /* state changed, push an update now */

static void on_signal(int sig)
{
	(void)sig;
	g_stop = 1;
}

static uint64_t now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000);
}

static void logf_(const char *fmt, ...)
{
	va_list ap;
	uint64_t t = now_ms() - g_t0;
	fprintf(stdout, "[%5llu.%03llu] ", (unsigned long long)(t / 1000), (unsigned long long)(t % 1000));
	va_start(ap, fmt);
	vfprintf(stdout, fmt, ap);
	va_end(ap);
	fputc('\n', stdout);
	fflush(stdout);
}

/* ---------------- address derivation ---------------- */
static uint16_t crc16_ccitt(const uint8_t *d, int n)
{
	uint16_t crc = 0xFFFF;
	int i, b;
	for (i = 0; i < n; i++) {
		crc ^= (uint16_t)d[i] << 8;
		for (b = 0; b < 8; b++)
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
	}
	return crc;
}

static void derive_ip(const char *sn, int attempt, char *out, size_t outsz)
{
	uint16_t h = crc16_ccitt((const uint8_t *)sn, (int)strlen(sn));
	int hi, lo;

	h = (uint16_t)(h + attempt * 0x0101);
	hi = (h >> 8) & 0xFF;
	lo = h & 0xFF;
	if (hi < 1)
		hi = 1;
	if (hi > 254)
		hi = 254;
	if (lo < 1)
		lo = 1;
	if (lo > 254)
		lo = 254;
	snprintf(out, outsz, "169.254.%d.%d", hi, lo);
}

/* ---------------- net helpers ---------------- */
static int get_mac(const char *ifn, uint8_t mac[6])
{
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	struct ifreq ifr;
	if (fd < 0)
		return -1;
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, ifn, IFNAMSIZ - 1);
	if (ioctl(fd, SIOCGIFHWADDR, &ifr) < 0) {
		close(fd);
		return -1;
	}
	memcpy(mac, ifr.ifr_hwaddr.sa_data, 6);
	close(fd);
	return 0;
}

static int set_ipv4(const char *ifn, const char *ip, const char *mask)
{
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	struct ifreq ifr;
	struct sockaddr_in *sin;
	if (fd < 0)
		return -1;

	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, ifn, IFNAMSIZ - 1);
	sin = (struct sockaddr_in *)&ifr.ifr_addr;
	sin->sin_family = AF_INET;
	inet_pton(AF_INET, ip, &sin->sin_addr);
	if (ioctl(fd, SIOCSIFADDR, &ifr) < 0) {
		perror("SIOCSIFADDR");
		close(fd);
		return -1;
	}

	memset(&ifr.ifr_netmask, 0, sizeof(ifr.ifr_netmask));
	sin = (struct sockaddr_in *)&ifr.ifr_netmask;
	sin->sin_family = AF_INET;
	inet_pton(AF_INET, mask, &sin->sin_addr);
	if (ioctl(fd, SIOCSIFNETMASK, &ifr) < 0) {
		perror("SIOCSIFNETMASK");
		close(fd);
		return -1;
	}

	if (ioctl(fd, SIOCGIFFLAGS, &ifr) == 0) {
		ifr.ifr_flags |= IFF_UP | IFF_RUNNING;
		ioctl(fd, SIOCSIFFLAGS, &ifr);
	}
	close(fd);
	return 0;
}

/* compare the first three octets (endian-safe: xor then mask) */
static int same_prefix24(const char *a, const char *b)
{
	struct in_addr x, y;
	if (inet_pton(AF_INET, a, &x) != 1)
		return 0;
	if (inet_pton(AF_INET, b, &y) != 1)
		return 0;
	return ((x.s_addr ^ y.s_addr) & htonl(0xFFFFFF00u)) == 0;
}

/* persisted address chosen by the host; empty means "not set" */
#define CONF_PATH "/etc/lidar_ip.conf"

static int load_static_ip(char *out, size_t outsz)
{
	FILE *f = fopen(CONF_PATH, "r");
	size_t n;
	if (!f)
		return -1;
	if (!fgets(out, (int)outsz, f)) {
		fclose(f);
		out[0] = 0;
		return -1;
	}
	fclose(f);
	n = strlen(out);
	while (n && (out[n - 1] == '\n' || out[n - 1] == '\r'))
		out[--n] = 0;
	return out[0] ? 0 : -1;
}

static void save_static_ip(const char *ip)
{
	FILE *f = fopen(CONF_PATH, "w");
	if (!f) {
		logf_("SETIP: cannot write %s (%s)", CONF_PATH, strerror(errno));
		return;
	}
	fprintf(f, "%s\n", ip);
	fclose(f);
}

/* bring the link up before ARP probing (AF_PACKET fails on a DOWN iface) */
static int iface_up(const char *ifn)
{
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	struct ifreq ifr;
	if (fd < 0)
		return -1;
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, ifn, IFNAMSIZ - 1);
	if (ioctl(fd, SIOCGIFFLAGS, &ifr) < 0) {
		close(fd);
		return -1;
	}
	if (!(ifr.ifr_flags & IFF_UP)) {
		ifr.ifr_flags |= IFF_UP | IFF_RUNNING;
		if (ioctl(fd, SIOCSIFFLAGS, &ifr) < 0) {
			close(fd);
			return -1;
		}
	}
	close(fd);
	return 0;
}

/* read the current primary IPv4 of an interface; "" if none */
static int get_ipv4(const char *ifn, char *out, size_t outsz)
{
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	struct ifreq ifr;
	struct sockaddr_in *sin;

	out[0] = 0;
	if (fd < 0)
		return -1;
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, ifn, IFNAMSIZ - 1);
	if (ioctl(fd, SIOCGIFADDR, &ifr) < 0) {
		close(fd);
		return -1;
	}
	sin = (struct sockaddr_in *)&ifr.ifr_addr;
	if (sin->sin_addr.s_addr == 0) {
		close(fd);
		return -1;
	}
	inet_ntop(AF_INET, &sin->sin_addr, out, (socklen_t)outsz);
	close(fd);
	return 0;
}

/* add a secondary IPv4 via the eth0:N alias mechanism */
static int add_ipv4_alias(const char *ifn, const char *ip, const char *mask, int idx)
{
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	struct ifreq ifr;
	struct sockaddr_in *sin;
	char alias[IFNAMSIZ];

	if (fd < 0)
		return -1;
	snprintf(alias, sizeof(alias), "%s:%d", ifn, idx);

	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, alias, IFNAMSIZ - 1);
	sin = (struct sockaddr_in *)&ifr.ifr_addr;
	sin->sin_family = AF_INET;
	inet_pton(AF_INET, ip, &sin->sin_addr);
	if (ioctl(fd, SIOCSIFADDR, &ifr) < 0) {
		perror("SIOCSIFADDR(alias)");
		close(fd);
		return -1;
	}

	memset(&ifr.ifr_netmask, 0, sizeof(ifr.ifr_netmask));
	sin = (struct sockaddr_in *)&ifr.ifr_netmask;
	sin->sin_family = AF_INET;
	inet_pton(AF_INET, mask, &sin->sin_addr);
	ioctl(fd, SIOCSIFNETMASK, &ifr);
	close(fd);
	return 0;
}

/* best-effort DHCP: run busybox udhcpc, let it configure the primary address.
   returns 0 if an address was obtained */
static int run_udhcpc(int tries, int try_secs)
{
	pid_t pid;
	int status = 0, waited = 0;
	FILE *f;
	char t1[8], t2[8];

	f = fopen("/tmp/udhcpc.script", "w");
	if (!f)
		return -1;
	fputs("#!/bin/sh\n"
	      "case \"$1\" in\n"
	      "  bound|renew) ifconfig \"$interface\" \"$ip\" netmask \"$subnet\" ;;\n"
	      "esac\n",
	      f);
	fclose(f);
	chmod("/tmp/udhcpc.script", 0755);

	snprintf(t1, sizeof(t1), "%d", tries);
	snprintf(t2, sizeof(t2), "%d", try_secs);

	pid = fork();
	if (pid == 0) {
		execlp("udhcpc", "udhcpc", "-i", g_ifname, "-n", "-q", "-t", t1, "-T", t2, "-s", "/tmp/udhcpc.script",
		       (char *)NULL);
		_exit(127);
	}
	if (pid < 0)
		return -1;

	/* never let a misbehaving udhcpc block startup */
	for (waited = 0; waited < 12000; waited += 100) {
		pid_t r = waitpid(pid, &status, WNOHANG);
		if (r == pid)
			break;
		if (r < 0)
			break;
		usleep(100 * 1000);
	}
	if (waited >= 12000) {
		kill(pid, SIGKILL);
		waitpid(pid, &status, 0);
		logf_("P0: udhcpc timeout, killed");
		return -1;
	}
	if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
		return -1;
	return 0;
}

/* ---------------- ARP ---------------- */
struct arp_frame {
	struct ether_header eth;
	struct arphdr arp;
	uint8_t sha[6];
	uint8_t spa[4];
	uint8_t tha[6];
	uint8_t tpa[4];
} __attribute__((packed));

static int arp_send(int fd, const uint8_t mac[6], uint16_t op, const uint8_t spa[4], const uint8_t tpa[4])
{
	struct arp_frame f;
	struct sockaddr_ll sa;
	int i;

	memset(&f, 0, sizeof(f));
	for (i = 0; i < 6; i++)
		f.eth.ether_dhost[i] = 0xFF;
	memcpy(f.eth.ether_shost, mac, 6);
	f.eth.ether_type = htons(ETH_P_ARP);

	f.arp.ar_hrd = htons(ARPHRD_ETHER);
	f.arp.ar_pro = htons(ETH_P_IP);
	f.arp.ar_hln = 6;
	f.arp.ar_pln = 4;
	f.arp.ar_op = htons(op);

	memcpy(f.sha, mac, 6);
	memcpy(f.spa, spa, 4);
	memcpy(f.tpa, tpa, 4);

	memset(&sa, 0, sizeof(sa));
	sa.sll_family = AF_PACKET;
	sa.sll_ifindex = g_ifindex;
	sa.sll_halen = 6;
	for (i = 0; i < 6; i++)
		sa.sll_addr[i] = 0xFF;

	if (sendto(fd, &f, sizeof(f), 0, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		perror("sendto(arp)");
		return -1;
	}
	return 0;
}

/* returns 1 if the candidate address is already taken; out_mac may be NULL */
static int arp_probe(int fd, const uint8_t mac[6], const char *cand, int rounds, int gap_ms, uint8_t *out_mac)
{
	uint8_t spa0[4] = { 0, 0, 0, 0 };
	uint8_t tpa[4];
	int r;
	uint8_t buf[256];

	inet_pton(AF_INET, cand, tpa);

	for (r = 0; r < rounds; r++) {
		uint64_t deadline;
		arp_send(fd, mac, ARPOP_REQUEST, spa0, tpa);
		deadline = now_ms() + (uint64_t)gap_ms;
		for (;;) {
			struct pollfd pfd;
			int left = (int)(deadline - now_ms());
			int n;
			if (left <= 0)
				break;
			pfd.fd = fd;
			pfd.events = POLLIN;
			pfd.revents = 0;
			n = poll(&pfd, 1, left);
			if (n <= 0)
				break;
			{
				ssize_t len = recv(fd, buf, sizeof(buf), 0);
				if (len >= (ssize_t)(sizeof(struct ether_header) + sizeof(struct arphdr) + 20)) {
					struct arp_frame *f = (struct arp_frame *)buf;
					uint16_t op = ntohs(f->arp.ar_op);
					if (memcmp(f->spa, tpa, 4) == 0) {
						if (memcmp(f->sha, mac, 6) != 0) {
							logf_("ACD: conflict detected (%s already in use by "
							      "%02x:%02x:%02x:%02x:%02x:%02x, op=%u)",
							      cand, f->sha[0], f->sha[1], f->sha[2], f->sha[3],
							      f->sha[4], f->sha[5], op);
							if (out_mac)
								memcpy(out_mac, f->sha, 6);
							return 1;
						}
					}
				}
			}
		}
	}
	return 0;
}

static void arp_announce(int fd, const uint8_t mac[6], const char *ip, int times)
{
	uint8_t a[4];
	int i;
	inet_pton(AF_INET, ip, a);
	for (i = 0; i < times; i++) {
		arp_send(fd, mac, ARPOP_REQUEST, a, a);
		if (i + 1 < times)
			usleep(200 * 1000);
	}
}

/* ---------------- DNS / mDNS ---------------- */
static int put_name(uint8_t *b, int o, const char *name)
{
	const char *p = name;
	while (*p) {
		const char *dot = strchr(p, '.');
		int len = dot ? (int)(dot - p) : (int)strlen(p);
		if (len <= 0)
			break;
		if (len > 63)
			return -1;
		b[o++] = (uint8_t)len;
		memcpy(b + o, p, (size_t)len);
		o += len;
		if (!dot)
			break;
		p = dot + 1;
	}
	b[o++] = 0;
	return o;
}

static int read_name(const uint8_t *pkt, int len, int off, char *out, int outsz)
{
	int o = off, guard = 0, outpos = 0, next = -1;

	while (o >= 0 && o < len && guard++ < 64) {
		uint8_t l = pkt[o];
		if (l == 0) {
			if (next < 0)
				next = o + 1;
			break;
		}
		if ((l & 0xC0) == 0xC0) {
			int ptr;
			if (o + 1 >= len)
				return -1;
			ptr = ((l & 0x3F) << 8) | pkt[o + 1];
			if (next < 0)
				next = o + 2;
			if (ptr >= o)
				return -1; /* only backward pointers */
			o = ptr;
			continue;
		}
		if (o + 1 + l > len)
			return -1;
		if (outpos + l + 2 >= outsz)
			return -1;
		if (outpos)
			out[outpos++] = '.';
		memcpy(out + outpos, pkt + o + 1, (size_t)l);
		outpos += l;
		o += 1 + l;
	}
	out[outpos] = 0;
	return (next < 0) ? o : next;
}

static int put16(uint8_t *b, int o, uint16_t v)
{
	b[o] = (uint8_t)(v >> 8);
	b[o + 1] = (uint8_t)v;
	return o + 2;
}
static int put32(uint8_t *b, int o, uint32_t v)
{
	b[o] = (uint8_t)(v >> 24);
	b[o + 1] = (uint8_t)(v >> 16);
	b[o + 2] = (uint8_t)(v >> 8);
	b[o + 3] = (uint8_t)v;
	return o + 4;
}

static int put_record(uint8_t *b, int o, const char *name, uint16_t type, uint16_t cls, uint32_t ttl,
		      const uint8_t *rdata, int rdlen)
{
	o = put_name(b, o, name);
	if (o < 0)
		return -1;
	o = put16(b, o, type);
	o = put16(b, o, cls);
	o = put32(b, o, ttl);
	o = put16(b, o, (uint16_t)rdlen);
	memcpy(b + o, rdata, (size_t)rdlen);
	return o + rdlen;
}

/* build a response; which = bitmask 1:A 2:PTR 4:SRV 8:TXT */
static int build_mdns(uint8_t *b, uint32_t ttl, int which)
{
	int o = 12, an = 0;
	uint8_t r[512];
	int rl;
	struct in_addr addr;
	uint8_t oct[4];

	inet_pton(AF_INET, g_ip, &addr);
	memcpy(oct, &addr.s_addr, 4);

	memset(b, 0, 12);
	b[2] = 0x84;
	b[3] = 0x00; /* response + authoritative */

	if (which & 1) {
		o = put_record(b, o, g_hostlocal, 1, 0x8001, ttl, oct, 4);
		if (o < 0)
			return -1;
		an++;
		if (g_ip2[0]) {
			struct in_addr a2;
			uint8_t oct2[4];
			if (inet_pton(AF_INET, g_ip2, &a2) == 1) {
				memcpy(oct2, &a2.s_addr, 4);
				o = put_record(b, o, g_hostlocal, 1, 0x8001, ttl, oct2, 4);
				if (o < 0)
					return -1;
				an++;
			}
		}
	}
	if (which & 2) {
		rl = put_name(r, 0, g_svcinst);
		o = put_record(b, o, g_svcname, 12, 0x0001, ttl, r, rl);
		if (o < 0)
			return -1;
		an++;
	}
	if (which & 4) {
		int n = 0;
		n = put16(r, n, 0); /* priority */
		n = put16(r, n, 0); /* weight   */
		n = put16(r, n, (uint16_t)CTRL_PORT);
		n = put_name(r, n, g_hostlocal);
		o = put_record(b, o, g_svcinst, 33, 0x8001, ttl, r, n);
		if (o < 0)
			return -1;
		an++;
	}
	if (which & 8) {
		char txt[4][80];
		int n = 0;
		int k;
		snprintf(txt[0], sizeof(txt[0]), "sn=%s", g_sn);
		snprintf(txt[1], sizeof(txt[1]), "state=%s", g_busy ? "LOCKED" : "FREE");
		snprintf(txt[2], sizeof(txt[2]), "ip=%s", g_ip);
		snprintf(txt[3], sizeof(txt[3]), "proto=1");
		for (k = 0; k < 4; k++) {
			int l = (int)strlen(txt[k]);
			if (n + 1 + l >= (int)sizeof(r))
				break;
			r[n++] = (uint8_t)l;
			memcpy(r + n, txt[k], (size_t)l);
			n += l;
		}
		o = put_record(b, o, g_svcinst, 16, 0x8001, ttl, r, n);
		if (o < 0)
			return -1;
		an++;
	}

	b[6] = (uint8_t)(an >> 8);
	b[7] = (uint8_t)an;
	/* qd=0, ns=0, ar=0 */
	return o;
}

static int mdns_open(void)
{
	int s = socket(AF_INET, SOCK_DGRAM, 0);
	struct sockaddr_in sa;
	struct ip_mreqn mreq;
	int on = 1, ttl = 255;
	if (s < 0)
		return -1;

	setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
	setsockopt(s, SOL_SOCKET, SO_REUSEPORT, &on, sizeof(on));
	/* non-blocking: the drain loop in main() must never block on recvfrom,
       otherwise one drained packet stalls the whole event loop */
	fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);

	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons(MDNS_PORT);
	sa.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(s, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		perror("bind 5353");
		close(s);
		return -1;
	}

	memset(&mreq, 0, sizeof(mreq));
	inet_pton(AF_INET, MDNS_GROUP, &mreq.imr_multiaddr);
	inet_pton(AF_INET, g_ip, &mreq.imr_address);
	mreq.imr_ifindex = g_ifindex;
	if (setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
		perror("IP_ADD_MEMBERSHIP");
	if (setsockopt(s, IPPROTO_IP, IP_MULTICAST_IF, &mreq, sizeof(mreq)) < 0)
		perror("IP_MULTICAST_IF");
	setsockopt(s, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
	return s;
}

static void mdns_send(int s, const uint8_t *buf, int len)
{
	struct sockaddr_in to;
	memset(&to, 0, sizeof(to));
	to.sin_family = AF_INET;
	to.sin_port = htons(MDNS_PORT);
	inet_pton(AF_INET, MDNS_GROUP, &to.sin_addr);
	sendto(s, buf, (size_t)len, 0, (struct sockaddr *)&to, sizeof(to));
}

static void mdns_announce(int s, uint32_t ttl, int times)
{
	uint8_t buf[900];
	int len = build_mdns(buf, ttl, 0xF);
	int i;
	if (len < 0)
		return;
	for (i = 0; i < times; i++) {
		mdns_send(s, buf, len);
		if (i + 1 < times)
			usleep(1000 * 1000);
	}
}

static void mdns_handle(int s, uint8_t *pkt, int len, const struct sockaddr_in *from)
{
	uint16_t flags, qd;
	int o, q;
	int want = 0;

	if (len < 12)
		return;
	flags = (uint16_t)((pkt[2] << 8) | pkt[3]);
	if (flags & 0x8000)
		return; /* ignore responses */
	qd = (uint16_t)((pkt[4] << 8) | pkt[5]);
	o = 12;

	for (q = 0; q < qd; q++) {
		char name[128];
		uint16_t type, cls;
		o = read_name(pkt, len, o, name, sizeof(name));
		if (o < 0 || o + 4 > len)
			break;
		type = (uint16_t)((pkt[o] << 8) | pkt[o + 1]);
		cls = (uint16_t)((pkt[o + 2] << 8) | pkt[o + 3]);
		o += 4;
		if (!(cls & 0x7FFF))
			continue;

		if (strcasecmp(name, g_hostlocal) == 0 && (type == 1 || type == 255))
			want |= 1;
		if (strcasecmp(name, g_svcname) == 0 && (type == 12 || type == 255))
			want |= 2 | 4 | 8 | 1;
		if (strcasecmp(name, g_svcinst) == 0 && (type == 33 || type == 16 || type == 255))
			want |= 4 | 8 | 1;
	}

	if (want) {
		uint8_t out[900];
		int n = build_mdns(out, DNS_TTL, want);
		if (n > 0)
			mdns_send(s, out, n);
		logf_("mDNS query from %s -> answered (mask 0x%x)", inet_ntoa(from->sin_addr), want);
	}
}

/* ---------------- control protocol ---------------- */
static void token_new(char *out, size_t n)
{
	static const char *hex = "0123456789abcdef";
	unsigned char raw[16];
	FILE *f = fopen("/dev/urandom", "rb");
	size_t i;
	if (f) {
		if (fread(raw, 1, sizeof(raw), f) != sizeof(raw))
			memset(raw, 0, sizeof(raw));
		fclose(f);
	} else {
		for (i = 0; i < sizeof(raw); i++)
			raw[i] = (unsigned char)(rand() & 0xFF);
	}
	for (i = 0; i < sizeof(raw) && (i * 2 + 1) < n; i++) {
		out[i * 2] = hex[raw[i] >> 4];
		out[i * 2 + 1] = hex[raw[i] & 0xF];
	}
	out[i * 2] = 0;
}

static int session_free_expired(void)
{
	if (g_busy && now_ms() >= g_expiry) {
		logf_("LEASE: expired, release (was owner %s)", g_owner);
		g_busy = 0;
		g_token[0] = 0;
		g_owner[0] = 0;
		g_dirty = 1;
		return 1;
	}
	return 0;
}

static void ctrl_handle(int cfd, const struct sockaddr_in *peer)
{
	char line[256];
	char reply[256];
	ssize_t n;
	char peerip[16];
	struct timeval tv = { 2, 0 }; /* PowerShell clients can be slow to send */

	setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	n = recv(cfd, line, sizeof(line) - 1, 0);
	if (n <= 0) {
		close(cfd);
		return;
	}
	line[n] = 0;
	while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'))
		line[--n] = 0;

	inet_ntop(AF_INET, &peer->sin_addr, peerip, sizeof(peerip));
	logf_("CTRL from %s: %s", peerip, line);

	if (strncasecmp(line, "LOCK", 4) == 0) {
		if (!g_busy || now_ms() >= g_expiry) {
			g_busy = 1;
			g_dirty = 1;
			token_new(g_token, sizeof(g_token));
			snprintf(g_owner, sizeof(g_owner), "%s", peerip);
			g_expiry = now_ms() + LEASE_MS;
			snprintf(reply, sizeof(reply), "OK %s lease=%d renew=%d\n", g_token, LEASE_MS,
				 LEASE_RENEW_HINT);
			logf_("LOCK granted to %s token=%s", peerip, g_token);
		} else {
			snprintf(reply, sizeof(reply), "BUSY owner=%s remain=%llu\n", g_owner,
				 (unsigned long long)(g_expiry - now_ms()));
			logf_("LOCK denied, held by %s", g_owner);
		}
	} else if (strncasecmp(line, "RENEW", 5) == 0) {
		const char *t = line + 5;
		while (*t == ' ')
			t++;
		if (g_busy && strcmp(t, g_token) == 0) {
			g_expiry = now_ms() + LEASE_MS;
			snprintf(reply, sizeof(reply), "OK lease=%d\n", LEASE_MS);
		} else {
			snprintf(reply, sizeof(reply), "GONE\n");
		}
	} else if (strncasecmp(line, "UNLOCK", 6) == 0) {
		const char *t = line + 6;
		while (*t == ' ')
			t++;
		if (g_busy && strcmp(t, g_token) == 0) {
			g_busy = 0;
			g_token[0] = 0;
			g_owner[0] = 0;
			g_dirty = 1;
			snprintf(reply, sizeof(reply), "OK\n");
			logf_("UNLOCK by %s", peerip);
		} else {
			snprintf(reply, sizeof(reply), "GONE\n");
		}
	} else if (strncasecmp(line, "STATUS", 6) == 0) {
		snprintf(reply, sizeof(reply), "STATE %s OWNER %s REMAIN %llu SN %s IP %s IP2 %s PORT %d\n",
			 g_busy ? "LOCKED" : "FREE", g_busy ? g_owner : "-",
			 g_busy ? (unsigned long long)(g_expiry - now_ms()) : 0ULL, g_sn, g_ip, g_ip2[0] ? g_ip2 : "-",
			 CTRL_PORT);
	} else if (strncasecmp(line, "IDENTIFY", 8) == 0) {
		int k;
		logf_("**************** IDENTIFY ****************");
		for (k = 0; k < 3; k++)
			logf_("***   I am %s   (%s)   ***", g_sn, g_ip);
		logf_("******************************************");
		snprintf(reply, sizeof(reply), "OK SN=%s IP=%s\n", g_sn, g_ip);
	} else if (strncasecmp(line, "SETIP", 5) == 0) {
		char nip[32] = { 0 };
		char tok[64] = { 0 };
		struct in_addr ia;
		int got = sscanf(line + 5, "%31s %63s", nip, tok);

		if (got < 1 || inet_pton(AF_INET, nip, &ia) != 1) {
			snprintf(reply, sizeof(reply), "BADREQ reason=format\n");
		} else if (!g_busy || strcmp(tok, g_token) != 0) {
			snprintf(reply, sizeof(reply), "LOCKED owner=%s\n", g_busy ? g_owner : "-");
		} else if (!same_prefix24(g_ip, nip)) {
			snprintf(reply, sizeof(reply), "BADREQ reason=prefix cur=%s\n", g_ip);
		} else if (((uint8_t *)&ia.s_addr)[3] == 0 || ((uint8_t *)&ia.s_addr)[3] == 255) {
			snprintf(reply, sizeof(reply), "BADREQ reason=range\n");
		} else {
			uint8_t cmac[6];
			if (arp_probe(g_arp_fd, g_mac, nip, 3, 200, cmac)) {
				snprintf(reply, sizeof(reply), "BUSY ip=%s mac=%02x:%02x:%02x:%02x:%02x:%02x\n", nip,
					 cmac[0], cmac[1], cmac[2], cmac[3], cmac[4], cmac[5]);
				logf_("SETIP: %s busy, rejected", nip);
			} else {
				snprintf(g_pending_ip, sizeof(g_pending_ip), "%s", nip);
				g_pending_at = now_ms() + 300;
				snprintf(reply, sizeof(reply), "OK ip=%s apply_in=300ms\n", nip);
				logf_("SETIP: %s -> %s accepted", g_ip, nip);
			}
		}
	} else if (strncasecmp(line, "INFO", 4) == 0) {
		snprintf(reply, sizeof(reply), "SN %s IP %s IP2 %s MAC %02x:%02x:%02x:%02x:%02x:%02x IF %s\n", g_sn,
			 g_ip, g_ip2[0] ? g_ip2 : "-", g_mac[0], g_mac[1], g_mac[2], g_mac[3], g_mac[4], g_mac[5],
			 g_ifname);
	} else {
		snprintf(reply, sizeof(reply), "ERR unknown command\n");
	}

	send(cfd, reply, strlen(reply), 0);
	close(cfd);
}

/* ---------------- broadcast ---------------- */
static int bcast_open(void)
{
	int s = socket(AF_INET, SOCK_DGRAM, 0);
	int on = 1;
	if (s < 0)
		return -1;
	setsockopt(s, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
	/* force the packet out of our own NIC: without this, 255.255.255.255
       needs a default route and sendto() fails with ENETUNREACH */
	if (setsockopt(s, SOL_SOCKET, SO_BINDTODEVICE, g_ifname, (socklen_t)strlen(g_ifname)) < 0)
		perror("SO_BINDTODEVICE");
	return s;
}

static void bcast_send(int s)
{
	struct sockaddr_in to;
	char msg[256];
	int len;
	static int warned[2] = { 0, 0 };
	static const char *dst[2] = { "255.255.255.255", "169.254.255.255" };
	int i;

	if (s < 0)
		return;

	len = snprintf(msg, sizeof(msg),
		       "{\"sn\":\"%s\",\"ip\":\"%s\",\"ip2\":\"%s\","
		       "\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\","
		       "\"state\":\"%s\",\"port\":%d,\"lease\":%d,\"remain\":%llu}\n",
		       g_sn, g_ip, g_ip2, g_mac[0], g_mac[1], g_mac[2], g_mac[3], g_mac[4], g_mac[5],
		       g_busy ? "LOCKED" : "FREE", CTRL_PORT, LEASE_MS,
		       g_busy ? (unsigned long long)(g_expiry - now_ms()) : 0ULL);

	for (i = 0; i < 2; i++) {
		ssize_t r;
		memset(&to, 0, sizeof(to));
		to.sin_family = AF_INET;
		to.sin_port = htons(BCAST_PORT);
		inet_pton(AF_INET, dst[i], &to.sin_addr);
		r = sendto(s, msg, (size_t)len, 0, (struct sockaddr *)&to, sizeof(to));
		if (r < 0 && !warned[i]) {
			warned[i] = 1;
			logf_("BCAST: sendto %s:%d failed: %s", dst[i], BCAST_PORT, strerror(errno));
		}
	}
}

static int is_our_ip(const uint8_t *ip4)
{
	uint8_t a[4];
	if (inet_pton(AF_INET, g_ip, a) == 1 && memcmp(ip4, a, 4) == 0)
		return 1;
	if (g_ip2[0] && inet_pton(AF_INET, g_ip2, a) == 1 && memcmp(ip4, a, 4) == 0)
		return 1;
	return 0;
}

/* ---------------- main ---------------- */
int main(int argc, char **argv)
{
	int arp_fd = -1, mdns_fd = -1, listen_fd = -1, bc_fd = -1;
	int attempt = 0, announced = 0;
	uint8_t buf[1500];
	char ll[16] = "";
	struct sigaction sa;

	if (argc < 2) {
		fprintf(stderr, "usage: %s <SN> [ifname]\n", argv[0]);
		return 1;
	}
	snprintf(g_sn, sizeof(g_sn), "%s", argv[1]);
	if (argc >= 3)
		snprintf(g_ifname, sizeof(g_ifname), "%s", argv[2]);

	g_t0 = now_ms();

	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = on_signal;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	snprintf(g_host, sizeof(g_host), "lidar-%s", g_sn);
	snprintf(g_hostlocal, sizeof(g_hostlocal), "%s.local", g_host);
	snprintf(g_svcinst, sizeof(g_svcinst), "%s.%s", g_host, g_svcname);

	if (get_mac(g_ifname, g_mac) < 0) {
		fprintf(stderr, "cannot read MAC of %s\n", g_ifname);
		return 1;
	}
	g_ifindex = (int)if_nametoindex(g_ifname);
	if (!g_ifindex) {
		fprintf(stderr, "no such interface %s\n", g_ifname);
		return 1;
	}

	logf_("lidar_agent %s  SN=%s  host=%s  if=%s  mac=%02x:%02x:%02x:%02x:%02x:%02x", BUILD_TAG, g_sn, g_hostlocal,
	      g_ifname, g_mac[0], g_mac[1], g_mac[2], g_mac[3], g_mac[4], g_mac[5]);

	/* --- P0: raw socket + probe --- */
	if (iface_up(g_ifname) < 0) {
		perror("iface_up");
		return 1;
	}
	arp_fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ARP));
	if (arp_fd < 0) {
		perror("socket(AF_PACKET)");
		return 1;
	}
	g_arp_fd = arp_fd;
	fcntl(arp_fd, F_SETFL, fcntl(arp_fd, F_GETFL, 0) | O_NONBLOCK);

	for (attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
		derive_ip(g_sn, attempt, ll, sizeof(ll));
		logf_("P0: link-local candidate %s (attempt %d)", ll, attempt);
		if (!arp_probe(arp_fd, g_mac, ll, 3, 200, NULL))
			break;
		logf_("P0: %s is taken, falling back", ll);
	}
	if (attempt >= MAX_ATTEMPTS) {
		fprintf(stderr, "no free link-local address after %d attempts\n", MAX_ATTEMPTS);
		return 1;
	}

	/* priority: saved static (set by host) > DHCP > link-local only */
	{
		int net_ok = 0;
		char saved[32] = "";

		if (load_static_ip(saved, sizeof(saved)) == 0) {
			logf_("P0: saved static candidate %s", saved);
			if (!arp_probe(arp_fd, g_mac, saved, 3, 200, NULL)) {
				if (set_ipv4(g_ifname, saved, "255.255.255.0") == 0) {
					snprintf(g_ip, sizeof(g_ip), "%s", saved);
					net_ok = 1;
					logf_("P0: using saved static %s", g_ip);
				}
			} else {
				logf_("P0: saved static %s is taken, ignored", saved);
			}
		}

		if (!net_ok && run_udhcpc(3, 2) == 0 && get_ipv4(g_ifname, g_ip, sizeof(g_ip)) == 0) {
			net_ok = 1;
			logf_("P0: DHCP address %s", g_ip);
		}

		if (net_ok) {
			if (add_ipv4_alias(g_ifname, ll, "255.255.0.0", 0) == 0) {
				snprintf(g_ip2, sizeof(g_ip2), "%s", ll);
				arp_announce(arp_fd, g_mac, ll, 2);
				logf_("P0: link-local %s kept as secondary", g_ip2);
			}
		} else {
			snprintf(g_ip, sizeof(g_ip), "%s", ll);
			if (set_ipv4(g_ifname, g_ip, "255.255.0.0") < 0)
				return 1;
			arp_announce(arp_fd, g_mac, g_ip, 2);
			logf_("P0: no DHCP, link-local only: %s/16", g_ip);
		}
		snprintf(g_ll, sizeof(g_ll), "%s", ll);
	}
	logf_("P0: ready  ip=%s  secondary=%s", g_ip, g_ip2[0] ? g_ip2 : "-");

	/* --- P1: mDNS --- */
	mdns_fd = mdns_open();
	if (mdns_fd >= 0) {
		mdns_announce(mdns_fd, DNS_TTL, 2);
		logf_("P1: mDNS announced %s", g_hostlocal);
	}

	/* --- P2: control socket --- */
	listen_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (listen_fd >= 0) {
		struct sockaddr_in sa4;
		int on = 1;
		setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
		memset(&sa4, 0, sizeof(sa4));
		sa4.sin_family = AF_INET;
		sa4.sin_port = htons(CTRL_PORT);
		sa4.sin_addr.s_addr = htonl(INADDR_ANY);
		if (bind(listen_fd, (struct sockaddr *)&sa4, sizeof(sa4)) < 0) {
			perror("bind 8080");
			close(listen_fd);
			listen_fd = -1;
		} else {
			listen(listen_fd, 4);
			fcntl(listen_fd, F_SETFL, fcntl(listen_fd, F_GETFL, 0) | O_NONBLOCK);
			logf_("P2: control on tcp/%d", CTRL_PORT);
		}
	}

	bc_fd = bcast_open();
	if (bc_fd >= 0)
		logf_("BCAST: udp/%d every %dms on %s", BCAST_PORT, BCAST_INTERVAL, g_ifname);

	bcast_send(bc_fd);
	g_last_bcast = now_ms();

	/* --- event loop --- */
	while (!g_stop) {
		struct pollfd pfd[3];
		int nf = 0, r;
		int arp_i = -1, mdns_i = -1, lst_i = -1;

		pfd[nf].fd = arp_fd;
		pfd[nf].events = POLLIN;
		pfd[nf].revents = 0;
		arp_i = nf++;
		if (mdns_fd >= 0) {
			pfd[nf].fd = mdns_fd;
			pfd[nf].events = POLLIN;
			pfd[nf].revents = 0;
			mdns_i = nf++;
		}
		if (listen_fd >= 0) {
			pfd[nf].fd = listen_fd;
			pfd[nf].events = POLLIN;
			pfd[nf].revents = 0;
			lst_i = nf++;
		}

		r = poll(pfd, (nfds_t)nf, TICK_MS);

		if (r > 0) {
			if (arp_i >= 0 && (pfd[arp_i].revents & POLLIN)) {
				/* drain, watch for conflicts on our address */
				for (;;) {
					ssize_t len = recv(arp_fd, buf, sizeof(buf), 0);
					if (len <= 0)
						break;
					if (len >=
					    (ssize_t)(sizeof(struct ether_header) + sizeof(struct arphdr) + 20)) {
						struct arp_frame *f = (struct arp_frame *)buf;
						if (is_our_ip(f->spa) && memcmp(f->sha, g_mac, 6) != 0) {
							char cip[16];
							inet_ntop(AF_INET, f->spa, cip, sizeof(cip));
							logf_("ACD: %s claimed by %02x:%02x:%02x:%02x:%02x:%02x (defend)",
							      cip, f->sha[0], f->sha[1], f->sha[2], f->sha[3],
							      f->sha[4], f->sha[5]);
							arp_announce(arp_fd, g_mac, cip, 1);
						}
					}
				}
			}
			if (mdns_i >= 0 && (pfd[mdns_i].revents & POLLIN)) {
				for (;;) {
					struct sockaddr_in from;
					socklen_t fl = sizeof(from);
					ssize_t len =
						recvfrom(mdns_fd, buf, sizeof(buf), 0, (struct sockaddr *)&from, &fl);
					if (len <= 0)
						break;
					mdns_handle(mdns_fd, buf, (int)len, &from);
				}
			}
			if (lst_i >= 0 && (pfd[lst_i].revents & POLLIN)) {
				for (;;) {
					struct sockaddr_in peer;
					socklen_t pl = sizeof(peer);
					int cfd = accept(listen_fd, (struct sockaddr *)&peer, &pl);
					if (cfd < 0)
						break;
					ctrl_handle(cfd, &peer);
				}
			}
		}

		if (!announced && now_ms() - g_t0 > 1500) {
			announced = 1;
			if (mdns_fd >= 0)
				mdns_announce(mdns_fd, DNS_TTL, 1);
		}

		if (g_dirty) {
			g_dirty = 0;
			if (mdns_fd >= 0)
				mdns_announce(mdns_fd, DNS_TTL, 1);
			bcast_send(bc_fd);
			g_last_bcast = now_ms();
		} else if (now_ms() - g_last_bcast >= BCAST_INTERVAL) {
			bcast_send(bc_fd);
			g_last_bcast = now_ms();
		}

		/* lease watchdog; if it fired, g_dirty makes the next tick push an update */
		session_free_expired();

		/* deferred apply of a host-assigned address (reply must reach the client first) */
		if (g_pending_ip[0] && now_ms() >= g_pending_at) {
			char old[16];
			snprintf(old, sizeof(old), "%s", g_ip);
			if (mdns_fd >= 0)
				mdns_announce(mdns_fd, 0, 1); /* goodbye: old addr */
			if (set_ipv4(g_ifname, g_pending_ip, "255.255.255.0") == 0) {
				snprintf(g_ip, sizeof(g_ip), "%s", g_pending_ip);
				if (!g_ip2[0] && g_ll[0] && add_ipv4_alias(g_ifname, g_ll, "255.255.0.0", 0) == 0)
					snprintf(g_ip2, sizeof(g_ip2), "%s", g_ll);
				arp_announce(arp_fd, g_mac, g_ip, 2);
				save_static_ip(g_ip);
				if (mdns_fd >= 0)
					mdns_announce(mdns_fd, DNS_TTL, 1);
				bcast_send(bc_fd);
				g_last_bcast = now_ms();
				logf_("SETIP: applied %s -> %s (saved)", old, g_ip);
			} else {
				logf_("SETIP: failed to apply %s", g_pending_ip);
			}
			g_pending_ip[0] = 0;
		}

		{
			static uint64_t next_alive = 0;
			if (now_ms() >= next_alive) {
				next_alive = now_ms() + 30000;
				logf_("ALIVE ip=%s state=%s", g_ip, g_busy ? "LOCKED" : "FREE");
			}
		}
	}

	logf_("shutting down, sending mDNS goodbye");
	if (mdns_fd >= 0)
		mdns_announce(mdns_fd, 0, 1);

	if (arp_fd >= 0)
		close(arp_fd);
	if (mdns_fd >= 0)
		close(mdns_fd);
	if (listen_fd >= 0)
		close(listen_fd);
	if (bc_fd >= 0)
		close(bc_fd);
	return 0;
}
