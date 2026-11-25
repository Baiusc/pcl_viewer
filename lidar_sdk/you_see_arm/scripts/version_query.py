#!/usr/bin/env python3
"""固件版本标识查询与校验工具 (指令 0x0302)

用法:
    python3 scripts/version_query.py <设备IP> [设备指令端口, 默认65000] [本机接收端口, 默认50026]

作用:
    按 V1 指令协议向板端查询"查看系统固件版本", 解析固件总包 + 各组件版本列表,
    并逐字段断言。用于验证固件版本标识功能, 也可作为上位机解析该应答的参考实现。

链路必须按此顺序, 否则收不到任何应答:
    1. 本机绑定本机接收端口(center_cmd_port)收包
    2. 先发 0x0002 握手, 把板端 center_ip/端口改写指向本机并置为已连接
       (respond_command() 只在已连接时才真正发包)
    3. 再发 0x0302 查询版本

协议依据: doc/网络通讯/固件版本标识指令-0x0302.md
板端实现: ys_business/ys_business_sys.c 的 get_sys_version()
"""

import socket
import struct
import subprocess
import sys
import time
import zlib

SOF = 0xAA
PROTO_VERSION = 0x01  # 报文头 version 字节, 必须为 1, 否则 pack() 的 switch 不匹配、不回包
CMD_TYPE_CMD = 0x00
CMD_TYPE_ACK = 0x01

CMD_SET_COMMON = 0x00
CMD_ID_HANDSHAKE = 0x02
CMD_SET_SYSTEM = 0x03
CMD_ID_GET_SYS_VERSION = 0x02
CMD_ID_GET_FPGA_VERSION_RETIRED = 0x00  # 已废弃, 应回 illegal_cmd

RET_SUCC = 0x00
RET_ILLEGAL_CMD = 0x01

# 组件 ID, 必须与 ys_business/ys_def.h 的 ys_component_id_t 一致
COMPONENTS = [
    (0, "you_see 应用(总成)"),
    (1, "ARM 驱动"),
    (2, "算法"),
    (3, "FPGA"),
    (4, "Web 服务程序"),
]

# 0x0302 应答头部: 10 个字段共 40 字节
HEAD_FMT = "<BBBBBB16s16s2x"
HEAD_SIZE = struct.calcsize(HEAD_FMT)

# 单条组件记录: 32 字节
COMP_FMT = "<BBBBBBB12sII5x"
COMP_SIZE = struct.calcsize(COMP_FMT)

assert HEAD_SIZE == 40, f"头部尺寸应为 40, 实际 {HEAD_SIZE}"
assert COMP_SIZE == 32, f"组件记录尺寸应为 32, 实际 {COMP_SIZE}"


def crc16_modbus(data: bytes) -> int:
    """与 core/ys_base.c 的 crc16_checkout() 逐位等价

    初值 0xFFFF, 反射多项式 0xA001, 结果不取反。
    注意: C 实现返回的是 (高字节<<8 | 低字节), 与常见的 Modbus CRC 表示互为字节交换,
    因此这里最后要交换高低字节 —— 否则板端 CRC 校验不通过, 帧会被直接丢弃。
    已与 C 实现实测比对: 前 7 字节包头 AA 01 02 00 00 00 00 两侧均为 0x691B。
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    # 与 C 实现的 (hi<<8 | lo) 对齐
    return ((crc & 0xFF) << 8) | (crc >> 8)


def build_frame(cmd_set: int, cmd_id: int, payload: bytes = b"", seq_num: int = 0) -> bytes:
    """按 V1 协议组帧: 包头 9 字节 + 数据段 + 尾部 CRC32 4 字节

    crc_16 覆盖包头前 7 字节(即不含 crc_16 字段本身);
    crc_32 覆盖整个数据段(cmd_set + cmd_id + payload)。
    """
    body = struct.pack("<BB", cmd_set, cmd_id) + payload
    head = struct.pack("<BBHBH", SOF, PROTO_VERSION, len(body), CMD_TYPE_CMD, seq_num)
    frame = head + struct.pack("<H", crc16_modbus(head))
    frame += body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)
    return frame


def parse_frame(data: bytes):
    """解析应答帧, 返回 (cmd_set, cmd_id, 数据段载荷)"""
    if len(data) < 15:
        raise ValueError(f"帧过短: {len(data)} 字节")
    sof, version, length, cmd_type, seq_num, crc16 = struct.unpack_from("<BBHBHH", data, 0)
    if sof != SOF:
        raise ValueError(f"起始字节错误: 0x{sof:02X}")
    if version != PROTO_VERSION:
        raise ValueError(f"协议版本错误: {version}")
    if crc16 != crc16_modbus(data[0:7]):
        raise ValueError("包头 CRC16 校验失败")
    body = data[9 : 9 + length]
    crc32 = struct.unpack_from("<I", data, 9 + length)[0]
    if crc32 != (zlib.crc32(body) & 0xFFFFFFFF):
        raise ValueError("数据段 CRC32 校验失败")
    return body[0], body[1], body[2:]


def local_ip_towards(target: str) -> str:
    """探测本机到目标 IP 的出口地址, 作为握手时告知板端的本机 IP"""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((target, 9))
        return probe.getsockname()[0]
    finally:
        probe.close()


def handshake(sock: socket.socket, dev_ip: str, dev_port: int, local_ip: str, local_port: int) -> bool:
    """发送 0x0002 握手, 让板端把应答指向本机

    载荷为 network_handshake_t: cmd_set + cmd_id + ip[4] + cmd_port + cloud_data_port + org_data_port
    板端校验: ip[0]!=0 且 ip[3]!=0 且无非 255 字节 且 ip[0]<=223, 端口范围 1..65535
    """
    ip_bytes = bytes(int(x) for x in local_ip.split("."))
    payload = struct.pack("<HHH", local_port, local_port, local_port)
    frame = build_frame(CMD_SET_COMMON, CMD_ID_HANDSHAKE, ip_bytes + payload)
    sock.sendto(frame, (dev_ip, dev_port))
    print(f"[握手] 已告知板端本机地址 {local_ip}:{local_port}")
    try:
        data, addr = sock.recvfrom(4096)
    except socket.timeout:
        print("[握手] 超时: 未收到应答。请确认设备 IP/端口正确, 且板端已启动")
        return False
    cmd_set, cmd_id, body = parse_frame(data)
    if (cmd_set, cmd_id) != (CMD_SET_COMMON, CMD_ID_HANDSHAKE):
        print(f"[握手] 应答异常: cmd_set=0x{cmd_set:02X} cmd_id=0x{cmd_id:02X}")
        return False
    print(f"[握手] 成功, 来自 {addr[0]}:{addr[1]}")
    return True


def query(sock: socket.socket, dev_ip: str, dev_port: int, cmd_id: int):
    """发送一条系统指令集指令并等待应答"""
    sock.sendto(build_frame(CMD_SET_SYSTEM, cmd_id), (dev_ip, dev_port))
    data, _ = sock.recvfrom(4096)
    return parse_frame(data)


def decode_version_reply(body: bytes) -> dict:
    """解析 0x0302 应答"""
    if len(body) < HEAD_SIZE:
        raise ValueError(f"应答载荷过短: {len(body)} 字节, 至少需要 {HEAD_SIZE}")

    (ret_code, comp_count, fw_major, fw_minor, fw_patch, _r0, product, release_id) = struct.unpack_from(
        HEAD_FMT, body, 0
    )

    components = []
    for i in range(comp_count):
        off = HEAD_SIZE + i * COMP_SIZE
        if off + COMP_SIZE > len(body):
            raise ValueError(f"组件记录 {i} 越界")
        (cid, valid, major, minor, patch, dirty, sha_len, sha_raw, btime, btime_hi) = struct.unpack_from(
            COMP_FMT, body, off
        )
        components.append(
            {
                "id": cid,
                "valid": valid,
                "version": (major, minor, patch),
                "dirty": dirty,
                "sha_len": sha_len,
                "sha": sha_raw[:sha_len].decode("ascii", "replace").rstrip("\x00"),
                "build_time": btime | (btime_hi << 32),
            }
        )

    return {
        "ret_code": ret_code,
        "component_count": comp_count,
        "fw_version": (fw_major, fw_minor, fw_patch),
        "product": product.split(b"\x00")[0].decode("ascii", "replace"),
        "release_id": release_id.split(b"\x00")[0].decode("ascii", "replace"),
        "components": components,
    }


def git_local_state():
    """采集本机仓库状态, 用于与板端上报值交叉比对(仅在仓库内运行时可用)"""
    try:
        sha = subprocess.check_output(["git", "rev-parse", "--short=7", "HEAD"], text=True).strip()
        dirty = bool(
            subprocess.check_output(
                ["git", "status", "--porcelain", "--ignore-submodules=all"], text=True
            ).strip()
        )
        return sha, dirty
    except Exception:
        return None, None


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    dev_ip = sys.argv[1]
    dev_port = int(sys.argv[2]) if len(sys.argv) > 2 else 65000
    local_port = int(sys.argv[3]) if len(sys.argv) > 3 else 50026

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", local_port))
    sock.settimeout(3.0)

    try:
        local_ip = local_ip_towards(dev_ip)
        if not handshake(sock, dev_ip, dev_port, local_ip, local_port):
            return 1

        print(f"\n[查询] 0x03/0x02 查看系统固件版本 -> {dev_ip}:{dev_port}")
        cmd_set, cmd_id, body = query(sock, dev_ip, dev_port, CMD_ID_GET_SYS_VERSION)
        if (cmd_set, cmd_id) != (CMD_SET_SYSTEM, CMD_ID_GET_SYS_VERSION):
            print(f"应答的 cmd_set/cmd_id 不符: 0x{cmd_set:02X}/0x{cmd_id:02X}")
            return 1

        info = decode_version_reply(body)

        print("\n================ 固件总包 ================")
        print(f"ret_code      : 0x{info['ret_code']:02X}")
        print(f"product       : {info['product']}")
        print(f"firmware_ver  : {info['fw_version'][0]}.{info['fw_version'][1]}.{info['fw_version'][2]}")
        print(f"release_id    : {info['release_id']}")

        print("\n================ 组件版本列表 ================")
        names = dict(COMPONENTS)
        for comp in info["components"]:
            name = names.get(comp["id"], f"未知组件({comp['id']})")
            if not comp["valid"]:
                print(f"  [{comp['id']}] {name:<22} 未接入(valid=0)")
                continue
            ver = ".".join(str(x) for x in comp["version"])
            tag = "  <<< 不可复现!" if comp["dirty"] else ""
            # FPGA 的 git_sha 字段存放的是构建 hash 的十六进制文本, 标签区分以便辨认
            sha_label = "hash" if comp["id"] == 3 else "sha "
            print(f"  [{comp['id']}] {name:<22} {ver}  {sha_label}={comp['sha'] or '(无)'}  dirty={comp['dirty']}{tag}")
            print(f"        build_time={comp['build_time']}  ({time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(comp['build_time']))})")

        print("\n================ 断言 ================")
        failures = []

        def check(cond, desc):
            print(f"  [{'PASS' if cond else 'FAIL'}] {desc}")
            if not cond:
                failures.append(desc)

        check(info["ret_code"] == RET_SUCC, "ret_code == 0x00 (成功)")
        check(info["component_count"] == len(COMPONENTS), f"component_count == {len(COMPONENTS)}")
        check(bool(info["product"]), "product 非空")

        yousee = next((c for c in info["components"] if c["id"] == 0), None)
        check(yousee is not None and yousee["valid"] == 1, "you_see 组件 valid == 1")
        if yousee:
            check(
                yousee["sha_len"] == len(yousee["sha"]),
                f"sha_len({yousee['sha_len']}) 与实际 sha 长度一致",
            )
            check(
                yousee["build_time"] > 1600000000,
                f"build_time 合理({yousee['build_time']})",
            )
            sha, dirty = git_local_state()
            if sha:
                check(yousee["sha"] == sha, f"板端 sha({yousee['sha']}) == 本机 HEAD({sha})")
                check(yousee["dirty"] == int(dirty), f"板端 dirty({yousee['dirty']}) == 本机工作区状态({int(dirty)})")

        # 算法(2) / Web 服务(4) 尚未接入, 必须保持 valid=0
        pending = [c for c in info["components"] if c["id"] in (2, 4)]
        check(
            len(pending) == 2 and all(c["valid"] == 0 for c in pending),
            "算法(2)/Web 服务(4) 尚未接入(valid == 0)",
        )

        # ARM 驱动(1) / FPGA(3) 视机型与硬件而定: 已接入时校验字段自洽, 未接入时给出提示
        arm = next((c for c in info["components"] if c["id"] == 1), None)
        if arm and arm["valid"]:
            check(arm["sha_len"] == len(arm["sha"]), f"ARM 驱动 sha_len 与实际 sha 长度一致({arm['sha_len']})")
            check(arm["dirty"] in (0, 1), f"ARM 驱动 dirty 取值合法({arm['dirty']})")
        else:
            print("  [INFO] ARM 驱动 valid=0: 该机型 liblidar.so 未提供版本接口(alpha/beta/fl6031 预期如此)")

        fpga = next((c for c in info["components"] if c["id"] == 3), None)
        if fpga and fpga["valid"]:
            check(fpga["sha_len"] in (0, 8), f"FPGA 哈希文本长度合法({fpga['sha_len']})")
            if fpga["sha_len"]:
                check(
                    all(ch in "0123456789abcdef" for ch in fpga["sha"]),
                    f"FPGA 哈希为十六进制文本({fpga['sha']})",
                )
            check(fpga["dirty"] in (0, 1), f"FPGA dirty(modify) 取值合法({fpga['dirty']})")
            check(1600000000 < fpga["build_time"] < 4102444800, f"FPGA 版本时间合理({fpga['build_time']})")
        else:
            print("  [INFO] FPGA valid=0: 版本寄存器读取失败/值非法, 或该机型未接入")

        print("\n[反例] 已废弃的 0x03/0x00 查看FPGA固件版本, 期望回 illegal_cmd")
        try:
            _, _, rbody = query(sock, dev_ip, dev_port, CMD_ID_GET_FPGA_VERSION_RETIRED)
            ret = rbody[0] if rbody else -1
            check(ret == RET_ILLEGAL_CMD, f"0x0300 回 illegal_cmd(0x01), 实际 0x{ret:02X}")
        except socket.timeout:
            check(False, "0x0300 无应答(期望回 illegal_cmd)")

        print()
        if failures:
            print(f"结果: {len(failures)} 项未通过")
            return 1
        print("结果: 全部通过")
        return 0
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
