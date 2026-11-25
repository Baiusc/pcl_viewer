#!/usr/bin/env python3
"""FL6532 Range LZ4 数据接收验证脚本 (模拟上位机)

流程: UDP 收包 -> 按 frame_id/seq 重组 -> LZ4F 解压 -> 按 push 逆转置
      -> 解析像素 (row,col) -> 校验数据完整性
"""
import socket
import struct
import sys
import time
import lz4.frame
import numpy as np

LISTEN_PORT = 50001
HDR_FMT = "<BBHHBBHHH"  # sof,ver,frame_id,seq,flag,sub,data_len,payload_len,rsv
HDR_LEN = struct.calcsize(HDR_FMT)
META_FMT = "<HI6BH2x" + "HHHBBBBf" * 4
META_LEN = struct.calcsize(META_FMT)
ROI_FMT = "<HHHBBBBf"


def parse_meta(buf):
    vals = struct.unpack(META_FMT, buf[:META_LEN])
    meta = {
        "pixel_len": vals[0],
        "total_pixels": vals[1],
        "push_cnt": vals[2],
        "group_cnt": vals[3],
        "roi_cnt": vals[4],
        "max_sub_frame_cnt": vals[5],
        "peak_num": vals[6],
        "group_row_unit": vals[7],
        "max_bin_cnt": vals[8],
    }
    rois = []
    for i in range(4):
        r = struct.unpack_from(ROI_FMT, buf, 16 + i * 14)
        rois.append(
            {
                "width": r[0],
                "height": r[1],
                "push_pixels": r[2],
                "group_cnt": r[3],
                "group_start": r[4],
                "sub_frame_cnt": r[5],
                "pcl_unit": r[6],
                "range_max": r[7],
            }
        )
    meta["roi"] = rois
    return meta


def process_frame(fid, fr, stats):
    meta = fr["meta"]
    if meta is None:
        print(f"[frm={fid}] END without START, drop")
        return

    comp = b"".join(fr["pkts"][i] for i in sorted(fr["pkts"]))
    try:
        raw = lz4.frame.decompress(comp)
    except Exception as e:
        print(f"[frm={fid}] lz4 decompress failed: {e}")
        return

    pixel_len = meta["pixel_len"]
    expect = meta["total_pixels"] * pixel_len
    if len(raw) != expect:
        print(f"[frm={fid}] raw size mismatch: {len(raw)} vs {expect}")
        return

    # 按 push 逆转置 (转置布局: trans[j*count+i] -> out[i*pixel_len+j])
    out = bytearray(len(raw))
    off = 0
    roi_info = []
    for r in range(meta["roi_cnt"]):
        roi = meta["roi"][r]
        count = roi["push_pixels"]
        n = count * pixel_len
        rows_min, rows_max, cols_min, cols_max = 255, 0, 255, 0
        for g in range(roi["group_cnt"]):
            t = np.frombuffer(raw[off : off + n], dtype=np.uint8).reshape(pixel_len, count)
            block = t.T.copy()
            out[off : off + n] = block.tobytes()
            # 解析像素 row/col (偏移 56/58)
            px = block.reshape(count, pixel_len)
            rows = px[:, 56:58].copy().view(np.uint16).flatten()
            cols = px[:, 58:60].copy().view(np.uint16).flatten()
            rows_min = min(rows_min, int(rows.min()))
            rows_max = max(rows_max, int(rows.max()))
            cols_min = min(cols_min, int(cols.min()))
            cols_max = max(cols_max, int(cols.max()))
            off += n
        roi_info.append((roi, rows_min, rows_max, cols_min, cols_max))

    stats["frames"] += 1
    stats["comp"] += len(comp)
    stats["raw"] += len(raw)
    stats["pkts"] += len(fr["pkts"])

    if stats["frames"] <= 3:
        print(
            f"[frm={fid}] OK pkts={len(fr['pkts'])} sub={fr['sub']} comp={len(comp)} "
            f"raw={len(raw)} ratio={len(raw)/len(comp):.2f}:1 pixel_len={pixel_len}"
        )
        for i, (roi, r0, r1, c0, c1) in enumerate(roi_info):
            print(
                f"    roi{i}: {roi['group_cnt']}x{roi['push_pixels']}px "
                f"rows {r0}..{r1} cols {c0}..{c1} ({roi['width']}x{roi['height']})"
            )


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
    sock.bind(("0.0.0.0", LISTEN_PORT))
    sock.settimeout(3.0)
    print(f"listening udp :{LISTEN_PORT} ...")

    frames = {}
    stats = {"frames": 0, "pkts": 0, "comp": 0, "raw": 0}
    deadline = time.time() + float(sys.argv[1]) if len(sys.argv) > 1 else None

    while True:
        if deadline and time.time() > deadline:
            break
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            if deadline:
                continue
            continue

        if len(data) < HDR_LEN:
            continue
        sof, ver, fid, seq, flag, sub, dlen, plen, _ = struct.unpack_from(HDR_FMT, data)
        if sof != 0xAA or ver != 1:
            continue
        hdr_len = HDR_LEN + (META_LEN if flag == 0 else 0)
        payload = data[hdr_len : hdr_len + plen]

        fr = frames.setdefault(fid, {"pkts": {}, "meta": None, "sub": sub})
        if flag == 0:
            fr["meta"] = parse_meta(data[HDR_LEN : HDR_LEN + META_LEN])
            fr["sub"] = sub
        fr["pkts"][seq] = payload

        if flag == 2:
            expect_seq = max(fr["pkts"].keys()) + 1
            if len(fr["pkts"]) == expect_seq:
                process_frame(fid, fr, stats)
            else:
                print(f"[frm={fid}] incomplete: {len(fr['pkts'])}/{expect_seq} pkts")
            del frames[fid]

    print(
        f"\n=== total: frames={stats['frames']} pkts={stats['pkts']} "
        f"comp={stats['comp']/1e6:.1f}MB raw={stats['raw']/1e6:.1f}MB "
        f"ratio={(stats['raw']/stats['comp']) if stats['comp'] else 0:.2f}:1 ==="
    )


if __name__ == "__main__":
    main()
