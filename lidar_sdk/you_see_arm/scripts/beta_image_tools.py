#!/usr/bin/env python3
"""Beta 镜像交付包的校验、硬件文件补齐和 JFFS2 制作工具；不连接或烧写设备。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess

CALIBRATION = ('ys_calibration.ini', 'ys_encoder_angle.csv')
REPORT = 'kit-report.json'


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1048576), b''):
            value.update(chunk)
    return value.hexdigest()


def inventory(root):
    result = {}
    for path in sorted(root.rglob('*')):
        name = path.relative_to(root).as_posix()
        if path.is_symlink():
            result[name] = {'link': os.readlink(path)}
        elif path.is_dir():
            result[name] = {'directory': True, 'mode': path.stat().st_mode & 0o777}
        elif path.is_file():
            result[name] = {'size': path.stat().st_size, 'sha256': digest(path),
                            'mode': path.stat().st_mode & 0o777}
    return result


def seed_hash(kit):
    return hashlib.sha256(json.dumps(inventory(kit/'third-partition'), sort_keys=True).encode()).hexdigest()


def elf(path):
    with path.open('rb') as stream:
        header = stream.read(20)
    if len(header) != 20 or header[:6] != b'\x7fELF\x02\x01' or struct.unpack_from('<H', header, 18)[0] != 183:
        raise ValueError('需要小端 AArch64 ELF 文件：'+str(path))


def read_report(kit):
    report = json.loads((kit/REPORT).read_text())
    if report.get('kit_format') != 1 or report.get('model') != 'CS96-beta':
        raise ValueError('不是受支持的 Beta 镜像交付目录')
    return report


def refresh(kit, report):
    motor = kit/'overlay/usr/libexec/lidar/platform/demo_i2c_motor'
    report['motor_provisioned'] = motor.is_file()
    report['calibration_provisioned'] = all((kit/'third-partition/lidar/calibration'/name).is_file()
                                            for name in CALIBRATION)
    report['missing_hardware_inputs'] = ([] if report['motor_provisioned'] else ['demo_i2c_motor'])
    if not report['calibration_provisioned']:
        report['missing_hardware_inputs'].append('本机三个标定文件，通用出厂镜像不预置其他设备的标定')
    # 清单不包含自身，避免递归摘要；符号链接单独记录目标，不解引用。
    files = inventory(kit)
    for name in (REPORT, 'SHA256SUMS'):
        files.pop(name, None)
    report['inventory'] = files
    (kit/REPORT).write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    regular = {name: entry for name, entry in files.items() if 'sha256' in entry}
    regular[REPORT] = {'sha256': digest(kit/REPORT)}
    (kit/'SHA256SUMS').write_text(''.join(entry['sha256']+'  '+name+'\n'
                                         for name, entry in sorted(regular.items())))


def provision(kit, motor=None, calibration=None):
    report = read_report(kit)
    pending = []
    if motor:
        elf(motor)
        pending.append((motor, kit/'overlay/usr/libexec/lidar/platform/demo_i2c_motor', 0o755))
    if calibration:
        for name in CALIBRATION:
            source = calibration/name
            if source.is_symlink() or not source.is_file() or not 0 < source.stat().st_size <= 1048576:
                raise ValueError('本机标定文件缺失、为空或不可信：'+str(source))
            pending.append((source, kit/'third-partition/lidar/calibration'/name, 0o600))
    if not pending:
        raise ValueError('至少提供 --motor-init 或 --calibration-dir')
    for source, target, _ in pending:
        if target.is_symlink() or (target.exists() and digest(source) != digest(target)):
            raise ValueError('拒绝覆盖已提供的硬件文件，请使用新的交付目录：'+str(target))
    for source, target, mode in pending:
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        target.chmod(mode)
    refresh(kit, report)
    print('硬件输入已补齐；如果补充了标定，请重新执行 make-jffs2。')


def make_image(kit, mkfs, erase=0x10000, size=0xff0000):
    report = read_report(kit)
    if erase < 4096 or erase & (erase-1) or size <= 0 or size % erase:
        raise ValueError('分区大小必须为擦除块的整数倍，擦除块必须为 2 的幂')
    source = kit/'third-partition'
    image = kit/'images/user.jffs2'
    image.parent.mkdir(exist_ok=True)
    temporary = image.with_suffix('.jffs2.tmp')
    # 当前内核启用 zlib/rtime，禁用镜像工具可能默认启用的 lzo。
    command = [str(mkfs), '--root='+str(source), '--output='+str(temporary),
               '--eraseblock='+str(erase), '--pagesize=4096', '--little-endian',
               '--squash-uids', '--faketime', '--disable-compressor=lzo', '--pad']
    try:
        subprocess.run(command, check=True, capture_output=True, text=True)
        used = temporary.stat().st_size
        if used > size:
            raise ValueError('第三分区镜像超出分区容量')
        # 填满整个分区，保留明确的擦除范围，避免旧 JFFS2 节点混入新文件系统。
        with temporary.open('ab') as stream:
            remaining = size-used
            chunk = b'\xff'*65536
            while remaining:
                length = min(remaining, len(chunk))
                stream.write(chunk[:length])
                remaining -= length
        temporary.replace(image)
    finally:
        temporary.unlink(missing_ok=True)
    report['jffs2'] = {'erase_size': erase, 'partition_size': size, 'page_size': 4096,
                       'used_image_bytes': used, 'padded_image_bytes': size,
                       'seed_sha256': seed_hash(kit), 'sha256': digest(image),
                       'calibration_provisioned': report['calibration_provisioned']}
    refresh(kit, report)
    print('已生成 '+str(image)+'；有效镜像 '+str(used)+' 字节，按分区容量填充至 '+str(size)+' 字节。')


def check(kit, ready=False):
    report = read_report(kit)
    actual = inventory(kit)
    actual.pop(REPORT, None)
    actual.pop('SHA256SUMS', None)
    if actual != report['inventory']:
        raise ValueError('交付文件、权限或链接发生变化，请重新生成清单')
    for line in (kit/'SHA256SUMS').read_text().splitlines():
        expected, name = line.split('  ', 1)
        if digest(kit/name) != expected:
            raise ValueError('交付摘要不匹配：'+name)
    data = kit/'third-partition/lidar'
    release = data/'current'
    if not release.is_symlink() or not release.resolve().is_relative_to((data/'releases').resolve()):
        raise ValueError('初始应用链接非法')
    manifest = json.loads((release/'manifest.json').read_text())
    if (release/'manifest.json').stat().st_size > 65536:
        raise ValueError('清单超过设备端解析上限')
    public = kit/'overlay/etc/lidar/release-public.pem'
    subprocess.run(['openssl', 'dgst', '-sha256', '-verify', str(public), '-signature',
                    str(release/'manifest.sig'), str(release/'manifest.json')], check=True, capture_output=True)
    actual_files = {name: {'size': entry['size'], 'sha256': entry['sha256']}
                    for name, entry in inventory(release.resolve()).items()
                    if name not in ('manifest.json', 'manifest.sig') and 'sha256' in entry}
    if actual_files != manifest['files']:
        raise ValueError('应用签名清单与实际文件不一致')
    for name in ('you_see', 'lidar_mgrd', 'lidar_mgrctl'):
        elf(release/'bin'/name)
    for name in ('lidar_boot', 'lidar_crypto'):
        elf(kit/'overlay/usr/libexec/lidar'/name)
    sn_provisioner = kit/'overlay/usr/libexec/lidar/lidar_sn_provision'
    elf(sn_provisioner)
    expected_sn_provisioner = report.get('sn_provisioner_sha256')
    if not expected_sn_provisioner or digest(sn_provisioner) != expected_sn_provisioner:
        raise ValueError('系统镜像中的 SN 写入程序与交付清单不匹配')
    if 'LIDAR_NETWORK_ENABLED=1' in (kit/'overlay/etc/lidar/service.conf').read_text():
        for name in ('lidar_netd', 'lidar_netctl', 'lidar_network_status.cgi'):
            elf(kit/'overlay/usr/libexec/lidar'/name)
        elf(release/'www/cgi-bin/lidar_network.cgi')
    if report.get('jffs2'):
        metadata = report['jffs2']
        image = kit/'images/user.jffs2'
        if metadata['seed_sha256'] != seed_hash(kit) or digest(image) != metadata['sha256']:
            raise ValueError('第三分区目录已变化，必须重新生成 user.jffs2')
        if image.stat().st_size != metadata['partition_size']:
            raise ValueError('JFFS2 镜像大小不匹配')
    if ready and report['missing_hardware_inputs']:
        raise ValueError('还不能进行真实采集验收：'+'；'.join(report['missing_hardware_inputs']))
    print('交付文件、应用签名、全部摘要、权限与链接检查通过。')
    for item in report['missing_hardware_inputs']:
        print('待提供：'+item)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    for command in ('check', 'provision', 'make-jffs2'):
        child = sub.add_parser(command)
        child.add_argument('kit', type=Path)
        if command == 'check':
            child.add_argument('--ready', action='store_true')
        elif command == 'provision':
            child.add_argument('--motor-init', type=Path)
            child.add_argument('--calibration-dir', type=Path)
        else:
            child.add_argument('--mkfs', type=Path, required=True)
            child.add_argument('--erase-size', type=lambda value: int(value, 0), default=0x10000)
            child.add_argument('--partition-size', type=lambda value: int(value, 0), default=0xff0000)
    args = parser.parse_args()
    kit = args.kit.resolve()
    if args.command == 'check':
        check(kit, args.ready)
    elif args.command == 'provision':
        provision(kit, args.motor_init, args.calibration_dir)
    else:
        make_image(kit, args.mkfs, args.erase_size, args.partition_size)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        raise SystemExit('操作失败：'+str(error))
