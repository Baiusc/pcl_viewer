#!/usr/bin/env python3
"""生成离线 Beta 启动集成包，不连接设备、不刷写分区。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import tarfile

from release_manifest import verify

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'deploy/runit'


def module_note_hash(path):
    data = path.read_bytes()
    if data[:6] != b'\x7fELF\x02\x01':
        raise ValueError('Beta 内核模块必须是小端 ELF64')
    offset = struct.unpack_from('<Q', data, 40)[0]
    size, count, names_index = struct.unpack_from('<HHH', data, 58)
    if size != 64 or not count or names_index >= count or offset + size * count > len(data):
        raise ValueError('模块 ELF 节表不完整')
    sections = [struct.unpack_from('<IIQQQQIIQQ', data, offset + i*size) for i in range(count)]
    names = sections[names_index]
    names_data = data[names[4]:names[4]+names[5]]
    for section in sections:
        name = names_data[section[0]:].split(b'\0', 1)[0]
        if name == b'.note.gnu.build-id':
            note = data[section[4]:section[4]+section[5]]
            if not note or len(note) != section[5]:
                raise ValueError('模块构建标识不完整')
            return hashlib.sha256(note).hexdigest()
    raise ValueError('模块缺少 .note.gnu.build-id，无法绑定已验证的配套')


def normalize(directory):
    for path in directory.rglob('*'):
        if path.is_symlink():
            continue
        path.chmod(0o755 if path.is_dir() or path.stat().st_mode & 0o111 else 0o644)


def elf_machine(path):
    with path.open('rb') as stream:
        header=stream.read(20)
    if len(header)!=20 or header[:6]!=b'\x7fELF\x02\x01':
        raise ValueError('目标程序必须是小端 ELF64：'+str(path))
    return struct.unpack_from('<H',header,18)[0]


def create(args, target_machine=183):
    application = args.application.resolve()
    public = args.public_key.resolve()
    manifest = verify(application, public)
    if manifest.get('model') != 'CS96-beta':
        raise ValueError('本阶段启动集成只开放 CS96-beta')
    for name in ('lidar_mgrd', 'lidar_mgrctl', 'lidar_crypto', 'lidar_boot', 'you_see'):
        if not (application/'bin'/name).is_file():
            raise ValueError('缺少构建产物：'+name)
    if not (application/'lib/liblidar.so').is_file():
        raise ValueError('缺少统一 liblidar.so')
    module_hash = module_note_hash(args.kernel_module)
    if args.motor_init is None and not getattr(args, 'allow_missing_motor', False):
        raise ValueError('缺少电机初始化程序；仅整理待补齐交付物时可显式允许缺失')
    output = args.output.resolve()
    if output.exists():
        raise ValueError('输出目录必须不存在')
    identity = args.release_id or 'beta-' + hashlib.sha256((application/'manifest.json').read_bytes()).hexdigest()[:16]
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]{0,63}', identity):
        raise ValueError('非法发布目录名称')
    for path in (public, args.kernel_module, args.sn_provisioner, *([args.motor_init] if args.motor_init else [])):
        if not path.is_file():
            raise ValueError('输入文件不存在：'+str(path))
    for name in ('lidar_status.cgi', 'lidar_switch.cgi'):
        if not (args.cgi_dir/name).is_file():
            raise ValueError('缺少已构建 HTTP CGI：'+name)
    network_dir = application/'libexec/lidar'
    network_names = ('lidar_netd', 'lidar_netctl', 'lidar_network_status.cgi')
    network_enabled = all((network_dir/name).is_file() for name in network_names)
    if any((network_dir/name).exists() for name in network_names) and not network_enabled:
        raise ValueError('网络系统组件不完整，拒绝生成部分启用的启动包')
    if network_enabled and not (application/'www/cgi-bin/lidar_network.cgi').is_file():
        raise ValueError('缺少配套网络业务 CGI')
    assets=[application/'bin'/name for name in ('you_see','lidar_mgrd','lidar_mgrctl','lidar_boot','lidar_crypto')]
    assets.append(args.sn_provisioner)
    assets += [application/'lib/liblidar.so', args.kernel_module]
    if network_enabled:
        assets += [network_dir/name for name in network_names]
        assets.append(application/'www/cgi-bin/lidar_network.cgi')
    if args.motor_init:
        assets.append(args.motor_init)
    assets += [args.cgi_dir/name for name in ('lidar_status.cgi','lidar_switch.cgi')]
    if args.www_root:
        assets.append(args.www_root/'sbin/lighttpd')
    for path in assets:
        if elf_machine(path)!=target_machine:
            raise ValueError('程序架构与目标机型不一致：'+str(path))
    output.mkdir(parents=True)
    overlay = output/'rootfs-overlay'
    shutil.copytree(SOURCE/'rootfs', overlay)
    if not network_enabled:
        shutil.rmtree(overlay/'etc/avahi')
    binary_dir = overlay/'usr/libexec/lidar'
    for name in ('lidar_boot', 'lidar_crypto'):
        shutil.copy2(application/'bin'/name, binary_dir/name)
    shutil.copy2(args.sn_provisioner, binary_dir/'lidar_sn_provision')
    if network_enabled:
        for name in network_names:
            shutil.copy2(network_dir/name, binary_dir/name)
    shutil.copy2(public, overlay/'etc/lidar/release-public.pem')
    shutil.copy2(SOURCE/'beta.boot.json', overlay/'etc/lidar/boot.json')
    if network_enabled:
        boot_path = overlay/'etc/lidar/boot.json'
        boot_settings = json.loads(boot_path.read_text())
        boot_settings['network_socket'] = '/run/lidar-network/control.sock'
        boot_path.write_text(json.dumps(boot_settings, ensure_ascii=False, indent=2)+'\n')
        network_path = overlay/'etc/lidar/network.json'
        network_settings = json.loads(network_path.read_text())
        network_settings['legacy_interfaces'] = '/etc/lidar/legacy-network.interfaces'
        network_path.write_text(json.dumps(network_settings, ensure_ascii=False, indent=2)+'\n')
    (overlay/'etc/lidar/http-bridge.json').write_text(
        json.dumps(dict(socket_path='/run/lidar/control.sock', timeout_ms=10000))+'\n')
    platform = binary_dir/'platform'
    platform.mkdir()
    shutil.copy2(args.kernel_module, platform/'yousee_lidar.ko')
    if args.motor_init:
        shutil.copy2(args.motor_init, platform/'demo_i2c_motor')
        (platform/'demo_i2c_motor').chmod(0o755)
    config = overlay/'etc/lidar/service.conf'
    config.write_text(config.read_text().replace('LIDAR_MODULE_ID=UNPROVISIONED', 'LIDAR_MODULE_ID='+module_hash))
    if network_enabled:
        config.write_text(config.read_text().replace('LIDAR_NETWORK_ENABLED=0', 'LIDAR_NETWORK_ENABLED=1'))
    if args.www_root:
        shutil.copytree(args.www_root, overlay/'www', symlinks=True)
    cgi = overlay/'www/cgi-bin'
    cgi.mkdir(parents=True, exist_ok=True)
    for name in ('lidar_status.cgi', 'lidar_switch.cgi'):
        shutil.copy2(args.cgi_dir/name, cgi/name)
        (cgi/name).chmod(0o755)
    if network_enabled:
        shutil.copy2(application/'www/cgi-bin/lidar_network.cgi', cgi/'lidar_network.cgi')
    # 旧模式脚本会绕过管理服务，初始集成不再提供这些后备路径。
    for name in ('switch_raw_mode.cgi', 'switch_point_mode.cgi'):
        (cgi/name).unlink(missing_ok=True)
    for name in ('ys_calibration.ini', 'ys_encoder_angle.csv'):
        (overlay/'etc'/name).symlink_to('/run/lidar-data/lidar/calibration/'+name)
    normalize(overlay)
    for path in (overlay/'etc/lidar').iterdir():
        path.chmod(0o600)
    for folder in (overlay/'etc/init.d', binary_dir, overlay/'usr/share/lidar/runit'):
        for path in folder.rglob('*'):
            if path.is_file() and (path.name in ('run', 'finish', 'check') or path.name.startswith(('S', 'lidar-'))):
                path.chmod(0o755)
    flash = output/'flash-seed/lidar'
    release = flash/'releases'/identity
    release.parent.mkdir(parents=True)
    release.parent.chmod(0o755)
    shutil.copytree(application, release)
    normalize(release)
    (flash/'current').symlink_to('releases/'+identity)
    for name in ('config', 'state', 'calibration'):
        (flash/name).mkdir(mode=0o700)
    flash.chmod(0o700)
    shutil.copy2(SOURCE/'inittab.fragment', output/'inittab.fragment')
    shutil.copy2(ROOT/'scripts/integrate_lidar_rootfs.py', output/'integrate_lidar_rootfs.py')
    if network_enabled:
        shutil.copytree(ROOT/'deploy/network', output/'network-config-fragments')
    report = dict(model='CS96-beta',release_id=identity,release=manifest.get('release', False),
                  version=manifest.get('version'),validated_work_modes=[1,2],
                  module_note_sha256=module_hash,hardware_scope='固定点云布局；不执行 DMA 停止或复位',
                  motor_provisioned=bool(args.motor_init),
                  sn_provisioner_sha256=hashlib.sha256(args.sn_provisioner.read_bytes()).hexdigest(),
                  network_enabled=network_enabled,
                  source_package_sha256=hashlib.sha256((application/'manifest.json').read_bytes()).hexdigest(),
                  www_root_provided=bool(args.www_root),
                  files={str(p.relative_to(overlay)):p.stat().st_size for p in overlay.rglob('*') if p.is_file() and not p.is_symlink()})
    (output/'integration-report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    with tarfile.open(output/'flash-seed.tar.gz','w:gz') as archive:
        def root_owner(info):
            info.uid=info.gid=0
            info.uname=info.gname='root'
            return info
        archive.add(flash,arcname='lidar',filter=root_owner)
    print('离线启动集成包：'+str(output))
    print('请将 rootfs-overlay 纳入新镜像；flash-seed 仅用于首次受控初始化，不覆盖已有部署。')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--application',type=Path,required=True,help='已经签名的统一 Beta 应用包')
    parser.add_argument('--public-key',type=Path,required=True,help='包外信任公钥，不接收私钥')
    parser.add_argument('--kernel-module',type=Path,required=True,help='系统同事确认的固定点云内核模块')
    parser.add_argument('--sn-provisioner',type=Path,required=True,help='单独构建的出厂 SN 写入程序，不属于第三分区应用包')
    parser.add_argument('--motor-init',type=Path,help='Beta 电机编译产物，例如 build/beta/platform/bin/demo_i2c_motor')
    parser.add_argument('--allow-missing-motor', action='store_true', help='允许整理待补齐目录；缺失时离线镜像合入仍会拒绝')
    parser.add_argument('--cgi-dir',type=Path,required=True,help='ARM64 两个新 CGI 所在目录')
    parser.add_argument('--www-root',type=Path,help='可选：完整目标平台 /www 内容；省略时由系统镜像提供')
    parser.add_argument('--release-id',help='可选的发布目录名称，默认由 manifest 摘要产生')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    try:
        create(args)
    except (ValueError,OSError,KeyError,struct.error,subprocess.CalledProcessError) as error:
        parser.exit(1,'生成失败：'+str(error)+'\n')


if __name__=='__main__':
    main()
