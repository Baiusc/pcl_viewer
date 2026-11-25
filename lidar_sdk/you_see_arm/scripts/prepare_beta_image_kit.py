#!/usr/bin/env python3
"""整理 Beta 系统 overlay、第三分区完整应用及离线集成工具，不修改 Buildroot 或设备。"""
import argparse
from argparse import Namespace
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import tarfile
import tempfile

from beta_image_tools import check, digest, elf, inventory, make_image, provision, refresh
from build import git_info
from prepare_boot_bundle import create
from release_manifest import files_in, sign

ROOT = Path(__file__).resolve().parents[1]
WEB_ARCHIVE = ROOT.parent/'you_see_www/model/CS96-beta/www_CS96-beta-0.0.1.6-0dd2c2c.tar.gz'
DISABLED = ('upgrade_firmware.cgi', 'upload.cgi', 'reboot.cgi', 'set_ssh.cgi',
            'switch_raw_mode.cgi', 'switch_point_mode.cgi')
DISABLED_CGI = '''#!/bin/sh
printf 'Status: 501 Not Implemented\r\nContent-Type: application/json; charset=utf-8\r\nCache-Control: no-store\r\n\r\n'
printf '%s\n' '{"code":501,"msg":"此旧入口已停用。模式切换使用 lidar_switch.cgi；本镜像测试包尚未实现在线升级、重启或 SSH 设置 API。"}'
'''


def unpack_web(archive_path, destination):
    """只接受 www 目录下的普通文件和目录，拒绝链接、路径穿越和特殊节点。"""
    with tarfile.open(archive_path, 'r:gz') as archive:
        members = archive.getmembers()
        if sum(member.size for member in members) > 32*1024*1024 or len(members) > 2048:
            raise ValueError('Web 归档超过合理大小')
        seen = set()
        for member in members:
            path = PurePosixPath(member.name)
            if (path.is_absolute() or '..' in path.parts or not path.parts or path.parts[0] != 'www'
                    or not (member.isdir() or member.isfile()) or path.as_posix() in seen):
                raise ValueError('Web 归档包含非法条目：'+member.name)
            seen.add(path.as_posix())
        for member in members:
            target = destination/PurePosixPath(member.name)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
                target.chmod(0o755)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.extractfile(member) as source, target.open('wb') as output:
                    shutil.copyfileobj(source, output)
                executable = member.name.startswith(('www/cgi-bin/', 'www/sbin/'))
                target.chmod(0o755 if executable else 0o644)
    return destination/'www'


def recovery_web(destination, source, cgi, network_dir=None):
    for name in ('sbin', 'lib', 'config', 'cgi-bin', 'webpages'):
        (destination/name).mkdir(parents=True, exist_ok=True)
    shutil.copy2(source/'sbin/lighttpd', destination/'sbin/lighttpd')
    for name in ('libpcre.so.1', 'mod_alias.so', 'mod_cgi.so', 'mod_setenv.so',
                 'mod_staticfile.so', 'mod_indexfile.so', 'mod_dirlisting.so'):
        shutil.copy2(source/'lib'/name, destination/'lib'/name)
    for name in ('lidar_status.cgi', 'lidar_switch.cgi'):
        shutil.copy2(cgi/name, destination/'cgi-bin'/name)
    if network_dir and (network_dir/'lidar_network_status.cgi').is_file():
        shutil.copy2(network_dir/'lidar_network_status.cgi', destination/'cgi-bin/lidar_network_status.cgi')
    (destination/'config/lighttpd.conf').write_text('''server.modules = ("mod_alias", "mod_setenv", "mod_cgi", "mod_staticfile")
server.port = 80
server.document-root = "/www/webpages"
server.errorlog = "/www/log/error.log"
server.pid-file = "/www/www.pid"
server.upload-dirs = ("/www/upload")
index-file.names = ("index.html")
mimetype.assign = (".html" => "text/html; charset=utf-8")
alias.url = ("/cgi-bin/" => "/www/cgi-bin/")
setenv.add-environment = ("LIDAR_HTTP_CONFIG" => "/etc/lidar/http-bridge.json")
$HTTP["url"] =~ "^/cgi-bin/" { cgi.assign = (".cgi" => "") }
''')
    (destination/'webpages/index.html').write_text('''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>雷达启动诊断</title><h1>雷达启动诊断</h1>
<p>业务应用包未通过启动校验，或 Web 文件缺失。当前页面由系统镜像提供。</p>
<p>请检查 /run/lidar-web/verify-error 和系统日志。本页面不执行固件擦写。</p>
<p><a href="/cgi-bin/lidar_status.cgi">查询管理状态</a></p>
<p><a href="/cgi-bin/lidar_network_status.cgi">查询系统网络状态</a></p></html>
''')


def create_kit(args):
    output = args.output.resolve()
    if output.exists():
        raise ValueError('输出目录必须不存在')
    output.parent.mkdir(parents=True, exist_ok=True)
    key = args.sign_key.resolve()
    if not key.is_file() or key.is_relative_to(output):
        raise ValueError('需要交付目录之外的签名私钥')
    # 重新编译并安装统一应用，构建入口会记录全部源码来源和开发状态。
    with tempfile.TemporaryDirectory(prefix='beta-image-kit-') as temporary:
        work = Path(temporary)
        application = work/'application'
        subprocess.run(['python3', str(ROOT/'scripts/build.py'), 'package', 'beta',
                        '--output', str(application)], check=True, cwd=ROOT)
        subprocess.run(['cmake', '--build', str(ROOT/'build/beta'), '--target', 'lidar_sn_provision'],
                       check=True, cwd=ROOT)
        sn_provisioner = ROOT/'build/beta/bin/lidar_sn_provision'
        if not sn_provisioner.is_file():
            raise ValueError('缺少独立构建的出厂 SN 写入程序：'+str(sn_provisioner))
        motor_init = args.motor_init or ROOT/'build/beta/platform/bin/demo_i2c_motor'
        cgi = work/'cgi'
        subprocess.run(['make', '-C', str(ROOT.parent/'you_see_www/you_see_cgi/src/lidar_management'),
                        'CC='+str(ROOT/'cc/bin/aarch64-buildroot-linux-gnu-gcc'),
                        'BIN_DIR='+str(cgi)], check=True)
        www = unpack_web(args.web_archive, work/'web')
        for path in [www/'sbin/lighttpd', *cgi.glob('*.cgi'), *www.glob('lib/*.so*')]:
            elf(path)
        for name in DISABLED:
            (www/'cgi-bin'/name).write_text(DISABLED_CGI)
            (www/'cgi-bin'/name).chmod(0o755)
        for name in ('lidar_status.cgi', 'lidar_switch.cgi'):
            shutil.copy2(cgi/name, www/'cgi-bin'/name)
        shutil.copy2(application/'www/cgi-bin/lidar_network.cgi', www/'cgi-bin/lidar_network.cgi')
        shutil.copytree(www, application/'www', dirs_exist_ok=True)
        manifest = json.loads((application/'manifest.json').read_text())
        manifest['sources']['management_cgi'] = git_info(ROOT.parent/'you_see_www/you_see_cgi')
        manifest['files'] = files_in(application)
        manifest['image_test'] = {'validated_work_modes': [1, 2], 'web_archive_sha256': digest(args.web_archive),
                                  'disabled_legacy_cgi': list(DISABLED), 'online_update_implemented': False}
        (application/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
        if (application/'manifest.json').stat().st_size > 65536:
            raise ValueError('完整应用清单超过当前设备解析上限')
        sign(application, key)
        public = work/'public.pem'
        subprocess.run(['openssl', 'pkey', '-in', str(key), '-pubout', '-out', str(public)],
                       check=True, capture_output=True)
        create(Namespace(application=application, public_key=public, kernel_module=args.kernel_module,
                         motor_init=motor_init, sn_provisioner=sn_provisioner, cgi_dir=cgi, www_root=None, release_id=None,
                         allow_missing_motor=False, output=output))
        (output/'rootfs-overlay').rename(output/'overlay')
        (output/'flash-seed').rename(output/'third-partition')
        # 第三分区应用已包括完整 Web；系统仅保留独立诊断所需的运行时。
        shutil.rmtree(output/'overlay/www')
        recovery_web(output/'overlay/usr/share/lidar/web-recovery', www, cgi, application/'libexec/lidar')
    service = output/'overlay/etc/lidar/service.conf'
    service.write_text(service.read_text()+'\nLIDAR_WEB_SOURCE=release\nLIDAR_WEB_RUN=/run/lidar-web\n')
    boot = output/'overlay/etc/lidar/boot.json'
    settings = json.loads(boot.read_text())
    settings.pop('legacy_config_dir', None)
    boot.write_text(json.dumps(settings, ensure_ascii=False, indent=2)+'\n')
    boot.chmod(0o600)
    tools = output/'tools'
    tools.mkdir()
    (output/'integrate_lidar_rootfs.py').rename(tools/'integrate_lidar_rootfs.py')
    shutil.copy2(ROOT/'scripts/beta_image_tools.py', tools/'beta_image_tools.py')
    shutil.copy2(ROOT/'deploy/beta-image/post-build.sh', tools/'post-build.sh')
    (tools/'post-build.sh').chmod(0o755)
    shutil.copy2(ROOT/'deploy/beta-image/集成流程.md', output/'README.md')
    reference = output/'reference'
    reference.mkdir()
    shutil.copy2(args.web_archive, reference/args.web_archive.name)
    for name in ('yousee_lidar_gray.ko', 'yousee_lidar_histogram.ko', 'yousee_lidar_pointcloud.ko'):
        source = ROOT/'file/CS96-beta'/name
        if source.is_file():
            shutil.copy2(source, reference/name)
    # 原初始化归档是在调整 Web 与配置之前生成的，移除以免与最终目录混用。
    (output/'flash-seed.tar.gz').unlink()
    (output/'integration-report.json').unlink()
    module_info = re.search(rb'vermagic=([^\x00]+)', args.kernel_module.read_bytes())
    report = {'kit_format': 1, 'model': 'CS96-beta', 'release_id': (output/'third-partition/lidar/current').readlink().name,
               'created_utc': datetime.now(timezone.utc).isoformat(), 'release': False,
               'purpose': 'Beta 开发镜像集成测试；不是正式产品发布',
               'kernel_module_source': str(args.kernel_module.resolve()),
               'sn_provisioner_sha256': digest(output/'overlay/usr/libexec/lidar/lidar_sn_provision'),
               'motor_init_source': str(motor_init.resolve()), 'motor_init_sha256': digest(motor_init),
               'kernel_vermagic': module_info.group(1).decode() if module_info else None,
               'calibration_source': args.calibration_label if args.calibration_dir else None,
               'web_archive_source': str(args.web_archive.resolve()), 'web_archive_sha256': digest(args.web_archive),
               'validated_work_modes': [1, 2], 'disabled_legacy_cgi': list(DISABLED),
               'online_update_implemented': False, 'fpga_bit_provided': False,
               'partition_layout': {'boot': [0, 0x200000], 'linux': [0x200000, 0xe00000],
                                    'user': [0x1000000, 0xff0000], 'sn': [0x1ff0000, 0x10000]}}
    report['application_bytes'] = sum(entry['size'] for entry in
        inventory((output/'third-partition/lidar/current').resolve()).values() if 'size' in entry)
    report['overlay_bytes'] = sum(entry['size'] for entry in inventory(output/'overlay').values() if 'size' in entry)
    report['two_version_uncompressed_budget'] = report['application_bytes']*2 + 16*0x10000
    if report['two_version_uncompressed_budget'] > 0xff0000:
        raise ValueError('双版本及基础文件系统余量超过当前第三分区容量，请先评估容量')
    for path in (output/'overlay').rglob('*'):
        if path.is_symlink():
            continue
        path.chmod(0o755 if path.is_dir() or path.stat().st_mode & 0o111 else 0o644)
    for path in (output/'overlay/etc/lidar').iterdir():
        path.chmod(0o600)
    # 初始配置与标定目录私有；默认配置由可信启动从已签名包中首次导入。
    for name in ('config', 'calibration', 'state'):
        (output/'third-partition/lidar'/name).chmod(0o700)
    refresh(output, report)
    if args.calibration_dir:
        provision(output, calibration=args.calibration_dir)
    if args.mkfs:
        make_image(output, args.mkfs)
    check(output)
    archive_path = output.with_suffix('.tar.gz')
    if archive_path.exists():
        raise ValueError('交付归档已存在，请另选输出目录')
    with tarfile.open(archive_path, 'w:gz') as archive:
        def root_owner(info):
            info.uid = info.gid = 0
            info.uname = info.gname = 'root'
            info.mtime = 0
            return info
        archive.add(output, arcname=output.name, filter=root_owner)
    archive_path.with_name(archive_path.name+'.sha256').write_text(digest(archive_path)+'  '+archive_path.name+'\n')
    print('交付目录：'+str(output))
    print('整包：'+str(archive_path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sign-key', type=Path, required=True, help='包外开发签名私钥，不复制到交付目录')
    parser.add_argument('--web-archive', type=Path, default=WEB_ARCHIVE)
    parser.add_argument('--kernel-module', type=Path, default=ROOT/'file/CS96-beta/yousee_lidar_pointcloud.ko')
    parser.add_argument('--motor-init', type=Path, help='可选的配套电机程序；默认使用本次 Beta 构建产物')
    parser.add_argument('--calibration-dir', type=Path)
    parser.add_argument('--calibration-label', default='用户提供；须绑定目标设备', help='标定来源说明，写入交付清单')
    parser.add_argument('--mkfs', type=Path, help='主机 mkfs.jffs2，可来自 Buildroot output/host/sbin')
    args = parser.parse_args()
    if args.kernel_module.name != 'yousee_lidar_pointcloud.ko':
        parser.error('本测试目标仅验证点云 1/2，请使用 pointcloud 内核模块，其他配套另行验收')
    create_kit(args)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        raise SystemExit('整理失败：'+str(error))
