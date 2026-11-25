#!/usr/bin/env python3
"""将离线 overlay 合入 Buildroot TARGET_DIR；保留系统 inittab 和原启动入口备份。"""
import argparse
import json
from pathlib import Path
import re
import shutil


def avahi_dbus_capability(target, header=None):
    if header is None:
        candidates=list((target.parent/'build').glob('avahi-*/config.h'))
        if len(candidates) != 1:
            raise ValueError('无法唯一定位 Buildroot Avahi config.h，请使用 --avahi-config-header 指定')
        header=candidates[0]
    if not header.is_file():
        raise ValueError('Avahi 编译配置不存在：'+str(header))
    text=header.read_text()
    if not re.search(r'^#define\s+PACKAGE_NAME\s+"Avahi"\s*$',text,re.MULTILINE|re.IGNORECASE):
        raise ValueError('提供的 config.h 不是 Avahi 编译配置：'+str(header))
    return bool(re.search(r'^#define\s+HAVE_DBUS\s+1\s*$',text,re.MULTILINE))


def network_migration(target, overlay, avahi_config_header=None):
    """先验证迁移条件，保留其他接口；在离线镜像内收敛 eth0 的配置所有权。"""
    service = overlay/'etc/lidar/service.conf'
    if not service.is_file() or not re.search(r'^LIDAR_NETWORK_ENABLED=1$', service.read_text(), re.MULTILINE):
        return None
    config=overlay/'etc/lidar/network.json'
    settings=json.loads(config.read_text())
    if type(settings.get('avahi_dbus_supported')) is not bool:
        raise ValueError('系统网络配置缺少布尔类型 avahi_dbus_supported')
    dbus_supported=avahi_dbus_capability(target, avahi_config_header)
    for name in ('usr/libexec/lidar/lidar_netd', 'usr/libexec/lidar/lidar_netctl',
                 'usr/libexec/lidar/lidar_network_status.cgi'):
        if not (overlay/name).is_file():
            raise ValueError('缺少网络系统组件：'+name)
    for name in ('sbin/udhcpc', 'usr/sbin/avahi-daemon'):
        source = target/name
        if not source.is_file() or not source.resolve().is_relative_to(target) or not source.stat().st_mode & 0o111:
            raise ValueError('有效 Buildroot 镜像缺少网络依赖：'+name)
    for script in (target/'etc/init.d').glob('S*'):
        if not script.is_file() or 'avahi' in script.name:
            continue
        text=script.read_text(errors='replace')
        if re.search(r'\b(lidar_agent|avahi-autoipd|zcip)\b|\bifconfig\s+eth0\b|\bip\s+addr\s+.*\beth0\b',text):
            raise ValueError('存在绕过统一服务的 eth0 配置入口，请先迁移：'+str(script))
    interfaces = target/'etc/network/interfaces'
    if interfaces.exists() and (not interfaces.is_file() or not interfaces.resolve().is_relative_to(target)):
        raise ValueError('原网络配置不是镜像内的普通文件')
    original = interfaces.read_text() if interfaces.exists() else ''
    output = []
    skipping = False
    for line in original.splitlines(keepends=True):
        text = line.strip()
        if not text or text.startswith('#'):
            if not skipping:
                output.append(line)
            continue
        words = text.split()
        if words[0] in ('source', 'source-directory', 'mapping'):
            raise ValueError('原网络配置包含外部片段或 mapping，需先明确 eth0 配置再启用独立网络服务')
        if words[0] == 'iface':
            if len(words) < 4:
                raise ValueError('原 interfaces 的 iface 定义不完整')
            skipping = words[1:3] == ['eth0', 'inet']
            if skipping and words[3] not in ('static', 'dhcp', 'manual'):
                raise ValueError('原 eth0 IPv4 模式无法自动迁移')
            if not skipping:
                output.append(line)
        elif words[0] == 'auto' or words[0].startswith('allow-'):
            skipping = False
            kept = [word for word in words[1:] if word != 'eth0']
            if kept:
                output.append(words[0]+' '+' '.join(kept)+'\n')
        elif not skipping:
            output.append(line)
    return original, ''.join(output), dbus_supported


def integrate(target, overlay, replace_legacy=False, avahi_config_header=None):
    target=target.resolve()
    overlay=overlay.resolve()
    if target == Path('/') or not (target/'etc/inittab').is_file():
        raise ValueError('必须指定离线 Buildroot TARGET_DIR，禁止操作正在运行的系统根目录')
    if not (target/'etc/inittab').resolve().is_relative_to(target):
        raise ValueError('inittab 链接越出 TARGET_DIR')
    for source in overlay.rglob('*'):
        relative=source.relative_to(overlay)
        for parent in relative.parents:
            destination=target/parent
            if destination.is_symlink() and not destination.resolve().is_relative_to(target):
                raise ValueError('目标父目录链接越出 TARGET_DIR：'+str(destination))
    network = network_migration(target, overlay, avahi_config_header)
    legacy=target/'etc/init.d/S99user'
    if legacy.exists() and not replace_legacy:
        raise ValueError('发现旧 S99user；核对后显式使用 --replace-legacy-startup 保留备份并移出启动目录')
    for script in (target/'etc/init.d').glob('S*'):
        if script.name == 'S99user' or not script.is_file():
            continue
        text=script.read_text(errors='replace')
        if '/root/you_see' in text or 'app_startup.sh' in text:
            raise ValueError('发现其他旧采集启动入口，请先处理：'+str(script))
        if script.name != 'S90lidar_web' and 'lighttpd' in script.name.lower():
            raise ValueError('已有 Web 启动入口，请先与 S90lidar_web 统一：'+str(script))
    web_source=(overlay/'etc/lidar/service.conf').read_text() if (overlay/'etc/lidar/service.conf').is_file() else ''
    release_web=bool(re.search(r'^LIDAR_WEB_SOURCE=release$', web_source, re.MULTILINE))
    web_root='usr/share/lidar/web-recovery' if release_web else 'www'
    required=('usr/libexec/lidar/lidar_boot','usr/libexec/lidar/lidar_crypto',
        'usr/libexec/lidar/lidar_sn_provision',
        'usr/libexec/lidar/platform/yousee_lidar.ko','usr/libexec/lidar/platform/demo_i2c_motor',
        'etc/lidar/release-public.pem','etc/lidar/boot.json','etc/lidar/service.conf',
        web_root+'/cgi-bin/lidar_status.cgi',web_root+'/cgi-bin/lidar_switch.cgi')
    if any(not (overlay/name).is_file() for name in required):
        raise ValueError('集成输入不完整，请使用 prepare_boot_bundle.py 生成的 rootfs-overlay')
    if not re.search(r'^LIDAR_MODULE_ID=[0-9a-f]{64}$',(overlay/'etc/lidar/service.conf').read_text(),re.MULTILINE):
        raise ValueError('内核模块配套尚未配置，禁止直接合入模板')
    if release_web and (not (overlay/'usr/libexec/lidar/lidar-web').is_file() or
                        (overlay/'www').exists() or (overlay/'www').is_symlink()):
        raise ValueError('第三分区 Web 必须由可信准备脚本生成，overlay 不直接覆盖 /www')
    for relative in (web_root+'/sbin/lighttpd',web_root+'/config/lighttpd.conf',web_root+'/lib'):
        if not (overlay/relative).exists() and not (target/relative).exists():
            raise ValueError('系统镜像缺少 Web 资产，请提供 --www-root 或先安装 lighttpd：'+relative)
    if legacy.exists():
        backup=target/'usr/share/lidar/legacy/S99user'
        backup.parent.mkdir(parents=True,exist_ok=True)
        if backup.exists():
            raise ValueError('旧启动脚本备份已经存在，请人工核对')
        legacy.rename(backup)
    # 先移除目标中的配置文件和符号链接，避免 copytree 追随旧链接写到 TARGET_DIR 外。
    if release_web and (target/'www').exists() and not (target/'www').is_symlink():
        backup=target/'usr/share/lidar/legacy/www'
        if backup.exists():
            raise ValueError('旧 Web 备份已存在，请核对后再合入')
        backup.parent.mkdir(parents=True,exist_ok=True)
        (target/'www').rename(backup)
    for source in overlay.rglob('*'):
        destination=target/source.relative_to(overlay)
        if (source.is_file() or source.is_symlink()) and destination.is_symlink():
            destination.unlink()
        if source.is_symlink() and destination.exists():
            if not destination.is_file():
                raise ValueError('标定目标不是普通文件：'+str(destination))
            backup=target/'usr/share/lidar/legacy'/destination.name
            backup.parent.mkdir(parents=True,exist_ok=True)
            if not backup.exists():
                shutil.copy2(destination,backup)
            destination.unlink()
    shutil.copytree(overlay,target,dirs_exist_ok=True,symlinks=True)
    if network is not None:
        original, updated, dbus_supported = network
        config = target/'etc/lidar/network.json'
        settings = json.loads(config.read_text())
        settings['avahi_dbus_supported'] = dbus_supported
        config.write_text(json.dumps(settings, ensure_ascii=False, indent=2)+'\n')
        config.chmod(0o600)
        backup = target/'etc/lidar/legacy-network.interfaces'
        if not backup.exists():
            backup.write_text(original)
            backup.chmod(0o600)
        interfaces = target/'etc/network/interfaces'
        interfaces.parent.mkdir(parents=True, exist_ok=True)
        interfaces.write_text(updated)
        # Avahi 的生命周期交给 lidar_netd；离线保留旧入口用于核查。
        for script in (target/'etc/init.d').glob('S*avahi*'):
            destination = target/'usr/share/lidar/legacy'/script.name
            destination.parent.mkdir(parents=True, exist_ok=True)
            if destination.exists():
                raise ValueError('已有 Avahi 启动备份，请核对：'+str(destination))
            script.rename(destination)
    if release_web:
        link=target/'www'
        if link.is_symlink():
            link.unlink()
        link.symlink_to('/run/lidar-web/current')
    if not release_web:
        for name in ('switch_raw_mode.cgi','switch_point_mode.cgi'):
            (target/'www/cgi-bin'/name).unlink(missing_ok=True)
    table=target/'etc/inittab'
    line='::respawn:/usr/libexec/lidar/lidar-service scan'
    text=table.read_text()
    if line not in text.splitlines():
        table.write_text(text.rstrip()+'\n\n# 雷达服务监督树，使用 BusyBox inittab 格式\n'+line+'\n')
    print('已集成启动工具并保留原 inittab；请核对网络、SSH 与 PTP 独立启动入口。')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target',type=Path)
    parser.add_argument('--overlay',type=Path,required=True)
    parser.add_argument('--replace-legacy-startup',action='store_true')
    parser.add_argument('--avahi-config-header',type=Path,help='目标 Buildroot Avahi 的 config.h；默认从 TARGET_DIR 同级 build/avahi-*/config.h 查找')
    args=parser.parse_args()
    try:
        integrate(args.target,args.overlay,args.replace_legacy_startup,args.avahi_config_header)
    except (ValueError,OSError) as error:
        parser.exit(1,'集成失败：'+str(error)+'\n')


if __name__=='__main__':
    main()
