#!/usr/bin/env python3
"""生成无雷达开发板的 RAM 测试包；使用真实验签和监督，硬件明确标记为模拟。"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import tempfile

ROOT=Path(__file__).resolve().parents[1]


def prepare(output, base='/tmp/lidar-board-test', port=8081):
    if not re.fullmatch(r'/tmp/[A-Za-z0-9_-]+',base):
        raise ValueError('RAM 测试目录必须是 /tmp 下一个明确的新目录')
    if not 1024<=port<=65535:
        raise ValueError('测试 HTTP 端口不合法')
    output=output.resolve();output.mkdir(parents=True,exist_ok=False)
    stage=output/'lidar-board-test';stage.mkdir()
    path=lambda name:base+'/'+name
    for name in ('tools','trust','factory','control','backup','history','http/lib','http/cgi-bin','template/http'):
        (stage/name).mkdir(parents=True,exist_ok=True)
    package=stage/'data/releases/mock-v1';(package/'bin').mkdir(parents=True)
    (package/'conf').mkdir()
    compiler=ROOT/'cc/bin/aarch64-buildroot-linux-gnu-gcc'
    subprocess.run([str(compiler),'-O2','-Wall','-Wextra','-Werror','-std=c11',
        str(ROOT/'scripts/devboard/mock_capture.c'),str(ROOT/'core/ys_management.c'),
        '-o',str(package/'bin/you_see')],check=True)
    for name in ('lidar_boot','lidar_crypto','lidar_mgrctl'):
        shutil.copy2(ROOT/'build/beta/bin'/name,stage/'tools'/name)
    shutil.copy2(ROOT/'build/beta/bin/lidar_mgrd',package/'bin/lidar_mgrd')
    for name in ('lidar-service','lidar-storage'):
        shutil.copy2(ROOT/'deploy/runit/rootfs/usr/libexec/lidar'/name,stage/'tools'/name)
    shutil.copy2(ROOT/'scripts/devboard/mock_adapter.sh',stage/'tools/mock-adapter')
    shutil.copy2(ROOT/'scripts/devboard/test.sh',stage/'test.sh')
    shutil.copy2(ROOT/'doc/开发板30-RAM模式管理人工验证.md',stage/'人工验证手册.md')
    shutil.copytree(ROOT/'deploy/runit/rootfs/usr/share/lidar/runit/lidar_mgrd',stage/'template/lidar_mgrd')
    ini='[system]\nwork_mode="2"\necho_mode="0"\necho_type="1"\ncoordinate_mode="0"\n[network]\ncenter_ip="127.0.0.1"\n'
    (package/'conf/ys_conf.ini').write_text(ini)
    (stage/'factory/ys_conf.ini').write_text(ini)
    for name in ('ys_calibration.ini','ys_encoder_angle.csv'):
        (stage/'factory'/name).write_text('开发板模拟占位，不是真实雷达标定；模拟应用不读取此文件。\n')
    manifest=dict(schema_version=1,model='DEVBOARD-MOCK',release=False,version='devboard-ram-test',
        supported_modes=['pointcloud'],hardware_validation='模拟采集，不使用真实雷达硬件',
        files={str(p.relative_to(package)):dict(size=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
               for p in package.rglob('*') if p.is_file()})
    (package/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    with tempfile.TemporaryDirectory(prefix='devboard-signing-') as temporary:
        key=Path(temporary)/'private.pem'
        for command in (
            ['openssl','genpkey','-algorithm','EC','-pkeyopt','ec_paramgen_curve:P-256','-out',str(key)],
            ['openssl','pkey','-in',str(key),'-pubout','-out',str(stage/'trust/test-public.pem')],
            ['openssl','dgst','-sha256','-sign',str(key),'-out',str(package/'manifest.sig'),str(package/'manifest.json')],
        ):subprocess.run(command,check=True,capture_output=True)
    (stage/'data/current').symlink_to('releases/mock-v1')
    settings=dict(model='DEVBOARD-MOCK',data_root=path('data'),release_root=path('data/releases'),
        package_dir=path('data/current'),public_key=path('trust/test-public.pem'),openssl=path('tools/lidar_crypto'),
        adapter=path('tools/mock-adapter'),boot_runtime=path('run'),runtime_dir=path('run/manager'),
        socket_path=path('run/control.sock'),state_dir=path('data/state'),base_config=path('data/config/ys_conf.ini'),
        user_settings=path('data/config/user-settings.json'),calibration_dir=path('data/calibration'),
        legacy_config_dir=path('factory'),initial_work_mode=2,validated_work_modes=[1,2],disable_file_log=True,
        verify_timeout_ms=2500,stop_timeout_ms=500,adapter_timeout_ms=500,settle_ms=100)
    (stage/'boot.json').write_text(json.dumps(settings,ensure_ascii=False,indent=2)+'\n')
    service=dict(LIDAR_STORAGE_MODE='directory',LIDAR_STORAGE_MOUNT=path('data'),LIDAR_STORAGE_FSTYPE='ramfs',
        LIDAR_RUN=path('run'),LIDAR_SCAN=path('services'),LIDAR_TEMPLATE=path('template/lidar_mgrd'),
        LIDAR_BOOT_CONFIG=path('boot.json'),LIDAR_BOOT=path('tools/lidar_boot'),
        LIDAR_RUNSV_DIR='/usr/bin/runsvdir',LIDAR_SV='/usr/bin/sv',LIDAR_LOGGER='/usr/bin/logger',
        LIDAR_MAX_FAILURES=3,LIDAR_STABLE_SECONDS=60,LIDAR_RETRY_SECONDS=1,LIDAR_STOP_SECONDS=12)
    (stage/'service.conf').write_text('\n'.join(f"{k}='{v}'" for k,v in service.items())+'\n')
    www=ROOT.parent/'you_see_www/www'
    shutil.copy2(www/'sbin/lighttpd',stage/'http/lighttpd')
    for name in ('libpcre.so.1','mod_alias.so','mod_cgi.so','mod_setenv.so','mod_staticfile.so','mod_indexfile.so','mod_dirlisting.so'):
        shutil.copy2(www/'lib'/name,stage/'http/lib'/name)
    for name in ('lidar_status.cgi','lidar_switch.cgi'):
        shutil.copy2(ROOT.parent/'you_see_www/you_see_cgi/build'/name,stage/'http/cgi-bin'/name)
    (stage/'http-bridge.json').write_text(json.dumps(dict(socket_path=path('run/control.sock'),timeout_ms=10000))+'\n')
    config=f'''server.modules = ("mod_alias", "mod_setenv", "mod_cgi", "mod_staticfile")
server.bind = "0.0.0.0"
server.port = {port}
server.document-root = "{path('http')}"
server.errorlog = "{path('http/error.log')}"
server.pid-file = "{path('http/lighttpd.pid')}"
server.upload-dirs = ("/tmp")
alias.url = ("/cgi-bin/" => "{path('http/cgi-bin/')}")
setenv.add-environment = ("LIDAR_HTTP_CONFIG" => "{path('http-bridge.json')}")
$HTTP["url"] =~ "^/cgi-bin/" {{ cgi.assign = (".cgi" => "") }}
'''
    (stage/'http/lighttpd.conf').write_text(config)
    (stage/'template/http/run').write_text(f'''#!/bin/sh
export LD_LIBRARY_PATH='{path('http/lib')}'
exec '{path('http/lighttpd')}' -D -f '{path('http/lighttpd.conf')}' -m '{path('http/lib')}'
''')
    (stage/'TEST_ENVIRONMENT').write_text('DEVBOARD-MOCK\n')
    (stage/'测试说明.txt').write_text(
        f'开发板 RAM 测试：BASE={base}，HTTP 端口 {port}。\n'
        '签名正常验证，simulation=false 只表示没有跳过验签；model=DEVBOARD-MOCK 明确表示模拟硬件。\n'
        '整板重启会清除全部测试文件；本轮不验证断电持久化或真实雷达数据。\n')
    for p in stage.rglob('*'):
        if p.is_symlink():continue
        p.chmod(0o700 if p.is_dir() else 0o600)
        if p.is_file() and (p.parent.name in ('bin','tools','cgi-bin') or p.name in ('test.sh','run','finish','check','lighttpd')):
            p.chmod(0o700)
    (stage/'bundle.sha256').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(stage))+'\n'
        for p in sorted(stage.rglob('*')) if p.is_file() and not p.is_symlink()))
    with tarfile.open(output/'lidar-board-test.tar.gz','w:gz') as archive:
        def owner(info):
            info.uid=info.gid=0;info.uname=info.gname='root';info.mtime=0;return info
        archive.add(stage,arcname=Path(base).name,filter=owner)
    print(output/'lidar-board-test.tar.gz')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    parser.add_argument('--base',default='/tmp/lidar-board-test')
    parser.add_argument('--port',type=int,default=8081)
    args=parser.parse_args()
    prepare(args.output,args.base,args.port)
