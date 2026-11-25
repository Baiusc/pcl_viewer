#!/usr/bin/env python3
"""生成仅用于 221 固定点云布局的签名开发测试包，不自动连接或部署设备。"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
ROOT=Path(__file__).resolve().parents[1]
MODULE_HASH='5c7a4c9da9807b1c57a45c20d2141c2bf16b898e67fb30d4a15c1b1b5b49fe16'

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path,help='新输出目录，必须不存在')
    args=parser.parse_args()
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    base=output/'beta-test';base.mkdir()
    # 临时测试私钥只留在构建主机临时目录，不纳入上传文件。
    with tempfile.TemporaryDirectory(prefix='ys-221-test-key-') as temp:
        key=Path(temp)/'private.pem'
        subprocess.run(['openssl','genpkey','-algorithm','EC','-pkeyopt','ec_paramgen_curve:P-256','-out',str(key)],check=True)
        subprocess.run([sys.executable,str(ROOT/'scripts/build.py'),'package','beta','--output',str(base/'payload'),'--sign-key',str(key)],cwd=ROOT,check=True)
        subprocess.run(['openssl','pkey','-in',str(key),'-pubout','-out',str(base/'test-public.pem')],check=True)
    (base/'tools').mkdir()
    shutil.copy2(base/'payload/bin/lidar_crypto',base/'tools/lidar_crypto')
    for name in ('test.sh','fixed_pointcloud_adapter.sh'):
        shutil.copy2(ROOT/'scripts/beta_221'/name,base/name)
        (base/name).chmod(0o700)
    config=dict(model='CS96-beta',package_dir='/tmp/beta-test/payload',public_key='/tmp/beta-test/test-public.pem',
                openssl='/tmp/beta-test/tools/lidar_crypto',state_dir='/tmp/beta-test/state',runtime_dir='/tmp/beta-test/runtime',
                socket_path='/tmp/beta-test/control.sock',base_config='/tmp/beta-test/device-test.ini',
                adapter='/tmp/beta-test/fixed_pointcloud_adapter.sh',initial_work_mode=2,
                validated_work_modes=[1,2],verify_timeout_ms=30000,stop_timeout_ms=5000,adapter_timeout_ms=5000,settle_ms=1000)
    (base/'manager.template.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n')
    (base/'module.sha256').write_text(MODULE_HASH+'  /sys/module/yousee_lidar/notes/.note.gnu.build-id\n')
    instructions='''# 221 固定点云测试包

只适用于已经核对过的 221 Beta 点云内核模块和 FPGA 配套。仅测试 work_mode 1/2，不能测试直方图或灰度。测试签名不是正式发布签名。

设备端 device-test.ini 由部署时复制当前 /etc/ys_conf.ini 并只修改日志路径生成。网络与标定沿用设备当前设置。

```sh
cd /tmp/beta-test
sh test.sh check
sh test.sh start
sleep 3
sh test.sh status
sh test.sh switch 1
sleep 3
sh test.sh status
sh test.sh switch 2
sleep 3
sh test.sh status
sh test.sh logs
sh test.sh restore
```

start 会先停止 /root/you_see，启动新管理服务；每次 start 建立新测试会话，旧记录保留，不用于验证跨会话断电恢复。

status 中 phase=COMMITTED、actual_mode 与目标一致、initialized=true、pipeline_verified=true，表示本轮应用流水线验证通过；计数单位是有效数据包，不能等同于完整帧或硬件格式全面验证。

固定点云适配器只检查已知内核模块构建标识和 /dev/lidar 无其他占用，允许同一 DMA 布局重新打开。不会切换、停止或复位 DMA，不能作为跨硬件格式的通用 recover 实现。正常应用退出也可能因硬件未确认停采返回非零，这是保守报告，不代表线程一定未回收。

stop 只停止本轮管理服务及采集应用；restore 会在设备无其他占用时重新启动原 /root/you_see。原程序、系统驱动库和配置未被覆盖。重启设备也会恢复原来的启动流程，但 /tmp 测试包和测试记录会丢失，请先取回日志。

失败时先保留 manager.log、app.log、active-config 指向的 JSON 及对应 state.* 目录；不要反复切换。可运行 sh test.sh restore 或重启雷达恢复。没有部署到 Flash，没有修改开机项，没有操作 238。
'''
    (base/'README.md').write_text(instructions)
    lines=[]
    for path in sorted(base.rglob('*')):
        if path.is_file():
            lines.append(hashlib.sha256(path.read_bytes()).hexdigest()+'  '+path.relative_to(base).as_posix())
    (base/'bundle.sha256').write_text('\n'.join(lines)+'\n')
    with tarfile.open(output/'beta-test.tar.gz','w:gz') as archive:archive.add(base,arcname='beta-test')
    print(output/'beta-test.tar.gz')
if __name__=='__main__':main()
