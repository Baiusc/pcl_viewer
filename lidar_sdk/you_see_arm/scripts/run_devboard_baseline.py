#!/usr/bin/env python3
"""在 30 新镜像上运行隔离的开发板验收；复用镜像程序，仅模拟采集源。"""
import argparse
from datetime import datetime, timezone
import hashlib
import http.client
import io
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import socket
import subprocess
import tarfile
import tempfile
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
MODEL = 'DEVBOARD-MOCK'


def healthy(value, mode=None):
    """HTTP 成功或管理进程就绪均不足以判定采集验证完成。"""
    return (value.get('code') == 200 and value.get('model') == MODEL
            and value.get('phase') == 'COMMITTED' and value.get('error') == ''
            and value.get('initialized') is True and value.get('pipeline_verified') is True
            and type(value.get('pid')) is int and value['pid'] > 0
            and type(value.get('actual_mode')) is int and type(value.get('confirmed_mode')) is int
            and value.get('actual_mode') in (1, 2)
            and value.get('actual_mode') == value.get('confirmed_mode')
            and (mode is None or value.get('actual_mode') == mode))


def require(value, message):
    if not value:
        raise RuntimeError(message)


def root_owner(info):
    info.uid=info.gid=0
    info.uname=info.gname='root'
    info.mtime=0
    return info


def checked_identity(run_id, port):
    require(re.fullmatch(r'run-[0-9]{8}-[0-9]{6}-[0-9a-f]{8}', run_id), '验收标识非法')
    require(1024 <= port <= 65535 and port not in (8080,), '验收 HTTP 端口非法')
    return '/run/lidar-acceptance-'+run_id, '/mnt/mtdblock2/lidar-acceptance/'+run_id


class Board:
    def __init__(self, host, output):
        require(host == '10.42.0.30', '本工具仅允许操作已约定的 10.42.0.30 开发板')
        self.host = host
        self.output = output
        password = os.environ.get('LIDAR_TEST_SSH_PASSWORD')
        self.env = os.environ.copy()
        self.ssh = ['ssh', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=8']
        if password:
            self.env['SSHPASS'] = password
            self.ssh = ['sshpass', '-e', *self.ssh]
        else:
            self.ssh += ['-o', 'BatchMode=yes']
        self.ssh += ['root@'+host]

    def command(self, command, *, data=None, check=True, timeout=25, binary=False):
        result = subprocess.run([*self.ssh, command], input=data, capture_output=True,
                                timeout=timeout, env=self.env)
        # 仅记录远端命令和结果，不记录 SSH 参数、密码或环境。
        with (self.output/'events.jsonl').open('a') as stream:
            stream.write(json.dumps({'command': command, 'exit': result.returncode,
                'stdout': {'bytes':len(result.stdout),'sha256':hashlib.sha256(result.stdout).hexdigest()} if binary
                    else result.stdout.decode(errors='replace') if data is None else '(传送验收归档)',
                'stderr': result.stderr.decode(errors='replace')}, ensure_ascii=False)+'\n')
        if check and result.returncode:
            raise RuntimeError('远端命令失败：'+result.stderr.decode(errors='replace')[-1200:])
        return result

    def text(self, command, **kwargs):
        return self.command(command, **kwargs).stdout.decode().strip()

    def control(self, runtime, action, *, check=True, timeout=25):
        return self.command('sh '+shlex.quote(runtime+'/control.sh')+' '+action,
                            check=check, timeout=timeout)

    def http(self, port, mode=None):
        connection = http.client.HTTPConnection(self.host, port, timeout=6)
        try:
            if mode is None:
                connection.request('GET', '/cgi-bin/lidar_status.cgi')
            else:
                body = json.dumps({'work_mode': mode})
                connection.request('POST', '/cgi-bin/lidar_switch.cgi', body,
                                   {'Content-Type': 'application/json'})
            response = connection.getresponse()
            payload = response.read(65537)
            require(len(payload) <= 65536, 'HTTP 响应过长')
            value = json.loads(payload)
            with (self.output/'events.jsonl').open('a') as stream:
                stream.write(json.dumps({'http_port':port, 'request_mode':mode,
                    'http_status':response.status, 'response':value}, ensure_ascii=False)+'\n')
            return response.status, value
        finally:
            connection.close()


def snapshot(board):
    command = ('sha256sum /mnt/mtdblock2/lidar/state/state.json /etc/inittab '
        '/etc/lidar/service.conf /etc/lidar/boot.json; '
        'readlink /mnt/mtdblock2/lidar/current; '
        'cat /run/lidar-services/lidar_mgrd/supervise/pid; '
        'cat /www/www.pid')
    return board.text(command)


def build_bundle(board, output, run_id, port):
    runtime, data_root = checked_identity(run_id, port)
    installed = board.text('readlink -f /mnt/mtdblock2/lidar/current')
    require(re.fullmatch(r'/mnt/mtdblock2/lidar/releases/[A-Za-z0-9._-]+', installed), '活动版本路径非法')
    stage = output/'runtime-template'
    stage.mkdir()
    package = stage/'seed/releases/mock-v1'
    for name in ('bin', 'conf'):
        (package/name).mkdir(parents=True, exist_ok=True)
    manager = board.command('cat '+shlex.quote(installed+'/bin/lidar_mgrd'),binary=True).stdout
    require(len(manager)>=20 and manager[:6] == b'\x7fELF\x02\x01'
            and int.from_bytes(manager[18:20],'little')==183, '已安装的管理程序不是预期 AArch64 ELF')
    (package/'bin/lidar_mgrd').write_bytes(manager)
    (package/'bin/lidar_mgrd').chmod(0o700)
    subprocess.run([str(ROOT/'cc/bin/aarch64-buildroot-linux-gnu-gcc'), '-O2', '-std=c11',
        '-Wall', '-Wextra', '-Werror', str(ROOT/'scripts/devboard/mock_capture.c'),
        str(ROOT/'core/ys_management.c'), '-o', str(package/'bin/you_see')], check=True)
    ini = '[system]\nwork_mode="2"\necho_mode="0"\necho_type="1"\ncoordinate_mode="0"\n[network]\ncenter_ip="127.0.0.1"\n'
    (package/'conf/ys_conf.ini').write_text(ini)
    files = {str(path.relative_to(package)): {'size':path.stat().st_size,
             'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
             for path in package.rglob('*') if path.is_file()}
    manifest = {'schema_version':1, 'model':MODEL, 'release':False, 'version':'baseline-mock-v1',
                'supported_modes':['pointcloud'], 'hardware_validation':'模拟健康信号，无真实雷达数据', 'files':files}
    (package/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    for name in ('trust', 'config', 'state', 'calibration'):
        (stage/'seed'/name).mkdir()
    (stage/'seed/current').symlink_to('releases/mock-v1')
    for name in ('ys_calibration.ini', 'ys_encoder_angle.csv'):
        (stage/'seed/calibration'/name).write_text('开发板模拟占位，不能用于真实雷达标定。\n')
    marker = MODEL+':'+run_id+'\n'
    (stage/'seed/TEST_ENVIRONMENT').write_text(marker)
    (stage/'TEST_ENVIRONMENT').write_text(marker)
    with tempfile.TemporaryDirectory(prefix='baseline-signing-') as temporary:
        private = Path(temporary)/'private.pem'
        for command in (
            ['openssl','genpkey','-algorithm','EC','-pkeyopt','ec_paramgen_curve:P-256','-out',str(private)],
            ['openssl','pkey','-in',str(private),'-pubout','-out',str(stage/'seed/trust/public.pem')],
            ['openssl','dgst','-sha256','-sign',str(private),'-out',str(package/'manifest.sig'),str(package/'manifest.json')],
        ):
            subprocess.run(command,check=True,capture_output=True)
    parent = '/run/lidar-services/acceptance_'+run_id
    (stage/'paths.conf').write_text('\n'.join(name+'='+shlex.quote(value) for name,value in
        {'RUN_ID':run_id,'DATA':data_root,'PARENT':parent,'CTL':installed+'/bin/lidar_mgrctl'}.items())+'\n')
    settings = {'model':MODEL,'data_root':data_root,'release_root':data_root+'/releases',
        'package_dir':data_root+'/current','public_key':data_root+'/trust/public.pem',
        'openssl':'/usr/libexec/lidar/lidar_crypto','adapter':runtime+'/mock-adapter',
        'boot_runtime':runtime,'runtime_dir':runtime+'/manager','socket_path':runtime+'/control.sock',
        'state_dir':data_root+'/state','base_config':data_root+'/config/ys_conf.ini',
        'user_settings':data_root+'/config/user-settings.json','calibration_dir':data_root+'/calibration',
        'initial_work_mode':2,'validated_work_modes':[1,2],'disable_file_log':True,
        'verify_timeout_ms':2500,'stop_timeout_ms':1000,'adapter_timeout_ms':1000,'settle_ms':300}
    (stage/'boot.json').write_text(json.dumps(settings,ensure_ascii=False,indent=2)+'\n')
    service = {'LIDAR_RUN':runtime,'LIDAR_SCAN':runtime+'/services',
        'LIDAR_TEMPLATE':'/usr/share/lidar/runit/lidar_mgrd','LIDAR_BOOT_CONFIG':runtime+'/boot.json',
        'LIDAR_BOOT':'/usr/libexec/lidar/lidar_boot','LIDAR_RUNSV_DIR':'/usr/bin/runsvdir',
        'LIDAR_SV':'/usr/bin/sv','LIDAR_LOGGER':'/usr/bin/logger','LIDAR_MAX_FAILURES':'3',
        'LIDAR_STABLE_SECONDS':'60','LIDAR_RETRY_SECONDS':'1','LIDAR_STOP_SECONDS':'12'}
    (stage/'service.conf').write_text('\n'.join(name+'='+shlex.quote(value) for name,value in service.items())+'\n')
    shutil.copy2(ROOT/'scripts/devboard/baseline_control.sh',stage/'control.sh')
    shutil.copy2(ROOT/'scripts/devboard/mock_adapter.sh',stage/'mock-adapter')
    for name in ('control','parent-template','http-template','http'):
        (stage/name).mkdir()
    (stage/'parent-template/run').write_text('#!/bin/sh\nexport LIDAR_SERVICE_CONFIG='+shlex.quote(runtime+'/service.conf')+
        '\nexport LIDAR_DEVBOARD_FLAGS='+shlex.quote(runtime+'/control')+
        '\nexec /usr/bin/runsvdir '+shlex.quote(runtime+'/services')+'\n')
    (stage/'http-template/run').write_text('#!/bin/sh\nexport LD_LIBRARY_PATH=/www/lib\nexec /www/sbin/lighttpd -D -f '+
        shlex.quote(runtime+'/http/lighttpd.conf')+' -m /www/lib\n')
    (stage/'http-bridge.json').write_text(json.dumps({'socket_path':runtime+'/control.sock','timeout_ms':10000})+'\n')
    (stage/'http/lighttpd.conf').write_text(f'''server.modules = ("mod_alias", "mod_setenv", "mod_cgi", "mod_staticfile")
server.port = {port}
server.document-root = "{runtime}/http"
server.errorlog = "{runtime}/http/error.log"
server.pid-file = "{runtime}/http/lighttpd.pid"
server.upload-dirs = ("/tmp")
alias.url = ("/cgi-bin/" => "/www/cgi-bin/")
setenv.add-environment = ("LIDAR_HTTP_CONFIG" => "{runtime}/http-bridge.json")
$HTTP["url"] =~ "^/cgi-bin/" {{ cgi.assign = (".cgi" => "") }}
''')
    for path in stage.rglob('*'):
        if not path.is_symlink():
            path.chmod(0o700 if path.is_dir() or path.stat().st_mode & 0o111 or
                       path.name in ('run','mock-adapter','control.sh') else 0o600)
    receipt = {'run_id':run_id,'runtime':runtime,'data_root':data_root,'port':port,'host':board.host,
        'manager_source':installed+'/bin/lidar_mgrd','manager_sha256':hashlib.sha256(manager).hexdigest(),
        'hardware':'模拟采集；不操作驱动、DMA、FPGA、电机；不代表真实点云验收'}
    (output/'receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
    archive_path = output/'runtime.tar.gz'
    with tarfile.open(archive_path,'w:gz') as archive:
        archive.add(stage,arcname='.',filter=root_owner)
    return receipt


class Acceptance:
    def __init__(self, board, receipt):
        self.board=board;self.receipt=receipt
        self.runtime=receipt['runtime'];self.port=receipt['port'];self.cases=[]

    def wait(self,predicate,timeout=15):
        deadline=time.monotonic()+timeout;last=None
        while time.monotonic()<deadline:
            try:
                _,last=self.board.http(self.port)
                if predicate(last):return last
            except (OSError,ValueError,http.client.HTTPException):
                pass
            time.sleep(.15)
        raise RuntimeError('状态未达到预期：'+json.dumps(last,ensure_ascii=False))

    def control(self,action,**kwargs):
        return self.board.control(self.runtime,action,**kwargs)

    def case(self,name,operation):
        try:
            details=operation()
            self.cases.append({'name':name,'result':'PASS','details':details})
            print('通过：'+name,flush=True)
            return details
        except Exception as error:
            self.cases.append({'name':name,'result':'FAIL','details':str(error)})
            raise

    def switch(self,mode):
        code,accepted=self.board.http(self.port,mode)
        require(code==202,'切换未受理：'+str(accepted))
        operation=accepted['operation_id']
        return self.wait(lambda value:healthy(value,mode) and value.get('operation_id')==operation)

    def suite(self):
        self.case('已安装管理程序通过签名包启动模拟采集',lambda:self.wait(lambda value:healthy(value,2)))
        self.case('HTTP 模式 2→1',lambda:self.switch(1))
        self.case('HTTP 模式 1→2',lambda:self.switch(2))
        def unsupported():
            code,value=self.board.http(self.port,4)
            require(code==422,'未拒绝白名单外模式')
            require(healthy(self.board.http(self.port)[1],2),'非法请求改变了现有状态')
            return value
        self.case('非法模式拒绝且保持原采集',unsupported)
        def rollback():
            self.control('fail-mode1')
            try:
                code,value=self.board.http(self.port,1);require(code==202,'未受理超时场景')
                code,busy=self.board.http(self.port,2);require(code==409,'切换中未拒绝并发请求')
                final=self.wait(lambda current:current.get('phase')=='COMMITTED'
                    and current.get('actual_mode')==2 and bool(current.get('error')))
                require(final['operation_id']==value['operation_id'],'回退任务标识不匹配')
                return {'busy':busy,'rollback':final}
            finally:
                self.control('clear-failure')
        self.case('切换互斥及无数据超时回退',rollback)
        self.case('故障注入移除后可正常切换',lambda:self.switch(1))
        def maintenance():
            self.control('stop');time.sleep(1.2)
            state=json.loads(self.board.text('cat '+shlex.quote(self.receipt['data_root']+'/state/state.json')))
            require(state['phase']=='STOPPED' and state['confirmed_mode']==1,'正常停机状态不符')
            code,_=self.board.http(self.port);require(code==503,'停止后管理接口不应继续报告可用')
            result=self.board.command('/usr/bin/sv status '+shlex.quote(self.runtime+'/services/lidar_mgrd')).stdout.decode()
            require(result.startswith('down:'),'监督器没有保持正常停机')
            return state
        self.case('正常停机后保持停止',maintenance)
        def restart():
            self.control('start')
            return self.wait(lambda value:healthy(value,1))
        self.case('从真实 Flash 恢复最后确认模式',restart)
        def crash():
            previous=self.board.http(self.port)[1]['pid']
            old_manager=self.control('manager-pid').stdout.decode().strip()
            self.control('crash-manager')
            value=self.wait(lambda current:healthy(current,1) and current['pid']!=previous)
            require(self.control('manager-pid').stdout.decode().strip()!=old_manager,'管理 PID 未更新')
            alive=self.board.command('test -e /proc/'+str(previous),check=False)
            require(alive.returncode!=0,'旧模拟采集进程未退出')
            return value
        self.case('管理进程崩溃后监督恢复并回收旧子进程',crash)
        def tamper():
            self.control('stop');self.control('tamper')
            try:
                require(self.control('verify',check=False).returncode!=0,'损坏包未被验签拒绝')
                self.control('start',check=False,timeout=25)
                count=self.control('failures',check=False).stdout.decode().strip()
                require(count=='3','连续失败没有按预算熔断：'+count)
                require(self.board.http(self.port)[0]==503,'损坏包仍有可用管理实例')
                return {'signature_rejected':True,'startup_failures':int(count)}
            finally:
                self.control('stop',check=False)
                self.control('restore-package')
                self.control('start')
        self.case('测试包损坏拒绝执行并停止无限重试',tamper)
        self.case('恢复原签名包后重新运行',lambda:self.wait(lambda value:healthy(value,1)))


def save_report(output,receipt,cases,error=None):
    report={'scope':receipt,'cases':cases,'error':error,'pending':[
        '整机重启后 Flash 状态与模式恢复：需人工重启后运行 resume',
        '真实采集、DMA、FPGA 与电机','OTA 安装与回退','SN 写入、网络配置与 PTP']}
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    lines=['# 开发板新镜像自动验收报告','',receipt.get('hardware',''),'','| 用例 | 结果 |','| --- | --- |']
    lines += ['| '+case['name']+' | '+case['result']+' |' for case in cases]
    if error:lines+=['','执行错误：'+error]
    lines+=['','## 本轮未验证','']+['- '+item for item in report['pending']]
    lines+=['','人工查看：`sh '+receipt['runtime']+'/control.sh status`。',
        '模拟服务保留运行，当前 Beta 部署不变。清理使用本工具 cleanup 子命令，Flash 测试数据保留。','']
    (output/'报告.md').write_text('\n'.join(lines))


def deploy(board,output,receipt,resume=False):
    runtime=receipt['runtime']
    # 恢复阶段只重建 RAM 文件，绝不将 seed 再覆盖到 Flash。
    if resume:
        archive_data=io.BytesIO()
        with tarfile.open(fileobj=archive_data,mode='w:gz') as archive:
            for path in (output/'runtime-template').iterdir():
                if path.name!='seed':archive.add(path,arcname=path.name,filter=root_owner)
        data=archive_data.getvalue()
    else:
        data=(output/'runtime.tar.gz').read_bytes()
    board.command('umask 077; test ! -e '+shlex.quote(runtime)+' && test ! -L '+shlex.quote(runtime)+
                  ' && mkdir '+shlex.quote(runtime)+' && tar -xzf - -C '+shlex.quote(runtime),data=data)
    board.control(runtime,'setup')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('run','resume','cleanup'))
    parser.add_argument('--host',default='10.42.0.30')
    parser.add_argument('--output',type=Path,required=True,help='本机结果目录；run 时必须不存在')
    parser.add_argument('--port',type=int,default=8081)
    args=parser.parse_args();output=args.output.resolve()
    if args.command=='run':output.mkdir(parents=True,exist_ok=False)
    else:require(output.is_dir(),'结果目录不存在')
    board=Board(args.host,output)
    if args.command=='run':
        run_id='run-'+datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]
        checked_identity(run_id,args.port)
        require(board.command('test -x /usr/bin/runsvdir && test -x /usr/bin/sv && test -d /run/lidar-services',check=False).returncode==0,'镜像监督环境未就绪')
        try:
            connection=socket.create_connection((args.host,args.port),timeout=3)
        except ConnectionRefusedError:
            pass
        else:
            connection.close()
            raise RuntimeError('验收端口已经被使用，拒绝覆盖现有服务')
        before=snapshot(board)
        receipt=build_bundle(board,output,run_id,args.port)
        receipt['baseline_before']=before
        receipt['boot_id']=board.text('cat /proc/sys/kernel/random/boot_id')
        (output/'receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
        acceptance=Acceptance(board,receipt);error=None
        try:
            deploy(board,output,receipt)
            acceptance.suite()
            receipt['checkpoint_mode']=1
            receipt['checkpoint_state']=board.text('cat '+shlex.quote(receipt['data_root']+'/state/state.json'))
            (output/'receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
        except Exception as failure:
            error=str(failure)
        finally:
            try:
                after=snapshot(board)
                acceptance.case('原 Beta 状态、系统配置和服务 PID 保持不变',
                    lambda:require(after==before,'原部署基线发生变化，请查看事件记录'))
            except Exception as failure:
                error=(error+'；' if error else '')+str(failure)
            save_report(output,receipt,acceptance.cases,error)
        require(error is None,error)
    else:
        receipt=json.loads((output/'receipt.json').read_text())
        runtime,data_root=checked_identity(receipt['run_id'],receipt['port'])
        require(receipt['host']==args.host and receipt['runtime']==runtime and receipt['data_root']==data_root,'验收收据路径不匹配')
        if args.command=='cleanup':
            print(board.control(runtime,'cleanup').stdout.decode())
            return
        require('checkpoint_mode' in receipt,'上次验收未完成，没有可验证的重启检查点')
        new_boot=board.text('cat /proc/sys/kernel/random/boot_id')
        require(new_boot!=receipt['boot_id'],'系统尚未重启，不能认定跨重启保留')
        state=board.text('cat '+shlex.quote(data_root+'/state/state.json'))
        require(state==receipt['checkpoint_state'],'重启前后 Flash 状态文件不一致')
        before=snapshot(board)
        deploy(board,output,receipt,resume=True)
        acceptance=Acceptance(board,receipt)
        acceptance.case('整机重启后读取同一 Flash 状态并恢复模式',
            lambda:acceptance.wait(lambda value:healthy(value,receipt['checkpoint_mode'])))
        require(snapshot(board)==before,'恢复测试改变了原 Beta 部署')
        (output/'reboot-report.json').write_text(json.dumps({'boot_before':receipt['boot_id'],
            'boot_after':new_boot,'cases':acceptance.cases,
            '说明':'运行目录由电脑端重建；验证 Flash 保留与手动恢复，不等同于测试实例开机自启。'},ensure_ascii=False,indent=2)+'\n')
    print('验收结果：'+str(output/'报告.md'))


if __name__=='__main__':
    try:main()
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as error:
        raise SystemExit('验收失败：'+str(error))
