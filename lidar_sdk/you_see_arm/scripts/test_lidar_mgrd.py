#!/usr/bin/env python3
"""真实守护进程集成测试：仅在临时目录模拟硬件，不连接雷达。"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
ROOT = Path(__file__).resolve().parents[1]
DAEMON = ROOT / 'build/manager-host/lidar_mgrd'

class ManagerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run(['cmake', '-S', str(ROOT/'manager'), '-B', str(ROOT/'build/manager-host'), '-G', 'Ninja'], check=True, stdout=subprocess.DEVNULL)
        subprocess.run(['cmake', '--build', str(ROOT/'build/manager-host')], check=True, stdout=subprocess.DEVNULL)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='lm-')
        self.base = Path(self.temp.name)
        self.package = self.base/'pkg'
        (self.package/'bin').mkdir(parents=True)
        self.behavior = self.base/'behavior.json'
        self.behavior.write_text('{}')
        self.adapter_log = self.base/'adapter.log'
        app = self.package/'bin/you_see'
        app.write_text(f'''#!{sys.executable}
import json,os,signal,socket,time
mode=int(os.environ['LIDAR_WORK_MODE'])
b=json.load(open({str(self.behavior)!r}))
action=b.get(str(mode),'ok')
if action=='crash': raise SystemExit(9)
if action=='ignore_term': signal.signal(signal.SIGTERM,signal.SIG_IGN)
else: signal.signal(signal.SIGTERM,lambda *_:exit(0))
s=socket.socket(fileno=int(os.environ['LIDAR_MGR_FD']))
nonce=os.environ['LIDAR_MGR_TOKEN'] if action!='stale' else '0'*32
if action!='timeout':
 for event in ['INITIALIZED','PIPELINE_PROGRESS']:
  if action=='initialized_only' and event=='PIPELINE_PROGRESS':continue
  s.send(json.dumps(dict(event=event,token=nonce,work_mode=mode,valid_packets=10)).encode())
while True:time.sleep(.02)
''')
        app.chmod(0o700)
        manifest = dict(schema_version=1, model='CS96-beta', supported_modes=['histogram','pointcloud','gray'],
                        files={'bin/you_see':dict(size=app.stat().st_size,sha256=hashlib.sha256(app.read_bytes()).hexdigest())})
        (self.package/'manifest.json').write_text(json.dumps(manifest))
        self.adapter = self.base/'adapter'
        self.adapter.write_text(f'''#!{sys.executable}
import sys,json
with open({str(self.adapter_log)!r},'a') as f:f.write(' '.join(sys.argv[1:])+'\\n')
b=json.load(open({str(self.behavior)!r}))
if int(sys.argv[2]) in b.get('adapter_fail',[]):raise SystemExit(1)
''')
        self.adapter.chmod(0o700)
        self.ini = self.base/'device.ini'
        self.ini.write_text('[system]\nwork_mode="2"\n[network]\ncenter_ip="10.42.0.1"\n')
        self.sock = self.base/'ctl.sock'
        self.config = self.base/'manager.json'
        self.config.write_text(json.dumps(dict(package_dir=str(self.package),state_dir=str(self.base/'state'),
            runtime_dir=str(self.base/'runtime'),socket_path=str(self.sock),adapter=str(self.adapter),
            base_config=str(self.ini),openssl=os.environ.get('LIDAR_TEST_CRYPTO',shutil.which('openssl')),model='CS96-beta',
            initial_work_mode=2,validated_work_modes=[0,1,2,3],verify_timeout_ms=600,
            stop_timeout_ms=200,adapter_timeout_ms=600,settle_ms=40)))
        self.process = None
        self.log = open(self.base/'daemon.log','w')
        self.start()
        self.wait(lambda s:s['phase']=='COMMITTED')

    def start(self):
        self.process=subprocess.Popen([str(DAEMON),str(self.config),'--simulation'], stdout=self.log,stderr=self.log)

    def tearDown(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        self.log.close()
        self.temp.cleanup()

    def call(self, request):
        with socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET) as sock:
            sock.settimeout(1)
            sock.connect(str(self.sock))
            sock.send(json.dumps(request).encode())
            return json.loads(sock.recv(8192))

    def wait(self, predicate, timeout=5):
        deadline=time.monotonic()+timeout
        last=None
        while time.monotonic()<deadline:
            if self.process.poll() is not None:raise AssertionError('守护进程提前退出')
            try:
                last=self.call(dict(op='status'))
                if predicate(last):return last
            except (OSError,json.JSONDecodeError):pass
            time.sleep(.02)
        raise AssertionError(last)

    def behavior_set(self, value):self.behavior.write_text(json.dumps(value))

    def test_switch_and_conflict(self):
        self.assertEqual(self.call(dict(op='switch',work_mode=0))['code'],202)
        self.assertEqual(self.call(dict(op='switch',work_mode=3))['code'],409)
        result=self.wait(lambda s:s['phase']=='COMMITTED' and s['actual_mode']==0)
        self.assertEqual(result['confirmed_mode'],0)
        cli=subprocess.run([str(DAEMON.parent/'lidar_mgrctl'),str(self.sock),'status'],capture_output=True,text=True,timeout=2)
        self.assertEqual(cli.returncode,0)
        self.assertEqual(json.loads(cli.stdout)['actual_mode'],0)
        self.assertIn('10.42.0.1',(self.base/'runtime/ys_conf.ini').read_text())
        self.assertIn('work_mode="2"',self.ini.read_text())
        self.assertEqual(self.call(dict(op='switch',work_mode=4))['code'],422)
        self.assertEqual(self.call(dict(op='upgrade'))['code'],501)

    def test_timeout_rolls_back(self):
        self.behavior_set({'0':'timeout'})
        self.call(dict(op='switch',work_mode=0))
        result=self.wait(lambda s:s['phase']=='COMMITTED' and '超时' in s['error'])
        self.assertEqual(result['actual_mode'],2)
        saved=json.loads((self.base/'state/state.json').read_text())
        self.assertEqual(saved['recovery_count'],1)

    def test_stale_and_initialized_only_are_not_verified(self):
        for action in ('stale','initialized_only'):
            self.behavior_set({'0':action})
            self.call(dict(op='switch',work_mode=0))
            self.wait(lambda s:s['phase']=='COMMITTED' and '超时' in s['error'])

    def test_adapter_failure_and_failed_rollback(self):
        self.behavior_set({'adapter_fail':[0,2]})
        self.call(dict(op='switch',work_mode=0))
        self.wait(lambda s:s['phase']=='FAULT')
        self.assertEqual(self.call(dict(op='switch',work_mode=3))['code'],503)

    def test_unexpected_exit_recovers_only_once(self):
        first=self.call(dict(op='status'))['pid']
        os.kill(first,signal.SIGKILL)
        result=self.wait(lambda s:s['phase']=='COMMITTED' and s['pid']!=first)
        os.kill(result['pid'],signal.SIGKILL)
        self.wait(lambda s:s['phase']=='FAULT')

    def test_interrupted_transaction_restores_confirmed_mode(self):
        self.behavior_set({'0':'timeout'})
        self.call(dict(op='switch',work_mode=0))
        self.wait(lambda s:s['phase']=='VERIFYING')
        self.process.kill();self.process.wait()
        self.start()
        result=self.wait(lambda s:s['phase']=='COMMITTED')
        self.assertEqual(result['actual_mode'],2)

    def test_tamper_does_not_stop_current_capture(self):
        old=self.call(dict(op='status'))['pid']
        with (self.package/'bin/you_see').open('a') as f:f.write('\n# 修改\n')
        self.assertEqual(self.call(dict(op='switch',work_mode=0))['code'],422)
        self.assertEqual(self.call(dict(op='status'))['pid'],old)

    def test_singleton(self):
        other=subprocess.run([str(DAEMON),str(self.config),'--simulation'],capture_output=True,timeout=3)
        self.assertNotEqual(other.returncode,0)
        self.assertEqual(self.call(dict(op='status'))['actual_mode'],2)

    def test_production_rejects_unsigned_package(self):
        other=subprocess.run([str(DAEMON),str(self.config)],capture_output=True,timeout=3)
        self.assertNotEqual(other.returncode,0)

    def test_intent_failure_keeps_current_process(self):
        state=self.base/'state/state.json'
        saved=state.read_bytes()
        state.unlink();state.mkdir()
        old=self.call(dict(op='status'))['pid']
        self.assertEqual(self.call(dict(op='switch',work_mode=0))['code'],500)
        result=self.call(dict(op='status'))
        self.assertEqual(result['pid'],old)
        self.assertEqual(result['actual_mode'],2)
        state.rmdir();state.write_bytes(saved)

    def test_production_signature_and_corrupted_state(self):
        self.process.terminate();self.process.wait(timeout=3)
        key=self.base/'key.pem';pub=self.base/'key.pub'
        subprocess.run(['openssl','genpkey','-algorithm','EC','-pkeyopt','ec_paramgen_curve:P-256','-out',str(key)],check=True,capture_output=True)
        subprocess.run(['openssl','pkey','-in',str(key),'-pubout','-out',str(pub)],check=True,capture_output=True)
        subprocess.run(['openssl','dgst','-sha256','-sign',str(key),'-out',str(self.package/'manifest.sig'),str(self.package/'manifest.json')],check=True,capture_output=True)
        config=json.loads(self.config.read_text());config['public_key']=str(pub)
        self.config.write_text(json.dumps(config))
        self.process=subprocess.Popen([str(DAEMON),str(self.config)],stdout=self.log,stderr=self.log)
        status=self.wait(lambda s:s['phase']=='COMMITTED')
        self.assertFalse(status['simulation'])
        self.process.terminate();self.process.wait(timeout=3)
        (self.base/'state/state.json').write_text('损坏状态')
        self.process=subprocess.Popen([str(DAEMON),str(self.config)],stdout=self.log,stderr=self.log)
        self.wait(lambda s:s['phase']=='FAULT')

    def test_interrupted_recovery_and_unknown_phase_fail_closed(self):
        for phase in ('RECOVERING','UNKNOWN'):
            self.process.terminate();self.process.wait(timeout=3)
            state=self.base/'state/state.json'
            state.write_text(json.dumps(dict(schema_version=1,model='CS96-beta',confirmed_mode=2,
                desired_mode=2,phase=phase,recovery_count=1)))
            self.start()
            self.wait(lambda s:s['phase']=='FAULT' and s['pid']==0)

    def test_forced_stop_requires_recover_adapter(self):
        self.behavior_set({'0':'ignore_term'})
        self.call(dict(op='switch',work_mode=0))
        self.wait(lambda s:s['phase']=='COMMITTED' and s['actual_mode']==0)
        self.call(dict(op='switch',work_mode=3))
        self.wait(lambda s:s['phase']=='COMMITTED' and s['actual_mode']==3)
        self.assertIn('recover 3 0',self.adapter_log.read_text())

    def test_network_forwarding_http_and_mode_arbitration(self):
        network_build = DAEMON.parent/'network'
        network_config = self.base/'network-service.json'
        network_socket = self.base/'network.sock'
        network_config.write_text(json.dumps(dict(model='CS96-beta', interface='test0',
            state_path=str(self.base/'network.json'), runtime_dir=str(self.base/'network-runtime'),
            socket_path=str(network_socket), sn_path=str(self.base/'missing-sn'),
            legacy_interfaces=str(self.base/'missing-interfaces'), legacy_flash_interfaces=str(self.base/'missing-flash-interfaces'),
            dhcp_timeout_ms=1000)))
        network_config.chmod(0o600)
        network = subprocess.Popen([str(network_build/'lidar_netd'), str(network_config), '--simulation'], stdout=self.log, stderr=self.log)
        try:
            self.process.terminate(); self.process.wait(timeout=3)
            settings=json.loads(self.config.read_text()); settings['network_socket']=str(network_socket)
            self.config.write_text(json.dumps(settings))
            self.start()
            self.wait(lambda value:value['phase']=='COMMITTED')
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                snapshot=self.call(dict(op='network_get'))
                if snapshot.get('ready'):break
                time.sleep(.02)
            self.assertTrue(snapshot['ready'])
            bridge=self.base/'http-bridge.json'
            bridge.write_text(json.dumps(dict(socket_path=str(self.sock),timeout_ms=1000)))
            bridge.chmod(0o600)
            def http(method='GET', value=None, **env):
                body=json.dumps(value).encode() if value is not None else b''
                settings=dict(os.environ,REQUEST_METHOD=method,QUERY_STRING='',CONTENT_TYPE='application/json',
                              CONTENT_LENGTH=str(len(body)),LIDAR_HTTP_CONFIG=str(bridge),SERVER_ADDR='10.42.0.70')
                settings.update(env)
                result=subprocess.run([str(network_build/'lidar_network.cgi')],input=body,env=settings,capture_output=True,timeout=3)
                self.assertEqual(result.returncode,0,result.stderr)
                header,data=result.stdout.split(b'\r\n\r\n',1)
                return int(header.split()[1]),json.loads(data)
            self.assertEqual(http()[0],200)
            configuration=dict(mode='pool',start='10.42.0.70',end='10.42.0.71',netmask='255.255.255.0')
            self.assertEqual(http('POST',dict(op='network_set',config=configuration,legacy=True))[0],400)
            self.assertEqual(http('POST',dict(op='network_set',config=dict(configuration,netmask='255.0.255.0')))[0],422)
            self.assertEqual(http('POST',dict(op='network_set',config=configuration),HTTP_ORIGIN='http://other',HTTP_HOST='device')[0],403)
            code,accepted=http('POST',dict(op='network_set',config=configuration))
            self.assertEqual(code,202)
            self.assertEqual(self.call(dict(op='switch',work_mode=0))['code'],409)
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                snapshot=self.call(dict(op='network_get'))
                if snapshot['transaction_phase']=='WAIT_CONFIRM':break
                time.sleep(.01)
            self.assertEqual(snapshot['transaction_phase'],'WAIT_CONFIRM')
            self.assertEqual(http('POST',dict(op='network_confirm',operation_id=accepted['operation_id']),SERVER_ADDR=snapshot['address'])[0],200)
            self.behavior_set({'0':'timeout'})
            self.assertEqual(self.call(dict(op='switch',work_mode=0))['code'],202)
            self.assertEqual(http('POST',dict(op='network_set',config=dict(mode='auto')))[0],409)
            self.assertEqual(http()[0],200)
        finally:
            network.terminate();network.wait(timeout=3)

if __name__=='__main__':unittest.main()
