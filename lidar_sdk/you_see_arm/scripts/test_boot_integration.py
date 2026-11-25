#!/usr/bin/env python3
"""用真实可信启动工具、签名包和可选的 BusyBox runit 验证启动集成，不连接设备。"""
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
from argparse import Namespace
from contextlib import redirect_stdout
import io

sys.path.insert(0,str(Path(__file__).resolve().parent))
from integrate_lidar_rootfs import integrate
from prepare_boot_bundle import create

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT/'build/manager-host'
OVERLAY = ROOT/'deploy/runit/rootfs'


class Fixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run(['cmake','-S',str(ROOT/'manager'),'-B',str(BIN),'-G','Ninja'],check=True,stdout=subprocess.DEVNULL)
        subprocess.run(['cmake','--build',str(BIN),'--parallel','4'],check=True,stdout=subprocess.DEVNULL)

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='lb-')
        self.root=Path(self.temp.name)
        self.data=self.root/'data';self.data.mkdir(mode=0o700)
        self.release=self.data/'releases/v1';self.release.mkdir(parents=True)
        (self.release/'bin').mkdir();(self.release/'conf').mkdir()
        shutil.copy2(BIN/'lidar_mgrd',self.release/'bin/lidar_mgrd')
        self.app=self.release/'bin/you_see'
        self.app.write_text(f'''#!{sys.executable}
import json,os,signal,socket,time
signal.signal(signal.SIGTERM,lambda *_:exit(0))
s=socket.socket(fileno=int(os.environ['LIDAR_MGR_FD']))
for event in ('INITIALIZED','PIPELINE_PROGRESS'):
 if event == 'PIPELINE_PROGRESS' and os.path.exists({str(self.root/'no-pipeline-progress')!r}): continue
 s.send(json.dumps(dict(event=event,token=os.environ['LIDAR_MGR_TOKEN'],work_mode=int(os.environ['LIDAR_WORK_MODE']),valid_packets=10)).encode())
while True: time.sleep(.02)
''')
        self.app.chmod(0o700)
        (self.release/'conf/ys_conf.ini').write_text('[system]\nwork_mode="2"\necho_mode="0"\necho_type="1"\ncoordinate_mode="0"\nlog_file_path="/tmp/unwanted.log"\n[network]\ncenter_ip="10.42.0.145"\n')
        self.adapter=self.root/'adapter'
        self.adapter.write_text('#!/bin/sh\nexit 0\n');self.adapter.chmod(0o700)
        self.public=self.root/'public.pem';self.key=self.root/'private.pem'
        for command in (
            ['openssl','genpkey','-algorithm','EC','-pkeyopt','ec_paramgen_curve:P-256','-out',str(self.key)],
            ['openssl','pkey','-in',str(self.key),'-pubout','-out',str(self.public)],
        ): subprocess.run(command,check=True,capture_output=True)
        self.public.chmod(0o600)
        self.crypto=self.root/'lidar_crypto'
        shutil.copy2(BIN/'lidar_crypto',self.crypto);self.crypto.chmod(0o700)
        self.sign()
        (self.data/'current').symlink_to('releases/v1')
        self.legacy=self.root/'factory';self.legacy.mkdir()
        shutil.copy2(self.release/'conf/ys_conf.ini',self.legacy/'ys_conf.ini')
        for name in ('ys_calibration.ini','ys_encoder_angle.csv'):
            (self.legacy/name).write_text('本机标定，不随软件版本更新\n')
        self.runtime=self.root/'run';self.scan=self.root/'services'
        self.config=self.root/'boot.json'
        self.settings=dict(model='CS96-beta',data_root=str(self.data),release_root=str(self.data/'releases'),
            package_dir=str(self.data/'current'),public_key=str(self.public),openssl=str(self.crypto),
            adapter=str(self.adapter),boot_runtime=str(self.runtime),runtime_dir=str(self.runtime/'manager'),
            socket_path=str(self.runtime/'control.sock'),state_dir=str(self.data/'state'),
            base_config=str(self.data/'config/ys_conf.ini'),user_settings=str(self.data/'config/user-settings.json'),
            calibration_dir=str(self.data/'calibration'),legacy_config_dir=str(self.legacy),initial_work_mode=2,
            validated_work_modes=[1,2],disable_file_log=True,verify_timeout_ms=1000,stop_timeout_ms=300,
            adapter_timeout_ms=1000,settle_ms=30)
        self.write_config()
        self.process=None
        self.log=(self.root/'test.log').open('w')

    def sign(self):
        files={str(p.relative_to(self.release)):dict(size=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
               for p in self.release.rglob('*') if p.is_file() and p.name not in ('manifest.json','manifest.sig')}
        (self.release/'manifest.json').write_text(json.dumps(dict(schema_version=1,model='CS96-beta',supported_modes=['pointcloud'],files=files)))
        subprocess.run(['openssl','dgst','-sha256','-sign',str(self.key),'-out',str(self.release/'manifest.sig'),
                        str(self.release/'manifest.json')],check=True,capture_output=True)
        # 测试主机可能采用 umask 0002；发布目录必须符合设备端的可信权限要求。
        (self.data/'releases').chmod(0o700)
        for path in self.release.rglob('*'):
            path.chmod(0o700 if path.is_dir() or path.stat().st_mode & 0o111 else 0o600)
        self.release.chmod(0o700)

    def write_config(self):
        self.config.write_text(json.dumps(self.settings));self.config.chmod(0o600)

    def boot(self, action='prepare'):
        return subprocess.run([str(BIN/'lidar_boot'),action,str(self.config)],capture_output=True,text=True,timeout=10)

    def call(self, value=None):
        with socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET) as sock:
            sock.settimeout(1)
            sock.connect(self.settings['socket_path'])
            sock.send(json.dumps(value or {'op':'status'}).encode())
            return json.loads(sock.recv(8192))

    def wait(self, predicate, timeout=8):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            try:
                status=self.call()
                if predicate(status):return status
            except (OSError,ValueError):pass
            time.sleep(.03)
        self.fail('状态未达到预期：'+(self.root/'test.log').read_text()[-3000:])

    def launch(self):
        self.process=subprocess.Popen([str(BIN/'lidar_boot'),'run',str(self.config)],stdout=self.log,stderr=self.log)
        return self.wait(lambda s:s['phase']=='COMMITTED')

    def stop(self):
        if self.process and self.process.poll() is None:
            self.process.terminate();self.process.wait(timeout=5)

    def tearDown(self):
        self.stop()
        self.log.close();self.temp.cleanup()


class BootTests(Fixture):
    def test_package_path_is_verified_and_does_not_seed_state(self):
        result=self.boot('package-path')
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(result.stdout.strip(),str(self.release))
        self.assertFalse(self.runtime.exists())
        self.app.write_text(self.app.read_text()+'\n# 改动后不能向 Web 返回路径\n')
        result=self.boot('package-path')
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(result.stdout,'')

    def test_verified_boot_and_persistent_mode(self):
        first=self.launch();self.assertFalse(first['simulation'])
        self.assertEqual(self.call({'op':'switch','work_mode':1})['code'],202)
        self.wait(lambda s:s['phase']=='COMMITTED' and s['actual_mode']==1)
        self.stop()
        self.assertEqual(json.loads((self.data/'state/state.json').read_text())['phase'],'STOPPED')
        self.assertEqual(self.launch()['actual_mode'],1)
        cfg=json.loads((self.runtime/'manager.json').read_text())
        self.assertEqual(cfg['package_dir'],str(self.release))

    def test_tamper_rejected_before_execution(self):
        self.app.write_text(self.app.read_text()+'\n# 修改\n')
        self.assertNotEqual(self.boot('run').returncode,0)
        self.assertFalse((self.data/'state/state.json').exists())

    def test_wrong_key_model_and_symlink_are_rejected(self):
        public=self.public.read_bytes()
        self.public.write_text('无效公钥')
        self.assertNotEqual(self.boot().returncode,0)
        self.public.write_bytes(public)
        self.settings['model']='CS96-gamma';self.write_config()
        self.assertNotEqual(self.boot().returncode,0)
        self.settings['model']='CS96-beta';self.write_config()
        self.app.unlink();self.app.symlink_to('/bin/true')
        self.assertNotEqual(self.boot().returncode,0)

    def test_parent_traversal_in_boot_paths_is_rejected(self):
        self.settings['base_config']=str(self.data/'config/../../outside.ini');self.write_config()
        self.assertNotEqual(self.boot().returncode,0)

    def test_seed_preserves_device_configuration(self):
        self.assertEqual(self.boot().returncode,0)
        base=self.data/'config/ys_conf.ini'
        base.write_text(base.read_text().replace('10.42.0.145','10.42.0.123'))
        (self.data/'calibration/ys_encoder_angle.csv').write_text('已有本机标定\n')
        self.assertEqual(self.boot().returncode,0)
        self.assertIn('10.42.0.123',base.read_text())
        self.assertEqual((self.data/'calibration/ys_encoder_angle.csv').read_text(),'已有本机标定\n')

    def test_user_parameters_survive_runtime_deletion(self):
        self.assertEqual(self.boot().returncode,0)
        path=self.data/'config/user-settings.json'
        path.write_text(json.dumps(dict(schema_version=1,echo_mode=1,echo_type=0,coordinate_mode=1)))
        path.chmod(0o600)
        self.launch()
        ini=(self.runtime/'manager/ys_conf.ini').read_text()
        self.assertIn('echo_mode',ini);self.assertNotIn('log_file_path',ini)
        self.assertRegex(ini,r'coordinate_mode\s*=\s*"1"')
        self.stop();shutil.rmtree(self.runtime)
        self.launch()
        self.assertRegex((self.runtime/'manager/ys_conf.ini').read_text(),r'echo_mode\s*=\s*"1"')

    def test_fault_is_api_ready_without_restart(self):
        self.assertEqual(self.boot().returncode,0)
        (self.data/'state/state.json').write_text('损坏状态')
        self.process=subprocess.Popen([str(BIN/'lidar_boot'),'run',str(self.config)],stdout=self.log,stderr=self.log)
        self.wait(lambda s:s['phase']=='FAULT')
        self.assertEqual(self.boot('ready').returncode,0)
        self.assertIsNone(self.process.poll())

    def test_bounded_tool_timeout(self):
        result=subprocess.run([str(BIN/'lidar_boot'),'bounded','1','/bin/sleep','5'],timeout=3)
        self.assertEqual(result.returncode,124)

    def test_completed_mode_survives_repeated_system_boots(self):
        self.launch()
        for unused in range(2):
            self.stop()
            path=self.data/'state/state.json'
            saved=json.loads(path.read_text())
            saved.update(phase='COMMITTED',recovery_count=1,boot_id='00000000-0000-0000-0000-000000000000')
            path.write_text(json.dumps(saved))
            self.assertEqual(self.launch()['actual_mode'],2)
            self.assertEqual(json.loads(path.read_text())['recovery_count'],0)

    def test_new_boot_does_not_clear_interrupted_recovery_or_fault(self):
        self.assertEqual(self.boot().returncode,0)
        for phase in ('RECOVERING','FAULT'):
            path=self.data/'state/state.json'
            path.write_text(json.dumps(dict(schema_version=1,model='CS96-beta',confirmed_mode=2,
                desired_mode=2,phase=phase,recovery_count=1,boot_id='00000000-0000-0000-0000-000000000000')))
            self.process=subprocess.Popen([str(BIN/'lidar_boot'),'run',str(self.config)],stdout=self.log,stderr=self.log)
            self.wait(lambda s:s['phase']=='FAULT' and s['pid']==0)
            self.stop()

    def test_malformed_boot_identifier_is_not_a_fresh_boot(self):
        self.launch();self.stop()
        path=self.data/'state/state.json'
        saved=json.loads(path.read_text())
        saved.update(phase='COMMITTED',recovery_count=1,boot_id='x'*36)
        path.write_text(json.dumps(saved))
        self.process=subprocess.Popen([str(BIN/'lidar_boot'),'run',str(self.config)],stdout=self.log,stderr=self.log)
        self.wait(lambda s:s['phase']=='FAULT' and s['pid']==0)


class FlashCopyTests(Fixture):
    def setUp(self):
        super().setUp()
        self.medium = self.root/'medium'
        self.medium.mkdir()
        self.data.rename(self.medium/'lidar')
        (self.medium/'you_see_arm').mkdir()
        self.legacy.rename(self.medium/'you_see_arm/config')
        self.mountpoint = self.root/'mount'
        self.mounts = self.root/'mounts'
        self.mounts.touch()
        self.commands = self.root/'commands'
        self.commands.mkdir()
        self.calls = self.root/'calls'
        self.storage = OVERLAY/'usr/libexec/lidar/lidar-storage'
        self.service = self.root/'service.conf'
        self.snapshot = self.root/'snapshot'
        self.data = self.snapshot/'lidar'
        self.settings.update(data_root=str(self.data), release_root=str(self.data/'releases'),
            package_dir=str(self.data/'current'), state_dir=str(self.data/'state'),
            base_config=str(self.data/'config/ys_conf.ini'), user_settings=str(self.data/'config/user-settings.json'),
            calibration_dir=str(self.data/'calibration'), legacy_config_dir=str(self.snapshot/'legacy-config'))
        self.write_config()
        config = dict(LIDAR_STORAGE_MODE='boot-copy', LIDAR_STORAGE_DEVICE='/dev/test-only',
            LIDAR_STORAGE_MOUNT=self.mountpoint, LIDAR_STORAGE_MOUNTS=self.mounts, LIDAR_RAM_DATA=self.snapshot,
            LIDAR_BOOT=BIN/'lidar_boot')
        self.service.write_text('\n'.join(f"{key}='{value}'" for key, value in config.items())+'\n')
        self.env = dict(os.environ, LIDAR_SERVICE_CONFIG=str(self.service), LIDAR_PERSIST_COMMAND=str(self.storage),
            PATH=str(self.commands)+os.pathsep+os.environ['PATH'], TEST_STORAGE=str(self.root))
        for tool, source in (
            ('mount', '''
printf '%s\n' "$4" >> "$TEST_STORAGE/calls"
case "$4" in remount,*) exit 0 ;; esac
mv "$6" "$TEST_STORAGE/empty"
mv "$TEST_STORAGE/medium" "$6"
printf '%s %s jffs2 %s 0 0\n' "$5" "$6" "$4" > "$TEST_STORAGE/mounts"
'''),
            ('umount', '''
printf 'umount\n' >> "$TEST_STORAGE/calls"
[ ! -e "$TEST_STORAGE/fail-unmount" ] || exit 1
mv "$1" "$TEST_STORAGE/medium"
mv "$TEST_STORAGE/empty" "$1"
: > "$TEST_STORAGE/mounts"
''')):
            executable = self.commands/tool
            executable.write_text('#!/bin/sh\nset -eu\n'+source)
            executable.chmod(0o700)

    def storage_call(self, *args):
        return subprocess.run([str(self.storage), *args], env=self.env, capture_output=True, text=True, timeout=10)

    def launch_from_ram(self):
        copied = self.storage_call('start')
        self.assertEqual(copied.returncode, 0, copied.stderr)
        self.assertEqual(self.mounts.read_text(), '')
        self.assertFalse((self.mountpoint/'lidar').exists())
        self.process = subprocess.Popen([str(BIN/'lidar_boot'), 'run', str(self.config)],
                                      env=self.env, stdout=self.log, stderr=self.log)

    def start_from_ram(self):
        self.launch_from_ram()
        return self.wait(lambda status: status['phase'] == 'COMMITTED')

    def test_boot_unmounts_and_confirmed_mode_survives_reloading_ram(self):
        self.assertEqual(self.start_from_ram()['confirmed_mode'], 2)
        self.assertEqual(self.calls.read_text().splitlines(), ['ro,nodev,nosuid,noexec', 'umount'])
        for name in ('ys_calibration.ini', 'ys_encoder_angle.csv'):
            self.assertEqual((self.data/'calibration'/name).read_text(),
                             (self.medium/'you_see_arm/config'/name).read_text())
        self.assertEqual(self.call(dict(op='switch', work_mode=1))['code'], 202)
        self.wait(lambda status: status['phase'] == 'COMMITTED' and status['confirmed_mode'] == 1)
        saved = json.loads((self.medium/'lidar/state/state.json').read_text())
        self.assertEqual(saved['confirmed_mode'], 1)
        self.assertEqual(self.calls.read_text().splitlines(),
                         ['ro,nodev,nosuid,noexec', 'umount', 'rw,nodev,nosuid,noexec', 'umount'])
        self.stop()
        shutil.rmtree(self.snapshot)
        self.assertEqual(self.start_from_ram()['confirmed_mode'], 1)
        self.assertEqual(self.calls.read_text().splitlines()[-2:], ['ro,nodev,nosuid,noexec', 'umount'])

    def test_first_verification_timeout_can_commit_on_next_boot(self):
        no_data = self.root/'no-pipeline-progress'
        no_data.touch()
        self.launch_from_ram()
        first = self.wait(lambda status: status['phase'] == 'FAULT' and status['pid'] == 0)
        self.assertEqual(first['error'], '数据验证超时')
        flash_state = self.medium/'lidar/state/state.json'
        saved = json.loads(flash_state.read_text())
        self.assertEqual(saved['fault_code'], 'verification_timeout')
        self.assertEqual(saved['confirmed_mode'], -1)
        self.stop()

        no_data.unlink()
        self.launch_from_ram()
        self.wait(lambda status: status['phase'] == 'FAULT' and status['pid'] == 0)
        self.assertEqual(json.loads((self.data/'state/state.json').read_text())['fault_code'], 'verification_timeout')
        self.assertEqual(json.loads(flash_state.read_text()), saved)
        self.stop()

        flash_state.write_text(json.dumps(dict(saved, boot_id='00000000-0000-0000-0000-000000000000')))
        shutil.rmtree(self.snapshot)
        self.assertEqual(self.start_from_ram()['actual_mode'], 2)
        committed = json.loads(flash_state.read_text())
        self.assertEqual(committed['phase'], 'COMMITTED')
        self.assertEqual(committed['confirmed_mode'], 2)
        self.assertEqual(committed['boot_id'], saved['boot_id'])
        self.assertNotIn('fault_code', committed)

    def test_timeout_retry_prefers_confirmed_mode_and_uses_one_attempt(self):
        (self.root/'no-pipeline-progress').touch()
        flash_state = self.medium/'lidar/state/state.json'
        flash_state.parent.mkdir(mode=0o700)
        flash_state.write_text(json.dumps(dict(schema_version=1, model='CS96-beta', confirmed_mode=1,
            desired_mode=2, phase='FAULT', recovery_count=1, fault_code='verification_timeout',
            boot_id='00000000-0000-0000-0000-000000000000')))
        flash_state.chmod(0o600)
        adapter_log = self.root/'adapter.log'
        self.adapter.write_text(f'#!/bin/sh\nprintf "%s\\n" "$*" >> "{adapter_log}"\n')
        self.launch_from_ram()
        fault = self.wait(lambda status: status['phase'] == 'FAULT' and status['pid'] == 0)
        self.assertEqual(fault['error'], '数据验证超时')
        self.assertEqual(fault['desired_mode'], 1)
        saved = json.loads(flash_state.read_text())
        self.assertEqual(saved['fault_code'], 'verification_timeout')
        self.assertEqual(saved['recovery_count'], 1)
        self.stop()

        self.launch_from_ram()
        self.wait(lambda status: status['phase'] == 'FAULT' and status['pid'] == 0)
        self.assertEqual(adapter_log.read_text().splitlines(), ['recover 1 1'])

    def test_failed_save_does_not_follow_links_and_unmounts(self):
        self.assertEqual(self.storage_call('start').returncode, 0)
        target = self.medium/'lidar/calibration'
        target.mkdir()
        original = self.root/'original.ini'
        original.write_text('原始标定\n')
        (target/'ys_calibration.ini').symlink_to(original)
        candidate = self.root/'new.ini'
        candidate.write_text('新标定\n')
        candidate.chmod(0o600)
        result = self.storage_call('save', 'calibration', str(candidate))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.mounts.read_text(), '')
        self.assertEqual(original.read_text(), '原始标定\n')
        self.assertEqual(self.calls.read_text().splitlines()[-2:], ['rw,nodev,nosuid,noexec', 'umount'])

    def test_saved_configuration_and_calibration_are_copied_at_next_boot(self):
        self.assertEqual(self.storage_call('start').returncode, 0)
        for kind, path in (
            ('base-config', 'config/ys_conf.ini'),
            ('calibration', 'calibration/ys_calibration.ini'),
            ('encoder-angle', 'calibration/ys_encoder_angle.csv'),
            ('user-settings', 'config/user-settings.json'),
            ('network', 'config/network.json'),
        ):
            candidate = self.root/kind
            candidate.write_text(kind+'\n')
            candidate.chmod(0o600)
            result = self.storage_call('save', kind, str(candidate))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(self.mounts.read_text(), '')
            self.assertEqual((self.medium/'lidar'/path).read_text(), kind+'\n')
            self.assertFalse((self.data/path).exists())
        shutil.rmtree(self.snapshot)
        self.assertEqual(self.storage_call('start').returncode, 0)
        self.assertEqual((self.data/'calibration/ys_encoder_angle.csv').read_text(), 'encoder-angle\n')
        self.assertEqual((self.data/'config/network.json').read_text(), 'network\n')

    def test_failed_boot_unmount_does_not_publish_the_ram_copy(self):
        (self.root/'fail-unmount').touch()
        result = self.storage_call('start')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.snapshot.exists())
        self.assertFalse(list(self.root.glob('snapshot.tmp.*')))
        self.assertIn('remount,ro,nodev,nosuid,noexec', self.calls.read_text().splitlines())


class RunitTests(Fixture):
    @classmethod
    def setUpClass(cls):
        cls.tools=Path(os.environ.get('LIDAR_TEST_RUNIT_BIN','/usr/bin'))
        if not all((cls.tools/name).exists() for name in ('runsv','runsvdir','sv')):
            raise unittest.SkipTest('需要真实 runit 工具，请设置 LIDAR_TEST_RUNIT_BIN；不使用模拟监督器代替')
        super().setUpClass()

    def setUp(self):
        super().setUp()
        self.control=OVERLAY/'usr/libexec/lidar/lidar-service'
        self.environment=os.environ.copy()
        self.environment['PATH']=str(self.tools)+os.pathsep+self.environment['PATH']
        self.environment['LIDAR_SERVICE_EXEC']=str(self.control)
        self.environment['LIDAR_SERVICE_CONFIG']=str(self.root/'service.conf')
        self.logger=self.root/'logger'
        self.logger.write_text('#!/bin/sh\nexec cat >> '+str(self.root/'service.log')+'\n');self.logger.chmod(0o700)
        values=dict(LIDAR_RUN=self.runtime,LIDAR_SCAN=self.scan,
            LIDAR_TEMPLATE=OVERLAY/'usr/share/lidar/runit/lidar_mgrd',LIDAR_BOOT_CONFIG=self.config,
            LIDAR_BOOT=BIN/'lidar_boot',LIDAR_RUNSV_DIR=self.tools/'runsvdir',LIDAR_SV=self.tools/'sv',
            LIDAR_LOGGER=self.logger,LIDAR_MAX_FAILURES=3,LIDAR_STABLE_SECONDS=60,LIDAR_RETRY_SECONDS=0,
            LIDAR_STOP_SECONDS=5)
        (self.root/'service.conf').write_text('\n'.join(f"{key}='{value}'" for key,value in values.items())+'\n')
        self.supervisor=None

    def command(self, action, check=True):
        return subprocess.run(['/bin/sh',str(self.control),action],env=self.environment,
                              capture_output=True,text=True,timeout=10,check=check)

    def supervise(self):
        self.command('enable')
        self.supervisor=subprocess.Popen(['/bin/sh',str(self.control),'scan'],env=self.environment,
                                         stdout=self.log,stderr=self.log,start_new_session=True)

    def tearDown(self):
        if self.supervisor:
            self.command('stop',check=False)
            # 只清理本测试自己创建的会话，避免遗留监督器与日志进程。
            try:os.killpg(self.supervisor.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            try:self.supervisor.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(self.supervisor.pid,signal.SIGKILL);self.supervisor.wait()
        super().tearDown()

    def test_start_stop_singleton_and_mode_restore(self):
        self.supervise();first=self.wait(lambda s:s['phase']=='COMMITTED')
        self.command('start');self.assertEqual(self.call()['pid'],first['pid'])
        self.call({'op':'switch','work_mode':1})
        self.wait(lambda s:s['phase']=='COMMITTED' and s['actual_mode']==1)
        self.command('stop');time.sleep(.3)
        with self.assertRaises(OSError):self.call()
        self.command('start')
        self.assertEqual(self.wait(lambda s:s['phase']=='COMMITTED')['actual_mode'],1)
        self.command('stop');self.command('enable')
        self.assertEqual(self.wait(lambda s:s['phase']=='COMMITTED')['actual_mode'],1)

    def test_manager_crash_is_supervised(self):
        self.supervise();first=self.wait(lambda s:s['phase']=='COMMITTED')
        manager=int((self.scan/'lidar_mgrd/supervise/pid').read_text())
        os.kill(manager,signal.SIGKILL)
        status=self.wait(lambda s:s['phase']=='COMMITTED' and s['pid']!=first['pid'])
        self.assertEqual(status['actual_mode'],2)

    def test_invalid_package_latches_after_three_attempts(self):
        self.app.write_text(self.app.read_text()+'\n# 修改\n')
        self.supervise()
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and not (self.runtime/'service-error').exists():time.sleep(.05)
        self.assertTrue((self.runtime/'service-error').exists())
        self.assertTrue((self.scan/'lidar_mgrd/down').exists())
        self.assertEqual((self.runtime/'failures').read_text().strip(),'3')
        time.sleep(.3)
        self.assertEqual((self.runtime/'failures').read_text().strip(),'3')
        self.assertFalse((self.data/'state/state.json').exists())

    def test_fault_remains_queryable_without_supervisor_restart(self):
        self.assertEqual(self.boot().returncode,0)
        (self.data/'state/state.json').write_text('损坏状态')
        self.supervise();self.wait(lambda s:s['phase']=='FAULT')
        pid=(self.scan/'lidar_mgrd/supervise/pid').read_text()
        self.command('start');time.sleep(.2)
        self.assertEqual(pid,(self.scan/'lidar_mgrd/supervise/pid').read_text())
        self.assertEqual(self.call()['phase'],'FAULT')


class WebPreparationTests(Fixture):
    def setUp(self):
        super().setUp()
        self.web=self.release/'www'
        for name in ('sbin','config','webpages'):
            (self.web/name).mkdir(parents=True)
        (self.web/'sbin/lighttpd').write_text('#!/bin/sh\nexit 0\n')
        (self.web/'sbin/lighttpd').chmod(0o755)
        (self.web/'config/lighttpd.conf').write_text('业务 Web 配置\n')
        (self.web/'webpages/index.html').write_text('业务页面\n')
        self.sign()
        self.recovery=self.root/'recovery'
        shutil.copytree(self.web,self.recovery)
        (self.recovery/'webpages/index.html').write_text('系统诊断页面\n')
        self.web_run=self.root/'web-run'
        self.service=self.root/'web-service.conf'
        self.service.write_text('\n'.join(f"{key}='{value}'" for key,value in dict(
            LIDAR_WEB_SOURCE='release',LIDAR_WEB_RUN=self.web_run,LIDAR_WEB_RECOVERY=self.recovery,
            LIDAR_BOOT=BIN/'lidar_boot',LIDAR_BOOT_CONFIG=self.config).items())+'\n')
        self.environment=dict(os.environ,LIDAR_SERVICE_CONFIG=str(self.service))

    def prepare_web(self):
        return subprocess.run(['/bin/sh',str(OVERLAY/'usr/libexec/lidar/lidar-web'),'prepare'],
                              env=self.environment,capture_output=True,text=True,timeout=10)

    def test_signed_web_copies_to_ram_and_preserves_package(self):
        result=self.prepare_web()
        self.assertEqual(result.returncode,0,result.stderr)
        current=self.web_run/'current'
        self.assertEqual((current/'webpages/index.html').read_text(),'业务页面\n')
        (current/'log/runtime.log').write_text('运行日志\n')
        self.assertFalse((self.web/'log/runtime.log').exists())
        old=current.resolve()
        self.assertEqual(self.prepare_web().returncode,0)
        self.assertFalse(old.exists())
        self.assertFalse((current/'log/runtime.log').exists())
        self.assertEqual(self.boot('verify').returncode,0)

    def test_corrupt_web_uses_system_recovery(self):
        (self.web/'webpages/index.html').write_text('未签名的修改\n')
        result=self.prepare_web()
        self.assertEqual(result.returncode,0,result.stderr)
        current=self.web_run/'current'
        self.assertEqual((current/'webpages/index.html').read_text(),'系统诊断页面\n')
        self.assertIn('RECOVERY',(current/'启动状态.txt').read_text())
        self.assertTrue((self.web_run/'verify-error').read_text())

    def test_running_web_prevents_switching_runtime_copy(self):
        self.assertEqual(self.prepare_web().returncode,0)
        current=self.web_run/'current'
        old=current.resolve()
        (current/'www.pid').write_text(str(os.getpid())+'\n')
        self.assertNotEqual(self.prepare_web().returncode,0)
        self.assertEqual(current.resolve(),old)

    def test_external_runtime_link_is_rejected(self):
        self.web_run.mkdir()
        (self.web_run/'current').symlink_to(self.root)
        self.assertNotEqual(self.prepare_web().returncode,0)


class BundleTests(Fixture):
    def setUp(self):
        super().setUp()
        for name in ('lidar_boot','lidar_crypto','lidar_mgrctl'):
            shutil.copy2(BIN/name,self.release/'bin'/name)
        shutil.copy2(BIN/'lidar_mgrctl',self.app)
        (self.release/'lib').mkdir()
        shutil.copy2(BIN/'lidar_crypto',self.release/'lib/liblidar.so')
        self.sign()
        self.module=self.root/'module.ko'
        subprocess.run(['gcc','-x','c','-Wl,--build-id','-o',str(self.module),'-'],
                       input='int main(void){return 0;}\n',text=True,check=True,capture_output=True)
        self.cgi=self.root/'cgi';self.cgi.mkdir()
        for name in ('lidar_status.cgi','lidar_switch.cgi'):
            shutil.copy2(BIN/'lidar_mgrctl',self.cgi/name)
        self.output=self.root/'bundle'
        self.args=Namespace(application=self.release,public_key=self.public,kernel_module=self.module,
                motor_init=self.module,sn_provisioner=self.module,cgi_dir=self.cgi,www_root=None,release_id=None,output=self.output)
        with redirect_stdout(io.StringIO()):
            create(self.args,target_machine=62)

    def target(self):
        target=self.root/'target'
        (target/'etc/init.d').mkdir(parents=True)
        (target/'etc/inittab').write_text('::sysinit:/etc/init.d/rcS\nconsole::respawn:/sbin/getty console\n')
        (target/'etc/init.d/S99user').write_text('#!/bin/sh\n/root/you_see &\n')
        (target/'www/sbin').mkdir(parents=True);(target/'www/lib').mkdir()
        (target/'www/config').mkdir()
        (target/'www/sbin/lighttpd').write_text('离线镜像内已有 Web 程序\n')
        (target/'www/config/lighttpd.conf').write_text('server.port = 80\n')
        return target

    def test_bundle_contains_boot_tools_and_root_owned_seed(self):
        import tarfile
        overlay=self.output/'rootfs-overlay'
        self.assertTrue(os.access(overlay/'usr/libexec/lidar/lidar-service',os.X_OK))
        self.assertEqual(os.readlink(overlay/'etc/ys_calibration.ini'),
                         '/run/lidar-data/lidar/calibration/ys_calibration.ini')
        self.assertEqual((overlay/'etc/lidar/release-public.pem').stat().st_mode & 0o777,0o600)
        with tarfile.open(self.output/'flash-seed.tar.gz') as archive:
            self.assertTrue(all(info.uid==0 and info.gid==0 for info in archive))

    def test_integration_preserves_console_and_disables_old_owner(self):
        target=self.target();overlay=self.output/'rootfs-overlay'
        with self.assertRaises(ValueError):integrate(target,overlay)
        with redirect_stdout(io.StringIO()):
            integrate(target,overlay,True)
            integrate(target,overlay)
        self.assertFalse((target/'etc/init.d/S99user').exists())
        self.assertTrue((target/'usr/share/lidar/legacy/S99user').exists())
        text=(target/'etc/inittab').read_text()
        self.assertIn('console::respawn:/sbin/getty console',text)
        self.assertEqual(text.count('::respawn:/usr/libexec/lidar/lidar-service scan'),1)

    def test_release_web_keeps_recovery_and_builds_runtime_link(self):
        target=self.target();overlay=self.output/'rootfs-overlay'
        recovery=overlay/'usr/share/lidar/web-recovery'
        shutil.copytree(overlay/'www',recovery)
        shutil.copytree(target/'www',recovery,dirs_exist_ok=True)
        shutil.rmtree(overlay/'www')
        with (overlay/'etc/lidar/service.conf').open('a') as stream:
            stream.write('\nLIDAR_WEB_SOURCE=release\n')
        with redirect_stdout(io.StringIO()):
            integrate(target,overlay,True)
            integrate(target,overlay)
        self.assertEqual(os.readlink(target/'www'),'/run/lidar-web/current')
        self.assertTrue((target/'usr/share/lidar/legacy/www/config/lighttpd.conf').is_file())
        self.assertTrue((target/'usr/share/lidar/web-recovery/cgi-bin/lidar_status.cgi').is_file())

    def test_external_parent_symlink_and_live_root_are_rejected(self):
        target=self.target();outside=self.root/'outside';outside.mkdir()
        (target/'usr').symlink_to(outside)
        with self.assertRaises(ValueError):integrate(target,self.output/'rootfs-overlay',True)
        self.assertFalse((outside/'libexec').exists())
        with self.assertRaises(ValueError):integrate(Path('/'),self.output/'rootfs-overlay',True)

    def test_arm_bundle_rejects_host_programs(self):
        self.args.output=self.root/'wrong-architecture'
        with self.assertRaises(ValueError):create(self.args)
        self.assertFalse(self.args.output.exists())

    def test_incomplete_template_does_not_remove_legacy_startup(self):
        target=self.target()
        (self.output/'rootfs-overlay/etc/lidar/release-public.pem').unlink()
        with self.assertRaises(ValueError):integrate(target,self.output/'rootfs-overlay',True)
        self.assertTrue((target/'etc/init.d/S99user').exists())

    def test_network_bundle_migrates_only_owned_interface(self):
        network_dir=self.release/'libexec/lidar'
        network_dir.mkdir(parents=True)
        for name in ('lidar_netd','lidar_netctl','lidar_network_status.cgi'):
            shutil.copy2(BIN/'network'/name,network_dir/name)
        (self.release/'www/cgi-bin').mkdir(parents=True)
        shutil.copy2(BIN/'network/lidar_network.cgi',self.release/'www/cgi-bin/lidar_network.cgi')
        self.sign()
        self.args.output=self.root/'network-bundle'
        with redirect_stdout(io.StringIO()):create(self.args,target_machine=62)
        overlay=self.args.output/'rootfs-overlay'
        self.assertIn('LIDAR_NETWORK_ENABLED=1',(overlay/'etc/lidar/service.conf').read_text())
        self.assertIn('network_socket',json.loads((overlay/'etc/lidar/boot.json').read_text()))
        target=self.target()
        original='auto lo eth0 eth1\niface lo inet loopback\niface eth0 inet static\n address 10.42.0.77\n netmask 255.255.255.0\n gateway 10.42.0.1\niface eth1 inet static\n address 192.168.4.2\n netmask 255.255.255.0\n'
        (target/'etc/network').mkdir()
        (target/'etc/network/interfaces').write_text(original)
        for name in ('sbin/udhcpc','usr/sbin/avahi-daemon'):
            tool=target/name;tool.parent.mkdir(parents=True,exist_ok=True);tool.write_text('系统组件\n')
            tool.chmod(0o755)
        avahi_header=target.parent/'build/avahi-0.8/config.h'
        avahi_header.parent.mkdir(parents=True)
        avahi_header.write_text('#define PACKAGE_NAME "avahi"\n/* #undef HAVE_DBUS */\n')
        (target/'etc/init.d/S50avahi-daemon').write_text('旧 Avahi 启动入口\n')
        with redirect_stdout(io.StringIO()):
            integrate(target,overlay,True)
            self.assertFalse(json.loads((target/'etc/lidar/network.json').read_text())['avahi_dbus_supported'])
            avahi_header.write_text('#define PACKAGE_NAME "avahi"\n#define HAVE_DBUS 1\n')
            integrate(target,overlay)
        self.assertTrue(json.loads((target/'etc/lidar/network.json').read_text())['avahi_dbus_supported'])
        updated=(target/'etc/network/interfaces').read_text()
        self.assertNotIn('eth0',updated)
        self.assertIn('iface eth1 inet static',updated)
        self.assertIn('iface lo inet loopback',updated)
        self.assertEqual((target/'etc/lidar/legacy-network.interfaces').read_text(),original)
        self.assertFalse((target/'etc/init.d/S50avahi-daemon').exists())
        self.assertTrue((target/'usr/share/lidar/legacy/S50avahi-daemon').is_file())


class PlatformTests(Fixture):
    def setUp(self):
        super().setUp()
        self.platform=OVERLAY/'usr/libexec/lidar/lidar-platform-beta'
        self.note=self.root/'note'
        self.marker=self.root/'platform'
        self.motor=self.root/'motor'
        self.motor_log=self.root/'motor.log'
        self.motor.write_text(f'''#!/bin/sh
: <<'VERSION'
beta-i2c-motor start-stop v1
VERSION
printf '%s\\n' "${{1:-start}}" >> '{self.motor_log}'
[ "${{1:-}}" != stop ] || [ "${{MOTOR_STOP_FAIL:-0}}" = 0 ]
''')
        self.motor.chmod(0o700)
        self.insmod=self.root/'insmod'
        self.insmod.write_text("#!/bin/sh\nprintf 'module note' > "+str(self.note)+'\n');self.insmod.chmod(0o700)
        self.fuser=self.root/'fuser'
        self.fuser.write_text('#!/bin/sh\nexit 1\n');self.fuser.chmod(0o700)
        self.rmmod=self.root/'rmmod'
        self.rmmod.write_text('#!/bin/sh\nrm '+str(self.note)+'\n');self.rmmod.chmod(0o700)
        self.profile=self.root/'ram/lidar/state/fpga-profile'
        config=self.root/'platform.conf'
        values=dict(LIDAR_BOOT=BIN/'lidar_boot',LIDAR_MODULE_ID=hashlib.sha256(b'module note').hexdigest(),
            LIDAR_MODULE_NOTE=self.note,LIDAR_PLATFORM_STATE=self.marker,LIDAR_MOTOR=self.motor,
            LIDAR_INSMOD=self.insmod,LIDAR_RMMOD=self.rmmod,LIDAR_RAM_DATA=self.root/'ram',
            LIDAR_KERNEL_MODULE=self.root/'unused.ko',LIDAR_FUSER=self.fuser,LIDAR_DEVICE='/dev/null')
        config.write_text('\n'.join(f"{k}='{v}'" for k,v in values.items())+'\n')
        self.env=dict(os.environ,LIDAR_SERVICE_CONFIG=str(config))

    def invoke(self,mode=2):
        return subprocess.run(['/bin/sh',str(self.platform),'recover',str(mode),'-1'],
                              env=self.env,capture_output=True,text=True,timeout=5)

    def quiesce(self):
        return subprocess.run(['/bin/sh', str(self.platform), 'quiesce'], env=self.env,
                              capture_output=True, text=True, timeout=5)

    def test_cold_initialization_once_then_fixed_layout_reopen(self):
        self.assertEqual(self.invoke().returncode,0)
        self.assertEqual(self.invoke(1).returncode,0)
        self.assertEqual(self.motor_log.read_text(),'start\n')
        self.note.write_text('错误模块')
        self.assertNotEqual(self.invoke().returncode,0)

    def test_unmanaged_driver_and_unvalidated_mode_are_rejected(self):
        self.assertNotEqual(self.invoke(0).returncode,0)
        self.note.write_text('module note')
        self.assertNotEqual(self.invoke().returncode,0)
        self.assertFalse(self.motor_log.exists())

    def test_cold_histogram_stops_motor_before_loading_driver(self):
        self.env.update(LIDAR_FPGA_ACTIVE_MODE='0', MOTOR_STOP_FAIL='1')
        self.assertNotEqual(self.invoke(0).returncode, 0)
        self.assertFalse(self.note.exists())
        self.env.pop('MOTOR_STOP_FAIL')
        self.assertEqual(self.invoke(0).returncode, 0)
        self.assertEqual(self.motor_log.read_text(), 'stop\nstop\n')

    def test_cold_gray_rejects_old_motor_binary_without_running_it(self):
        self.env['LIDAR_FPGA_ACTIVE_MODE'] = '3'
        motor = self.motor.read_text()
        self.motor.write_text(motor.replace('beta-i2c-motor start-stop v1\n', ''))
        result = self.invoke(3)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('需要支持停转的 Beta 电机程序', result.stderr)
        self.assertFalse(self.motor_log.exists())
        self.assertFalse(self.note.exists())
        self.motor.write_text(motor)
        self.assertEqual(self.invoke(3).returncode, 0)
        self.assertEqual(self.motor_log.read_text(), 'stop\n')

    def test_spot_starts_motor_without_loading_capture_module(self):
        self.assertNotEqual(self.invoke(4).returncode, 0)
        self.env['LIDAR_FPGA_ACTIVE_MODE'] = '4'
        self.assertEqual(self.invoke(4).returncode, 0)
        self.assertEqual(self.invoke(4).returncode, 0)
        self.assertFalse(self.note.exists())
        self.assertFalse((self.marker/'initialized').exists())
        self.assertEqual(self.motor_log.read_text(), 'start\n')
        self.assertEqual(self.quiesce().returncode, 0)
        self.assertEqual(self.motor_log.read_text(), 'start\nstop\n')

    def test_fpga_detected_layout_stops_and_restarts_motor_across_reload(self):
        self.assertEqual(self.invoke(2).returncode, 0)
        self.env['MOTOR_STOP_FAIL'] = '1'
        self.assertNotEqual(self.quiesce().returncode, 0)
        self.assertTrue(self.note.exists())
        self.env.pop('MOTOR_STOP_FAIL')
        self.assertEqual(self.quiesce().returncode, 0)
        self.assertFalse(self.note.exists())
        self.assertTrue((self.marker/'reload').is_file())
        self.profile.parent.mkdir(parents=True)
        self.profile.write_text('pending 2 '+'1'*64+'\n')
        self.assertNotEqual(self.invoke(3).returncode, 0)
        self.env['LIDAR_FPGA_ACTIVE_MODE'] = '3'
        self.assertEqual(self.invoke(3).returncode, 0)
        self.env.pop('LIDAR_FPGA_ACTIVE_MODE')
        self.assertEqual(self.motor_log.read_text(), 'start\nstop\nstop\nstop\n')
        self.assertFalse((self.marker/'reload').exists())
        self.profile.write_text('3 '+'2'*64+'\n')
        self.assertEqual(self.invoke(3).returncode, 0)
        self.assertNotEqual(self.invoke(1).returncode, 0)
        self.assertEqual(self.quiesce().returncode, 0)
        self.env['LIDAR_FPGA_ACTIVE_MODE'] = '2'
        self.assertEqual(self.invoke(1).returncode, 0)
        self.assertEqual(self.invoke(2).returncode, 0)
        self.assertEqual(self.motor_log.read_text(), 'start\n' + 'stop\n' * 5 + 'start\n')


if __name__=='__main__':
    unittest.main()
