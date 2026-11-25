#!/usr/bin/env python3
"""运行真实网络守护进程的隔离集成测试；模拟后端不打开或修改任何主机网卡。"""
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / 'build/network-host'


class NetworkTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run(['cmake', '-S', str(ROOT/'network'), '-B', str(BUILD), '-G', 'Ninja'], check=True, stdout=subprocess.DEVNULL)
        subprocess.run(['cmake', '--build', str(BUILD)], check=True, stdout=subprocess.DEVNULL)
        subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', str(ROOT/'scripts/fixtures/network_rules.c'),
                        str(BUILD/'liblidar_network.a'), str(ROOT/'third_party/cJSON/cJSON.c'), '-lm',
                        '-o', str(BUILD/'network_rules')], check=True)
        subprocess.run([str(BUILD/'network_rules')], check=True)
        subprocess.run(['cc', '-std=gnu11', '-DCS96_BETA=1', '-DLOCAL_TEST=0',
                        '-ffunction-sections', '-fdata-sections', '-I'+str(ROOT/'driver/inc'),
                        '-I'+str(ROOT/'third_party/libuv/include'), str(ROOT/'scripts/fixtures/network_udp_receive.c'),
                        '-Wl,--gc-sections', '-luv', '-pthread', '-lm', '-ldl',
                        '-o', str(BUILD/'network_udp_receive')], check=True)
        subprocess.run([str(BUILD/'network_udp_receive')], check=True)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='ln-')
        self.root = Path(self.temp.name)
        self.state = self.root/'network.json'
        self.socket = self.root/'control.sock'
        self.settings = self.root/'settings.json'
        self.settings.write_text(json.dumps(dict(model='CS96-beta', interface='test0',
            runtime_dir=str(self.root/'runtime'), state_path=str(self.state), socket_path=str(self.socket),
            sn_path=str(self.root/'sn'), legacy_interfaces=str(self.root/'interfaces'),
            legacy_flash_interfaces=str(self.root/'flash-interfaces'),
            confirm_timeout_ms=20000, dhcp_timeout_ms=1000)))
        self.settings.chmod(0o600)
        self.process = None
        self.log = (self.root/'daemon.log').open('w')
        self.start()
        self.wait(lambda value: value['ready'])

    def start(self):
        self.process = subprocess.Popen([str(BUILD/'lidar_netd'), str(self.settings), '--simulation'], stdout=self.log, stderr=self.log)

    def stop(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            self.process.wait(timeout=3)

    def tearDown(self):
        self.stop()
        self.log.close()
        self.temp.cleanup()

    def call(self, op='network_get', **fields):
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as connection:
            connection.settimeout(1)
            connection.connect(str(self.socket))
            connection.send(json.dumps(dict(op=op, **fields)).encode())
            return json.loads(connection.recv(8192))

    def wait(self, predicate, timeout=4):
        deadline = time.monotonic()+timeout
        latest = None
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise AssertionError((self.root/'daemon.log').read_text())
            try:
                latest = self.call()
                if predicate(latest):
                    return latest
            except (OSError, json.JSONDecodeError):
                pass
            time.sleep(.005)
        raise AssertionError(latest)

    @staticmethod
    def pool():
        return dict(mode='pool', start='10.42.0.70', end='10.42.0.71', netmask='255.255.255.0', gateway='10.42.0.1')

    def confirm(self, value):
        return self.call('network_confirm', operation_id=value['operation_id'], via_address=value['address'])

    def collide(self, target, probing=False):
        return self.call('simulate_arp', sender='0.0.0.0' if probing else target, target=target, mac='02:00:00:00:00:99')

    def test_pool_simultaneous_probe_and_confirm(self):
        result = self.call('network_set', config=self.pool())
        self.assertEqual(result['code'], 202)
        first = result['candidate_address']
        self.assertIn(first, ('10.42.0.70', '10.42.0.71'))
        self.collide(first, probing=True)
        result = self.wait(lambda value: value['transaction_phase'] == 'WAIT_CONFIRM')
        self.assertNotEqual(result['address'], first)
        self.assertEqual(self.call('network_confirm', operation_id=result['operation_id'], via_address=first)['code'], 409)
        self.assertEqual(self.call('network_confirm', operation_id='0'*32, via_address=result['address'])['code'], 409)
        committed = self.confirm(result)
        self.assertEqual(committed['code'], 200)
        saved = json.loads(self.state.read_text())
        self.assertFalse(saved['pending'])
        self.assertEqual(saved['confirmed_config']['mode'], 'pool')
        self.assertEqual(saved['confirmed_address'], result['address'])
        self.stop()
        self.start()
        restored = self.wait(lambda value: value['ready'])
        self.assertEqual(restored['address'], result['address'])

    def test_pool_exhaustion_keeps_confirmed_config(self):
        old = self.call()['address']
        result = self.call('network_set', config=self.pool())
        seen = set()
        for _ in range(2):
            target = result['candidate_address']
            self.assertNotIn(target, seen)
            seen.add(target)
            result = self.collide(target)
        restored = self.wait(lambda value: value['ready'] and not value['pending'])
        self.assertEqual(restored['confirmed_config']['mode'], 'auto')
        self.assertEqual(restored['address'], old)
        self.assertEqual(restored['operation_code'], 409)
        self.assertEqual(seen, {'10.42.0.70', '10.42.0.71'})

    def test_invalid_ranges_and_masks_do_not_change_network(self):
        old = self.call()['address']
        invalid = [dict(self.pool(), end='10.42.1.1'), dict(self.pool(), end='10.42.0.69'),
                   dict(self.pool(), netmask='255.0.255.0'), dict(self.pool(), start='10.42.0.0'),
                   dict(self.pool(), gateway='192.168.0.1'), dict(mode='auto', address='10.42.0.88'),
                   dict(mode='static', address='169.254.1.5', netmask='255.255.0.0'),
                   dict(self.pool(), start='10.42.0.1', end='10.42.0.1')]
        for config in invalid:
            self.assertEqual(self.call('network_set', config=config)['code'], 422, config)
            self.assertEqual(self.call()['address'], old)

    def test_confirmation_timeout_rolls_back(self):
        old = self.call()['address']
        self.call('network_set', config=dict(mode='static', address='192.168.20.88', netmask='255.255.255.0'))
        self.wait(lambda value: value['transaction_phase'] == 'WAIT_CONFIRM')
        restored = self.wait(lambda value: value['ready'] and not value['pending'])
        self.assertEqual(restored['address'], old)
        self.assertEqual(restored['operation_code'], 408)
        self.assertEqual(json.loads(self.state.read_text())['confirmed_config']['mode'], 'auto')

    def test_interrupted_change_restores_confirmed_config(self):
        self.call('network_set', config=self.pool())
        pending = self.wait(lambda value: value['transaction_phase'] == 'WAIT_CONFIRM')
        self.process.kill()
        self.process.wait()
        self.start()
        restored = self.wait(lambda value: value['ready'])
        self.assertEqual(restored['confirmed_config']['mode'], 'auto')
        self.assertFalse(restored['pending'])
        self.assertEqual(self.call('network_confirm', operation_id=pending['operation_id'], via_address=pending['address'])['code'], 409)

    def test_running_conflict_defends_then_reselects_inside_pool(self):
        self.call('network_set', config=self.pool())
        result = self.wait(lambda value: value['transaction_phase'] == 'WAIT_CONFIRM')
        self.confirm(result)
        old = result['address']
        self.assertEqual(self.collide(old)['address'], old)
        self.collide(old)
        changed = self.wait(lambda value: value['ready'] and value['address'] != old)
        self.assertIn(changed['address'], ('10.42.0.70', '10.42.0.71'))
        self.assertEqual(changed['confirmed_config']['mode'], 'pool')

    def test_dhcp_address_and_renewal(self):
        lease = dict(interface='test0', event='bound', address='10.5.0.70', netmask='255.255.255.0', gateway='10.5.0.1', lease_seconds=120)
        self.assertEqual(self.call('dhcp_event', **lease)['code'], 202)
        ready = self.wait(lambda value: value['ready'] and value['source'] == 'dhcp')
        self.assertEqual(ready['address'], lease['address'])
        self.assertEqual(self.call('dhcp_event', **dict(lease, event='renew'))['code'], 200)
        self.call('dhcp_event', interface='test0', event='deconfig')
        fallback = self.wait(lambda value: value['ready'] and value['source'] == 'linklocal')
        self.assertTrue(fallback['address'].startswith('169.254.'))
        self.assertEqual(fallback['gateway'], '')

    def test_legacy_ack_before_address_switch(self):
        before = self.call()['address']
        self.call('network_set', config=self.pool(), legacy=True)
        prepared = self.wait(lambda value: value['transaction_phase'] == 'PREPARED')
        self.assertEqual(prepared['address'], before)
        self.assertEqual(self.call('network_apply', operation_id=prepared['operation_id'])['code'], 202)
        changed = self.wait(lambda value: value['transaction_phase'] == 'WAIT_CONFIRM')
        self.assertEqual(self.confirm(changed)['code'], 200)

    def test_state_failure_does_not_change_current_address(self):
        before = self.call()['address']
        content = self.state.read_bytes()
        self.state.unlink()
        self.state.mkdir()
        self.assertEqual(self.call('network_set', config=self.pool())['code'], 500)
        self.assertEqual(self.call()['address'], before)
        self.state.rmdir()
        self.state.write_bytes(content)

    def test_corrupted_state_is_queryable_and_not_overwritten(self):
        self.stop()
        self.state.write_text('损坏的配置')
        self.start()
        fault = self.wait(lambda value: value['phase'] == 'FAULT')
        self.assertFalse(fault['ready'])
        self.assertEqual(self.call('network_set', config=self.pool())['code'], 503)
        recovered=self.wait(lambda value:value['recovery'] and value['address'].startswith('169.254.'))
        self.assertFalse(recovered['configuration_valid'])
        self.assertEqual(self.state.read_text(), '损坏的配置')

    def test_unavailable_storage_still_provides_temporary_diagnostics(self):
        self.stop()
        settings=json.loads(self.settings.read_text())
        unavailable=self.root/'missing-store/network.json'
        settings['state_path']=str(unavailable)
        self.settings.write_text(json.dumps(settings))
        self.start()
        fault=self.wait(lambda value:value['phase']=='FAULT' and value['address'].startswith('169.254.'))
        self.assertTrue(fault['recovery'])
        self.assertFalse(fault['ready'])
        self.assertEqual(self.call('network_set',config=self.pool())['code'],503)
        self.assertFalse(unavailable.exists())

    def test_storage_failure_after_apply_withdraws_unconfirmed_address(self):
        self.call('network_set',config=self.pool())
        pending=self.wait(lambda value:value['transaction_phase']=='WAIT_CONFIRM')
        self.state.unlink()
        self.state.mkdir()
        fault=self.wait(lambda value:value['phase']=='FAULT' and value['address'].startswith('169.254.'))
        self.assertNotEqual(fault['address'],pending['address'])
        self.assertFalse(fault['ready'])
        self.assertFalse(fault['pending'])
        self.assertTrue(fault['recovery'])
        self.assertTrue(self.state.is_dir())

    def test_singleton_and_native_client(self):
        other = subprocess.run([str(BUILD/'lidar_netd'), str(self.settings), '--simulation'], capture_output=True, timeout=3)
        self.assertNotEqual(other.returncode, 0)
        result = subprocess.run([str(BUILD/'lidar_netctl'), str(self.socket), 'get'], capture_output=True, text=True, timeout=3)
        self.assertEqual(result.returncode, 0)
        self.assertTrue(json.loads(result.stdout)['simulation'])

    def test_legacy_static_configuration_is_imported_once(self):
        self.stop()
        self.state.unlink()
        (self.root/'interfaces').write_text('auto lo test0\niface lo inet loopback\niface test0 inet static\n address 10.42.0.88\n netmask 255.255.255.0\n gateway 10.42.0.1\n')
        self.start()
        selected=self.wait(lambda value:value['ready'])
        self.assertEqual(selected['address'],'10.42.0.88')
        self.assertEqual(selected['confirmed_config']['mode'],'static')
        self.stop()
        (self.root/'interfaces').write_text('损坏的旧配置不应覆盖已经迁移的网络设置\n')
        self.start()
        self.assertEqual(self.wait(lambda value:value['ready'])['address'],'10.42.0.88')

    def test_flash_legacy_configuration_takes_precedence_over_system_default(self):
        self.stop()
        self.state.unlink()
        (self.root/'interfaces').write_text('iface test0 inet static\n address 10.42.0.13\n netmask 255.255.255.0\n')
        (self.root/'flash-interfaces').write_text('iface test0 inet static\n address 10.42.0.77\n netmask 255.255.255.0\n')
        self.start()
        self.assertEqual(self.wait(lambda value:value['ready'])['address'],'10.42.0.77')
        self.stop()
        self.state.unlink()
        (self.root/'flash-interfaces').write_text('iface test0 inet dhcp\n')
        self.start()
        self.assertEqual(self.wait(lambda value:value['ready'])['confirmed_config']['mode'],'auto')

    def test_static_startup_conflict_has_explicit_recovery_address(self):
        self.stop()
        saved=json.loads(self.state.read_text())
        fixed=dict(mode='static',address='10.42.0.99',netmask='255.255.255.0',gateway='')
        saved.update(confirmed_config=fixed,candidate_config=fixed,confirmed_address='10.42.0.99')
        self.state.write_text(json.dumps(saved))
        self.start()
        self.wait(lambda value:value['phase']=='PROBING')
        self.collide('10.42.0.99')
        recovered=self.wait(lambda value:value['phase']=='READY' and value['recovery'])
        self.assertFalse(recovered['ready'])
        self.assertEqual(recovered['source'],'recovery')
        self.assertTrue(recovered['address'].startswith('169.254.'))
        self.assertEqual(recovered['gateway'],'')
        stored=json.loads(self.state.read_text())
        self.assertEqual(stored['confirmed_config'],fixed)
        self.assertEqual(stored['confirmed_address'],'10.42.0.99')

    def test_dhcp_conflict_does_not_starve_linklocal(self):
        lease=dict(interface='test0',event='bound',address='10.5.0.70',netmask='255.255.255.0',gateway='',lease_seconds=120)
        self.call('dhcp_event',**lease)
        self.collide(lease['address'],probing=True)
        self.assertEqual(self.call('dhcp_event',**lease)['code'],409)
        result=self.wait(lambda value:value['ready'] and value['source']=='linklocal')
        self.assertNotEqual(result['address'],lease['address'])

    def test_link_reconnect_withdraws_old_address_before_probing(self):
        self.call('simulate_link',up=False)
        disconnected=self.wait(lambda value:value['phase']=='LINK_DOWN')
        self.assertFalse(disconnected['ready'])
        self.assertEqual(disconnected['address'],'')
        self.call('simulate_link',up=True)
        probing=self.wait(lambda value:value['phase']=='PROBING')
        self.assertEqual(probing['address'],'')
        self.assertTrue(self.wait(lambda value:value['ready'])['address'].startswith('169.254.'))


if __name__ == '__main__':
    unittest.main()
