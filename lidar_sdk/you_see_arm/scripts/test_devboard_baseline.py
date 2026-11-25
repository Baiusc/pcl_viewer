#!/usr/bin/env python3
"""验证验收判据、目标板限制和归档权限；不连接设备。"""
import copy
import io
import os
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch

from run_devboard_baseline import Board, checked_identity, healthy, root_owner


class BaselineTests(unittest.TestCase):
    def setUp(self):
        self.value={'code':200,'model':'DEVBOARD-MOCK','phase':'COMMITTED','error':'',
                    'initialized':True,'pipeline_verified':True,'pid':123,
                    'actual_mode':1,'confirmed_mode':1}

    def test_healthy_requires_pipeline_and_confirmed_mode(self):
        self.assertTrue(healthy(self.value,1))
        for key,value in (('phase','FAULT'),('pipeline_verified',False),('initialized',False),
                          ('pid',0),('error','已经回退'),('confirmed_mode',2),('model','CS96-beta')):
            sample=copy.deepcopy(self.value);sample[key]=value
            with self.subTest(field=key):self.assertFalse(healthy(sample,1))

    def test_boolean_is_not_a_process_id_or_mode(self):
        for key in ('pid','actual_mode','confirmed_mode'):
            sample=copy.deepcopy(self.value);sample[key]=True
            self.assertFalse(healthy(sample,1))

    def test_paths_are_confined_to_test_namespace(self):
        runtime,data=checked_identity('run-20261111-120000-aabbccdd',8081)
        self.assertTrue(data.startswith('/mnt/mtdblock2/lidar-acceptance/'))
        self.assertTrue(runtime.startswith('/run/lidar-acceptance-'))
        for run_id in ('../lidar','run-20261111-120000-aabbccdd/../../lidar',''):
            with self.assertRaises(RuntimeError):checked_identity(run_id,8081)
        for port in (80,8080,65536):
            with self.assertRaises(RuntimeError):checked_identity('run-20261111-120000-aabbccdd',port)

    def test_production_hosts_are_not_allowed(self):
        for host in ('10.42.0.221','10.42.0.238','-oProxyCommand=example'):
            with self.assertRaises(RuntimeError):Board(host,Path('/tmp'))

    def test_password_is_not_an_ssh_argument(self):
        with patch.dict(os.environ,{'LIDAR_TEST_SSH_PASSWORD':'仅测试字符串'}):
            board=Board('10.42.0.30',Path('/tmp'))
            self.assertNotIn('仅测试字符串',board.ssh)
            self.assertIn('StrictHostKeyChecking=yes',board.ssh)

    def test_runtime_archive_uses_root_ownership(self):
        with tempfile.TemporaryDirectory() as temporary:
            source=Path(temporary)/'boot.json';source.write_text('{}');source.chmod(0o600)
            output=io.BytesIO()
            with tarfile.open(fileobj=output,mode='w') as archive:
                archive.add(source,arcname='boot.json',filter=root_owner)
            output.seek(0)
            with tarfile.open(fileobj=output) as archive:
                info=archive.getmember('boot.json')
                self.assertEqual((info.uid,info.gid,info.mode),(0,0,0o600))


if __name__=='__main__':unittest.main()
