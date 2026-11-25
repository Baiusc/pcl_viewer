#!/usr/bin/env python3
"""验证存储挂载所有权和文件系统参数，不进行真实 mount/umount。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
SCRIPT=ROOT/'deploy/runit/rootfs/usr/libexec/lidar/lidar-storage'


class StorageTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='storage-')
        self.root=Path(self.temp.name)
        self.data=self.root/'data';self.data.mkdir()
        self.commands=self.root/'bin';self.commands.mkdir()
        self.calls=self.root/'calls'
        for name in ('mount','umount'):
            tool=self.commands/name
            tool.write_text(f'#!/bin/sh\nprintf "%s\\n" "{name} $*" >> "{self.calls}"\n')
            tool.chmod(0o700)
        self.env=dict(os.environ,PATH=str(self.commands)+os.pathsep+os.environ['PATH'],
                      LIDAR_SERVICE_CONFIG=str(self.root/'service.conf'))

    def tearDown(self):self.temp.cleanup()

    def call(self,mode,action='start',fstype='ext4',mount=None):
        values=dict(LIDAR_STORAGE_MODE=mode,LIDAR_STORAGE_MOUNT=mount or self.data,
                    LIDAR_STORAGE_DEVICE='/dev/test-only',LIDAR_STORAGE_FSTYPE=fstype,LIDAR_BOOT='/bin/false',
                    LIDAR_BOOT_CONFIG='/unused')
        (self.root/'service.conf').write_text('\n'.join(f"{k}='{v}'" for k,v in values.items())+'\n')
        return subprocess.run(['/bin/sh',str(SCRIPT),action],env=self.env,capture_output=True,text=True)

    def test_existing_directory_never_mounts_or_unmounts(self):
        self.assertEqual(self.call('directory').returncode,0)
        self.assertEqual(self.call('directory','stop').returncode,0)
        self.assertFalse(self.calls.exists())

    def test_managed_mode_passes_explicit_filesystem(self):
        self.assertEqual(self.call('managed').returncode,0)
        self.assertEqual(self.calls.read_text(),f'mount -t ext4 /dev/test-only {self.data}\n')

    def test_missing_or_linked_directory_is_rejected(self):
        self.assertNotEqual(self.call('directory',mount=self.root/'missing').returncode,0)
        link=self.root/'link';link.symlink_to(self.data)
        self.assertNotEqual(self.call('directory',mount=link).returncode,0)
        self.assertFalse(self.calls.exists())

    def test_unknown_mode_is_rejected(self):
        self.assertNotEqual(self.call('unexpected').returncode,0)
        self.assertFalse(self.calls.exists())


if __name__=='__main__':unittest.main()
