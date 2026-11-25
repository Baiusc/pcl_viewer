#!/usr/bin/env python3
"""验证 Web 输入归档边界及本机标定补齐时的覆盖保护。"""
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest

from beta_image_tools import CALIBRATION, provision
from prepare_beta_image_kit import unpack_web


class ImageKitTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory()
        self.root=Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def archive(self, name, link=False):
        target=self.root/'web.tar.gz'
        with tarfile.open(target,'w:gz') as archive:
            info=tarfile.TarInfo(name)
            if link:
                info.type=tarfile.SYMTYPE
                info.linkname='/etc'
                archive.addfile(info)
            else:
                data=b'#!/bin/sh\nexit 0\n'
                info.size=len(data)
                archive.addfile(info,io.BytesIO(data))
        return target

    def test_valid_web_is_extracted_and_cgi_is_executable(self):
        result=unpack_web(self.archive('www/cgi-bin/status.cgi'),self.root/'out')
        self.assertTrue((result/'cgi-bin/status.cgi').stat().st_mode & 0o111)

    def test_archive_parent_traversal_is_rejected_before_writing(self):
        with self.assertRaises(ValueError):
            unpack_web(self.archive('www/../../escape'),self.root/'out')
        self.assertFalse((self.root/'out').exists())
        self.assertFalse((self.root/'escape').exists())

    def test_archive_symlink_is_rejected(self):
        with self.assertRaises(ValueError):
            unpack_web(self.archive('www/config',True),self.root/'out')
        self.assertFalse((self.root/'out').exists())

    def test_calibration_mismatch_does_not_partially_copy(self):
        kit=self.root/'kit';kit.mkdir()
        (kit/'kit-report.json').write_text(json.dumps({'kit_format':1,'model':'CS96-beta'}))
        source=self.root/'calibration';source.mkdir()
        for name in CALIBRATION:
            (source/name).write_text('本次提供的标定\n')
        target=kit/'third-partition/lidar/calibration';target.mkdir(parents=True)
        (target/CALIBRATION[1]).write_text('已经存在的另一份本机标定\n')
        with self.assertRaises(ValueError):
            provision(kit,calibration=source)
        self.assertFalse((target/CALIBRATION[0]).exists())
        self.assertEqual((target/CALIBRATION[1]).read_text(),'已经存在的另一份本机标定\n')


if __name__=='__main__':
    unittest.main()
