#!/usr/bin/env python3
"""验证真实 OpenSSL 签名、错误公钥、清单及载荷篡改拒绝。"""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from release_manifest import create_manifest, sign, verify, release_source_check

class ManifestTest(unittest.TestCase):
    def test_authentication_and_payload(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            root = base / 'payload'
            root.mkdir()
            (root / 'you_see').write_bytes('应用'.encode())
            for name in ('release', 'wrong'):
                subprocess.run(['openssl', 'genpkey', '-algorithm', 'EC', '-pkeyopt', 'ec_paramgen_curve:P-256', '-out', str(base/name)], check=True, capture_output=True)
                subprocess.run(['openssl', 'pkey', '-in', str(base/name), '-pubout', '-out', str(base/(name+'.pub'))], check=True, capture_output=True)
            catalog = {'models': {'CS96-beta': {'modes': {'gray': 5}}}}
            provenance = {'model': 'CS96-beta', 'sources': {}, 'toolchain': {}, 'build_type': 'Release', 'compiler_version': 'test', 'supported_modes': ['gray'], 'reflectivity': True, 'spotslope': True, 'hardware_validation': '待验证', 'driver_abi': 1}
            create_manifest(root, provenance, catalog)
            sign(root, base/'release')
            verify(root, base/'release.pub')
            with self.assertRaises(subprocess.CalledProcessError): verify(root, base/'wrong.pub')
            original = (root/'manifest.json').read_bytes()
            (root/'manifest.json').write_bytes(original+b' ')
            with self.assertRaises(subprocess.CalledProcessError): verify(root, base/'release.pub')
            (root/'manifest.json').write_bytes(original)
            (root/'you_see').write_bytes('篡改'.encode())
            with self.assertRaises(ValueError): verify(root, base/'release.pub')
            (root/'you_see').write_bytes('应用'.encode())
            (root/'extra').write_bytes('注入'.encode())
            with self.assertRaises(ValueError): verify(root, base/'release.pub')
            (root/'extra').unlink()
            (root/'link').symlink_to(base/'release')
            with self.assertRaises(ValueError): verify(root, base/'release.pub')

    def test_release_requires_clean_provenance(self):
        for source in ({'commit': None, 'changes': ''}, {'commit': '123', 'changes': None}, {'commit': '123', 'changes': ' M file'}):
            with self.assertRaises(ValueError): release_source_check({'application': source})

    def test_release_requires_version_tag(self):
        with tempfile.TemporaryDirectory() as temp:
            source = {'commit': '123', 'changes': ''}
            provenance = {'model': 'CS96-beta', 'sources': {'application': source},
                          'toolchain': source}
            catalog = {'models': {'CS96-beta': {}}}
            for tag in (None, '', 'v1.2'):
                provenance['release_tag'] = tag
                with self.assertRaisesRegex(ValueError, 'vX.Y.Z'):
                    create_manifest(Path(temp), provenance, catalog, release=True)

if __name__ == '__main__': unittest.main()
