#!/usr/bin/env python3
"""验证轻量工具与 OpenSSL 的摘要、RSA/ECDSA 签名互通及失败拒绝。"""
import hashlib
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT/'build/manager-host/lidar_crypto'

class CryptoTest(unittest.TestCase):
    def run_tool(self, *args):
        return subprocess.run([str(TOOL), 'dgst', '-sha256', *map(str,args)], capture_output=True, text=True)

    def test_streaming_digest(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'data'
            for data in (b'', b'abc', bytes(range(256))*1000):
                path.write_bytes(data)
                result=self.run_tool('-r',path)
                self.assertEqual(result.returncode,0,result.stderr)
                self.assertEqual(result.stdout[:64],hashlib.sha256(data).hexdigest())
            self.assertNotEqual(self.run_tool('-r',Path(temp)/'missing').returncode,0)

    def test_openssl_signature_interoperability(self):
        with tempfile.TemporaryDirectory() as temp:
            base=Path(temp);data=base/'data';signature=base/'sig';key=base/'key';public=base/'pub'
            for algorithm, options in [('EC',['-pkeyopt','ec_paramgen_curve:P-256']),('RSA',['-pkeyopt','rsa_keygen_bits:2048'])]:
                subprocess.run(['openssl','genpkey','-algorithm',algorithm,*options,'-out',str(key)],check=True,capture_output=True)
                subprocess.run(['openssl','pkey','-in',str(key),'-pubout','-out',str(public)],check=True,capture_output=True)
                data.write_bytes(b'firmware'*10000)
                subprocess.run(['openssl','dgst','-sha256','-sign',str(key),'-out',str(signature),str(data)],check=True,capture_output=True)
                args=('-verify',public,'-signature',signature,data)
                self.assertEqual(self.run_tool(*args).returncode,0)
                data.write_bytes(b'tampered')
                self.assertNotEqual(self.run_tool(*args).returncode,0)
                signature.write_bytes(b'invalid signature')
                self.assertNotEqual(self.run_tool(*args).returncode,0)
                public.write_text('not a key')
                self.assertNotEqual(self.run_tool(*args).returncode,0)

if __name__ == '__main__': unittest.main()
