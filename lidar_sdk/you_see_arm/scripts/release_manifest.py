#!/usr/bin/env python3
"""生成发布文件清单；使用 OpenSSL 签名及受信任外部公钥验签，不生成发布密钥。"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess


def files_in(root):
    result = {}
    for path in sorted(root.rglob('*')):
        relative = path.relative_to(root).as_posix()
        if path.is_symlink():
            raise ValueError(f'包内不允许符号链接：{relative}')
        if path.is_file() and relative not in ('manifest.json', 'manifest.sig'):
            data = path.read_bytes()
            result[relative] = {'size': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
    if not result:
        raise ValueError('软件包为空')
    return result


def release_source_check(sources):
    for name, source in sources.items():
        if not source.get('commit') or source.get('changes') is None or source['changes']:
            raise ValueError(f'正式发布要求来源可追溯且工作区干净：{name}')


def create_manifest(root, provenance, catalog, release=False):
    model = provenance['model']
    definition = catalog['models'][model]
    if release:
        release_source_check(provenance['sources'])
        release_source_check({'toolchain': provenance['toolchain']})
        if not re.fullmatch(r'v\d+\.\d+\.\d+', provenance.get('release_tag') or ''):
            raise ValueError('正式发布要求 HEAD 准确位于 vX.Y.Z 标签')
    manifest = {
        'schema_version': 1,
        'model': model,
        'release': release,
        'version': provenance.get('release_tag'),
        'sources': provenance['sources'],
        'toolchain': provenance['toolchain'],
        'build_type': provenance['build_type'],
        'compiler_version': provenance['compiler_version'],
        'supported_modes': provenance['supported_modes'],
        'mode_formats': {m: definition['modes'][m] for m in provenance['supported_modes']},
        'driver_abi': provenance.get('driver_abi'),
        'algorithms': {'reflectivity': provenance['reflectivity'], 'spotslope': provenance['spotslope']},
        'hardware_validation': provenance['hardware_validation'],
        'files': files_in(root),
    }
    (root / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    return manifest


def sign(root, key):
    if key.resolve().is_relative_to(root.resolve()):
        raise ValueError('签名私钥不得放入软件包')
    subprocess.run(['openssl', 'dgst', '-sha256', '-sign', str(key),
                    '-out', str(root / 'manifest.sig'), str(root / 'manifest.json')], check=True)


def verify(root, public_key):
    if public_key.resolve().is_relative_to(root.resolve()):
        raise ValueError('必须使用包外受信任公钥')
    # 先验证原始字节签名，再解释能力及路径；公钥必须由调用方信任，不能从包内取。
    subprocess.run(['openssl', 'dgst', '-sha256', '-verify', str(public_key),
                    '-signature', str(root / 'manifest.sig'), str(root / 'manifest.json')],
                   check=True, stdout=subprocess.DEVNULL)
    manifest = json.loads((root / 'manifest.json').read_text())
    if manifest.get('schema_version') != 1:
        raise ValueError('不支持的清单版本')
    for name in manifest['files']:
        path = PurePosixPath(name)
        if path.is_absolute() or '..' in path.parts or path.as_posix() != name:
            raise ValueError(f'非法包内路径：{name}')
    if files_in(root) != manifest['files']:
        raise ValueError('软件包文件列表、大小或摘要不匹配')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--public-key', type=Path, required=True, help='受信任的包外公钥')
    args = parser.parse_args()
    verify(args.directory.resolve(), args.public_key.resolve())
    print('签名和包内全部文件校验通过')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, OSError, subprocess.CalledProcessError) as error:
        raise SystemExit(f'验证失败：{error}')
