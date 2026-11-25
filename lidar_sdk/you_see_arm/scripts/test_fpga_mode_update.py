#!/usr/bin/env python3
"""使用真实的启动器、管理服务和存储脚本验证 FPGA 更新；可生成开发板 RAM 测试件。"""
import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
SWUPDATE_URL = 'http://127.0.0.1:8080'


def prepare(output, binaries, compiler):
    output.mkdir()
    tools = output / 'tools'
    tools.mkdir()
    for name in ('lidar_boot', 'lidar_crypto'):
        shutil.copy2(binaries / name, tools / name)
    for source in (
        ROOT / 'deploy/runit/rootfs/usr/libexec/lidar/lidar-storage',
        ROOT / 'deploy/runit/rootfs/usr/libexec/lidar/lidar-service',
        ROOT / 'deploy/swu/fpga-mode-hook.sh',
        ROOT / 'scripts/devboard/test_fpga_mode_update.sh',
    ):
        target = output / source.name
        shutil.copy2(source, target)
        target.chmod(0o700)
    shutil.copytree(ROOT / 'deploy/runit/rootfs/usr/share/lidar/runit/lidar_mgrd', output / 'service-template')

    package = output / 'package'
    (package / 'bin').mkdir(parents=True)
    (package / 'conf').mkdir()
    shutil.copy2(binaries / 'lidar_mgrd', package / 'bin/lidar_mgrd')
    subprocess.run([compiler, '-O2', '-std=c11', ROOT / 'scripts/devboard/mock_capture.c',
                    ROOT / 'core/ys_management.c', '-o', package / 'bin/you_see'], check=True)
    (package / 'conf/ys_conf.ini').write_text(
        '[system]\nwork_mode="2"\n[network]\ndefault_state="0"\n[config]\n'
        'set_spad="1"\nset_vop_bias="1"\nset_pl_threshold="1"\nset_pl_zero_angle="1"\n'
        'set_pl_side_en="1"\nset_pl_standard_encoder="1"\n')
    files = {path.relative_to(package).as_posix(): {
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'size': path.stat().st_size}
        for path in package.rglob('*') if path.is_file()}
    (package / 'manifest.json').write_text(json.dumps(dict(schema_version=1, model='CS96-beta',
        supported_modes=['histogram', 'pointcloud', 'gray', 'spot'], driver_abi=2, files=files)))
    with tempfile.TemporaryDirectory(prefix='fpga-test-signing-') as temporary:
        key = Path(temporary) / 'private.pem'
        manifest = package / 'manifest.json'
        for command in (
            ['genpkey', '-algorithm', 'EC', '-pkeyopt', 'ec_paramgen_curve:P-256', '-out', key],
            ['pkey', '-in', key, '-pubout', '-out', output / 'test-public.pem'],
            ['dgst', '-sha256', '-sign', key, '-out', package / 'manifest.sig', manifest],
        ):
            subprocess.run(['openssl', *command], check=True, capture_output=True)
    for path in output.rglob('*'):
        path.chmod(0o700 if path.is_dir() or path.stat().st_mode & 0o111 else 0o600)


def swu_package(root, format_name, certificate, key):
    directory = root / format_name
    directory.mkdir()
    (directory / 'target.bit').write_text(format_name + '\n')
    payload = directory / 'fpga-mode.tar.gz'
    hook = directory / 'fpga-mode-hook.sh'
    shutil.copy2(root / hook.name, hook)
    with tarfile.open(payload, 'w:gz') as archive:
        archive.add(directory / 'target.bit', arcname='target.bit')
        archive.add(root / 'mock-bin/burner', arcname='lattice-flash')
    template = (ROOT / 'deploy/swu/sw-description.in').read_text()
    description = directory / 'sw-description'
    description.write_text(template.replace('/run/lidar-fpga-stage', str(root / 'stage'))
                           .replace('@RELEASE_VERSION@', f'test-{format_name}')
                           .replace('@PAYLOAD_SHA256@', hashlib.sha256(payload.read_bytes()).hexdigest())
                           .replace('@HOOK_SHA256@', hashlib.sha256(hook.read_bytes()).hexdigest()))
    signature = directory / 'sw-description.sig'
    subprocess.run(['openssl', 'cms', '-sign', '-in', description, '-out', signature, '-signer',
                    certificate, '-inkey', key, '-outform', 'DER', '-nosmimecap', '-binary'], check=True)
    update = directory / f'{format_name}.swu'
    filenames = (description, signature, payload, hook)
    with update.open('wb') as stream:
        subprocess.run(['cpio', '--create', '--format=crc', '--quiet'], cwd=directory, stdout=stream,
                       input=''.join(path.name + '\n' for path in filenames).encode(), check=True)
    return update


def curl(path, *args):
    return subprocess.run(['curl', '--noproxy', '*', '-fsS', '--max-time', '5', *args, SWUPDATE_URL + path],
                          capture_output=True, text=True)


@contextmanager
def swupdate_server(root, binary, certificate):
    work = Path(tempfile.mkdtemp(prefix='swu-', dir=root))
    www = work / 'www'
    www.mkdir()
    (www / 'index.html').write_text('FPGA-SWUPDATE-TEST')
    config = work / 'swupdate.cfg'
    config.write_text('globals = { verbose = true; };\n')
    env = dict(os.environ, TMPDIR=str(work), FPGA_TEST_ROOT=str(root),
               LIDAR_SERVICE_CONFIG=str(root / 'service.conf'), LIDAR_DEVBOARD_FLAGS=str(root / 'flags'),
               PATH=str(root / 'mock-bin') + os.pathsep + os.environ['PATH'])
    log = work / 'swupdate.log'
    with log.open('w') as output:
        service = subprocess.Popen([binary, '-v', '-f', config, '-k', certificate, '-M', '-m',
                                    '-w', f'-r {www} -p 127.0.0.1:8080'],
                                   env=env, stdout=output, stderr=subprocess.STDOUT)
        try:
            for _ in range(50):
                if service.poll() is not None:
                    break
                if curl('/').stdout.strip() == 'FPGA-SWUPDATE-TEST':
                    yield log
                    return
                time.sleep(0.1)
            raise RuntimeError(f'SWUpdate 的 8080 未就绪\n{log.read_text(errors="replace")}')
        finally:
            service.terminate()
            service.wait(timeout=10)


def upload(update, log, success):
    with log.open('r', encoding='utf-8', errors='replace') as stream:
        stream.seek(0, os.SEEK_END)
        response = curl('/upload', '-F', f'file=@{update}')
        if response.returncode:
            raise RuntimeError(response.stderr)
        received = ''
        for _ in range(100):
            received += stream.read()
            if 'SWUPDATE successful !' in received or 'Installation failed !' in received:
                break
            time.sleep(0.1)
        result = 'SWUPDATE successful !' if success else 'Installation failed !'
        assert result in received and 'Verified OK' in received, received


def test_http(root, binary):
    subprocess.run([root / 'test_fpga_mode_update.sh', 'setup'], check=True)
    certificate = root / 'swupdate-test.crt'
    with tempfile.TemporaryDirectory(prefix='swu-cms-key-') as temporary:
        key = Path(temporary) / 'test.key'
        subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '1',
                        '-subj', '/CN=FPGA SWUpdate Local Test', '-addext', 'extendedKeyUsage=emailProtection',
                        '-keyout', key, '-out', certificate], check=True, capture_output=True)
        gray = swu_package(root, 'gray', certificate, key)
        histogram = swu_package(root, 'histogram', certificate, key)
        pointcloud = swu_package(root, 'pointcloud', certificate, key)

    state = root / 'medium/lidar/state'
    assert not (state / 'fpga-profile').exists() and not (state / 'state.json').exists()
    with swupdate_server(root, binary, certificate) as log:
        upload(gray, log, success=True)
        assert not (state / 'state.json').exists()
        assert (state / 'fpga-profile').read_text() == 'installed ' + hashlib.sha256(b'gray\n').hexdigest() + '\n'
        assert not (root / 'service-up').exists()
        print('通过：首次从 8080 上传签名灰度 SWU，完成烧录并保持停采', flush=True)

        fault = root / 'fail-readback'
        fault.touch()
        upload(histogram, log, success=False)
        fault.unlink()
        assert not (state / 'state.json').exists()
        assert (state / 'fpga-profile').read_text().startswith('pending ')
        assert not (root / 'service-up').exists()
        assert curl('/').stdout.strip() == 'FPGA-SWUPDATE-TEST'

    # 重新加载模拟 Flash 到新的 RAM 快照，并重新拉起 8080 服务。
    (root / 'ram').rename(root / 'old-ram')
    env = dict(os.environ, LIDAR_SERVICE_CONFIG=str(root / 'service.conf'), FPGA_TEST_ROOT=str(root),
               PATH=str(root / 'mock-bin') + os.pathsep + os.environ['PATH'])
    subprocess.run([root / 'lidar-storage', 'start'], env=env, check=True)
    boot = [root / 'tools/lidar_boot', 'prepare', root / 'boot.json']
    assert subprocess.run(boot, capture_output=True).returncode != 0
    with swupdate_server(root, binary, certificate) as log:
        upload(pointcloud, log, success=True)
        assert not (state / 'state.json').exists()
        assert (state / 'fpga-profile').read_text() == 'installed ' + hashlib.sha256(b'pointcloud\n').hexdigest() + '\n'
        assert not (root / 'service-up').exists()
        subprocess.run(boot, env=env, check=True)
        assert json.loads((root / 'run/manager.json').read_text())['initial_work_mode'] == 2
        print('通过：直方图回读失败后模拟重启，重传点云 SWU 后由正常启动识别模式 2', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    choices = parser.add_mutually_exclusive_group()
    choices.add_argument('--board-tree', type=Path)
    choices.add_argument('--swupdate', type=Path, help='使用有 CMS、archive 和 Web 支持的宿主 SWUpdate 测试 8080')
    args = parser.parse_args()
    if args.board_tree:
        prepare(args.board_tree, ROOT / 'build/beta/bin', ROOT / 'cc/bin/aarch64-buildroot-linux-gnu-gcc')
    else:
        with tempfile.TemporaryDirectory(prefix='fpga-update-test-') as temporary:
            build = Path(temporary) / 'build'
            subprocess.run(['cmake', '-S', ROOT / 'manager', '-B', build], check=True, stdout=subprocess.DEVNULL)
            subprocess.run(['cmake', '--build', build, '-j', '4'], check=True, stdout=subprocess.DEVNULL)
            output = Path(temporary) / 'test'
            prepare(output, build, 'cc')
            if args.swupdate:
                test_http(output, args.swupdate.resolve())
            else:
                subprocess.run([output / 'test_fpga_mode_update.sh'], check=True)
