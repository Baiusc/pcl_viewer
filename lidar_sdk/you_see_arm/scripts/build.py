#!/usr/bin/env python3
"""统一构建入口；机型能力只从 configs/build_catalog.json 读取。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from release_manifest import create_manifest, sign

ROOT = Path(__file__).resolve().parents[1]
CATALOG = json.loads((ROOT / "configs/build_catalog.json").read_text())


def combinations():
    result = {}
    for model, definition in CATALOG["models"].items():
        if definition.get("runtime_modes"):
            result[definition["alias"]] = {"MODEL": model}
            continue
        for mode in definition["modes"]:
            name = f'{definition["alias"]}-{mode}'
            result[name] = {"MODEL": model, "DATA_MODE": mode}
            slope = CATALOG["algorithms"]["spotslope"]
            if model in slope["models"] and mode in slope["modes"]:
                result[name + "-spotslope"] = dict(result[name], YS_SPOT_SLOPE="ON")
    return result


def presets():
    configure, build = [], []
    for name, values in combinations().items():
        for kind in ("Release", "Debug"):
            full = name if kind == "Release" else name + "-debug"
            configure.append({
                "name": full, "displayName": f'{values["MODEL"]} / {values.get("DATA_MODE", "runtime")} / {"完整功能" if "DATA_MODE" not in values else "无算法"} / {kind}',
                "generator": "Ninja", "binaryDir": "${sourceDir}/build/" + full,
                "cacheVariables": dict(YS_REFLECTIVITY="ON" if "DATA_MODE" not in values else "OFF", YS_SPOT_SLOPE="OFF",
                                       CMAKE_BUILD_TYPE=kind, **{k: v for k, v in values.items()
                                                             if k != "YS_SPOT_SLOPE"})
            })
            configure[-1]["cacheVariables"]["YS_SPOT_SLOPE"] = values.get("YS_SPOT_SLOPE", "ON" if "DATA_MODE" not in values and values["MODEL"] == "CS96-beta" else "OFF")
            build.append({"name": full, "configurePreset": full})
    return {"version": 3, "cmakeMinimumRequired": {"major": 3, "minor": 21, "patch": 0},
            "configurePresets": configure, "buildPresets": build}


def run(args, **kwargs):
    subprocess.run(list(map(str, args)), cwd=ROOT, check=True, **kwargs)


def git_info(path):
    def query(*args):
        result = subprocess.run(["git", "-C", str(path), *args], capture_output=True, text=True)
        return result.stdout.strip() if result.returncode == 0 else None
    return {"commit": query("rev-parse", "HEAD"),
            "changes": query("status", "--porcelain", "--ignore-submodules=all")}


def manifest(directory):
    directory = Path(directory).resolve()
    info = json.loads((directory / "generated/build-info.json").read_text())
    cache = {}
    for line in (directory / "CMakeCache.txt").read_text().splitlines():
        if "=" in line and ":" in line.split("=", 1)[0] and not line.startswith("//"):
            key, value = line.split("=", 1)
            cache[key.split(":", 1)[0]] = value
    info["sources"] = {"application": git_info(ROOT)}
    exact_tag = subprocess.run(["git", "describe", "--tags", "--exact-match", "--match", "v[0-9]*", "HEAD"], cwd=ROOT, capture_output=True, text=True)
    info["release_tag"] = exact_tag.stdout.strip() if exact_tag.returncode == 0 else None
    if info["driver_kind"] in ("source", "v4l2"):
        info["sources"]["driver"] = git_info(cache["YS_DRIVER_SOURCE_DIR"])
    for enabled, folder in (("reflectivity", "calib_reflectivity"), ("spotslope", "spot_slope_alog")):
        if info[enabled]:
            info["sources"][enabled] = git_info(ROOT / "algo" / folder)
    info["supported_modes"] = list(CATALOG["models"][info["model"]]["modes"]) if info["data_mode"] == "runtime" else [info["data_mode"]]
    info["driver_abi"] = 2 if info["data_mode"] == "runtime" else None
    info["hardware_validation"] = "待板端验证，必须匹配内核模块与 FPGA 数据格式"
    info["toolchain"] = git_info(cache.get("YS_TOOLCHAIN_ROOT", ROOT / "cc"))
    artifacts = [directory / "bin/you_see", directory / "generated/ys_conf.ini"]
    artifacts.extend(directory / 'bin' / name for name in
                     ('lidar_netd', 'lidar_netctl', 'lidar_network.cgi', 'lidar_network_status.cgi'))
    if info["data_mode"] == "runtime":
        artifacts.extend([directory / "bin/lidar_mgrd", directory / "bin/lidar_mgrctl", directory / "bin/lidar_crypto", directory / "bin/lidar_boot"])
    if info["driver_kind"] in ("source", "v4l2"):
        artifacts.append(directory / "lib/liblidar.so")
    elif info["driver_kind"] == "prebuilt":
        artifacts.append(Path(cache["YS_PREBUILT_DRIVER"]))
        info["driver_mode_verified"] = False
    else:
        artifacts.extend((ROOT / "third_party/fs_sdk/bsp/rockchip/rv1126b/sdk_libs").glob("*.so*"))
    missing = [str(p) for p in artifacts if not p.is_file()]
    if missing:
        raise ValueError(f"构建产物缺失：{missing}")
    info["sha256"] = {str(p.relative_to(directory) if p.is_relative_to(directory) else p):
                      hashlib.sha256(p.read_bytes()).hexdigest() for p in artifacts if p.is_file()}
    (directory / "manifest.json").write_text(json.dumps(info, ensure_ascii=False, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list", help="列出合法组合")
    generation = sub.add_parser("generate", help="从关系表生成 CMake 预设")
    generation.add_argument("--check", action="store_true", help="检查预设是否与关系表一致")
    prepare = sub.add_parser("prepare", help="从本地已有分支准备独立工具链和驱动，不切换当前分支")
    prepare.add_argument("model", choices=list(CATALOG["models"]))
    record = sub.add_parser("manifest", help="记录构建产物")
    record.add_argument("directory")
    for action in ("configure", "build", "package"):
        p = sub.add_parser(action, help={"configure": "配置", "build": "配置并编译", "package": "编译并打包"}[action])
        p.add_argument("preset", choices=[x["name"] for x in presets()["configurePresets"]])
        p.add_argument("-j", "--jobs", type=int, default=min(os.cpu_count() or 1, 8))
        if action == "package":
            p.add_argument("--output", type=Path, help="输出目录，必须不存在")
            p.add_argument("--sign-key", type=Path, help="包外发布私钥，不复制到包内")
            p.add_argument("--release", action="store_true", help="正式发布：必须签名且源码干净并准确命中版本标签")
        p.add_argument("--toolchain-root", type=Path, help="工具链根目录")
    args = parser.parse_args()
    if args.command == "list":
        for name, values in combinations().items():
            print(f'{name:30} {values["MODEL"]:14} {values.get("DATA_MODE", "runtime"):12} '
                  f'算法：{"完整功能" if "DATA_MODE" not in values else "SpotSlope" if values.get("YS_SPOT_SLOPE") else "无"}')
        print("每个预设追加 -debug 可构建调试版；Beta/Gamma 编入完整功能，算法运行开关默认关闭。")
    elif args.command == "generate":
        content = json.dumps(presets(), ensure_ascii=False, indent=2) + "\n"
        path = ROOT / "CMakePresets.json"
        if args.check:
            if not path.exists() or path.read_text() != content:
                parser.error("预设与关系表不一致，请执行 python3 scripts/build.py generate")
        else:
            path.write_text(content)
    elif args.command == "prepare":
        definition = CATALOG["models"][args.model]
        dependencies = [("cc", "toolchains", definition["toolchain"])]
        if definition["driver"] in ("source", "v4l2"):
            dependencies.append(("driver", "drivers", definition["driver_platform"]))
        for repo, group, reference in dependencies:
            destination = ROOT / "build" / group / reference
            if destination.exists():
                print(f"保留已有依赖：{destination}")
                continue
            destination.parent.mkdir(parents=True, exist_ok=True)
            run(["git", "-C", ROOT / repo, "worktree", "add", "--detach", destination, reference])
        print("依赖准备完毕；后续构建使用这些固定检出的版本，清单记录实际提交。")
    elif args.command == "manifest":
        manifest(args.directory)
    else:
        if args.command == "package" and args.release and not args.sign_key:
            parser.error("正式发布必须提供 --sign-key")
        if args.jobs < 1:
            parser.error("并行数必须大于 0")
        command = ["cmake", "--preset", args.preset]
        if args.toolchain_root:
            command.append("-DYS_TOOLCHAIN_ROOT=" + str(args.toolchain_root.resolve()))
        run(command)
        if args.command != "configure":
            run(["cmake", "--build", "--preset", args.preset, "--parallel", args.jobs])
        if args.command == "package":
            destination = (args.output or ROOT / "dist" / args.preset).resolve()
            if destination.exists():
                parser.error(f"输出目录已存在，请另选 --output：{destination}")
            destination.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(prefix=".ys-package-", dir=destination.parent) as temporary:
                stage = Path(temporary) / "payload"
                run(["cmake", "--install", ROOT / "build" / args.preset, "--prefix", stage])
                provenance = json.loads((stage / "manifest.json").read_text())
                create_manifest(stage, provenance, CATALOG, args.release)
                if args.sign_key:
                    sign(stage, args.sign_key.resolve())
                stage.rename(destination)
            print(f"已生成{'签名' if args.sign_key else '未签名开发'}包：{destination}")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError) as error:
        sys.exit(str(error))
    except subprocess.CalledProcessError as error:
        sys.exit(error.returncode)
