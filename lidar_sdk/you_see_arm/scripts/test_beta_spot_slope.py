#!/usr/bin/env python3
"""在临时目录编译算法库，验证 Beta 按面修正和坏表拒绝行为。"""
import pathlib
import struct
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
ALGO = ROOT / "algo/spot_slope_alog"


def table(order=(0, 1, 2, 3), angles=(-60.0, 60.0), correction=None):
    result = bytes([len(order)])
    for face in order:
        result += struct.pack("<BHH", face, 0, 2)
        for angle in angles:
            result += struct.pack("<ff", angle, face + 1 if correction is None else correction)
    return result


with tempfile.TemporaryDirectory(prefix="beta-slope-test-") as directory:
    work = pathlib.Path(directory)
    cases = {
        "valid": table((3, 1, 0, 2)),
        "zero": table(correction=0),
        "short": table()[:-1],
        "duplicate": table((0, 0, 2, 3)),
        "missing": table((0, 1, 2)),
        "unsorted": table(angles=(60, -60)),
        "equal": table(angles=(0, 0)),
        "nan": table(correction=float("nan")),
    }
    for name, data in cases.items():
        (work / name).write_bytes(data)
    source = work / "test.cpp"
    source.write_text(r'''
#include "algo_calib_rotate_angle_interface.h"
#include <cassert>
#include <cmath>
int main() {
    auto h = calibRotateAngle_Create();
    assert(calibRotateAngle_DoCalibByFace(h, 12, 0) == 12);
    assert(calibRotateAngle_LoadMapByFace(h, "valid") == 1);
    for (int face = 0; face < 4; ++face)
        for (float angle : {-70.f, -60.f, 0.f, 60.f, 70.f})
            assert(std::fabs(calibRotateAngle_DoCalibByFace(h, angle, face)
                             - angle - face - 1) < 0.0001f);
    assert(calibRotateAngle_DoCalibByFace(h, 12, 255) == 12);
    for (auto file : {"short", "duplicate", "missing", "unsorted", "equal", "nan", "absent"})
        assert(calibRotateAngle_LoadMapByFace(h, file) == 0);
    assert(calibRotateAngle_LoadMapByFace(h, "zero") == 1);
    assert(calibRotateAngle_DoCalibByFace(h, 12, 0) == 12);
    calibRotateAngle_Destroy(h);
}
'''.replace('#include <cassert>', '#include <cassert>\n#include <initializer_list>'))
    executable = work / "test"
    subprocess.run([
        "g++", "-std=c++17", "-I" + str(ALGO / "include"), str(source),
        *map(str, (ALGO / "src").glob("*.cpp")), "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], cwd=work, check=True)
    print("Beta 修正算法验证通过：面号排序、插值及外推、零修正、非法面号、坏表拒绝。")
