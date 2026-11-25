#!/usr/bin/env python3
"""验证统一机型构建、不兼容配置拒绝和实际编译参数。"""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from build import ROOT, presets

class BuildConfigurationTest(unittest.TestCase):
    def test_list_runtime_models(self):
        result = subprocess.run(['python3', str(ROOT/'scripts/build.py'), 'list'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        for model in ('CS96-beta', 'CS96-gamma'):
            self.assertTrue(any(model in line and 'runtime' in line and '完整功能' in line
                                for line in result.stdout.splitlines()))

    def configure(self, directory, variables):
        return subprocess.run(['cmake', '-S', str(ROOT), '-B', str(directory), '-G', 'Ninja',
            *[f'-D{k}={v}' for k,v in variables.items()]], capture_output=True, text=True)

    def test_presets_match_catalog(self):
        self.assertEqual(json.loads((ROOT/'CMakePresets.json').read_text()), presets())
        names = {p['name'] for p in presets()['configurePresets']}
        self.assertTrue({'beta', 'gamma', 'beta-debug', 'gamma-debug'} <= names)
        self.assertFalse(any(n.startswith(('beta-', 'gamma-')) and not n.endswith('-debug') for n in names))

    def test_invalid_combinations_fail_early(self):
        cases = [({'MODEL':'未知'}, '必须指定有效 MODEL'),
                 ({'MODEL':'CS96-beta','DATA_MODE':'gray'}, '已按机型收敛'),
                 ({'MODEL':'CS96-beta','YS_SPOT_SLOPE':'OFF'}, '完整算法'),
                 ({'MODEL':'CS96-gamma','YS_SPOT_SLOPE':'ON'}, '算法 spotslope 不支持'),
                 ({'MODEL':'MF150-fl6532','DATA_MODE':'gray'}, '不支持 DATA_MODE')]
        with tempfile.TemporaryDirectory() as temp:
            for i,(variables, diagnostic) in enumerate(cases):
                result = self.configure(Path(temp)/str(i),variables)
                self.assertNotEqual(result.returncode,0)
                self.assertIn(diagnostic,result.stdout+result.stderr)
                self.assertNotIn('compiler identification',result.stdout)

    def test_runtime_model_compilation(self):
        with tempfile.TemporaryDirectory() as temp:
            for model, alias, formats in [('CS96-beta', 'beta', (4, 0, 3)),
                                          ('CS96-gamma', 'gamma', (5, 1, 2))]:
                directory = Path(temp)/alias
                result=self.configure(directory,{'MODEL':model})
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                commands=json.loads((directory/'compile_commands.json').read_text())
                core=next(c for c in commands if c['file'].endswith('/core/ys_core_system.c'))
                self.assertIn('-DYS_WORK_MODE_MASK=15',core['command'])
                self.assertIn('-DYS_REFLECTIVITY=1',core['command'])
                for name, mode in zip(('HISTOGRAM', 'POINTCLOUD', 'GRAY'), formats):
                    self.assertIn(f'-DYS_FORMAT_{name}={mode}', core['command'])
                driver=next(c for c in commands if c['file'].endswith('/driver/yousee_lidar_api.c'))
                self.assertNotIn('DEF_RCV_MODE',driver['command'])
                self.assertIn('work_mode = "2"',(directory/'generated/ys_conf.ini').read_text())
                self.assertTrue(any('/algo/calib_reflectivity/' in c['file'] for c in commands))
                self.assertEqual(any('/algo/spot_slope_alog/' in c['file'] for c in commands),alias=='beta')
                self.assertEqual(any('/platform/beta/motor/' in c['file'] for c in commands),alias=='beta')

if __name__=='__main__': unittest.main()
