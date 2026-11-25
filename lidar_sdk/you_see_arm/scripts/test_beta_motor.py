#!/usr/bin/env python3
"""编译真实电机源码，用链接包装隔离硬件并验证启停和失败退出。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class BetaMotorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix='beta-motor-test-')
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.program = Path(cls.temporary.name) / 'demo_i2c_motor'
        calls = ('open', 'ioctl', 'write', 'close', 'sleep', 'usleep')
        subprocess.run([
            'gcc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-O2',
            str(ROOT / 'platform/beta/motor/demo_i2c_motor.c'),
            str(ROOT / 'scripts/fixtures/beta_motor_io.c'),
            *['-Wl,--wrap=' + name for name in calls], '-o', str(cls.program),
        ], check=True, capture_output=True)

    def invoke(self, scenario='success', action=''):
        return subprocess.run([str(self.program), *([action] if action else [])],
                              capture_output=True, text=True, timeout=3,
                              env=dict(os.environ, MOTOR_TEST_CASE=scenario, MOTOR_TEST_ACTION=action))

    def test_registers_speed_pulse_and_descriptor_zero(self):
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        writes = [line for line in result.stderr.splitlines() if line.startswith('REG_WRITE:')]
        self.assertEqual(writes, ['REG_WRITE:04=03', 'REG_WRITE:02=4a', 'REG_WRITE:03=40'])
        self.assertEqual(result.stderr.count('POLL\n'), 1)
        self.assertIn('GPIO_VALUE:1\nGPIO_VALUE:0', result.stderr)
        self.assertIn('I2C_CLOSED', result.stderr)

    def test_already_exported_gpio_is_accepted(self):
        result = self.invoke('export_busy')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('GPIO_VALUE:0', result.stderr)

    def test_speed_may_settle_after_five_samples(self):
        result = self.invoke('delayed')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr.count('POLL\n'), 7)

    def test_timeout_is_failure_without_gpio_pulse(self):
        result = self.invoke('timeout')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stderr.count('POLL\n'), 10)
        self.assertNotIn('GPIO_VALUE:', result.stderr)
        self.assertIn('I2C_CLOSED', result.stderr)

    def test_open_error_is_failure(self):
        result = self.invoke('open_fail')
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('I2C_CLOSED', result.stderr)

    def test_io_errors_and_partial_transfers_are_failures(self):
        for scenario in ('setup_fail', 'write_fail', 'write_partial', 'read_fail', 'read_partial',
                         'export_fail', 'gpio_partial', 'close_fail', 'sleep_fail'):
            with self.subTest(scenario=scenario):
                result = self.invoke(scenario)
                self.assertNotEqual(result.returncode, 0, result.stderr)
                self.assertIn('I2C_CLOSED', result.stderr)

    def test_pulse_failure_attempts_to_restore_low_level(self):
        for scenario in ('gpio_fail', 'pulse_fail'):
            with self.subTest(scenario=scenario):
                result = self.invoke(scenario)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('GPIO_VALUE:0', result.stderr)
                self.assertIn('I2C_CLOSED', result.stderr)

    def test_stop_waits_for_zero_without_gpio_pulse(self):
        result = self.invoke('delayed', 'stop')
        self.assertEqual(result.returncode, 0, result.stderr)
        writes = [line for line in result.stderr.splitlines() if line.startswith('REG_WRITE:')]
        self.assertEqual(writes, ['REG_WRITE:02=00', 'REG_WRITE:03=00', 'REG_WRITE:04=03'])
        self.assertEqual(result.stderr.count('POLL\n'), 7)
        self.assertNotIn('GPIO_VALUE:', result.stderr)

    def test_stop_fails_if_still_moving_or_i2c_fails(self):
        for scenario in ('timeout', 'write_fail', 'read_fail'):
            with self.subTest(scenario=scenario):
                result = self.invoke(scenario, 'stop')
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertNotIn('GPIO_VALUE:', result.stderr)
                self.assertIn('I2C_CLOSED', result.stderr)
                if scenario == 'timeout':
                    self.assertEqual(result.stderr.count('POLL\n'), 10)


if __name__ == '__main__':
    unittest.main()
