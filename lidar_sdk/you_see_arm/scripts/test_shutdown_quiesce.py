#!/usr/bin/env python3
"""编译真实状态机，验证硬件停采未实现时不会误报成功，线程仍被回收。"""
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]
SOURCE = r'''
#include "ys_business/ys_state_machine.h"
#include "core/ys_pthread.h"
#include <assert.h>
static int pool_stopped, threads_stopped, hardware_calls;
void log_log(int level, const char *file, int line, const char *fmt, ...) {}
void set_memory_pool_run_flag(uint8_t flag) { assert(flag == 0); ++pool_stopped; }
void thpool_exit(thpool_t *pool) { assert(pool == &data_thpool); ++threads_stopped; }
int sys_workmode_set(unsigned int state) { ++hardware_calls; return 0; }
int main(void)
{
    int result = stop_capture_for_shutdown();
#if LOCAL_TEST
    assert(result == 0);
#else
    assert(result == -1);
#endif
    assert(pool_stopped == 1 && threads_stopped == 1 && hardware_calls == 0);
}
'''
with tempfile.TemporaryDirectory(prefix='ys-quiesce-') as folder:
    root = Path(folder)
    source = root/'test.c'
    source.write_text(SOURCE)
    for model, local in [('CS96_BETA', 0), ('CS96_GAMMA', 0), ('CS96_BETA', 1)]:
        target = root/f'{model}-{local}'
        subprocess.run(['gcc', '-std=gnu11', '-ffunction-sections', '-fdata-sections',
            '-D'+model+'=1', f'-DLOCAL_TEST={local}', '-I'+str(ROOT),
            '-I'+str(ROOT/'driver/inc'), '-I'+str(ROOT/'third_party/libuv/include'),
            str(source), str(ROOT/'ys_business/ys_state_machine.c'),
            '-Wl,--gc-sections', '-pthread', '-o', str(target)], check=True)
        subprocess.run([str(target)], check=True)
print('停采保护验证通过：Beta/Gamma 硬件未确认时返回失败，线程仍回收，主机模拟不受影响')
