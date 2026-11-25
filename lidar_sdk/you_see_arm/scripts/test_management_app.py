#!/usr/bin/env python3
"""验证采集应用真实管理模块的通知、连续计数与 SIGTERM 处理。"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
SOURCE=r'''
#include "core/ys_management.h"
#include <assert.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>
int main(void)
{
    assert(ys_management_init()==0);
    assert(ys_management_enabled());
    ys_management_initialized(2);
    for(int i=0;i<5;++i)ys_management_progress(2,1);
    ys_management_progress(2,0);
    for(int i=0;i<9;++i)ys_management_progress(2,1);
    char byte;
    assert(recv(atoi(getenv("LIDAR_MGR_FD")),&byte,1,0)==1);
    ys_management_progress(2,1);
    while(!ys_shutdown_requested())usleep(1000);
    ys_management_close();
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='lm-app-') as folder:
    path=Path(folder);(path/'test.c').write_text(SOURCE)
    subprocess.run(['gcc','-std=gnu11','-Wall','-Wextra','-Werror','-I'+str(ROOT),str(path/'test.c'),str(ROOT/'core/ys_management.c'),'-o',str(path/'test')],check=True)
    parent,child=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
    with parent,child:
        env=dict(os.environ,LIDAR_MGR_FD=str(child.fileno()),LIDAR_MGR_TOKEN='a'*32)
        process=subprocess.Popen([str(path/'test')],env=env,pass_fds=(child.fileno(),))
        try:
            parent.settimeout(1)
            first=json.loads(parent.recv(4096));assert first['event']=='INITIALIZED'
            parent.settimeout(.05)
            try:parent.recv(4096);raise AssertionError('连续有效包计数不足时不能确认')
            except socket.timeout:pass
            parent.send(b'G')
            parent.settimeout(1)
            second=json.loads(parent.recv(4096));assert second['event']=='PIPELINE_PROGRESS' and second['valid_packets']==10
            assert second['token']=='a'*32 and second['work_mode']==2
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=2)==0
        finally:
            if process.poll() is None:process.kill();process.wait()
print('应用管理模块验证通过：私有通知通道、无效包清零、连续有效包计数和安全退出标志')
