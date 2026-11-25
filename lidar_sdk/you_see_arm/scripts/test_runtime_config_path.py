#!/usr/bin/env python3
"""用真实配置加载和参数写回函数，验证管理运行配置不会写回系统默认路径。"""
from pathlib import Path
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
SOURCE=r'''
#include "ys_business/ys_ps_config.h"
#include "common/ys_protocal.h"
#include "core/ys_user_settings.h"
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
static const char *expected_path;
static int last_ack;
FILE *__real_fopen(const char *path,const char *mode);
FILE *__wrap_fopen(const char *path,const char *mode)
{
    if (strchr(mode,'w')) assert(expected_path && !strcmp(path,expected_path));
    return __real_fopen(path,mode);
}
int pack(void *arg,uint8_t *data,uint16_t length,uint8_t type)
{ (void)arg;(void)length;(void)type;last_ack=*data;return 0; }
int main(int argc,char **argv)
{
    assert(argc==3);
    assert(!strcmp(ys_runtime_config_path(),DEFAULT_CONF_PATH));
    expected_path=argv[1];
    assert(!setenv("YS_CONFIG_PATH",expected_path,1));
    assert(load_config()==0);
    assert(!strcmp(ys_runtime_config_path(),expected_path));
    unsetenv("YS_CONFIG_PATH");
    uint8_t request[]={0,0,0,1};
    management_id_t message={.buf=request};
    set_echo_parm(&message);
    request[2]=1;
    set_coordinate_mode(&message);
    dictionary *saved=iniparser_load(expected_path);
    assert(saved);
    assert(iniparser_getint(saved,"system:echo_type",-1)==1);
    assert(iniparser_getint(saved,"system:coordinate_mode",-1)==1);
    assert(!strcmp(iniparser_getstring(saved,"network:center_ip",""),"10.42.0.145"));
    iniparser_freedict(saved);
    assert(!setenv("YS_USER_SETTINGS_PATH",argv[2],1));
    request[2]=1;request[3]=0;
    set_echo_parm(&message);
    assert(last_ack==0);
    ys_user_settings_t persisted;
    assert(ys_user_settings_load(argv[2],&persisted)==0);
    assert(persisted.echo_mode==1 && persisted.echo_type==0 && persisted.coordinate_mode==1);
    struct stat before,after;
    assert(!stat(argv[2],&before));
    set_echo_parm(&message);
    assert(!stat(argv[2],&after) && before.st_ino==after.st_ino);
    int broken=open(argv[2],O_WRONLY|O_TRUNC);assert(broken>=0);
    assert(write(broken,"broken",6)==6);close(broken);
    request[2]=0;
    set_echo_parm(&message);
    assert(last_ack!=0 && config.echo_mode==1);
    assert(!setenv("YS_CONFIG_PATH","relative.ini",1));
    assert(load_config()==-1);
    assert(!strcmp(ys_runtime_config_path(),expected_path));
    char too_long[PATH_MAX+1];memset(too_long,'x',sizeof(too_long)-1);too_long[0]='/';too_long[PATH_MAX]=0;
    assert(!setenv("YS_CONFIG_PATH",too_long,1));
    assert(load_config()==-1);
    destroy_config();
}
'''
with tempfile.TemporaryDirectory(prefix='ys-runtime-config-') as temp:
    folder=Path(temp);source=folder/'test.c';source.write_text(SOURCE)
    config=folder/'runtime.ini';config.write_text('[system]\nwork_mode="2"\n[network]\ncenter_ip="10.42.0.145"\n')
    target=folder/'test'
    subprocess.run(['gcc','-g','-fsanitize=address','-std=gnu11','-DCS96_BETA=1','-DLOCAL_TEST=0','-DYS_WORK_MODE_MASK=15',
        '-ffunction-sections','-fdata-sections','-I'+str(ROOT),'-I'+str(ROOT/'driver/inc'),
        '-I'+str(ROOT/'third_party/libuv/include'),str(source),str(ROOT/'ys_business/ys_ps_config.c'),
        str(ROOT/'core/ys_conf.c'),str(ROOT/'core/ys_user_settings.c'),str(ROOT/'third_party/cJSON/cJSON.c'),str(ROOT/'third_party/iniparser/iniparser.c'),
        str(ROOT/'third_party/iniparser/dictionary.c'),str(ROOT/'third_party/log.c/log.c'),
        '-Wl,--gc-sections','-Wl,--wrap=fopen','-pthread','-lm','-o',str(target)],check=True)
    result=subprocess.run([str(target),str(config),str(folder/'user-settings.json')],cwd=folder,capture_output=True,text=True)
    if result.returncode:
        print(result.stdout);print(result.stderr);raise SystemExit(result.returncode)
print('配置验证通过：实际读写路径隔离、用户参数原子保存、重复设置不重写、损坏设置拒绝且不改变内存参数')
