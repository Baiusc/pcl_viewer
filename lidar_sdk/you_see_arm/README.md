<!--
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 03:59:12
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-04-09 02:08:34
 * @FilePath     : /you_see_arm/README.md
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
-->

# 编译环境

## 自动安装编译环境

初次编译可直接执行 sh 目录下的初始化脚本进行环境的初始化

```shell
sudo ./sh/init_env.sh
```

请保持网络条件畅通，如果需要手动编译请查看 [手动安装化编译环境](#手动安装编译环境) 。

自动安装编译环境后即可进行应用程序编译 [编译应用程序](#编译应用程序)

## 手动安装编译环境

### libuv

本项目使用 libuv 库进行事件管理，使用前请安装 libuv

#### 本地环境编译 libuv

```shell
sudo apt install autoconf libtool -y
git clone https://github.com/libuv/libuv.git
cd libuv/
sh autogen.sh
./configure
make
make check
sudo make install
```

#### 交叉编译工具链编译 libuv

```shell
mkdir build
cd build
```

在 build 文件夹下创建一个文件 toolchain.cmake，内容为：

```shell
SET(CMAKE_SYSTEM_NAME Linux)

# specify the cross compiler
SET(CMAKE_C_COMPILER   /home/vot/aarch64-buildroot-linux-gnu_sdk-buildroot/bin/aarch64-buildroot-linux-gnu-gcc)
SET(CMAKE_CXX_COMPILER /home/vot/aarch64-buildroot-linux-gnu_sdk-buildroot/bin/aarch64-buildroot-linux-gnu-g++)

# specify where to search for the target environment
SET(CMAKE_FIND_ROOT_PATH  /home/vot/aarch64-buildroot-linux-gnu_sdk-buildroot/)
```

然后编译

```shell
cmake .. -DCMAKE_TOOLCHAIN_FILE=./toolchain.cmake
```

### D-Bus

本项目使用 D-Bus 作为进程间通信的方式

编译 D-Bus 前需要安装 expat

下载：https://distfiles.macports.org/expat/expat-2.2.9.tar.bz2

#### 本地环境编译 expat

```shell
export CFLAGS=""
./configure
make
sudo make install
```

#### 交叉编译工具链编译 expat

```shell
mkdir build
export CFLAGS=""

./configure --prefix=/home/vot/Documents/Tools/expat-2.2.9/build --host=aarch64-buildroot-linux-gnu \
CC=/home/vot/aarch64-buildroot-linux-gnu_sdk-buildroot/bin/aarch64-buildroot-linux-gnu-gcc

make
make install
```

编译成功后将 build 下的编译文件都复制到交叉编译工具的目录中

下载 D-Bus：https://dbus.freedesktop.org/releases/dbus/dbus-1.12.0.tar.gz

#### 本地环境编译 D-Bus

```shell
sudo apt-get install pkg-config -y
./configure
make
sudo make install
sudo ln -s /usr/local/lib/libdbus-1.so /usr/local/lib/libdbus.so
```

#### 交叉编译工具链编译 D-Bus

```shell
sudo apt-get install pkg-config

mkdir build

CFLAGS="-I/home/vot/aarch64-buildroot-linux-gnu_sdk-buildroot/include" LDFLAGS="-L/home/vot/aarch64-buildroot-linux-gnu_sdk-buildroot/lib" \
./configure --prefix=/home/vot/Documents/Tools/dbus-1.12.0_arm/build --host=aarch64-buildroot-linux-gnu \
CC=/home/vot/aarch64-buildroot-linux-gnu_sdk-buildroot/bin/aarch64-buildroot-linux-gnu-gcc \
CXX=/home/vot/aarch64-buildroot-linux-gnu_sdk-buildroot/bin/aarch64-buildroot-linux-gnu-g++

make
make install
```

### ~~编译 jemalloc~~ (已弃用)

```shell
./configure --with-jemalloc-prefix=je_ --prefix=/home/vot/jemalloc_install --host=aarch64-buildroot-linux-gnu CC=/home/vot/aarch64-buildroot-linux-gnu_sdk-buildroot/bin/aarch64-buildroot-linux-gnu-gcc CXX=/home/vot/aarch64-buildroot-linux-gnu_sdk-buildroot/bin/aarch64-buildroot-linux-gnu-g++
make -j4
make install
```

## 编译应用程序

统一使用 CMake 构建预设，机型、数据模式和算法关系定义在 `configs/build_catalog.json`。

```shell
python3 scripts/build.py list
python3 scripts/build.py build beta-pointcloud
python3 scripts/build.py build beta-pointcloud-spotslope
python3 scripts/build.py package beta-gray
```

调试版在预设名后追加 `-debug`。程序位于 `build/<预设>/bin/you_see`，完整包位于 `dist/<预设>/`。也可使用 `make PRESET=beta-gray`；旧的 `make MODEL=...` 请改用明确预设。

构建不会自动切换工具链或驱动分支。首次使用 MF150 时，可先执行 `python3 scripts/build.py prepare MF150-fl6031` 或 `python3 scripts/build.py prepare MF150-fl6532`，从本地分支准备独立依赖目录。

完整的组合表、算法边界、依赖准备与验证方法见 [构建管理](doc/构建管理.md)。

# 测试

## 单元测试

单元测试使用 gtest，按照以下步骤安装

gtest 的源码编译安装需要使用 cmake，请先安装 cmake

```shell
sudo apt-get install libssl-dev cmake -y
在官网下载 cmake 代码 https://cmake.org/download/
解压
cd cmake-3.29.3/
./bootstrap
make -j8
sudo make install
cmake --version
```

### 本地环境编译 gtest

```shell
git clone https://github.com/google/googletest.git
cd googletest/
mkdir build
cd build/
cmake ..
make
sudo make install
```

### 使用

```shell
cd test/build
cmake ..
make

# 测试所有项目
ctest

# 单元测试
./某个单元测试程序
```

## 内存泄漏检测

内存泄漏检测使用 Valgrind，按照以下步骤安装

### 本地环境编译

```shell
在官网下载代码 https://valgrind.org/downloads/current.html#current
tar -xvf valgrind-3.23.0.tar.bz2
cd valgrind-3.23.0/
sudo apt install automake
./autogen.sh
./configure
make -j8
sudo make install
sudo apt-get install libc6-dbg
```

### 使用

```shell
valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all ./sample
```

## 静态检查

静态检查使用 splint，按照以下步骤安装

```shell
sudo apt install splint
```

使用

```shell
splint -strict main.c
```
