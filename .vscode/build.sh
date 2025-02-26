BUILD_DIR="build"

# 检查是否存在该文件夹
if [ ! -d "$BUILD_DIR" ]; then
    mkdir "$BUILD_DIR"
else
    echo "Cleaning $BUILD_DIR directory..."
    rm -rf "$BUILD_DIR"/*
fi

# 进入 build 目录
cd "$BUILD_DIR"

# 获取当前脚本目录
Local_Dir=$(cd "$(dirname "$0")"; pwd)
echo "Now working in directory: $Local_Dir"

# 运行 cmake
echo "Running cmake..."
cmake -DCMAKE_BUILD_TYPE=Debug ..
if [ $? -ne 0 ]; then
    echo "CMake configuration failed!"
    exit 1
fi

# 运行 make，启用多线程编译
echo "Running make with multithreading..."
make -j$(nproc)
if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

echo "Build completed successfully!"