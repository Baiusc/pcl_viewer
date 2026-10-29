# pcl_viewer

# 激光雷达点云可视化助手（主要面向离线保存的单帧或多帧点云）（基于LIDAR原始点云解析）（支持 alpha转镜雷达、beta薄款转镜雷达、面阵全固态雷达（FL6031、FL6532芯片）

俯仰角范围：-12.2917° ~ 7.5°
俯仰角0度pixel_idx=59
---
graph TD
    A[开始] --> B{初始化参数};
    B --> C{For dist in list_dist};
    C --> D{For pan_angle in pan_angle_range};
    D --> E{For pitch_angle in list_theta};
    E --> F[俯仰转台旋转到pitch_angle];
    F --> G[采集不同pixel的行点云数据];
    G --> H{俯仰转台反向旋转到初始位置};
    H --> I[记录俯仰角数据];
    I --> E;
    E --> J{转盘旋转到pan_angle};
    J --> K[采集不同反射率的ROI云数据];
    K --> D;
    D --> L[轨道运动到dist];
    L --> M[采集不同在离下的ROI云数据];
    M --> C;
    C --> N[结束];

## 一、子目录说明
- `data`   数据
- `doc`   文档
- `include`   头
- `src`   源文件

## 二、Git
推送`git push origin main`

清理 .gitignore 中已被跟踪的文件：
```bash
git rm -r --cached . 
git add .
git commit 
```
其它常用指令：
```bash
# 添加远程仓库
git remote add github git@192.168.110.19:ys_algorithm/pcl_viewer.git
# 查看已添加的远程仓库
git remote -v
# 删除分支
git push github --delete main_github
# 查看所有分支
git branch -a
# 关联本地和远程分支
git branch --set-upstream-to=gitlab/dev dev
# 推送分支
git push github dev
# 合并dev到main
git checkout main
git pull gitlab main
git merge dev
# 忽略 data 目录并停止 Git 对该目录的跟踪
git rm -r --cached data 
git add .gitignore
git commit -m "Stop tracking data directory and add it to .gitignore"
git status
git push gitlab main
```


## 三、CMake
### 0.linux + vscode 自动化构建、编译、运行(调试)

克隆项目后，点击 `vscode`的`运行和调试`选项卡，选择 `构建+编译+调试`选项，脚本会自动构建、编译、运行(可调试)。

### 1.构建
先清除缓存变量 linux: `[ -d build ] && rm -rf build/*`  windows ：`rmdir /s /q build`

在主目录用命令行 `cmake -B build -S .`  

或者指定构建类型为`Debug`：`cmake -B build -S . -DCMAKE_BUILD_TYPE=Debug`

or指定构建类型为`Release`：`cmake -B build -S . -DCMAKE_BUILD_TYPE=Release`

### 2.编译
windows+vs:在主目录用命令行：`cmake --build build --config Debug -- /m:%NUMBER_OF_PROCESSORS%`

windows+vs:在主目录用命令行：`cmake --build build --config Release -- /m:%NUMBER_OF_PROCESSORS%`

linux+vscode:在主目录用命令行：`cmake --build build --config Debug -- -j$(nproc)`

linux+vscode:在主目录用命令行：`cmake --build build --config Release -- -j$(nproc)`


### 2.安装 （本算法项目未用到）
在主目录用命令行`cmake --install build --config Release`


## 四、vs中Release模式下调试代码

使用Release模式调试代码（Release模式下需要改动的几个设置选项）

> 配置都是选中：Release

1. 右键项目 -> C/C++ -> 常规 -> 调试信息格式 -> “程序数据库(/Zi)”
2. 右键项目 -> C/C++ -> 优化 -> 优化 -> “已禁用(/Od)”
3. 右键项目 -> 链接器 -> 调试 -> 生成调试信息 -> “生成调试信息(/DEBUG)”

> 解决，Release无法调试问题

CMAKE语句：
```bash
# Release模式下调试代码
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    # 配置 C/C++ 的调试信息格式为 “程序数据库(/Zi)”
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
    
    # 配置 C/C++ 的优化级别为 “已禁用(/Od)”
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Od")
    
    # 配置链接器生成调试信息 “生成调试信息(/DEBUG)”
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG")
endif()
```

## 五、将DLL复制到运行时输出目录

CMAKE语句：
```bash
# 将DLL复制到运行时输出目录
if (WIN32)
  add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:${PROJECT_NAME}> $<TARGET_FILE_DIR:${PROJECT_NAME}>
    COMMAND_EXPAND_LISTS
  )
endif ()
```

## 六、距离标定计算流程
### A版
1.算法输入：激光测距得到的r值 ，Lidar出的极坐标点云

2.寻找中心点，算出 d0：

找到零水平角所在的slot（从右往左0~799取中间的slot：idx = 399、400），

找到零俯仰角所在的pixel（从上到下0~95取中间的pixel：idx = 47、48），

在上述pixel内，找到距离为r的中心像素点center的极坐标值（R,θ） , 算出 d0（ R = r + d0 ，故 d0 = R - r）， 

3.计算 di , i = 0~95

暂不关心水平角，只关心slot[400]这一列中的96个pixel点：

di = d0 / cosφ

di = Ri - r / cosφ

### B版
1.算法输入：激光测距得到的r值 ，Lidar出的回波数据
再 二次水平角标定（墙面法向与4个平面的法向差异）

## 八、单独标定水平角
1.读取原始点云，不用标定。

2.在白墙上贴低反射率条，保存原始点云。初步框选目标所在的 slot_indices ，取出每行的峰值最低点(or用k-means聚类)，形成中轴线 (96个点)。

3.对中轴线按 slot_angle 取中位数/均值 median 。然后得到每行的水平角偏移值 cur_sa_offset = cur_slot_angle - median

## 九、距离标定的改进优化
1.我们认为墙面法向和雷达点云的X轴(第400个slot，极坐标水平角0度)是存在不可避免的误差的，不能忽略该误差。但可以用ROI区域的均值来平差

2.为了ROI均值平差，我们要保证ROI区域的中间slot就是测距照准的slot。可以通过在桌子上平放测距仪，照准反光条的水平中心点来实现。

3.我们认为雷达与墙面的Z轴一致的，忽略Z轴误差，在求单行点云的均值距离时，计算xy二维距离。

4.修改computeLinearity，计算sum_diff作为误差度，单行每个点的 dist = r*cos(theta)，diff = (dist - work_dist)^2
在 slot_center 中找到零俯仰角所在的 pixel_center（最接近0的，如果有两个，则取其一），

3.默认参数 d_defult = 5 mm ，d_offset = 0.5mm，d_start = d_defult - d_offset = 4.5mm 。 取出每一行，对于每一行，求直线度，

求直线度时，先根据 work_dist 确立 refer_line ，然后遍历单行的每一个点，求点到 refer_line 的距离, 得到均值距离作为直线度。

4.四片点云前后参差的问题。

方法一：在不同工作距离的场景中多跑几次算法，看96个最佳 r_offset 的变化

方法二：与水平角标定联合，微调水平角标定的结果

## 七、距离与二次水平角联合标定
### C版    联合标定
1.先 距离标定 （beta版）

2.再 二次水平角标定（墙面法向与4个平面的法向差异）

## 八、单独标定水平角
1.读取原始点云，不用标定。

2.在白墙上贴低反射率条，保存原始点云。初步框选目标所在的 slot_indices ，取出每行的峰值最低点(or用k-means聚类)，形成中轴线 (96个点)。

3.对中轴线按 slot_angle 取中位数/均值 median 。然后得到每行的水平角偏移值 cur_sa_offset = cur_slot_angle - median

## 九、距离标定的改进优化
1.我们认为墙面法向和雷达点云的X轴(第400个slot，极坐标水平角0度)是存在不可避免的误差的，不能忽略该误差。但可以用ROI区域的均值来平差

2.为了ROI均值平差，我们要保证ROI区域的中间slot就是测距照准的slot。可以通过在桌子上平放测距仪，照准反光条的水平中心点来实现。

3.我们认为雷达与墙面的Z轴一致的，忽略Z轴误差，在求单行点云的均值距离时，计算xy二维距离。

4.修改computeLinearity，计算sum_diff作为误差度，单行每个点的 dist = r*cos(theta)，diff = (dist - work_dist)^2