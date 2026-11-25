# CS96 Beta 光斑斜率修正编译与测试

## 代码版本

修改前基线为 `c2f5dae768a96ffb407f9cec58466626e5090343`，原分支为 `dev/fs_sdk`。
算法改动位于 `feat/cs96-beta-spot-slope`，提交前改动仍属于工作区；切换分支不会自动隔离未提交改动。
`algo/spot_slope_alog` 是独立 Git 子模块，本次修改了其接口头文件和实现文件，
也建立了同名功能分支。保存版本时先提交子模块接口改动，再由主仓提交子模块引用
和业务代码；共享时两个仓库的提交都需要推送。
如需同时保留两个源码目录，可从基线创建独立工作树，并初始化其所需子模块：

```sh
git worktree add --detach ../you_see_arm-baseline c2f5dae768a96ffb407f9cec58466626e5090343
```

日常测试推荐同一功能分支通过编译开关生成两种版本。关闭开关保留原有水平角计算；
它与历史提交的二进制不保证逐字节一致，版本信息和句柄清理等代码已有变化。

## 编译

```sh
# 原有水平角计算，默认也是关闭
make MODEL=CS96-beta BETA_SPOT_SLOPE=0 -j4
mkdir -p build/beta-spot-slope/off
cp you_see build/beta-spot-slope/off/you_see
cp lib/CS96-beta/liblidar.so build/beta-spot-slope/off/liblidar.so

# 启用 Beta 光斑斜率修正
make MODEL=CS96-beta BETA_SPOT_SLOPE=1 -j4
mkdir -p build/beta-spot-slope/on
cp you_see build/beta-spot-slope/on/you_see
cp lib/CS96-beta/liblidar.so build/beta-spot-slope/on/liblidar.so
```

切换机型、算法开关或编译参数会通过构建配置文件触发重编。由于源码目录内共享目标文件，
不要在同一工作目录同时编译两个配置。其他机型使用原来的 `make MODEL=机型`；
非 Beta 机型不能指定 `BETA_SPOT_SLOPE=1`。Alpha 原有修正行为不受这个开关控制。

产物目录中的驱动库只是配套留档。部署仍需沿用项目的程序和库目录布局，
不能把该目录当成完整安装包。

## 标定和实际行为

实机读取 `/etc/spot_slope_map.bin`，本地测试读取 `./file/spot_slope_map.bin`。
开启开关但文件不存在时保持原角度；文件存在但校验失败时初始化失败。
关闭开关沿用原有初始化逻辑，但 Beta 水平角不调用修正。

Beta 非零 SLOT 的输入角为 `H = 2 × wrap(A - S[face]) + omega0`，单位为度。
表中 `angles` 对应 H，`diffs` 为目标角减 H，输出为 H 加插值修正量，之后才加逐线补偿。
不得直接沿用未经核对的 Alpha 标定数据；调整角度基准后需要重新核对标定表。
SLOT 0 保持原行为，不参与修正。

二进制格式沿用小端格式：一字节面数；每面包含一字节面号、两字节起始编码值、
两字节点数，以及若干对四字节浮点角度和修正量。必须恰好包含面 0～3，
每面至少两个点，角度和修正量必须有限，角度严格递增。Beta 加载时按面号排序，
起始编码值仅兼容文件格式，不参与选面。超出表范围时沿用算法库的线性外推。

当前 Beta 驱动配置为灰度接收模式 5。灰度和原始数据透传不会执行正式点云角度修正。
实机效果验证需要另行配套点云采集驱动、工作模式及 Beta 标定表。

## 验证

```sh
python3 scripts/test_beta_spot_slope.py
```

脚本在临时目录编译真实算法库，检查面排序、固定修正量、零修正、边界外推、非法面号，
以及截断、缺面、重复面、非递增角度、非有限修正量和文件缺失的拒绝行为。
实机还应以四面点云检查修正方向、跨零连续性和帧处理耗时。
