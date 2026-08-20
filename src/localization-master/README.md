# localization — 多传感器融合定位算法库

模块化 C++ 定位算法库。模块之间互相解耦：`sensors` / `estimator` / `io` 只依赖 `core`，组合只发生在最外层（demo 子项目或用户自建 main）。

## 模块边界

| 目录 | CMake 目标 | 职责 |
|------|------------|------|
| `modules/core` | `msf_core` | 共享底座：`math/`（常量/姿态/坐标/地球模型）、`types/`（消息/状态/输出契约）、`interfaces/`（`MeasFactor` 因子接缝）、`config/`（全局配置）、`msf_algorithms/`（ZUPT 等通用算法） |
| `modules/sensors/gnss` | `msf_gnss` | GNSS 质量门控、名义状态初始化、位置/速度/航向因子（只依赖 core） |
| `modules/sensors/ins` | `msf_ins` | IMU 补偿与名义状态递推（只依赖 core） |
| `msf_lidar` | LiDAR 前端（去畸变/滤波/外参）、ikd-Tree 局部地图、点面因子、pose_meas 位姿因子、点云可视化（只依赖 core） |
| `modules/estimator/ekf` | `msf_ekf` | 误差状态 EKF：`Ekf` 类（时间/量测更新、残差预检、误差反馈）+ 状态容器（只依赖 core） |
| `modules/io` | `msf_io` | YAML 配置读取（`GlobalConfig`）、离线 IMU / GNSS / Heading / LiDAR 数据读取（只依赖 core） |
| `demo/GNSS_INS` | `gnss_ins` | GNSS/INS 组合导航 demo（组合层） |
| `demo/LIO` | `lio` | LIO demo（GNSS 仅用于初始化，组合层） |
| `demo/GNSS_LIDAR_INS` | `gnss_lidar_ins` | GNSS/INS/LiDAR 组合导航 demo（点面 IEKF） |
| `demo/GNSS_LIO_INS` | `gnss_lio_ins` | GNSS/LIO/INS 组合导航 demo（pose_meas：GNSS 锁位置，LiDAR 默认只改正姿态） |
| `demo/unit_test` | 多个测试目标 | 单元测试（组合层） |

依赖规则：库模块只允许依赖 `msf_core` 与第三方库（CMake 层由 `msf_module()` 强制校验）；demo 与用户 main 可以任意组合模块。

## 已落地组件

| 组件 | 说明 |
|------|------|
| `msf::Ekf` | 误差状态 EKF 类：`TimeUpdate` / `MeasUpdate(H,R,Z)` / `MeasUpdate(MeasBlock)` / `CalMeasResidual` / `StateFeedback`；内部含 INS 误差时间模型与按块动态组装的 F/G/Q |
| `EkfState` | `NominalState` + `earth` + `ErrorState` + `StateLayout`（15/18 维，支持自定义块） |
| `MeasFactor::MeasBlock` | 传感器 → 优化器中性接缝：(H, R, z, timestamp)；因子通过 `BuildMeasBlock()` 产出 |
| `gnss::GnssPosFactor / GnssVelFactor / HeadingFactor` | GNSS 量测因子，数学与 POST_MSF `set_*_meas` 一致（含数值对比测试） |
| `gnss::InitializeNominal` | 前向搜索 + 名义状态初始化（与 POST_MSF 一致） |
| `InitializeLIO` | 静基座 IMU 均值初始化（调平 + 陀螺零偏；yaw=0；位置取 gloc_origin） |
| `algorithms::ZuptDetector` | 零速检测 + 零速量测构建（core/msf_algorithms，单文件单类） |
| `GlobalConfig` | 全量配置（yaml 根节点全小节 + calib.json 外参），纯头文件；main 一次加载后显式传参 |
| `ScanDeskewer` | LiDAR 帧内去畸变（位姿链 + Hermite 插值 + 反向补偿），只依赖 `InertialPropagator` 接口 |
| `LocalMap` | ikd-Tree 局部地图：FOV 滑动、平面匹配、增量插入 |
| `LidarVisualizer` | 世界系点云/轨迹可视化（旁路，不参与融合） |
| `EstimateLidarPose` / `LidarPoseFactor` | 副本点面 Gauss-Newton 产出 C_meas/p_meas/R_pose，再以 3/6 维 pose_meas 更新主滤波 |
| `DataReader` | 离线 IMU / GNSS / Heading / LiDAR 数据读取 |
| `Coordinate` | 坐标转换统一入口（lla/xyz/enu/utm，弧度约定） |
| `VehiclePose` | 车体导航输出契约（`types/output.h`） |

## Demo 与单元测试

demo 下每个子目录是一个独立小项目（各自维护 CMakeLists.txt）。一键脚本：

```bash
./demo/process.sh list                                  # 列出可用子项目
./demo/process.sh gnss_ins --config=config/parameter_yulin.yaml --output=demo/GNSS_INS/out
./demo/process.sh lio --config=config/parameter_yulin.yaml --output=demo/LIO/out
./demo/process.sh gnss_lidar_ins --config=config/parameter_yulin.yaml --output=demo/GNSS_LIDAR_INS/out
./demo/process.sh gnss_lio_ins --config=config/parameter_yankuang.yaml
./demo/process.sh test [CTest 正则]                     # 全部或单个测试
```

手动构建：

```bash
cmake -S . -B build -DLOC_BUILD_DEMO=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

可执行文件统一输出到 `build/bin/`。

## 解码 Cyber Record

需要能 `import cyber` / `modules.msgs.*_pb2`（先 `source /breton_pilot/scripts/env_setup.sh`）。
IMU / INSPVAX / calib / LiDAR 调用 `/breton_pilot` 已安装 BADP 工具；GNSS pos/vel/heading 为本仓库自研 CSV 解码。

```bash
python3 tools/decode_cyber/decode_main.py /abs/path/to/record.00000 --list
python3 tools/decode_cyber/decode_main.py /abs/path/to/record.00000
python3 tools/decode_cyber/decode_main.py /abs/path/to/record.00000 \
    --topics imu,gnsspos,gnssvel,heading,inspvax,gloc,calib,lidar_front \
    --output /abs/path/to/raw_data
```

默认写到 `<record 所在目录>/raw_data/`（与 BADP 解包目录一致）。详细类型与 channel 见 `tools/decode_cyber/decode_main.py` 头部说明。

画原始传感器（`tools/plot_raw_data/`，读 `raw_data/`，不依赖后处理 output）：

```bash
python3 tools/plot_raw_data/plot_imu.py testdata/xiwan/0702/201/raw_data
python3 tools/plot_raw_data/plot_gnss.py testdata/xiwan/0702/201/raw_data
python3 tools/plot_raw_data/plot_inspvax.py testdata/xiwan/0702/201/raw_data
python3 tools/plot_raw_data/plot_compare_inspvax_gnss.py testdata/xiwan/0702/201/raw_data
```

GNSS/INS 结果与 POST_MSF 参考组对比：

```bash
python3 tools/compare_result.py demo/GNSS_INS/ref_out/gloc_result.txt \
    demo/GNSS_INS/out/gloc_result.txt --tol=0.01
```

## 参考项目 POST_MSF

本仓库是 POST_MSF 的重构 / 移植版本，算法移植与数值基准均以 POST_MSF 为对照：

- 源码：`/home/liyanjie/breton/POST_MSF`
- 参考可执行：`/home/liyanjie/breton/POST_MSF/post_msf/build/post_msf`（已开启 GNSS_VEL 速度更新；原版备份 `/tmp/post_main_backup.cpp`）
- 参考配置 / 数据：`post_msf/config/parameter_yulin.yaml`、`/home/liyanjie/breton/POST_MSF/data/20260702/decode_cyber`

## 构建

```bash
cmake -S . -B build
cmake --build build -j
```

## 扩展方式

- 新增传感器：在 `modules/sensors/<name>` 建独立 target（只依赖 core），实现 `MeasFactor::BuildMeasBlock`，配置放 `GlobalConfig`。
- 新增优化器：在 `modules/estimator/<name>` 建独立 target（只依赖 core）。
- 通用算法（NHC / IGG-III 等）：放 `modules/core/msf_algorithms/`，每个算法一个文件。
- 组合：在 demo 子项目或用户 main 中装配。
