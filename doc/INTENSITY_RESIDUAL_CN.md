# 激光强度残差：抗几何退化实现与测试

日期：2026-09-20。修改基线：`24ee6ab`。测试环境：Ubuntu 22.04.5、ROS2 Humble、GCC 11.4、C++17、Release。

本次为 KILO-MAP 增加了**局部平面切向强度残差**，并接入原有 ESKF / IESKF。强度沿表面发生变化时，这个残差可以补充点到平面约束缺失的平面内运动信息。旧配置默认关闭；新增 `legkilo/config/mid360_intensity.yaml` 可直接启用。

用户目前没有实采 rosbag，因此测试范围是：完整工程编译、数值与异常输入测试、合成退化配准，以及直接调用 `KILO::process()` 的多帧集成测试。本文中的误差是合成场景的位姿误差，**不是实测轨迹 ATE / RPE**。尚未验证真实走廊、隧道、跨设备和长期运行效果。

## 1. 为什么需要增加独立的强度残差

原实现先尝试点到平面匹配，失败时才尝试 NDT。点云预处理保留了 `PointXYZINormal::intensity`，但原 `GaussPoint` 仅包含坐标和协方差，强度在进入前端高斯点云时丢失。

对世界系平面法向 $n$、中心 $q$，点到平面预测为：

$$
h_G = n^T(Rp_b+t-q).
$$

它对平移的导数为 $n^T$。平面内位移 $delta t$ 满足 $n^T\delta t=0$，因此无法通过这个观测消除。只根据强度给已有几何残差乘一个权重，不会产生新的雅可比方向，也不能恢复这些零空间。

新增强度预测的平移导数为表面切向梯度 $g^T$。只要 $g^T\delta t\ne0$，原先无法观测的移动就会引起强度误差。不同位置、不同方向的纹理共同提供更多约束；单一梯度方向仍有无法恢复的自由度。

联合几何与光度目标约束表面法向和切向，是 Colored Point Cloud Registration 的主要思路；COIN-LIO 则验证了强度光度观测融合至迭代滤波器的可行性。本项目采用直接在体素内拟合的三维局部强度模型，没有复现它们的完整图像处理或特征选择系统。参考：[Park 等，ICCV 2017](https://openaccess.thecvf.com/content_iccv_2017/html/Park_Colored_Point_Cloud_ICCV_2017_paper.html)、[Pfreundschuh 等，COIN-LIO](https://arxiv.org/abs/2310.01235)。

## 2. 数学模型和代码约定

### 2.1 固定尺度归一化

原始强度 $I$ 通过固定的 `intensity_scale` 归一化：

$$
\bar I = I/s.
$$

默认 $s=255$，适用于本项目 Livox 路径传入的反射率量纲。NaN、Inf、负值和超过 $s$ 的数值不参与强度建图和匹配，不做截断。零值有效，但整片常量强度无法通过梯度门控。不同雷达需要按驱动输出调整尺度；Ouster 现有代码仍取 `intensity`，未改成 `reflectivity`。

每帧单独做 min-max 归一化会改变地图与当前帧之间的强度对应关系，因此本实现保持固定尺度。这一步仅统一量纲，不等于距离、入射角或自动增益标定。

### 2.2 体素内局部线性拟合

每个含有效强度的父体素按需分配 `IntensityModel`。使用 Welford 中心化增量统计维护点位置均值 $\mu_p$、强度均值 $\mu_I$、位置协方差 $C_{pp}$、位置与强度交叉协方差 $C_{pI}$，无需保存历史点云。

令 $B\in\mathbb R^{3\times2}$ 为平面切向正交基：

$$
C_T = B^T C_{pp}B,\qquad
g = B C_T^{-1}B^T C_{pI}.
$$

通过二维切向拟合满足 $n^Tg=0$，避免把平面厚度中的噪声拟合为虚假的法向强度梯度。两个切向空间特征值都必须超过下限；采样近似一条线时直接拒绝，而不是用正则项创造约束。

模型输出：

$$
\hat I(p_w)=\mu_I+g^T(p_w-\mu_p),\qquad p_w=Rp_b+t.
$$

至少积累 `intensity_min_points` 个有效样本后拟合，最多累计 `intensity_max_points` 个。达到上限后统计冻结；地图体素被 LRU 淘汰时模型一起释放。几何平面仍使用已有的 `voxel_max_num` 规则。两个统计上限独立。

### 2.3 创新、雅可比与噪声

本项目 `ObsShared::pt_z` 存放“测量减预测”的创新，`pt_h` 是**预测函数**对误差状态的导数。右乘旋转扰动 $R\leftarrow R\operatorname{Exp}(\delta\theta)$ 下：

$$
z_I=\frac{\bar I-\hat I(Rp_b+t)}{\sigma_I},\qquad
H_I=\frac1{\sigma_I}\begin{bmatrix}-g^TR[p_b]_\times & g^T\end{bmatrix}.
$$

因此有限差分检验应比较 $H_I$ 与创新导数的负值。雅可比没有新增状态列，只使用现有 24 维状态的前 6 列位姿误差。

令 $N$ 为强度模型样本数，$s_f^2$ 为拟合残差平方和除以 $N-3$，$\sigma_m$ 为配置的测量标准差，$C_b$ 为当前点的机体系测量协方差，$d_T^2$ 为点相对强度中心的切向马氏距离平方：

$$
\sigma_I^2=\sigma_m^2+s_f^2+
\max(s_f^2,\sigma_m^2)\frac{1+d_T^2}{N}
+g^T R C_b R^T g.
$$

代码对最后一项取非负下限以处理舍入误差。拟合误差与有限样本的预测不确定性可减弱不可靠纹理的作用。没有再次将滤波器位姿协方差计入这个测量噪声。该噪声模型是工程近似，未包含完整的点间、几何与强度之间的相关性。

先按 `intensity_residual_gate` 拒绝过大的白化创新，再应用 Huber 权重。`intensity_weight` 为信息倍率 $\alpha$，`intensity_huber_delta` 为 $\delta$：

$$
R_I=\frac{\max(1,|z_I|/\delta)}{\alpha}.
$$

最终附加信息为 $H_I^TR_I^{-1}H_I$。噪声、对应关系和鲁棒权重在一次线性化内视为固定值；IESKF 下一轮重新计算。不会对这些权重继续求导。

### 2.4 匹配与拒绝条件

强度观测仅附加在已有有效 P2P 或 NDT 几何匹配的点上；它使用同一体素邻域搜索设置。候选必须满足：

1. 几何平面有效，强度模型点数和二维覆盖充分。
2. 梯度幅值在上下限之间，拟合 RMSE 与 $R^2$ 合格。
3. 到几何平面的距离不超过 `intensity_max_normal_distance`。
4. 切向马氏距离不超过 `intensity_max_mahalanobis`，到强度模型中心的欧氏距离不超过一个父体素边长。
5. 归一化强度有效，白化创新未超门限。

候选根据到模型中心的几何距离选择，选择后才计算强度误差。不会遍历选择“强度最相似”的体素来人为降低残差。强度与几何匹配独立选取局部模型，可能来自不同的相邻体素；跨结构边界仍是潜在误匹配来源。

## 3. 接入位置与修改文件

| 文件 | 修改内容 |
| --- | --- |
| `legkilo/src/common/pcl_types.h` | `GaussPoint` 增加原始强度，缺失默认 NaN；转换回 PCL 时保留强度 |
| `legkilo/src/core/slam/frontend/intensity_model.h/.cc` | 参数校验、归一化、增量拟合、切向梯度和预测方差 |
| `legkilo/src/core/slam/frontend/gaussian_voxel_map.h/.cc` | 按需建强度模型、邻域匹配、创新 / 雅可比 / 方差构造；NDT-only 配置也维护拟合所需平面 |
| `legkilo/src/core/slam/frontend/KILO.h/.cc` | 首帧、逐点处理、反投影、输出保留强度；附加观测；诊断统计 |
| `legkilo/config/mid360_intensity.yaml` | 从当前 Mid360 配置复制的启用示例，输出到独立 `intensity_trial` 目录 |
| `legkilo/test/test_intensity.cc` | 数值、退化配准、多帧前端和诊断 CSV 测试 |
| `legkilo/CMakeLists.txt`、`legkilo/test/CMakeLists.txt` | 编译新实现，并注册 4 组 CTest |

双阶段模式 `two_step_lidar_eskf: true`：Stage-1 保留原几何去畸变，Stage-2 将强度观测与几何观测一起迭代。单阶段模式：在 Stage-1 时间桶内附加强度观测。这样强度不会在两个阶段被重复当作独立测量使用；原工程自身的两阶段几何更新策略保持原样。

地图仅在当前帧估计完成后插入，残差不会与刚插入的当前帧自身匹配。查询接口为 const，逐点强度残差可沿用 TBB 并行处理。

`ProcessResult::intensity_count` 单独记录附加标量观测数；`success_pts_size` 和匹配类型仍反映几何匹配点，避免把一个点的两个观测误计为两个成功点。

当前策略是固定小权重与质量门控，没有加入几何退化特征向量投影或按退化程度自适应调权。抗退化来自新增观测的方向信息。信息矩阵特征值用于诊断，不作为触发开关。

## 4. 配置与运行

完整默认参数见 `legkilo/config/mid360_intensity.yaml`，核心参数如下：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `intensity_enable` | `false`，示例为 `true` | 是否建图并融合强度 |
| `intensity_scale` | 255 | 原始强度上界和归一化除数 |
| `intensity_min_points` / `intensity_max_points` | 12 / 200 | 每体素强度样本下限 / 累积上限 |
| `intensity_min_spatial_eigenvalue` | 0.0001 | 两个切向空间方差下限，单位 $m^2$ |
| `intensity_min_gradient` / `intensity_max_gradient` | 0.02 / 5.0 | 归一化强度每米的梯度范围 |
| `intensity_max_fit_rmse` | 0.04 | 归一化强度的拟合 RMSE 上限 |
| `intensity_min_fit_r2` | 0.3 | 最低解释比例 |
| `intensity_measurement_sigma` | 0.03 | 归一化强度标准差，平方后进入方差 |
| `intensity_max_normal_distance` | 0.1 | 法向匹配距离上限，米 |
| `intensity_max_mahalanobis` | 3.0 | 切向支持范围 |
| `intensity_residual_gate` | 4.0 | 白化创新的硬拒绝门限 |
| `intensity_weight` | 0.2 | 信息倍率，合法范围 $(0,1]$ |
| `intensity_huber_delta` | 1.5 | 白化创新的 Huber 阈值 |

新参数读入后会验证数值范围，拒绝零尺度、非正噪声、点数上下限错误等配置。启用双阶段强度时，IESKF 最大迭代次数必须至少为 1。

工作区已有依赖安装的复现命令：

```bash
cd /home/x/workspace/austin_code/kilo_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cmake -S src/kilo-map/legkilo -B build/legkilo -DLEGKILO_BUILD_TESTS=ON
cmake --build build/legkilo -j2
ctest --test-dir build/legkilo -R '^intensity_' --output-on-failure
```

使用刚编译的节点运行启用配置：

```bash
./build/legkilo/legkilo_node --config_file \
  "$PWD/src/kilo-map/legkilo/config/mid360_intensity.yaml"
```

本次已通过 `cmake --install build/legkilo` 更新当前工作区 install 中的节点与配置，可使用 `ros2 launch`。后续再次修改代码时，需要重新编译安装，例如在工作区执行：

```bash
colcon build --packages-select legkilo --parallel-workers 1 \
  --cmake-args -DLEGKILO_BUILD_TESTS=ON
source install/setup.bash
ros2 launch legkilo mid360.py \
  config_file:="$PWD/src/kilo-map/legkilo/config/mid360_intensity.yaml"
```

实际读取的两阶段开关名是 `two_step_lidar_eskf`。原 Mid360 文件中存在 `enable_second_step_eskf`，前端并未读取这个名称；新增示例显式设置了正确键。示例沿用现有话题和外参，需要与实际雷达安装一致。

## 5. 测试结果

完整工程编译成功，包括 `legkilo_node` 和测试可执行文件。**4/4 组 CTest 全部通过**，本次总测试墙钟时间约 0.08 秒；该时间只表示小规模合成测试耗时，不能换算为实采处理帧率。完整输出见 [测试原始记录](test_results/intensity_test_output.txt)。

| 合成场景 | 最终平移误差 | 最终旋转误差 | 最小信息特征值 | 强度观测数 |
| --- | ---: | ---: | ---: | ---: |
| 仅几何，二维纹理 | 100.000 mm | 1.432394° | 约 0 | 0 |
| 几何 + 强度，二维纹理 | **0.166 mm** | **0.002575°** | 2972.064 | 784 |
| 常量纹理，强度关闭 | 100.000 mm | 1.432394° | 约 0 | 0 |
| 常量纹理，强度开启 | 100.000 mm | 1.432394° | 约 0 | 0 |
| 只有 x 方向梯度，强度开启 | 60.000 mm | 0.001759° | 约 0 | 784 |

无噪声线性纹理条件十分理想，因此亚毫米误差只用于确认残差与优化行为，不能作为真实雷达精度预期。只沿 x 变化的强度未恢复 y 方向位移；这说明增加强度并不自动使所有自由度可观测。

加入归一化强度标准差 0.01（原始量纲约 2.55）的噪声后，种子 0–9 的 10 次运行全部通过：平均平移误差 **1.838 mm**，最大 **3.108 mm**，最大旋转误差 **0.071404°**。解析雅可比与六维中央差分的最大绝对差为 **$5.8431\times10^{-10}$**，低于 $10^{-6}$ 判定阈值。

16 种前端组合全部通过：4 个“强度开启且有纹理”的组合各累计使用 1323 条强度观测（3 个非初始化帧，每帧 441 条），其余组合均为 0；输出点云强度与 CSV 观测计数检查通过。

合成配准使用 4×4 个 1 米体素、每体素 7×7 个点，共 784 个点，全部处于 $z=1.25$ 的同一平面。相邻体素交替使用沿 x / y 的线性强度纹理，归一化梯度为 0.4/m。真实位姿为单位变换，初始误差为平移 $(0.08,-0.06,0.02)$ 米、yaw 0.025 弧度，初始状态协方差为 $0.2I$，最多 10 次 IESKF 迭代。

配准测试调用生产版地图残差与 `ESKF::updateByCloud()`，几何观测使用 Stage-2 噪声与阈值 3 的 Cauchy 权重。它隔离验证残差与滤波器的作用，不模拟真实雷达扫描运动、IMU 或后端。多帧集成测试另外直接调用生产版 `KILO::process()`；采用 LO 模式和微小逐帧移动，覆盖单 / 双阶段、强度开 / 关、平坦 / 有纹理、P2P / NDT-only 的 16 种组合，每组 4 帧，并检查 CSV 列与实际观测数一致。

测试组细节：

- `intensity_model`：中心化拟合、大坐标偏移、样本上限、共线 / 稀疏 / 常量 / 高噪声 / 过大梯度拒绝、非法参数、缺失强度、NaN / Inf / 越界值、Huber 降权、法向与支持范围门控、尺度一致性、PCL 强度保留、并行查询、NDT-only 支持、几何残差兼容。
- `intensity_jacobian`：非单位旋转与非零平移处的 6 维右扰动中央差分，步长 $10^{-6}$；在零创新处检查，避免将冻结权重的导数误算进雅可比。
- `intensity_registration`：同一几何下对比开关；验证常量纹理回退、单方向纹理仍退化；加入归一化标准差 0.01 的独立强度噪声，运行固定种子 0–9。
- `intensity_frontend`：64 个合成输入帧，检查完整前端路径、观测调度、输出强度与新增诊断 CSV。

## 6. 如何判断实采数据是否有效

诊断文件位于 `legkilo/result/<temp_result_save_folder>/frontend_diagnostics.csv`。新增字段为：

| 字段 | 解释 |
| --- | --- |
| `intensity_count` | 本帧最终使用的强度观测数；双阶段取最后一次迭代，单阶段汇总时间桶 |
| `intensity_residual_median` / `intensity_residual_p95` | 白化创新绝对值统计 |
| `intensity_weight_mean` | 平均有效权重，已包含信息倍率 $\alpha$ |
| `geometry_information_min_eigenvalue` | 几何观测信息矩阵最小特征值 |
| `information_min_eigenvalue` | 几何加已采用强度观测的信息矩阵最小特征值 |

初始化帧或没有观测时，部分统计为空。信息矩阵混合旋转和平移单位，其特征值与点数、尺度有关；只能在相同设置下辅助比较。单阶段跨时间桶累加的矩阵还使用了各桶的线性化位姿，不应解释为精确的同一时刻可观测性分析。融合后特征值上升也不能单独证明匹配正确或轨迹更准。

实采 A/B 比较应使用两份完全相同的传感器、几何、下采样与后端配置，仅改变 `intensity_enable` 并分开输出目录。记录有效强度比例、残差、弱方向漂移、帧耗时；有真值再计算同一对齐方式下的 ATE / RPE。先确认采集的是稳定强度 / 反射率，再调整 `scale`、测量噪声和拟合门控；不要为了增加观测数直接放宽所有门限。

若 `intensity_count` 长期为零，可依次检查字段与量纲、每体素点数、切向覆盖、局部梯度和拟合误差。地图初期尚未积累足够样本时，计数为零是正常现象。

## 7. 使用边界与后续工作

1. **无纹理不恢复。** 全部强度相同、梯度方向单一或纹理重复时，部分自由度依然不可观测。本实现会对弱纹理回退，但无法保证识别所有错误的重复纹理对应。
2. **不是辐射标定。** 没有距离 / 入射角补偿、跨帧增益偏置估计；缓慢而一致的强度偏差可能通过门控并造成位姿偏差。
3. **局部收敛。** 强度线性模型、体素边界和支持范围限制了可捕获位移，不能替代大范围重定位。几何完全无匹配时不会启动纯强度定位。
4. **地图会老化。** 模型样本达到上限后冻结，无时间衰减或重建检测；长期材质变化、动态物体需额外处理。
5. **先验与数据相关性。** 未重构现有两阶段滤波和地图估计相关性，新增信息量不应视为严格标定后的独立概率观测。
6. **性能尚无实采结论。** 每个含有效强度的体素增加固定大小统计量；每个候选点增加一次邻域搜索。未在实际传感器数据上验证实时性、长期内存与轨迹收益，也未编译 ROS1。

后续优先用同一段实际走廊 / 隧道数据做开关对比，再决定是否加入反射率校正、跨帧仿射亮度参数和基于弱方向的自适应选点与权重。
