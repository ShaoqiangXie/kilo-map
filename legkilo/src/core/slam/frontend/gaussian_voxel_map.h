// SPDX-License-Identifier: MIT
// @file gaussian_voxel_map.h
// @brief 高斯体素地图：每个体素维护一个"平面 (Plane)" 和一个 3×3×3 的子体素栅格 (SubGrid)，
//        分别支撑 Point-to-Plane (P2P) 与 NDT (分布到分布) 两种残差。
// @author Ou Guangjun
// @created 2025-11-29
// @maintainer ouguangjun98@gmail.com
//
// -----------------------------------------------------------------------------
// 【地图结构总览】
//
//   世界空间被 voxel_size (~0.5m) 的立方体划分成父体素 (Voxel)。
//   每个父体素内部再有 2×2×2 = 8 个"子体素" (SubVoxel)：
//
//         Voxel (edge = voxel_size)
//         ┌───────────┬───────────┐
//         │  SubVoxel │  SubVoxel │      每个 SubVoxel 记录：
//         ├───────────┼───────────┤        count / mean / cov / noise_sum
//         │  SubVoxel │  SubVoxel │      → 供 NDT 拟合小尺度分布
//         └───────────┴───────────┘
//
//   同时每个 Voxel 维护一个 Plane：把落入该体素的所有点做增量协方差 → 特征分解 → 拟合平面，
//   用于 Point-to-Plane 残差。
//
// 【残差构造】
//   - buildPoint2PlaneResidual： 邻域搜索 nearby_grids_ (CENTER/6/26) 内的父体素平面，
//                                挑选一个最匹配的返回 (r=有符号距离, J=对 δθ,δp 的雅可比)
//   - buildNdtResidual        ： 邻域搜索父体素的 SubGrid，找到最近的 SubVoxel 作为高斯分布，
//                                通过 Cholesky 白化后返回 3D 残差
//
// 【存储与淘汰】
//   voxel_list_ (std::list) + voxel_map_ (unordered_map<VoxelKey, list iterator>)
//   = LRU：新插入或再访问的体素被 splice 到链表头，超过 capacity 时淘汰链表尾。
//
// 【线程模型】
//   - insertPoints：并行分组 → 串行写入（每个体素单独 addPoint / 平面 update）
//   - build*Residual：完全 const，可被 tbb::parallel_for 并发调用
// -----------------------------------------------------------------------------
#ifndef LEG_KILO_GAUSSIAN_VOXEL_MAP_H
#define LEG_KILO_GAUSSIAN_VOXEL_MAP_H

#include <array>
#include <limits>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

#include "common/math_utils.hpp"
#include "common/pcl_types.h"
#include "core/slam/frontend/voxel_map_utils.hpp"
#include "core/slam/frontend/intensity_model.h"

namespace legkilo {
// 体素键：把 3D 整数索引 (ix, iy, iz) 编码进一个 int64（各占 21 位，有偏移），详见 math_utils.hpp
using VoxelKey = int64_t;

// 近邻搜索模板：CENTER=只看自己；NEARBY6=面邻居；NEARBY26=面/边/角所有 3×3×3-1 邻居
enum class NearByType { CENTER = 0, NEARBY6, NEARBY26 };

// ============================================================
// 【结构体】SubVoxel
// 作用：3×3×3 子网格里的单个小体素，维护该子体素内所有点的 (count, mean, cov)。
//       为 NDT 提供小尺度局部高斯分布。
// 关键成员：
//   - count      : 点数
//   - mean       : 当前均值 μ
//   - cov        : Welford 增量算出的样本协方差 Σ (人口协方差)
//   - noise_sum  : 累积的点测量噪声之和，NDT 拟合时用 noise_sum/count 做去噪补偿
// 【关键算法】Welford / Chan 增量协方差公式（数值稳定，避免二次差近乎相等相消）：
//   μ_{n+1} = μ_n + (x - μ_n) / (n+1)
//   Σ_{n+1} = (n/(n+1)) Σ_n + (1/(n+1)) (x - μ_n)(x - μ_{n+1})^T
// ============================================================
struct SubVoxel {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t count = 0;                                              // 已累积的点数
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();                // 均值 μ
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();                 // 人口协方差 Σ
    Eigen::Matrix3d noise_sum = Eigen::Matrix3d::Zero();           // 点测量噪声累积和

    inline static double kJitter = 1e-6;  // 初始化时给 cov 加一点扰动，防止 Cholesky 失败

    // 增量加入一个点 (p, noise)。整个过程 O(1)，无需保存历史点。
    inline void addPoint(const Eigen::Vector3d& p, const Eigen::Matrix3d& noise) {
        const double n_old = static_cast<double>(count);
        const double n_new = n_old + 1.0;
        if (count == 0) {
            // 【首点】：μ = p，Σ 初始化为 jitter·I（避免后续白化时数值失败）
            mean = p;
            cov = Eigen::Matrix3d::Identity() * kJitter;
            noise_sum = noise;
            count = 1;
            return;
        }
        // 【Welford / Chan 更新】
        const Eigen::Vector3d delta = p - mean;                 // x - μ_n (更新前偏差)
        const Eigen::Vector3d mean_new = mean + delta / n_new;  // μ_{n+1}
        const Eigen::Vector3d delta2 = p - mean_new;            // x - μ_{n+1} (更新后偏差)
        // Σ_{n+1} = (n/(n+1)) Σ_n + (1/(n+1)) δ δ'^T   (population covariance)
        cov *= (n_old / n_new);
        cov.noalias() += (delta * delta2.transpose()) / n_new;
        noise_sum += noise;
        mean = mean_new;
        count += 1;
    }
};

// ============================================================
// 【结构体】SubGrid
// 作用：把父体素细分成 kDim×kDim×kDim 个子体素（当前 kDim=2 ⇒ 8 个 cell），
//       为 NDT 提供"父体素内多个高斯分布"，从而应对同一体素内跨越多平面/结构的情况。
// 关键成员：
//   - cells          : 8 个 SubVoxel，按 (ix,iy,iz) → flatten 索引
//   - occupancy_mask : 位掩码，bit i = 1 表示 cell[i] 至少收到过一个点
//   - active_count   : 已占用的 cell 数
//   - origin         : 父体素的最小角落坐标 (世界系)
//   - sub_size       : 子体素边长 = parent_voxel_size / kDim
// 【注意】原注释写的是"3×3×3"，但 kDim=2 ⇒ 实际是 2×2×2 = 8 个 cell（以代码为准）。
// ============================================================
struct SubGrid {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr int kDim = 2;                       // 3x3x3
    static constexpr int kCellNum = kDim * kDim * kDim;  // 27

    std::array<SubVoxel, kCellNum> cells;
    uint32_t occupancy_mask = 0;  // 位 i = 1 表示 cells[i] 已有点
    uint8_t active_count = 0;     // 当前非空的子体素数量

    Eigen::Vector3d origin = Eigen::Vector3d::Zero();  // 父体素的最小角坐标
    double sub_size = 0.0;                             // 子体素边长
    double inv_sub_size = 0.0;                         // 1/sub_size 预算，供 fast_floor 使用

    SubGrid(const Eigen::Vector3d& origin_min, double parent_voxel_size) {
        origin = origin_min;
        sub_size = parent_voxel_size / static_cast<double>(kDim);
        inv_sub_size = 1.0 / sub_size;
    }

    // 三维索引展成一维：iz*kDim² + iy*kDim + ix（列优先展开等价）
    inline static int flatten(int ix, int iy, int iz) { return iz * (kDim * kDim) + iy * kDim + ix; }
    // 数值兜底：把 float→int 后可能越界的下标夹回 [0, kDim-1]
    inline static int clamp(int v) { return (v < 0) ? 0 : (v >= kDim ? (kDim - 1) : v); }

    // 把点 p 累积到它所属的子体素。同时更新 occupancy_mask / active_count。
    void addPoint(const Eigen::Vector3d& p, const Eigen::Matrix3d& noise);
    // 返回 p 落入的那个子体素指针；若该 cell 为空则返回 nullptr。
    const SubVoxel* nearest(const Eigen::Vector3d& p) const;
};

// ============================================================
// 【结构体】Plane
// 作用：父体素内一个"当前最佳拟合平面"。通过增量收集点 (addPoint) 及惰性拟合 (fit)，
//       为 P2P 残差提供平面参数 (normal, center) 和其协方差 cov_nq (6×6)。
//
// 【拟合流程】
//   1) addPoint 累积 sum_p、sum_ppT、以及 PlaneCovAccumulator (供 6×6 cov_nq 计算)
//   2) update 时：
//      - 计算 center = sum_p / n；covariance = sum_ppT / n - center·centerᵀ
//      - 若满足触发条件 (刚初始化点够多 or 距上次更新累积到间隔阈值) ⇒ 调 fit()
//      - 若 count > kMaxPointsNum ⇒ finalized = true (冻结，不再更新，节省内存)
//   3) fit() 做 self-adjoint 特征分解：
//      - 最小特征值 λ₀ 对应"平面厚度"，其特征向量 = 法向 normal
//      - 平面性判据：λ₀ / Σλ < kPlanarRatio 且 λ₀ < kPlanarThickness²
//      - 计算 6×6 cov_nq (法向 + 中心的联合协方差；由 PlaneCovAccumulator 输出)
//
// 【类静态阈值】
//   - kMinPointsNumForPlaneInit          : 首次拟合最少需要的点数 (5)
//   - kMinPointsNumForPlaneUpdateInterval: 已初始化后每间隔多少新点重新 fit 一次 (2)
//   - kMaxPointsNum                      : 达到该点数后冻结，不再更新 (100)
//   - kPlanarRatio / kPlanarThickness2   : 平面判据阈值 (从 YAML 读取)
// ============================================================
struct Plane {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // ---- 对外输出（P2P 残差用得到的量） ----
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();                              // 平面法向 (最小特征值方向)
    Eigen::Vector3d center = Eigen::Vector3d::Zero();                              // 平面中心 = 点均值
    Eigen::Matrix<double, 6, 6> cov_nq = Eigen::Matrix<double, 6, 6>::Identity();  // [n; q] 的 6×6 联合协方差
    double planarity = 0.0;                                                        // 平面性度量 = λ₀ (越小越平)
    bool valid = false;                                                            // 平面拟合是否有效
    bool inited = false;                                                           // 是否已经首次拟合

    // 增量加入一个点 + 其协方差 (世界系)
    void addPoint(const Eigen::Vector3d& p, const Eigen::Matrix3d& p_cov);
    // 更新（重算 center / covariance；条件满足时触发 fit()）
    void update();

   private:
    void fit();   // 特征分解 + 平面判据 + cov_nq 计算

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();                       // 点云 3×3 协方差
    bool updated = false;                                                       // 本轮是否已 update
    bool finalized = false;                                                     // 是否已冻结（点数够多不再更新）
    size_t count = 0;                                                           // 总点数
    size_t count_fit = 0;                                                       // 距上次 fit 累积的点数
    Eigen::Vector3d sum_p = Eigen::Vector3d::Zero();                            // Σ p
    Eigen::Matrix3d sum_ppT = Eigen::Matrix3d::Zero();                          // Σ p·pᵀ
    std::unique_ptr<VoxelMapUtils::PlaneCovAccumulator> plane_cov_accumulator;  // 用于 cov_nq 增量累计

   public:
    inline static size_t kMinPointsNumForPlaneInit = 5;              // 首次拟合门槛
    inline static size_t kMinPointsNumForPlaneUpdateInterval = 2;    // 后续每 N 个新点重新拟合
    inline static size_t kMaxPointsNum = 100;                        // 到此冻结
    inline static float kPlanarRatio = 0.1f;                         // λ₀/Σλ < 0.1 才判为平面
    inline static float kPlanarThickness2 = 0.0025f;                 // λ₀ < 0.05² ≈ 5cm 才判为平面
};

// ============================================================
// 【结构体】Voxel
// 作用：地图的最小单元（父体素），同时持有：
//   - plane   : P2P 用；一直持有（即使暂未拟合成功，valid=false）
//   - subgrid : NDT 用；懒创建（第一次 subgridAddPoint 才 make_unique）
// ============================================================
struct Voxel {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Voxel();
    ~Voxel() = default;

    Plane plane;
    std::unique_ptr<IntensityModel> intensity_model;  // allocated only when intensity is enabled and available
    std::unique_ptr<SubGrid> subgrid;  // 懒创建：只有启用 NDT 且落进点时才分配

    // 便捷入口：懒创建 SubGrid 后追加一个点
    inline void subgridAddPoint(const Eigen::Vector3d& p, const Eigen::Matrix3d& noise,
                                const Eigen::Vector3d& origin_min, double parent_voxel_size) {
        if (!subgrid) subgrid = std::make_unique<SubGrid>(origin_min, parent_voxel_size);
        subgrid->addPoint(p, noise);
    }
};
using VoxelPtr = std::shared_ptr<Voxel>;

// ============================================================
// 【结构体】KNearestInput
// 作用：残差构造需要用到的所有输入指针的一次性打包（避免多参数长签名）。
//   - point_body      : 待查询点在机体系的位置
//   - point_cov_body  : 上者在机体系的协方差
//   - point_world     : 待查询点在世界系的位置（由 R·p_body + t 得到）
//   - point_cov_world : 世界系点协方差（叠加了位姿不确定性，见 KILO::computeWorldPointCov）
//   - rot_predict     : 当前预测/迭代位姿的 R（用于把机体系雅可比旋到世界系）
// 【注意】全部是"const 指针"，KNearestInput 不拥有，仅是视图。
// ============================================================
struct KNearestInput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KNearestInput(const Eigen::Vector3d* p_body, const Eigen::Matrix3d* p_cov_body, const Eigen::Vector3d* p_world,
                  const Eigen::Matrix3d* p_cov_world, const Eigen::Matrix3d* rot,
                  double raw_intensity = std::numeric_limits<double>::quiet_NaN())
        : point_body(p_body),
          point_cov_body(p_cov_body),
          point_world(p_world),
          point_cov_world(p_cov_world),
          rot_predict(rot), intensity(raw_intensity) {}
    const Eigen::Vector3d* point_body = nullptr;
    const Eigen::Matrix3d* point_cov_body = nullptr;
    const Eigen::Vector3d* point_world = nullptr;
    const Eigen::Matrix3d* point_cov_world = nullptr;
    const Eigen::Matrix3d* rot_predict = nullptr;
    double intensity;
};

// ============================================================
// 【模板结构体】KNearestRes<DIM>
// 作用：残差构造的输出。
//   - DIM = 1 ： P2P (点到平面有符号距离)
//   - DIM = 3 ： NDT (三维白化位移)
// 关键字段：
//   - J     : ∂r/∂[δθ, δp] (DIM × 6)
//   - r     : 残差
//   - R     : 观测协方差（当前实现里都是单位阵，实际权重通过 pt_R 打入 ObsShared）
//   - score : 选择"最匹配"邻域时用的评分（P2P=高斯概率密度；NDT=-距离²）
//   - valid : 是否找到合法邻域
// ============================================================
template <int DIM>
struct KNearestRes {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<double, DIM, 6> J;
    Eigen::Matrix<double, DIM, 1> r;
    Eigen::Matrix<double, DIM, DIM> R;
    double score = std::numeric_limits<double>::lowest();   // 越大越好，用于挑选最优邻域
    bool valid = false;
};

// ============================================================
// 【类】GaussianVoxelMap
// 作用：整张地图。负责：
//   1) 点云插入 (insertPoints) —— 并行 group-by 体素，再串行更新 plane / subgrid
//   2) 残差构造 (buildPoint2PlaneResidual / buildNdtResidual)  —— const，并发安全
//   3) LRU 淘汰：voxel_list_ 保序（最近访问在头），超过 capacity 时删除尾部
// ============================================================
class GaussianVoxelMap {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ---- 建图相关配置（从 YAML 传入） ----
    struct Config {
        double voxel_size = 0.5;                  // 父体素边长 (m)
        size_t capacity = 100000;                 // LRU 保留的最大体素数（超过则淘汰）
        NearByType nearby_type = NearByType::CENTER;  // 邻域搜索模板
        float planar_ratio = 0.1f;                // 平面判据 λ₀/Σλ
        float planar_thickness = 0.05f;           // 平面判据 √λ₀（m）
        size_t voxel_max_num = 50;                // 单体素最多点数 (触发 Plane 冻结)
        bool p2p_enable = true;                   // 是否启用 P2P
        bool ndt_enable = false;                  // 是否启用 NDT
        size_t ndt_min_points = 5;                // NDT 分布拟合的最少点数
        double ndt_jitter = 1e-6;                 // NDT 协方差对角小扰动
        bool ndt_eigenvalue_regularization = true;// 是否做特征值下限正则化
        double ndt_min_eigenvalue = 1e-6;         // 最小特征值下限
        double ndt_max_condition = 100.0;         // 最大条件数（限制 λ_max / λ_min）
        IntensityConfig intensity;
    };

    // ---- 残差构造时使用的测量噪声档位 (Stage-1/2 有不同配置) ----
    struct ResidualNoise {
        double p2plane_meas_ratio = 10.0;   // P2P：把 (点+平面) 传播的方差再放大的倍率
        double p2plane_min_noise = 1e-3;    // P2P：方差下限
        double ndt_meas_ratio = 10.0;       // NDT：Sigma 放大倍率
        double ndt_min_noise = 1e-3;        // NDT：对角下限
    };

    explicit GaussianVoxelMap(const Config& config);

    /**
     * @brief 把一帧世界系高斯点云插入地图。
     *   步骤：
     *     [1] 并行为每个点算 voxel key
     *     [2] parallel_sort 按 key 排序，把同一体素的点聚集
     *     [3] 顺序遍历分组：找到/新建 Voxel，把点喂进 plane / subgrid，并做 LRU 维护
     *   淘汰：voxel_list_.size() > capacity 时，从链表尾丢一个（最久未访问的体素）。
     */
    void insertPoints(const GaussCloud& cloud);

    // r is the innovation (measured - predicted); J differentiates the prediction.
    // R contains the intensity information multiplier and Huber variance scale.
    bool buildIntensityResidual(const KNearestInput& input, KNearestRes<1>& result) const;

    /**
     * @brief 构造点到平面 (P2P) 残差；遍历 nearby_grids_ 的候选体素，挑选高斯概率最大的平面。
     * @return 是否找到有效平面
     */
    bool buildPoint2PlaneResidual(const KNearestInput& knn_input, KNearestRes<1>& knn_res,
                                  const ResidualNoise& residual_noise) const;

    /**
     * @brief 构造 NDT (点到分布) 残差；遍历 nearby_grids_ 的所有 SubGrid cell，挑最近的高斯。
     * @return 是否找到有效分布
     */
    bool buildNdtResidual(const KNearestInput& knn_input, KNearestRes<3>& knn_res,
                          const ResidualNoise& residual_noise) const;

   private:
    // 根据 config_.nearby_type 生成搜索偏移列表（CENTER→1, NEARBY6→7, NEARBY26→27 个偏移）
    void generateNearbyGrids();

    // 对 NDT 协方差做数值正则化：+jitter → 特征值下限截断 + 最大条件数约束
    Eigen::Matrix3d regularizeNdtCovariance(const Eigen::Matrix3d& covariance) const;

    Config config_;

    double voxel_size_ = 0.5;
    double inv_voxel_size_ = 2.0;                 // 1 / voxel_size_，预算供 fast_floor

    // LRU 存储：list 保序（头=最近访问），map 提供 O(1) 查表
    std::list<std::pair<VoxelKey, VoxelPtr>> voxel_list_;
    std::unordered_map<VoxelKey, std::list<std::pair<VoxelKey, VoxelPtr>>::iterator> voxel_map_;
    std::vector<Eigen::Vector3i> nearby_grids_;   // 邻域搜索偏移
};
}  // namespace legkilo
#endif  // LEG_KILO_GAUSSIAN_VOXEL_MAP_H
