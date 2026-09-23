#include "core/slam/frontend/gaussian_voxel_map.h"

#include <algorithm>

#include <glog/logging.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include "common/math_utils.hpp"

namespace legkilo {

// --------------------------------------------------------------------------
// SubGrid::addPoint
// 把点 p 依据它相对父体素 origin 的位置定位到 kDim×kDim×kDim 中的一个 cell，
// 更新对应 SubVoxel 并维护 occupancy_mask / active_count。
// --------------------------------------------------------------------------
void SubGrid::addPoint(const Eigen::Vector3d& p, const Eigen::Matrix3d& noise) {
    // [步骤 1] 相对坐标 → 子网格整数索引
    const Eigen::Vector3d rel = p - origin;
    Eigen::Vector3i sub_idx = voxelKeyFastFloor(rel, inv_sub_size);
    int ix = clamp(sub_idx.x());
    int iy = clamp(sub_idx.y());
    int iz = clamp(sub_idx.z());
    const int idx = flatten(ix, iy, iz);
    // [步骤 2] 累积到该 cell（走 Welford 增量协方差）
    SubVoxel& cell = cells[idx];
    const bool was_empty = (cell.count == 0);
    cell.addPoint(p, noise);
    // [步骤 3] 首次被占用时更新位掩码
    if (was_empty) {
        occupancy_mask |= (1u << idx);
        ++active_count;
    }
}

// --------------------------------------------------------------------------
// SubGrid::nearest
// 返回 p 落入的那个 cell（若为空返回 nullptr）。注意：此函数并没有做真正的
// 最近邻搜索，只是"直接命中的 cell"。真正的 NDT 邻域搜索由外层
// GaussianVoxelMap::buildNdtResidual 遍历 nearby_grids_ 完成。
// --------------------------------------------------------------------------
const SubVoxel* SubGrid::nearest(const Eigen::Vector3d& p) const {
    if (occupancy_mask == 0) return nullptr;
    const Eigen::Vector3d rel = p - origin;
    Eigen::Vector3i sub_idx = voxelKeyFastFloor(rel, inv_sub_size);
    int ix = clamp(sub_idx.x());
    int iy = clamp(sub_idx.y());
    int iz = clamp(sub_idx.z());
    const int idx = flatten(ix, iy, iz);
    if (((occupancy_mask >> idx) & 1u) == 0u) return nullptr;
    return &cells[idx];
}

// 默认构造：Plane / subgrid 各自初始化；subgrid 是 unique_ptr，此处保持 nullptr（懒创建）
Voxel::Voxel() {}

// --------------------------------------------------------------------------
// Plane::addPoint
// 只累加原始 (sum_p, sum_ppT) 与噪声累加器，不立即 fit。真正的拟合在 update() 里做。
// 一旦 finalized（点数达到上限），后续点被忽略以节省内存与算力。
// --------------------------------------------------------------------------
void Plane::addPoint(const Eigen::Vector3d& p, const Eigen::Matrix3d& p_cov) {
    if (finalized) return;
    updated = false;               // 标记本轮尚未 update
    ++count;
    ++count_fit;
    sum_p += p;                    // Σ p
    sum_ppT.noalias() += p * p.transpose();  // Σ p·pᵀ
    if (!plane_cov_accumulator) { plane_cov_accumulator = std::make_unique<VoxelMapUtils::PlaneCovAccumulator>(); }
    plane_cov_accumulator->addPoint(p, p_cov);   // 增量维护 cov_nq 所需的高阶统计
}

// --------------------------------------------------------------------------
// Plane::update
// 【时机】每次 insertPoints 遍历完一个体素的点后立即调用一次。
// 【逻辑】
//   1) 每轮重算 center = Σp/n 和样本协方差 covariance = Σppᵀ/n - center·centerᵀ
//   2) 触发条件下调 fit()：
//       - 未 inited 且点数 ≥ kMinPointsNumForPlaneInit           (首次拟合)
//       - 已 inited 且距上次 fit 积累 ≥ kMinPointsNumForPlaneUpdateInterval  (再拟合)
//   3) 若 count > kMaxPointsNum：finalized = true，释放累加器，永久冻结
// --------------------------------------------------------------------------
void Plane::update() {
    if (updated || finalized) return;

    const double n = static_cast<double>(count);
    if (n <= 0.0) return;
    center = sum_p / n;                                       // [步骤 1] 重算均值
    covariance = sum_ppT / n - center * center.transpose();    // [步骤 2] 重算样本协方差

    // 【步骤 3】决定要不要重新拟合（惰性触发）
    bool if_update_plane =
        (inited && count_fit >= kMinPointsNumForPlaneUpdateInterval) || (!inited && count >= kMinPointsNumForPlaneInit);
    if (if_update_plane) { this->fit(); }

    // 【步骤 4】达到上限就冻结，避免地图长期驻留大量小体素占用内存
    if (count > kMaxPointsNum) {
        finalized = true;
        plane_cov_accumulator.reset();
    }

    updated = true;
    return;
}

// --------------------------------------------------------------------------
// Plane::fit
// 【算法】对样本协方差做自伴特征分解 (升序返回)：
//   λ₀ ≤ λ₁ ≤ λ₂ 与对应特征向量 u₀, u₁, u₂
//   若最小方向足够扁平 (λ₀/Σλ 与 λ₀ 都够小)，则 normal = u₀（该方向就是平面法向）。
// 【平面性判据】(两个都要满足)
//   planar_ratio    = λ₀ / (λ₀+λ₁+λ₂) < kPlanarRatio        (类比"扁平度")
//   planar_thickness² = λ₀              < kPlanarThickness²   (类比"平面厚度平方")
// 【cov_nq】计算平面参数 [normal; center] 的 6×6 联合协方差（详见 PlaneCovAccumulator）。
// --------------------------------------------------------------------------
void Plane::fit() {
    // 对称化：数值稳定，避免非对称浮点误差破坏 SelfAdjointEigenSolver
    Eigen::Matrix3d cov_sym = 0.5 * (covariance + covariance.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov_sym);
    if (eig.info() != Eigen::Success) return;
    inited = true;
    Eigen::Vector3d evals = eig.eigenvalues();     // 升序：evals[0] = λ_min
    Eigen::Matrix3d evecs = eig.eigenvectors();

    // 平面判据两个数值：ratio + 绝对厚度
    const double planar_ratio = evals[0] / (evals.sum() + 1e-6);   // +1e-6 防除 0
    const double planar_thickness2 = evals[0];

    planarity = planar_thickness2;

    if (planar_ratio > kPlanarRatio || planar_thickness2 > kPlanarThickness2) {
        // 判为非平面（可能是墙角、树叶等结构）
        valid = false;
    } else {
        // 判为平面：法向 = 最小特征值对应的特征向量
        valid = true;
        normal = evecs.col(0).normalized();
        // 计算 6×6 cov_nq (n_x n_y n_z q_x q_y q_z)，供 P2P 残差里的方差传播使用
        plane_cov_accumulator->computePlaneCov(evecs, evals[0], evals[1], evals[2], cov_nq);
    }
    count_fit = 0;    // 重置"距离上次 fit 的新点计数"，等下次积累够再触发
    return;
}

// --------------------------------------------------------------------------
// GaussianVoxelMap 构造：把 Config 里的静态阈值写进 Plane（那三个 kXxx 是静态变量），
// 再生成邻域搜索模板 nearby_grids_（CENTER/NEARBY6/NEARBY26）。
// 【为什么静态】所有 Plane 实例共用同一组阈值；在建图开始前一次性初始化即可。
// --------------------------------------------------------------------------
GaussianVoxelMap::GaussianVoxelMap(const Config& config) : config_(config) {
    config_.intensity.validate();
    voxel_size_ = config_.voxel_size;
    inv_voxel_size_ = 1.0 / voxel_size_;
    Plane::kMaxPointsNum = config_.voxel_max_num;
    Plane::kPlanarRatio = config_.planar_ratio;
    Plane::kPlanarThickness2 = config_.planar_thickness * config_.planar_thickness;  // 用平方值，避免每次 sqrt
    generateNearbyGrids();
}

// --------------------------------------------------------------------------
// NDT 协方差数值正则化（保证 Cholesky/求逆都稳定）：
//   1) 对称化 + 主对角 + jitter（消除微小非对称、避免奇异）
//   2) 若开启特征值正则化：
//      - 拉低特征值下限到 max(config.ndt_min_eigenvalue, jitter)
//      - 再用 λ_max / config.ndt_max_condition 进一步抬高（限制条件数）
//   3) 用新特征值重建 Σ = U · diag(λ') · Uᵀ
//
// 【为什么要正则化】NDT 里要对 Σ 做 Cholesky。若某方向方差 ≈ 0（例如落进平面上的
// 点几乎共面），LLT 会失败或结果爆炸。正则化保证：
//   - λ_min ≥ ndt_min_eigenvalue
//   - λ_max / λ_min ≤ ndt_max_condition
// --------------------------------------------------------------------------
Eigen::Matrix3d GaussianVoxelMap::regularizeNdtCovariance(const Eigen::Matrix3d& covariance) const {
    Eigen::Matrix3d covariance_sym = 0.5 * (covariance + covariance.transpose());
    const double jitter = std::max(config_.ndt_jitter, 0.0);
    covariance_sym.diagonal().array() += jitter;   // 主对角先加 jitter 提升数值稳定性

    if (!config_.ndt_eigenvalue_regularization) return covariance_sym;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance_sym);
    if (eig.info() != Eigen::Success) return covariance_sym;   // 分解失败则退回仅 jitter 的版本

    // λ 全部下限到 jitter，避免出现 ≤0
    Eigen::Vector3d evals = eig.eigenvalues().cwiseMax(jitter);
    const double lambda_max = std::max(evals.maxCoeff(), jitter);
    // 计算最终下限：先满足 min_eigenvalue，再用 max_condition 提升
    double lambda_floor = std::max(config_.ndt_min_eigenvalue, jitter);
    if (config_.ndt_max_condition > 1.0) {
        lambda_floor = std::max(lambda_floor, lambda_max / config_.ndt_max_condition);
    }
    evals = evals.cwiseMax(lambda_floor);   // 应用下限

    // 用修正后的特征值重建 Σ_reg = U · diag(λ_reg) · Uᵀ
    return eig.eigenvectors() * evals.asDiagonal() * eig.eigenvectors().transpose();
}

// --------------------------------------------------------------------------
// GaussianVoxelMap::insertPoints
// 【总体思路】把一帧世界系点云"按体素分组"后依次注入各自的 Voxel。
//
//   [并行阶段] (对每个点独立计算 key，可并发)
//     for i in [0, N): key = encodeKey( voxelKeyFastFloor(p_i, inv_voxel_size_) )
//     voxel_entries[i] = (key, i)
//
//   [排序阶段] tbb::parallel_sort，按 key 升序 → 同一 voxel 的点相邻
//
//   [串行阶段] 逐组处理：
//     - 若 voxel_map_ 中已存在该 key → splice 到链表头 (LRU 更新)
//     - 否则新建 Voxel push_front，如果超过 capacity 淘汰链表尾
//     - 把组内所有点喂进 plane.addPoint / subgridAddPoint
//     - P2P 启用则调用一次 plane.update() 触发拟合
//
// 【为什么不并行注入】共享 voxel_list_ / voxel_map_ 有插入删除，并发风险高；
// 而查 key 才是重头戏（每点独立），排序也是高度并行操作。
// --------------------------------------------------------------------------
void GaussianVoxelMap::insertPoints(const GaussCloud& cloud) {
    const size_t N = cloud.size();
    if (N == 0) return;
    std::vector<std::pair<int64_t, size_t>> voxel_entries;   // (voxel_key_code, 原始索引)
    voxel_entries.resize(N);

    // [并行阶段] 每点计算所在体素 key。粒度 1024 平衡任务开销与并行度。
    tbb::parallel_for(tbb::blocked_range<size_t>(0, N, 1024), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
            const auto& point = cloud[i];
            const auto key = voxelKeyFastFloor(point.pt, inv_voxel_size_);
            voxel_entries[i].first = encodeKey(key);
            voxel_entries[i].second = i;
        }
    });

    // [排序阶段] 按 key 排序 → 让同一体素的点相邻，便于线性扫描分组
    tbb::parallel_sort(
        voxel_entries.begin(), voxel_entries.end(),
        [](const std::pair<int64_t, size_t>& a, const std::pair<int64_t, size_t>& b) { return a.first < b.first; });

    // [串行阶段] 分组处理，每组对应一个体素
    size_t start = 0;
    while (start < N) {
        size_t begin = start;
        size_t end = start + 1;
        // 找到当前 key 的右边界
        while (end < N && voxel_entries[end].first == voxel_entries[begin].first) { ++end; }

        const VoxelKey key_code = voxel_entries[begin].first;
        auto it = voxel_map_.find(key_code);
        std::list<std::pair<VoxelKey, VoxelPtr>>::iterator list_it;
        if (it == voxel_map_.end()) {
            // 【新体素】：push_front + emplace，然后视容量淘汰尾部
            VoxelPtr voxel_ptr = std::make_shared<Voxel>();
            voxel_list_.push_front(std::make_pair(key_code, voxel_ptr));
            voxel_map_.emplace(key_code, voxel_list_.begin());
            list_it = voxel_list_.begin();
            if (voxel_list_.size() > config_.capacity) {
                // LRU 淘汰：链表最尾 = 最久未访问
                voxel_map_.erase(voxel_list_.back().first);
                voxel_list_.pop_back();
            }
        } else {
            // 【已有体素】：splice 到链表头，更新访问时间；地址稳定，map 里的 iterator 仍然有效
            voxel_list_.splice(voxel_list_.begin(), voxel_list_, it->second);
            list_it = it->second;
        }

        VoxelPtr& voxel_ptr = list_it->second;
        // ---- P2P 通道：把点喂进 plane 累加器，一次性 update() ----
        if (config_.p2p_enable || config_.intensity.enable) {
            for (size_t i = begin; i < end; ++i) {
                const auto& point = cloud[voxel_entries[i].second];
                voxel_ptr->plane.addPoint(point.pt, point.cov);
            }
            voxel_ptr->plane.update();
        }

        if (config_.intensity.enable) {
            for (size_t i = begin; i < end; ++i) {
                const auto& point = cloud[voxel_entries[i].second];
                double normalized = 0.0;
                if (!config_.intensity.normalize(point.intensity, normalized)) continue;
                if (!voxel_ptr->intensity_model) voxel_ptr->intensity_model = std::make_unique<IntensityModel>();
                voxel_ptr->intensity_model->addPoint(point.pt, normalized, config_.intensity.max_points);
            }
            if (voxel_ptr->intensity_model) {
                voxel_ptr->intensity_model->fit(voxel_ptr->plane.valid ? voxel_ptr->plane.normal
                                                                     : Eigen::Vector3d::Zero(), config_.intensity);
            }
        }

        // ---- NDT 通道：懒创建 SubGrid，喂到对应的 SubVoxel ----
        if (config_.ndt_enable) {
            // 由 voxel key 反向解码出父体素的整数索引，再乘以 voxel_size_ 得到最小角坐标
            const Eigen::Vector3i grid_idx = decodeKey(key_code);
            const Eigen::Vector3d origin_min = grid_idx.cast<double>() * voxel_size_;
            for (size_t i = begin; i < end; ++i) {
                const auto& point = cloud[voxel_entries[i].second];
                voxel_ptr->subgridAddPoint(point.pt, point.cov, origin_min, voxel_size_);
            }
        }

        start = end;
    }
}

// --------------------------------------------------------------------------
// GaussianVoxelMap::buildPoint2PlaneResidual
// 【物理意义】把点到平面距离作为观测残差：d = nᵀ · (p_world - center)  期望 = 0。
// 【流程】
//   1) 找到 p_world 所在的父体素坐标 p_floor
//   2) 遍历 nearby_grids_ (CENTER/6/26) 中的候选体素
//   3) 对每个"平面有效"的体素：
//      a) 距离 d = nᵀ(p_w - c)
//      b) 计算方差 σ² = σ²_plane + σ²_point (来自 cov_nq 和 cov_world 的传播)
//      c) 卡方检验 d² > 9·σ² ⇒ 直接舍弃 (99.7% 3σ 阈值)
//      d) 用高斯概率评分挑选"最匹配"的一个平面
//   4) 用最佳平面构造归一化残差 r/√σ² 与雅可比 J = [ -nᵀR·[p_b]×,  nᵀ ] / √σ²
//
// 【残差归一化】把测量方差合并进 r 和 J 后，ObsShared 里 R 只需保留 kernel 的方差放大部分。
// 后续 Cauchy 核会再乘一个方差 scale，所以此处 R 用 Identity 就够。
//
// 【雅可比推导】
//   r = nᵀ · (R·p_b + t - c)          (右扰动：R → R·Exp(δθ))
//   ∂r/∂δθ = nᵀ · R · [-p_b]× = -nᵀ·R·[p_b]×
//   ∂r/∂δp = nᵀ
//
// 【σ²_plane vs σ²_point 的两种雅可比】
//   Jw_nq = [ (p-c)ᵀ, -nᵀ ] : 对 [n; q] 的雅可比，用于 σ²_plane = Jw_nq·cov_nq·Jw_nqᵀ
//   Jw_p  = nᵀ              : 对 p_world 的雅可比，用于 σ²_point = Jw_p·cov_world·Jw_pᵀ
// --------------------------------------------------------------------------
bool GaussianVoxelMap::buildPoint2PlaneResidual(const KNearestInput& knn_input, KNearestRes<1>& knn_res,
                                                const ResidualNoise& residual_noise) const {
    static constexpr double kSigmaTimes2 = 9.0;   // 3σ 卡方阈值：d² < 9·σ² 才通过
    static constexpr double kMinVar = 1e-12;      // 防除 0

    const Eigen::Vector3d& p_world = *knn_input.point_world;
    const Eigen::Vector3d& p_body = *knn_input.point_body;
    const Eigen::Matrix3d& cov_world = *knn_input.point_cov_world;
    const Eigen::Matrix3d& cov_body = *knn_input.point_cov_body;
    const Eigen::Matrix3d& rot_predict = *knn_input.rot_predict;

    // 待查询点所在的父体素索引
    const Eigen::Vector3i p_floor = voxelKeyFastFloor(p_world, inv_voxel_size_);

    // 遍历所有邻域体素，挑选高斯概率最高的一个作为最佳匹配
    for (size_t i = 0; i < nearby_grids_.size(); ++i) {
        Eigen::Vector3i cur_floor = p_floor + nearby_grids_[i];
        VoxelKey key = encodeKey(cur_floor);

        auto iter = voxel_map_.find(key);
        if (iter == voxel_map_.end()) continue;         // 该邻居体素为空

        const VoxelPtr cur_voxel = iter->second->second;
        if (!cur_voxel->plane.valid) continue;          // 该体素平面尚未拟合成功

        // [步骤 1] 计算有符号距离
        Eigen::RowVector3d normalT = cur_voxel->plane.normal.transpose();
        Eigen::Vector3d diff = p_world - cur_voxel->plane.center;
        double d = normalT * diff;                       // d = nᵀ · (p_w - c)

        // [步骤 2] 方差传播（用于卡方判据和归一化）
        Eigen::Matrix<double, 1, 6> Jw_nq;               // 对 [n; q]  的雅可比 (评估平面参数不确定性)
        Eigen::Matrix<double, 1, 3> Jw_p;                // 对 p_world 的雅可比 (评估点位置不确定性)
        Jw_nq.block<1, 3>(0, 0) = diff.transpose();      // ∂d/∂n = (p-c)ᵀ
        Jw_nq.block<1, 3>(0, 3) = -normalT;              // ∂d/∂c = -nᵀ
        Jw_p = normalT;                                  // ∂d/∂p = nᵀ

        double sigma2_plane = (Jw_nq * cur_voxel->plane.cov_nq * Jw_nq.transpose())(0, 0);
        double sigma2_point = (Jw_p * cov_world * Jw_p.transpose())(0, 0);
        double sigma2 = sigma2_plane + sigma2_point;

        // [步骤 3] 3σ 卡方判据（马氏距离）：过大就是 outlier，直接跳过
        if (d * d > kSigmaTimes2 * sigma2) continue;

        // [步骤 4] 用高斯概率密度作为"匹配得分"，越大越好；保留全场最高分
        double probability = gaussianPdf1D(d, std::sqrt(std::max(sigma2, kMinVar)));
        if (probability > knn_res.score) {
            knn_res.valid = true;
            knn_res.score = probability;

            // [步骤 5] 使用 Stage 特定放大倍率 + 下限，构造最终观测方差
            //   var_R = ratio * (σ²_plane + σ²_point) + min_noise
            // 归一化：把 √var_R 因子直接乘进 r 和 J 中（ObsShared.R 保留 kernel scale 用）
            Eigen::Matrix<double, 1, 3> Jv_p = normalT * rot_predict;                // nᵀ·R (用于 body cov 传播)
            double var_plane_R = sigma2_plane;
            double var_point_R = (Jv_p * cov_body * Jv_p.transpose())(0, 0);          // 用 body 系点协方差重算 σ²_point
            double var_R =
                residual_noise.p2plane_meas_ratio * (var_plane_R + var_point_R) + residual_noise.p2plane_min_noise;
            const double inv_std = 1.0 / std::sqrt(var_R);

            // 归一化残差 & 雅可比
            knn_res.r(0, 0) = -d * inv_std;                                          // 残差取 -d：目标是让 d→0
            // ∂r/∂δθ = -nᵀ·R·[p_b]×
            knn_res.J.block<1, 3>(0, 0) = -normalT * rot_predict * SKEW_SYM_MATRIX(p_body) * inv_std;
            // ∂r/∂δp = nᵀ
            knn_res.J.block<1, 3>(0, 3) = normalT * inv_std;
            knn_res.R.setIdentity();
        }
    }

    return knn_res.valid;
}

// Local tangent intensity: h = mean_I + g^T (R p_body + t - mean_p).
// The filter uses innovation measured_I - h and prediction Jacobian dh/d(delta).
// Covariance and association are held fixed within each linearization.
bool GaussianVoxelMap::buildIntensityResidual(const KNearestInput& input, KNearestRes<1>& result) const {
    result = KNearestRes<1>{};
    const auto& cfg = config_.intensity;
    double measured = 0.0;
    if (!cfg.enable || !cfg.normalize(input.intensity, measured)) return false;
    const auto& p = *input.point_world;
    if (!p.allFinite() || !input.point_body->allFinite() || !input.point_cov_body->allFinite() ||
        !input.rot_predict->allFinite()) return false;
    const auto key = voxelKeyFastFloor(p, inv_voxel_size_);
    const IntensityModel* best = nullptr;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto& offset : nearby_grids_) {
        const auto it = voxel_map_.find(encodeKey(key + offset));
        if (it == voxel_map_.end()) continue;
        const auto& voxel = *it->second->second;
        if (!voxel.plane.valid || !voxel.intensity_model || !voxel.intensity_model->valid()) continue;
        const auto& model = *voxel.intensity_model;
        if (std::abs(voxel.plane.normal.dot(p - voxel.plane.center)) > cfg.max_normal_distance) continue;
        const double support = model.supportDistance2(p);
        const double distance = (p - model.center()).squaredNorm();
        if (!std::isfinite(support) || support > cfg.max_mahalanobis * cfg.max_mahalanobis ||
            distance > voxel_size_ * voxel_size_) continue;
        // Select using geometry only, never search for a convenient intensity match.
        if (distance < best_distance) {
            best = &model;
            best_distance = distance;
        }
    }
    if (!best) return false;
    const Eigen::RowVector3d g = best->gradient().transpose();
    const Eigen::RowVector3d g_body = g * *input.rot_predict;
    const double point_variance = (g_body * *input.point_cov_body * g_body.transpose())(0, 0);
    const double measurement_variance = cfg.measurement_sigma * cfg.measurement_sigma;
    const double variance = measurement_variance + best->predictionVariance(p, measurement_variance) +
                            std::max(0.0, point_variance);
    if (!std::isfinite(variance) || variance <= 0.0) return false;
    const double inv_std = 1.0 / std::sqrt(variance);
    const double innovation = (measured - best->predict(p)) * inv_std;
    if (!std::isfinite(innovation) || std::abs(innovation) > cfg.residual_gate) return false;
    result.r(0) = innovation;
    result.J.leftCols<3>() = -g_body * SKEW_SYM_MATRIX(*input.point_body) * inv_std;
    result.J.rightCols<3>() = g * inv_std;
    result.R(0, 0) = std::max(1.0, std::abs(innovation) / cfg.huber_delta) / cfg.weight;
    result.score = -best_distance;
    result.valid = result.J.allFinite() && result.R.allFinite();
    return result.valid;
}

// --------------------------------------------------------------------------
// GaussianVoxelMap::buildNdtResidual
// 【NDT (Normal Distributions Transform) 残差】
//   把待查询点 p_w 与地图中最近的高斯分布 (μ_cell, Σ_cell) 做马氏距离型残差：
//     r3 = p_w - μ_cell
//     S  = Σ_cell + noise_avg + R·Σ_body·Rᵀ     (地图分布 + 测量噪声 + 位姿传播)
//     S  = ratio · regularize(S) + min_noise·I  (数值稳定 + 观测噪声底)
//     LLT: S = L·Lᵀ
//     r_white = L⁻¹ · r3               (三维白化残差)
//     J_white = L⁻¹ · H3               (H3 是残差对 [δθ, δp] 的雅可比: [-R·[p_b]×, I])
//
// 【和 P2P 的区别】
//   - P2P 只在有效"平面"体素找匹配，残差是 1D 有符号距离；
//   - NDT 在所有非空 SubVoxel 找最近的高斯，残差是 3D 白化位移；
//     适合非平面结构（角、树、灌木、粗糙表面）。
//
// 【几何裁剪】距离² > (0.5·voxel_size)² 就跳过 —— 只考虑不超过半个体素边长的匹配，
// 防止跨越结构边界导致错误匹配。
//
// 【为什么用 LLT.solve 而不显式求 L⁻¹】
//   Eigen 的 LLT.solve 通过前向/后向代入实现 L⁻¹·b，数值上比 inverse(L)·b 更稳定，
//   同一 LLT 分解可以反复用于向量 r3 和矩阵 H3，都很高效。
// --------------------------------------------------------------------------
bool GaussianVoxelMap::buildNdtResidual(const KNearestInput& knn_input, KNearestRes<3>& knn_res,
                                        const ResidualNoise& residual_noise) const {
    knn_res.valid = false;
    const Eigen::Vector3d& p_world = *knn_input.point_world;
    const Eigen::Vector3d& p_body = *knn_input.point_body;
    const Eigen::Matrix3d& cov_body = *knn_input.point_cov_body;
    const Eigen::Matrix3d& rot_predict = *knn_input.rot_predict;

    double best_dist2 = std::numeric_limits<double>::infinity();
    // 最大匹配距离² = (0.5·voxel_size)²  ⇒  只接受体素中心半径内的点
    const double kMaxDistance2 = 0.5 * 0.5 * voxel_size_ * voxel_size_;

    // 未白化的雅可比 H3 = [ -R·[p_b]× ,  I₃ ]，稍后与 L⁻¹ 相乘得到白化雅可比
    Eigen::Matrix<double, 3, 6> H3;
    H3.block<3, 3>(0, 0) = -rot_predict * SKEW_SYM_MATRIX(p_body);
    H3.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    const Eigen::Vector3i p_floor = voxelKeyFastFloor(p_world, inv_voxel_size_);
    for (size_t i = 0; i < nearby_grids_.size(); ++i) {
        const Eigen::Vector3i cur_floor = p_floor + nearby_grids_[i];
        const VoxelKey key = encodeKey(cur_floor);

        auto iter = voxel_map_.find(key);
        if (iter == voxel_map_.end()) continue;

        const VoxelPtr vox = iter->second->second;
        if (!vox || !vox->subgrid) continue;         // 该体素没启用 SubGrid（懒创建）

        const SubGrid& sg = *vox->subgrid;
        const uint32_t mask = sg.occupancy_mask;

        // 遍历该父体素内所有非空 SubVoxel
        for (int idx = 0; idx < SubGrid::kCellNum; ++idx) {
            if (((mask >> idx) & 1u) == 0u) continue;                    // 该 cell 空
            const SubVoxel& cell = sg.cells[idx];
            if (cell.count < config_.ndt_min_points) continue;            // 点太少，分布不稳定

            // [步骤 1] 位置残差 & 快速裁剪
            const Eigen::Vector3d r3 = p_world - cell.mean;
            const double dist2 = r3.squaredNorm();
            if (dist2 > kMaxDistance2) continue;                          // 距离太远，不予考虑

            // [步骤 2] 组装总协方差 S = 分布协方差 + 平均测量噪声 + 位姿传播的点噪声
            Eigen::Matrix3d S = cell.cov + cell.noise_sum / static_cast<double>(cell.count) +
                                rot_predict * cov_body * rot_predict.transpose();
            // [步骤 3] 放大 + 特征值/条件数正则化 + 下限对角 + 对称化
            S = residual_noise.ndt_meas_ratio * regularizeNdtCovariance(S);
            S.diagonal().array() += residual_noise.ndt_min_noise;
            S = 0.5 * (S + S.transpose());

            // [步骤 4] Cholesky: S = L·Lᵀ；失败则跳过
            Eigen::LLT<Eigen::Matrix3d> llt(S);
            if (llt.info() != Eigen::Success) continue;
            const auto Ltri = llt.matrixL();

            // [步骤 5] 白化残差 r_w = L⁻¹·r3（LLT.solve 内部走前向代入，比显式 inverse 快且稳）
            const Eigen::Vector3d r_w = Ltri.solve(r3);

            // [步骤 6] 择优：dist2 最小的胜出（等价于挑最近的分布中心）
            if (dist2 < best_dist2) {
                const Eigen::Matrix<double, 3, 6> H_w = Ltri.solve(H3);   // 白化雅可比 L⁻¹·H3
                best_dist2 = dist2;
                knn_res.J = H_w;
                knn_res.r = -r_w;         // 取负：目标是让残差 → 0
                knn_res.R.setIdentity();
                knn_res.score = -dist2;   // 越大越好；这里用 -dist2 就能保持"距离越小得分越高"
                knn_res.valid = true;
            }
        }
    }

    return knn_res.valid;
}

// --------------------------------------------------------------------------
// 生成邻域搜索模板：
//   CENTER   : 只查自己所在体素 (1 个偏移)
//   NEARBY6  : 面邻居 (7 个偏移：中心 + 6 面)
//   NEARBY26 : 3×3×3 全部 (27 个偏移；对应 26 个真正邻居 + 中心)
//
// 【选择建议】
//   - 平面丰富 / 快速：CENTER
//   - 常规均衡：NEARBY6
//   - 追求鲁棒（更宽松匹配）：NEARBY26，代价是每点查询 27 次哈希表
// --------------------------------------------------------------------------
void GaussianVoxelMap::generateNearbyGrids() {
    nearby_grids_.clear();
    if (config_.nearby_type == NearByType::CENTER) {
        nearby_grids_ = {Eigen::Vector3i(0, 0, 0)};
    } else if (config_.nearby_type == NearByType::NEARBY6) {
        nearby_grids_ = {Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(-1, 0, 0), Eigen::Vector3i(1, 0, 0),
                         Eigen::Vector3i(0, 1, 0), Eigen::Vector3i(0, -1, 0), Eigen::Vector3i(0, 0, -1),
                         Eigen::Vector3i(0, 0, 1)};
    } else if (config_.nearby_type == NearByType::NEARBY26) {
        nearby_grids_.reserve(27);
        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                for (int z = -1; z <= 1; z++) { nearby_grids_.emplace_back(Eigen::Vector3i(x, y, z)); }
            }
        }
    } else {
        // 兜底：非法配置也不要挂掉，退回 CENTER 并打错误日志
        LOG(ERROR) << "Undefined NearbyType !";
        nearby_grids_ = {Eigen::Vector3i(0, 0, 0)};
    }
}
}  // namespace legkilo
