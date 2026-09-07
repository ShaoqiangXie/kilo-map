#include "core/slam/backend/loop_closure/loop_detector/euclidean_method.h"

#include <array>

#include "KDTreeVectorOfVectorsAdaptor.h"
#include "nanoflann.hpp"

namespace legkilo {

// setSearchRadius / setMinIdSeparation：加下限保护，避免设成 0 造成无候选或除零
void EuclideanLoopDetector::setSearchRadius(double r) { search_radius_ = std::max(0.1, r); }

void EuclideanLoopDetector::setMinIdSeparation(int s) { min_id_separation_ = std::max(1, s); }

// ---------------------------------------------------------------
// detectLatest：欧氏候选检测的主体
//   核心思想：把所有子地图的平移放进 KD-Tree，对每个子地图查询 r 邻居；
//             再用 ID 间隔阈值过滤掉那些"其实是时间相邻"的对。
//
// 步骤：
//   [1] 少于 2 个子地图直接返回
//   [2] 抽取每个子地图的平移和 ID
//   [3] 转成 nanoflann 兼容的 std::vector<std::array<double,3>>
//   [4] 构建 3D KD-Tree（L2）
//   [5] 依次做半径查询：过滤 id_i < id_j 且 id_j - id_i >= min_id_separation_
//       - 只输出 id_i < id_j 是为了避免重复对（(a,b) 与 (b,a) 只保留一份）
// ---------------------------------------------------------------
//根据子地图优化后的空间位置，找出"距离足够近、但序号相隔足够远"的子地图对，作为回环候选。
std::vector<std::pair<NodeType, NodeType>> EuclideanLoopDetector::detectLatest(
    const std::vector<std::pair<NodeType, Eigen::Isometry3d>>& poses) const {
    std::vector<std::pair<NodeType, NodeType>> out;
    const size_t n = poses.size();//子地图数量
    if (n < 2) return out;//少于两个子地图不可能形成回环，直接返回空结果。

    // [步骤 2] 拆分为 平移向量数组 + ID 数组
    std::vector<Eigen::Vector3d> pts;//保存每个子地图的平移向量
    pts.reserve(n);
    std::vector<NodeType> ids;//保存每个子地图的 ID
    ids.reserve(n);
    for (const auto& pj : poses) {
        pts.emplace_back(pj.second.translation());
        ids.emplace_back(pj.first);
    }

    // [步骤 3] 转为 KD-Tree 需要的容器形式
    std::vector<std::array<double, 3>> vv;//std::array 是 C++ 标准库提供的固定长度数组
    vv.reserve(n);
    for (const auto& p : pts) vv.push_back({p.x(), p.y(), p.z()});
    // 构建三维KD树
    // [步骤 4] 用 nanoflann 提供的 vector-of-vectors 适配器构建 3D 树
    using VVT = std::vector<std::array<double, 3>>;
    using KDAdaptor = KDTreeVectorOfVectorsAdaptor<VVT, double, 3, nanoflann::metric_L2_Simple>;
    KDAdaptor kd(3, vv);
    kd.index->buildIndex();

    // 注意：nanoflann 的 radiusSearch 用的是"距离平方"作为阈值
    const double r2 = search_radius_ * search_radius_;//准备搜索半径
    using SearchParamsT = nanoflann::SearchParameters;
    SearchParamsT params{};

    // [步骤 5] 遍历所有子地图，做 r-NN 查询 + 过滤
    for (size_t j = 0; j < n; ++j) {//查询每个子地图的邻近点
        const double query_pt[3] = {pts[j].x(), pts[j].y(), pts[j].z()};//查询点的坐标
        std::vector<nanoflann::ResultItem<size_t, double>> matches;//保存搜索结果，ResultItem 是一个结构体，包含两个成员：first 表示匹配点的索引，second 表示匹配点的距离平方
        kd.index->radiusSearch(query_pt, r2, matches, params);//搜索点 搜索距离 搜索结果 搜索参数

        const NodeType id_j = ids[j];//当下的子地图 ID
        for (const auto& kv : matches) {//对搜索点匹配出来的点进行遍历，kv.first 是匹配点的索引，kv.second 是匹配点的距离平方
            const size_t i = kv.first;//已经查找到的匹配点的索引
            if (i == j) continue;//如果是同一个子地图
            const NodeType id_i = ids[i];//匹配点的ID
            // 关键过滤：
            //   条件 A: id_i < id_j        —— 只输出一半，避免 (a,b)(b,a) 双写
            //   条件 B: id_j - id_i >= min_id_separation_ —— ID 相隔足够远才可能是真回环
            if (id_i < id_j && (id_j - id_i) >= static_cast<NodeType>(min_id_separation_)) {
                out.emplace_back(id_i, id_j);
            }
        }
    }

    return out;
}

}  // namespace legkilo
