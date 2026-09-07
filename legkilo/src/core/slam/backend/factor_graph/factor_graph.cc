#include "core/slam/backend/factor_graph/factor_graph.h"

#include <algorithm>
#include <cmath>

#include <glog/logging.h>

#include "common/yaml_helper.hpp"


//ceres输出相关
#include <fstream>
#include <iomanip>//设置浮点数输出精度

namespace legkilo {

namespace {
// ---------------------------------------------------------------
// 根据平移和旋转的标准差，生成一个 6×6 的"平方根信息矩阵"，用于给图优化误差设置权重。
//   数学背景：Ceres 的残差项目标函数是 0.5 * ||r||^2，
//   为了让不同物理量的残差可比，通常先把 r 左乘 sqrt(Ω) = sqrt(Σ^{-1}) = diag(1/σ)。
//   这里的信息矩阵是对角形式（各分量独立），因此 sqrt 之后仍为对角矩阵。
// 布局：前 3 行 3 列对应平移残差，后 3 行 3 列对应旋转残差（2*δq.vec()）。
// ---------------------------------------------------------------
Eigen::Matrix<double, 6, 6> MakeSqrtInformation(double trans_sigma, double rot_sigma_deg) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

    const double safe_trans_sigma = trans_sigma;
    const double safe_rot_sigma = rot_sigma_deg * kDegToRad;  // 角度 → 弧度，保证与 delta_q 单位一致

    Eigen::Matrix<double, 6, 6> sqrt_information = Eigen::Matrix<double, 6, 6>::Zero();
    sqrt_information.diagonal().head<3>().setConstant(1.0 / safe_trans_sigma);  // 平移权重
    sqrt_information.diagonal().tail<3>().setConstant(1.0 / safe_rot_sigma);    // 旋转权重
    return sqrt_information;
}

}  // namespace

// ---------------------------------------------------------------
// 构造：创建空的 ceres::Problem，读入所有优化参数
// ---------------------------------------------------------------
FactorGraph::FactorGraph(const std::string &yaml_file) : problem_(std::make_unique<ceres::Problem>()) {
    YamlHelper yaml(yaml_file);

    // 每次运行的 g2o 输出路径，落到与 Recorder 同级的结果目录
    const std::string result_folder = yaml.get<std::string>("temp_result_save_folder", "temp");
    g2o_output_path_ = std::string(ROOT_DIR) + "result/" + result_folder + "/factor_graph.g2o";

    max_iterations_ = yaml.get<size_t>("factor_graph_max_iterations", 100);//最大迭代次数
    odom_near_trans_sigma_ = yaml.get<double>("odom_near_trans_sigma", 0.1);//相邻子地图之间的平移测量，典型误差大约在 10 cm 这个量级。
    odom_near_rot_sigma_deg_ = yaml.get<double>("odom_near_rot_sigma_deg", 5.0);//相邻子地图之间的旋转测量，典型误差大约在 5° 这个量级。
    // "跳一跃"里程计边：给相隔一个节点的两个 submap 也挂一条弱一点的相对位姿约束
    odom_next_trans_sigma_multiplier_ = yaml.get<double>("odom_next_trans_sigma_multiplier", 2.0);
    odom_next_rot_sigma_multiplier_ = yaml.get<double>("odom_next_rot_sigma_multiplier", 2.0);
    // 回环边：sigma 显著更大 + Cauchy 鲁棒核，抑制错误回环
    loopclosure_trans_sigma_multiplier_ = yaml.get<double>("loopclosure_trans_sigma_multiplier", 10.0);
    loopclosure_rot_sigma_multiplier_ = yaml.get<double>("loopclosure_rot_sigma_multiplier", 4.0);
    loopclosure_loss_scale_ = yaml.get<double>("loopclosure_loss_scale", 3.0);
    LOG(INFO) << "FactorGraph is constructed";
}

FactorGraph::~FactorGraph() { LOG(INFO) << "FactorGraph is destructed"; }

// ---------------------------------------------------------------
// addNode：新增一个 Submap 节点，并挂里程计边
//   [1] 已存在则退出
//   [2] 保存初始位姿 & 记录添加顺序
//   [3] 在 Ceres 里注册 (t, q) 两个参数块；q 挂 EigenQuaternionManifold（xyzw + 流形更新）
//   [4] 若为第一个节点：固定 (t, q) 消除全局 gauge 自由度
//   [5] 挂里程计边（详见 addOdometryEdgesForNewestNode）
// ---------------------------------------------------------------
void FactorGraph::addNode(NodeType id, const Eigen::Isometry3d &initial_pose) {
    if (initial_poses_.find(id) != initial_poses_.end()) return;// 节点已经存在，避免重复添加参数块和里程计边
    initial_poses_.insert({id, initial_pose});//保存未经图优化修改的初始位姿： 节点 ID → 初始位姿
    node_ids_.push_back(id);//记录节点添加顺序

    // emplace 会构造 PoseParam(initial_pose)，其中 t/q 是双精度 C 数组，Ceres 直接读写
    auto it = poses_.emplace(id, graph::PoseParam(initial_pose)).first;//emplace() 尝试插入 {id, PoseParam}，
    // 返回：std::pair<iterator, bool> .first：指向对应元素的迭代器 .second：是否真的插入成功
    graph::PoseParam &node = it->second;  //指向PoseParam
    problem_->AddParameterBlock(node.t, 3);  // 3 维平移
    problem_->AddParameterBlock(node.q, 4);  // 4 维四元数（xyzw）
    graph::QuaternionUpdateRule::Attach(*problem_, node.q);  // 挂上四元数流形，保证优化时保持单位模长

    // 首个节点固定 —— 否则整个 PoseGraph 存在 6 自由度全局 gauge，优化不唯一
    if (node_ids_.size() == 1) {
        problem_->SetParameterBlockConstant(node.t);//固定第一个节点
        problem_->SetParameterBlockConstant(node.q);//固定第一个节点的旋转
    }

    addOdometryEdgesForNewestNode();//添加里程计边：
}

// ---------------------------------------------------------------
// addLoopClosureEdge：加入一条经过验证的回环约束
//   [1] 去重（同一对 (i,j) 只加一次）
//   [2] 两端节点必须都已存在
//   [3] 记录到 loop_edges_ 便于查询
//   [4] 以 (trans_sigma * loopclosure_trans_sigma_multiplier_, rot_sigma * loopclosure_rot_sigma_multiplier_) 建边
//       并挂 CauchyLoss(loopclosure_loss_scale_) —— 抗误回环
// ---------------------------------------------------------------
void FactorGraph::addLoopClosureEdge(NodeType i, NodeType j, const Eigen::Isometry3d &meas) {
    if (loop_edges_.find({i, j}) != loop_edges_.end()) return;
    if (initial_poses_.find(i) == initial_poses_.end() || initial_poses_.find(j) == initial_poses_.end()) return;
    LOG(INFO) << "Adding loop closure edge between " << i << " and " << j;
    loop_edges_.insert({{i, j}, graph::LoopEdge(i, j, meas)});
    //加入一条边
    addRelativeEdge(i, j, meas, odom_near_trans_sigma_ * loopclosure_trans_sigma_multiplier_,
                    odom_near_rot_sigma_deg_ * loopclosure_rot_sigma_multiplier_,
                    new ceres::CauchyLoss(loopclosure_loss_scale_));
}

// 从 poses_ 中取出优化后的 (t, q) 并组装成 Isometry3d 返回
bool FactorGraph::getPose(NodeType node_id, Eigen::Isometry3d &pose) const {
    if (poses_.find(node_id) == poses_.end()) return false;
    pose = poses_.at(node_id).toIsometry();
    return true;
}

// ---------------------------------------------------------------
// addRelativeEdge：通用相对位姿边构建
//   传入的 trans_sigma / rot_sigma_deg 决定该测量的可信度（越小越信）；
//   loss_function 为可选的鲁棒核，nullptr 表示纯 L2。
//
//   参数块顺序必须与 RelativePoseFactor::operator() 保持一致：t_i, q_i, t_j, q_j
// ---------------------------------------------------------------
//最后一项是鲁棒核函数，当某条边的误差特别大时，降低它对整体优化的影响。
void FactorGraph::addRelativeEdge(NodeType i, NodeType j, const Eigen::Isometry3d &meas, double trans_sigma,
                                  double rot_sigma_deg, ceres::LossFunction *loss_function) {
    auto it_i = poses_.find(i);
    auto it_j = poses_.find(j);
    if (it_i == poses_.end() || it_j == poses_.end()) return;

    // 由 sigma 生成 sqrt(Ω)（对角矩阵，前 3 平移，后 3 旋转）
    const Eigen::Matrix<double, 6, 6> sqrt_information = MakeSqrtInformation(trans_sigma, rot_sigma_deg);

    edges_.push_back({i, j, meas, sqrt_information});//将边存储起来

    // Create 内部会 new AutoDiffCostFunction<...,6,3,4,3,4>，残差 6 维，参数块四段
    ceres::CostFunction *cost = graph::RelativePoseFactor::Create(meas, sqrt_information);//
    problem_->AddResidualBlock(cost, loss_function, it_i->second.t, it_i->second.q, it_j->second.t, it_j->second.q);
}

// ---------------------------------------------------------------
// addOdometryEdgesForNewestNode：为最新加入的节点自动挂里程计边
//   规则：
//     A) 若 num_nodes ≥ 2：挂 prev → newest 的"近邻"约束（sigma = odom_near_*）
//     B) 若 num_nodes ≥ 3：再挂 prev2 → newest 的"跳一跃"约束
//        （sigma 放大 next_multiplier，充当额外的软约束以提高一致性）
//   相对位姿 T_ij_meas = T_i^-1 * T_j 直接取 initial_poses_ 里的前端估计
// ---------------------------------------------------------------
//添加里程计边
void FactorGraph::addOdometryEdgesForNewestNode() {
    const size_t num_nodes = node_ids_.size();
    if (num_nodes < 2) return;   // 只有一个节点，没有相对约束可加

    const NodeType newest_id = node_ids_.back();//num_nodes - 1
    const NodeType prev_id = node_ids_[num_nodes - 2];
    // A: 相邻两节点的相对位姿约束 = 前端里程计的两次快照相减（in the SE(3) sense）
    addRelativeEdge(prev_id, newest_id, initial_poses_.at(prev_id).inverse() * initial_poses_.at(newest_id),
                    odom_near_trans_sigma_, odom_near_rot_sigma_deg_);//两个参数表示对这条测量的信任程度

    if (num_nodes >= 3) {
        // B: "跳一跃"约束 —— 使用更大的 sigma
        const NodeType prev2_id = node_ids_[num_nodes - 3];
        addRelativeEdge(prev2_id, newest_id, initial_poses_.at(prev2_id).inverse() * initial_poses_.at(newest_id),
                        odom_near_trans_sigma_ * odom_next_trans_sigma_multiplier_,
                        odom_near_rot_sigma_deg_ * odom_next_rot_sigma_multiplier_);
    }
}

// ---------------------------------------------------------------
// optimize：运行 Ceres 求解器
//   算法要点：
//     - SPARSE_NORMAL_CHOLESKY：对 PoseGraph 这类稀疏问题最合适（H = J^T J 是稀疏带状）
//     - num_threads=1：保证可复现（也可根据机器改成更大值以加速）
//     - 优化完成后立即导出 g2o，方便离线检查/画图
// ---------------------------------------------------------------
void FactorGraph::optimize() {
    if (poses_.empty()) return;

    ceres::Solver::Options options;//创建配置器
    options.max_num_iterations = static_cast<int>(max_iterations_);//设定最大迭代次数
    options.linear_solver_type = linear_solver_type_;//设定线性求解器类型
    options.minimizer_progress_to_stdout = false;//不在终端输出优化过程
    options.num_threads = 1;//设定线程数为1，避免多线程导致的不可重复性

    ceres::Solver::Summary summary;//创建优化结果汇总对象


//     ceres::Solve(
//     options,         // 告诉 Ceres 应该怎样优化
//     problem_.get(),  // 告诉 Ceres 要优化哪个问题
//     &summary         // 告诉 Ceres 把优化结果写到哪里
// );
    ceres::Solve(options, problem_.get(), &summary);
    LOG(INFO) << "FactorGraph optimization done. Iterations: " << summary.iterations.size()
              << ", final cost: " << summary.final_cost;

    saveG2o(g2o_output_path_);//将优化后的因子图保存为 g2o 文件
}


// ---------------------------------------------------------------
// saveG2o：把当前图导出为标准 g2o 文本格式
//   格式说明：
//     VERTEX_SE3:QUAT id  tx ty tz  qx qy qz qw
//     EDGE_SE3:QUAT   i j tx ty tz  qx qy qz qw  上三角信息矩阵 21 个元素
//   备注：这里写的是 sqrt(Ω) 而非 Ω，对 g2o_viewer 等工具展示够用（不严格符合 g2o 规范）。
// ---------------------------------------------------------------
bool FactorGraph::saveG2o(const std::string &file_path) const {
    std::ofstream out(file_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        LOG(ERROR) << "Failed to open file: " << file_path;
        return false;
    }

    out <<std::setprecision(17);   // 用最大双精度尾数，避免读写往返丢失精度


    // Save nodes —— 每个节点一行 VERTEX_SE3:QUAT
    for (const auto & [id, pose] : poses_) {

        const Eigen::Vector3d t = pose.tEigen();
        const Eigen::Quaterniond q = pose.qEigen().normalized();   // 归一化避免数值漂移



        out<< "VERTEX_SE3:QUAT " << id << " " << t.x() << " " << t.y() << " " << t.z() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }

    // Save edges —— 每条边一行 EDGE_SE3:QUAT + 21 个信息矩阵元素（上三角）
    for (const auto &edge : edges_) {
        const Eigen::Vector3d t = edge.meas.translation();
        const Eigen::Quaterniond q = Eigen::Quaterniond(edge.meas.rotation()).normalized(); // Ensure the quaternion is normalized

        out << "EDGE_SE3:QUAT " << edge.i << " " << edge.j << " "<< t.x() << " " << t.y() << " " << t.z() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() ;

        // 依次写上三角：(0,0)(0,1)..(0,5)(1,1)..(5,5) 共 21 个元素
        for (int row = 0; row < 6; ++row) {
            for (int col = row; col < 6; ++col) {
                out << " " << edge.sqrt_information(row, col);
            }
        }
        out << std::endl;
    }

    out.flush();

    if (!out.good())        
    {
       LOG(ERROR) << "Failed to write to file: " << file_path;
       return false;
    }
    

    return true;
}

}  // namespace legkilo
