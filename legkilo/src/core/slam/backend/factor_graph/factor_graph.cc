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
// 根据平移和旋转的标准差，生成一个 6×6 的“平方根信息矩阵”，用于给图优化误差设置权重。
Eigen::Matrix<double, 6, 6> MakeSqrtInformation(double trans_sigma, double rot_sigma_deg) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

    const double safe_trans_sigma = trans_sigma;
    const double safe_rot_sigma = rot_sigma_deg * kDegToRad;

    Eigen::Matrix<double, 6, 6> sqrt_information = Eigen::Matrix<double, 6, 6>::Zero();
    sqrt_information.diagonal().head<3>().setConstant(1.0 / safe_trans_sigma);
    sqrt_information.diagonal().tail<3>().setConstant(1.0 / safe_rot_sigma);
    return sqrt_information;
}

}  // namespace

FactorGraph::FactorGraph(const std::string &yaml_file) : problem_(std::make_unique<ceres::Problem>()) {
    YamlHelper yaml(yaml_file);

    const std::string result_folder = yaml.get<std::string>("temp_result_save_folder", "temp");
    g2o_output_path_ = std::string(ROOT_DIR) + "result/" + result_folder + "/factor_graph.g2o";

    max_iterations_ = yaml.get<size_t>("factor_graph_max_iterations", 100);//最大迭代次数
    odom_near_trans_sigma_ = yaml.get<double>("odom_near_trans_sigma", 0.1);//相邻子地图之间的平移测量，典型误差大约在 10 cm 这个量级。
    odom_near_rot_sigma_deg_ = yaml.get<double>("odom_near_rot_sigma_deg", 5.0);//相邻子地图之间的旋转测量，典型误差大约在 5° 这个量级。
    odom_next_trans_sigma_multiplier_ = yaml.get<double>("odom_next_trans_sigma_multiplier", 2.0);
    odom_next_rot_sigma_multiplier_ = yaml.get<double>("odom_next_rot_sigma_multiplier", 2.0);
    loopclosure_trans_sigma_multiplier_ = yaml.get<double>("loopclosure_trans_sigma_multiplier", 10.0);
    loopclosure_rot_sigma_multiplier_ = yaml.get<double>("loopclosure_rot_sigma_multiplier", 4.0);
    loopclosure_loss_scale_ = yaml.get<double>("loopclosure_loss_scale", 3.0);
    LOG(INFO) << "FactorGraph is constructed";
}

FactorGraph::~FactorGraph() { LOG(INFO) << "FactorGraph is destructed"; }

void FactorGraph::addNode(NodeType id, const Eigen::Isometry3d &initial_pose) {
    if (initial_poses_.find(id) != initial_poses_.end()) return;
    initial_poses_.insert({id, initial_pose});
    node_ids_.push_back(id);

    auto it = poses_.emplace(id, graph::PoseParam(initial_pose)).first;
    graph::PoseParam &node = it->second;
    problem_->AddParameterBlock(node.t, 3);
    problem_->AddParameterBlock(node.q, 4);
    graph::QuaternionUpdateRule::Attach(*problem_, node.q);

    if (node_ids_.size() == 1) {
        problem_->SetParameterBlockConstant(node.t);//固定第一个节点
        problem_->SetParameterBlockConstant(node.q);//固定第一个节点的旋转
    }

    addOdometryEdgesForNewestNode();//添加里程计边：
}

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

bool FactorGraph::getPose(NodeType node_id, Eigen::Isometry3d &pose) const {
    if (poses_.find(node_id) == poses_.end()) return false;
    pose = poses_.at(node_id).toIsometry();
    return true;
}

//最后一项是鲁棒核函数，当某条边的误差特别大时，降低它对整体优化的影响。
void FactorGraph::addRelativeEdge(NodeType i, NodeType j, const Eigen::Isometry3d &meas, double trans_sigma,
                                  double rot_sigma_deg, ceres::LossFunction *loss_function) {
    auto it_i = poses_.find(i);
    auto it_j = poses_.find(j);
    if (it_i == poses_.end() || it_j == poses_.end()) return;

    const Eigen::Matrix<double, 6, 6> sqrt_information = MakeSqrtInformation(trans_sigma, rot_sigma_deg);

    edges_.push_back({i, j, meas, sqrt_information});//将边存储起来

    ceres::CostFunction *cost = graph::RelativePoseFactor::Create(meas, sqrt_information);//
    problem_->AddResidualBlock(cost, loss_function, it_i->second.t, it_i->second.q, it_j->second.t, it_j->second.q);
}

//添加里程计边
void FactorGraph::addOdometryEdgesForNewestNode() {
    const size_t num_nodes = node_ids_.size();
    if (num_nodes < 2) return;

    const NodeType newest_id = node_ids_.back();//num_nodes - 1
    const NodeType prev_id = node_ids_[num_nodes - 2];
    addRelativeEdge(prev_id, newest_id, initial_poses_.at(prev_id).inverse() * initial_poses_.at(newest_id),
                    odom_near_trans_sigma_, odom_near_rot_sigma_deg_);//两个参数表示对这条测量的信任程度

    if (num_nodes >= 3) {
        const NodeType prev2_id = node_ids_[num_nodes - 3];
        addRelativeEdge(prev2_id, newest_id, initial_poses_.at(prev2_id).inverse() * initial_poses_.at(newest_id),
                        odom_near_trans_sigma_ * odom_next_trans_sigma_multiplier_,
                        odom_near_rot_sigma_deg_ * odom_next_rot_sigma_multiplier_);
    }
}

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


bool FactorGraph::saveG2o(const std::string &file_path) const {
    std::ofstream out(file_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        LOG(ERROR) << "Failed to open file: " << file_path;
        return false;
    }

    out <<std::setprecision(17);


    // Save nodes
    for (const auto & [id, pose] : poses_) {

        const Eigen::Vector3d t = pose.tEigen();
        const Eigen::Quaterniond q = pose.qEigen().normalized();



        out<< "VERTEX_SE3:QUAT " << id << " " << t.x() << " " << t.y() << " " << t.z() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }

    // Save edges
    for (const auto &edge : edges_) {
        const Eigen::Vector3d t = edge.meas.translation();
        const Eigen::Quaterniond q = Eigen::Quaterniond(edge.meas.rotation()).normalized(); // Ensure the quaternion is normalized

        out << "EDGE_SE3:QUAT " << edge.i << " " << edge.j << " "<< t.x() << " " << t.y() << " " << t.z() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() ;

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
