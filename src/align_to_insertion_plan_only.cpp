// V1: plan a short Cartesian alignment from the LIVE completed reach to saved insertion.
// This executable has NO action client and NO execution code. Never drives hardware.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/conversions.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace {
constexpr std::array<const char*, 6> kJoints = {
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
};
constexpr double kPi = 3.14159265358979323846;

std::array<double, 6> readTransportEnd(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Missing reference reach trajectory: " + path);
  std::string header, line, last;
  if (!std::getline(in, header)) throw std::runtime_error("Empty reference CSV");
  while (std::getline(in, line)) if (!line.empty()) last = line;
  if (last.empty()) throw std::runtime_error("Reference reach trajectory has no rows");
  std::vector<std::string> columns, fields;
  {
    std::stringstream ss(header);
    std::string val;
    while (std::getline(ss, val, ',')) {
    if (!val.empty() && val.back() == '\r') val.pop_back();
    columns.push_back(val);
  }
  }
  {
    std::stringstream ss(last);
    std::string val;
    while (std::getline(ss, val, ',')) fields.push_back(val);
  }
  if (fields.size() != columns.size()) throw std::runtime_error("Malformed last CSV row");
  std::array<double, 6> expected{};
  for (size_t i=0; i<kJoints.size(); ++i) {
    auto it = std::find(columns.begin(), columns.end(), kJoints[i]);
    if (it == columns.end()) throw std::runtime_error(std::string("Missing CSV joint: ") + kJoints[i]);
    expected[i] = std::stod(fields[std::distance(columns.begin(), it)]);
    if (!std::isfinite(expected[i])) throw std::runtime_error("Nonfinite reference joint");
  }
  return expected;
}

geometry_msgs::msg::Pose relativePose(const moveit::core::RobotState& state) {
  const Eigen::Isometry3d& base = state.getGlobalLinkTransform("base");
  const Eigen::Isometry3d& tip = state.getGlobalLinkTransform("gripper_tcp");
  Eigen::Isometry3d relative = base.inverse() * tip;
  Eigen::Quaterniond quat(relative.rotation());
  quat.normalize();
  geometry_msgs::msg::Pose pose;
  pose.position.x = relative.translation().x();
  pose.position.y = relative.translation().y();
  pose.position.z = relative.translation().z();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  return pose;
}

Eigen::Quaterniond quatOf(const geometry_msgs::msg::Pose& p) {
  Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  if (!std::isfinite(q.norm()) || q.norm() < 1e-9)
    throw std::runtime_error("Invalid TCP quaternion");
  q.normalize();
  return q;
}

double positionDistance(const geometry_msgs::msg::Pose& a,
                        const geometry_msgs::msg::Pose& b) {
  return std::hypot(a.position.x-b.position.x,
                    a.position.y-b.position.y,
                    a.position.z-b.position.z);
}

double angularDistance(const geometry_msgs::msg::Pose& a,
                       const geometry_msgs::msg::Pose& b) {
  const auto qa = quatOf(a), qb = quatOf(b);
  return 2.0 * std::acos(std::clamp(std::abs(qa.dot(qb)), 0.0, 1.0));
}

class Spinner {
public:
  explicit Spinner(const rclcpp::Node::SharedPtr& node) {
    executor_.add_node(node);
    worker_ = std::thread([this](){ executor_.spin(); });
  }
  ~Spinner() {
    executor_.cancel();
    if (worker_.joinable()) worker_.join();
  }
private:
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread worker_;
};
} // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("align_to_insertion_plan_only");
  auto log = node->get_logger();
  int code = 1;
  try {
    // Guard this diagnostic against accidental use in the physical-robot domain.
    const char* domain = std::getenv("ROS_DOMAIN_ID");
    if (!domain || std::string(domain) != "73")
      throw std::runtime_error("PLAN-ONLY V1 alignment requires ROS_DOMAIN_ID=73");
    const std::string urdf = node->declare_parameter<std::string>("robot_description", "");
    if (
        urdf.find("mock_components/GenericSystem") == std::string::npos ||
        urdf.find("ur_robot_driver/URPositionHardwareInterface") != std::string::npos)
      throw std::runtime_error("Parameters must contain the FAKE URDF, not real hardware");

    const auto grip_path = node->declare_parameter<std::string>("insertion_pose_file", "");
    const auto reach_csv = node->declare_parameter<std::string>("expected_transport_joint_csv", "");
    if (grip_path.empty() || reach_csv.empty())
      throw std::runtime_error("Missing insertion_pose_file or expected_transport_joint_csv");
    const auto expected = readTransportEnd(reach_csv);
    const auto yaml = YAML::LoadFile(grip_path);
    if (yaml["robot"].as<std::string>() != "ur3e" ||
        yaml["base_frame"].as<std::string>() != "base" ||
        yaml["tool_frame"].as<std::string>() != "gripper_tcp")
      throw std::runtime_error("Insertion YAML must describe UR3e base -> gripper_tcp");
    const auto xyz = yaml["tcp"]["position_m"];
    const auto xyzw = yaml["tcp"]["quaternion_xyzw"];
    if (!xyz.IsSequence() || xyz.size()!=3 || !xyzw.IsSequence() || xyzw.size()!=4)
      throw std::runtime_error("Invalid insertion TCP pose format");
    geometry_msgs::msg::Pose goal;
    goal.position.x=xyz[0].as<double>(); goal.position.y=xyz[1].as<double>(); goal.position.z=xyz[2].as<double>();
    goal.orientation.x=xyzw[0].as<double>(); goal.orientation.y=xyzw[1].as<double>();
    goal.orientation.z=xyzw[2].as<double>(); goal.orientation.w=xyzw[3].as<double>();
    for(double v: {goal.position.x, goal.position.y, goal.position.z,
                   goal.orientation.x, goal.orientation.y, goal.orientation.z,goal.orientation.w}) {
      if (!std::isfinite(v)) throw std::runtime_error("Nonfinite saved insertion pose");
    }
    const auto gq = quatOf(goal);
    goal.orientation.x=gq.x(); goal.orientation.y=gq.y();
    goal.orientation.z=gq.z(); goal.orientation.w=gq.w();

    Spinner spinner(node);
    moveit::planning_interface::MoveGroupInterface group(node,"ur_onrobot_manipulator");
    group.setPoseReferenceFrame("base");
    if (!group.setEndEffectorLink("gripper_tcp"))
      throw std::runtime_error("Could not set gripper_tcp as the end effector");
    auto state = group.getCurrentState(5.0);
    if (!state) throw std::runtime_error("No fresh current joint state");
    if (!state->getRobotModel()->hasLinkModel("base") ||
        !state->getRobotModel()->hasLinkModel("gripper_tcp"))
      throw std::runtime_error("Missing base or gripper_tcp link in robot model");
    state->update();
    double max_start_error = 0.0;
    for (size_t i=0; i<kJoints.size(); ++i) {
      double actual = state->getVariablePosition(kJoints[i]);
      double delta = actual - expected[i];
      if (i==5) delta=std::remainder(delta,2.0*kPi); // continuous Wrist 3 representation
      max_start_error = std::max(max_start_error,std::abs(delta));
      RCLCPP_INFO(log, "Start %-22s measured=%+.5f expected=%+.5f delta=%.5f rad",
                  kJoints[i],actual,expected[i],std::abs(delta));
    }
    if (max_start_error > 0.015)
      throw std::runtime_error("Sim robot is not at end of learned V1 transport (max joint error > 0.03 rad)");

    const auto start = relativePose(*state);
    const double distance = positionDistance(start,goal);
    const double angle = angularDistance(start,goal);
    RCLCPP_INFO(log,"Current TCP (base): [%.6f %.6f %.6f] m",
                start.position.x,start.position.y,start.position.z);
    RCLCPP_INFO(log,"Saved insertion TCP:     [%.6f %.6f %.6f] m",
                goal.position.x,goal.position.y,goal.position.z);
    RCLCPP_INFO(log,"ALIGNMENT REQUIRED: %.2f mm translation, %.2f deg rotation",
                distance*1000.0,angle*180.0/kPi);
    if (distance > 0.03 || angle > 15.0*kPi/180.0)
      throw std::runtime_error("Alignment is too large; wrong TCP frame or start state?");
    if (distance < 0.00001 && angle < 0.0001)
      throw std::runtime_error("Already at insertion-ready pose; no motion to plan");

    // Smooth interpolation of both position and orientation; NO pre-position path.
    const int segments = std::max(2,static_cast<int>(std::ceil(std::max(
      distance/0.001, angle/(1.0*kPi/180.0)))));
    const Eigen::Quaterniond q0=quatOf(start);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(segments);
    for (int i=1; i<=segments; ++i) {
      const double f=static_cast<double>(i)/segments;
      geometry_msgs::msg::Pose p;
      p.position.x=start.position.x+f*(goal.position.x-start.position.x);
      p.position.y=start.position.y+f*(goal.position.y-start.position.y);
      p.position.z=start.position.z+f*(goal.position.z-start.position.z);
      Eigen::Quaterniond q=q0.slerp(f,gq);
      p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();
      waypoints.push_back(p);
    }
    group.setStartState(*state);
    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    double fraction=group.computeCartesianPath(waypoints,0.001,0.0,trajectory_msg,true);
    RCLCPP_INFO(log,"CARTESIAN ALIGNMENT FRACTION: %.1f%% (%d target samples)",fraction*100.0,segments);
    if (fraction < 0.999)
      throw std::runtime_error("Alignment planning did not reach 100% WITH collision checking");

    robot_trajectory::RobotTrajectory rt(state->getRobotModel(),"ur_onrobot_manipulator");
    rt.setRobotTrajectoryMsg(*state,trajectory_msg);
    trajectory_processing::IterativeParabolicTimeParameterization retimer;
    if (!retimer.computeTimeStamps(rt,0.05,0.05))
      throw std::runtime_error("MoveIt could not time-parameterise alignment");
    rt.getRobotTrajectoryMsg(trajectory_msg);
    auto& jt=trajectory_msg.joint_trajectory;
    if (jt.points.size()<2)
      throw std::runtime_error("Insufficient joint trajectory points");
    auto it=std::find(jt.joint_names.begin(),jt.joint_names.end(),"wrist_3_joint");
    if (it==jt.joint_names.end()) throw std::runtime_error("Wrist 3 missing from trajectory");
    size_t wrist_idx=std::distance(jt.joint_names.begin(),it);
    const double branch_shift=2*kPi*std::round((expected[5]-jt.points.front().positions.at(wrist_idx))/(2*kPi));
    for (auto& point: jt.points) point.positions.at(wrist_idx)+=branch_shift;
    double max_step=0.0;
    for(size_t i=1; i<jt.points.size(); ++i) {
      for(size_t j=0; j<jt.joint_names.size(); ++j) {
        max_step=std::max(max_step,std::abs(jt.points[i].positions[j]-jt.points[i-1].positions[j]));
      }
    }
    if (max_step>0.05) throw std::runtime_error("Joint trajectory has a >0.05 rad step");
    const auto end = relativePose(rt.getLastWayPoint());
    const auto pos_err=positionDistance(end,goal);
    const auto rot_err=angularDistance(end,goal);
    RCLCPP_INFO(log,"Final FK error: %.4f mm, %.4f deg",pos_err*1000.0,rot_err*180.0/kPi);
    if(pos_err>0.001 || rot_err>1.0*kPi/180.0)
      throw std::runtime_error("Final TCP pose differs from saved insertion beyond tolerance");

    // V1-INSERTION-ALIGNMENT-CSV-EXPORT-ONLY: save the SAME validated, wrist-corrected
    // joint trajectory shown in RViz. No controller/action interaction.
    const std::string export_csv = "/tmp/v1_align_to_insertion_joint_plan_001.csv";
    {
      std::ofstream out(export_csv, std::ios::trunc);
      if (!out) throw std::runtime_error("Could not open alignment CSV export");
      out << "time";
      std::array<size_t, 6> indexes{};
      for (size_t j = 0; j < kJoints.size(); ++j) {
        auto ji = std::find(jt.joint_names.begin(), jt.joint_names.end(), kJoints[j]);
        if (ji == jt.joint_names.end())
          throw std::runtime_error(std::string("Alignment joint missing: ") + kJoints[j]);
        indexes[j] = std::distance(jt.joint_names.begin(), ji);
        out << "," << kJoints[j];
      }
      out << "\n";
      out << std::setprecision(16);
      for (const auto& pt : jt.points) {
        out << (static_cast<double>(pt.time_from_start.sec) +
                static_cast<double>(pt.time_from_start.nanosec) * 1e-9);
        for (size_t j = 0; j < kJoints.size(); ++j)
          out << "," << pt.positions.at(indexes[j]);
        out << "\n";
        if (!out) throw std::runtime_error("Alignment CSV write failed");
      }
      out.close();
      if (!out) throw std::runtime_error("Alignment CSV close failed");
    }
    RCLCPP_INFO(log, "Saved validated alignment joint plan: %s", export_csv.c_str());

    auto display_pub=node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",rclcpp::QoS(1).reliable());
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id=state->getRobotModel()->getName();
    moveit::core::robotStateToRobotStateMsg(*state,display.trajectory_start);
    // Match visualization's Wrist 3 start branch to the corrected trajectory.
    auto& names=display.trajectory_start.joint_state.name;
    auto wi=std::find(names.begin(),names.end(),"wrist_3_joint");
    if(wi!=names.end()) {
      auto& start_wrist = display.trajectory_start.joint_state.position.at(
        std::distance(names.begin(),wi));
      start_wrist += 2*kPi*std::round((expected[5]-start_wrist)/(2*kPi));
    }
    display.trajectory.push_back(trajectory_msg);
    display_pub->publish(display);
    const auto duration=jt.points.back().time_from_start;
    RCLCPP_INFO(log,"ALIGN_TO_INSERTION PLAN ONLY: PASS. %zu trajectory points; %.3f s; max step %.5f rad",
                jt.points.size(), duration.sec+duration.nanosec*1e-9,max_step);
    RCLCPP_INFO(log,"NO MOVEMENT COMMANDED. Inspect alignment in RViz; physical clearance still unverified.");
    std::this_thread::sleep_for(2s);
    code=0;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(log,"ALIGNMENT BLOCKED: %s",e.what());
  }
  rclcpp::shutdown();
  return code;
}
