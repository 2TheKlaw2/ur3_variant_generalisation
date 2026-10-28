// Capture a stationary V1 checkpoint from /joint_states and TF (base -> gripper_tcp).
// Read-only ROS2 node: it does not publish commands, enable freedrive, or alter home.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

namespace {

constexpr std::array<const char *, 6> kJoints = {
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
};
struct Observation {
  Clock::time_point stamp;
  std::array<double, 6> joints{};
  double max_reported_velocity = 0.0;
};

class SnapshotNode : public rclcpp::Node {
public:
  SnapshotNode() : Node("variant_save_named_pose") {
    base_ = declare_parameter<std::string>("base_frame", "base");
    tool_ = declare_parameter<std::string>("tool_frame", "gripper_tcp");
    dir_ = declare_parameter<std::string>("output_dir", "config/saved_poses");
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { onJointState(*msg); });
    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  }

  bool ready(std::string & why) const {
    if (observations_.size() < 10) {
      why = "Waiting for 10 valid joint-state messages";
      return false;
    }
    const double window = std::chrono::duration<double>(
      observations_.back().stamp - observations_.front().stamp).count();
    if (window < 0.7) {
      why = "Waiting for a 0.7 second stationary observation window";
      return false;
    }
    if (std::chrono::duration<double>(Clock::now() - observations_.back().stamp).count() > 0.5) {
      why = "Joint states are stale";
      return false;
    }
    for (std::size_t j = 0; j < kJoints.size(); ++j) {
      double lo = observations_[0].joints[j];
      double hi = lo;
      for (const auto & obs : observations_) {
        lo = std::min(lo, obs.joints[j]);
        hi = std::max(hi, obs.joints[j]);
      }
      if (hi - lo > 0.005) {
        why = std::string("Robot moving at ") + kJoints[j] + " (span > 0.005 rad)";
        return false;
      }
    }
    for (const auto & obs : observations_) {
      if (obs.max_reported_velocity > 0.03) {
        why = "Robot moving (joint speed > 0.03 rad/s)";
        return false;
      }
    }
    why.clear();
    return true;
  }

  bool captureTF(geometry_msgs::msg::TransformStamped & out, std::string & reason) {
    try {
      out = buffer_->lookupTransform(base_, tool_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      reason = std::string("TF unavailable: ") + e.what();
      return false;
    }
    const double age = (now() - rclcpp::Time(out.header.stamp)).seconds();
    if (age > 2.0 || age < -1.0) {
      reason = "TF is stale or has mismatched clock (age " + std::to_string(age) + " s)";
      return false;
    }
    const auto & q = out.transform.rotation;
    const double qnorm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(qnorm) || std::abs(qnorm-1.0) > 0.05) {
      reason = "Invalid TCP quaternion";
      return false;
    }
    for (double v : {out.transform.translation.x, out.transform.translation.y,
                     out.transform.translation.z}) {
      if (!std::isfinite(v)) {reason = "Invalid TCP position"; return false;}
    }
    return true;
  }

  const Observation & last() const {return observations_.back();}
  const std::string & base() const {return base_;}
  const std::string & tool() const {return tool_;}
  const std::string & directory() const {return dir_;}

private:
  void onJointState(const sensor_msgs::msg::JointState & msg) {
    if (msg.position.size() < msg.name.size()) {return;}
    Observation obs;
    obs.stamp = Clock::now();
    std::map<std::string, std::size_t> indices;
    for (std::size_t i = 0; i < msg.name.size(); ++i) {indices[msg.name[i]] = i;}
    for (std::size_t i = 0; i < kJoints.size(); ++i) {
      auto found = indices.find(kJoints[i]);
      if (found == indices.end()) {return;}
      const std::size_t j = found->second;
      if (!std::isfinite(msg.position[j])) {return;}
      obs.joints[i] = msg.position[j];
      if (j < msg.velocity.size()) {
        if (!std::isfinite(msg.velocity[j])) {return;}
        obs.max_reported_velocity = std::max(obs.max_reported_velocity, std::abs(msg.velocity[j]));
      }
    }
    observations_.push_back(obs);
    if (observations_.size() > 45) {observations_.erase(observations_.begin());}
  }
  std::string base_, tool_, dir_;
  std::vector<Observation> observations_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

bool writeNewFile(const fs::path & filename, const std::string & contents) {
  // O_EXCL prevents silently destroying a previously calibrated pose.
  const int fd = ::open(filename.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
  if (fd < 0) {
    std::cerr << "ERROR: Cannot create " << filename << ": " << std::strerror(errno) << '\n';
    return false;
  }
  std::size_t done = 0;
  while (done < contents.size()) {
    const ssize_t n = ::write(fd, contents.data() + done, contents.size() - done);
    if (n <= 0) {
      std::cerr << "ERROR: Failed writing " << filename << '\n';
      ::close(fd);
      ::unlink(filename.c_str());
      return false;
    }
    done += static_cast<std::size_t>(n);
  }
  if (::close(fd) != 0) {
    std::cerr << "ERROR: Close failed for " << filename << '\n';
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv) {
  if (argc < 2 || !std::regex_match(argv[1], std::regex("v1_[A-Za-z0-9_]+"))) {
    std::cerr << "Usage: ros2 run ur3_variant_generalisation save_named_pose v1_grip_pose --ros-args -p output_dir:=...\n";
    return 2;
  }
  const std::string name = argv[1];
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SnapshotNode>();
  std::cout << "Sampling stationary robot joint states and TF " << node->base()
            << " -> " << node->tool() << " (12 s timeout)...\n";
  auto began = Clock::now();
  bool ok = false;
  geometry_msgs::msg::TransformStamped tf;
  std::string status;
  while (rclcpp::ok() &&
         std::chrono::duration<double>(Clock::now() - began).count() < 12.0) {
    rclcpp::spin_some(node);
    if (node->ready(status) && node->captureTF(tf, status)) {ok = true; break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
  }
  if (!ok) {
    std::cerr << "ERROR: Did not capture a stationary valid pose: " << status << '\n';
    rclcpp::shutdown();
    return 1;
  }
  fs::path dir(node->directory());
  try {fs::create_directories(dir);}
  catch (const fs::filesystem_error & e) {
    std::cerr << "ERROR: " << e.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  const fs::path file = dir / (name + ".yaml");
  std::ostringstream yaml;
  yaml << std::setprecision(12)
       << "# Stationary TCP + joints, measurement ONLY (not a controller command)\n"
       << "name: " << name << "\n"
       << "robot: ur3e\n"
       << "base_frame: " << node->base() << "\n"
       << "tool_frame: " << node->tool() << "\n"
       << "arm:\n  joint_names:\n";
  for (const char * j : kJoints) {yaml << "    - " << j << "\n";}
  yaml << "  positions_rad:\n";
  for (double p : node->last().joints) {yaml << "    - " << p << "\n";}
  const auto & t = tf.transform.translation;
  const auto & q = tf.transform.rotation;
  yaml << "tcp:\n"
       << "  position_m: [" << t.x << ", " << t.y << ", " << t.z << "]\n"
       << "  quaternion_xyzw: [" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << "]\n";
  if (!writeNewFile(file, yaml.str())) {
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "SAVED: " << file << "\n"
            << "This pose does not overwrite config/home.yaml and does not move the robot.\n";
  rclcpp::shutdown();
  return 0;
}
