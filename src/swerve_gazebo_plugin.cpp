// SPDX-License-Identifier: Apache-2.0
// Local Gazebo Classic joint actuator bridge. No simulated sensors.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace rover_sim {
class SwerveActuators : public gazebo::ModelPlugin {
  struct Actuator {
    gazebo::physics::JointPtr joint;
    bool steering;
    double effort;
    double target = 0;
    double integral = 0;
  };
  std::map<std::string, Actuator> actuators_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::physics::WorldPtr world_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr epoch_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr ack_sub_;
  gazebo::event::ConnectionPtr update_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point received_{};
  double last_publish_ = 0;
  double last_update_ = -1;
  double watchdog_timeout_ = 0.3;
  double minimum_command_time_ = 0;
  uint64_t epoch_ = 0;
  bool reset_pending_ = true;
  bool acknowledged_ = false;
 public:
  ~SwerveActuators() override { update_.reset(); sub_.reset(); ack_sub_.reset(); }
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    node_ = gazebo_ros::Node::Get(sdf);
    epoch_ = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
    world_ = model->GetWorld();
    watchdog_timeout_ = sdf->Get<double>("watchdog_timeout", 0.3).first;
    if (!std::isfinite(watchdog_timeout_) || watchdog_timeout_ <= 0) {
      RCLCPP_ERROR(node_->get_logger(), "watchdog_timeout must be finite and positive; bridge disabled");
      return;
    }
    for (auto &joint : model->GetJoints()) {
      const auto name = joint->GetName();
      const bool steering = name.size() >= 8 && name.compare(name.size()-8, 8, "_kingpin") == 0;
      const bool drive = name.size() >= 12 && name.compare(name.size()-12, 12, "_wheel_joint") == 0;
      if (steering || drive) {
        actuators_.emplace(name, Actuator{joint, steering, joint->GetEffortLimit(0), 0.0});
      }
    }
    if (actuators_.size() != 8) {
      RCLCPP_ERROR(node_->get_logger(), "Expected four kingpins and four drive joints; actuator bridge disabled");
      return;
    }
    epoch_pub_ = node_->create_publisher<std_msgs::msg::UInt64>(
      "/swerve/reset_epoch", rclcpp::QoS(1).transient_local());
    ack_sub_ = node_->create_subscription<std_msgs::msg::UInt64>(
      "/swerve/reset_ack", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::UInt64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (reset_pending_ || acknowledged_ || msg->data != epoch_) return;
        acknowledged_ = true;
        minimum_command_time_ = world_->SimTime().Double();
        received_ = {};
        for (auto &[name, a] : actuators_) {
          a.target = a.steering ? a.joint->Position(0) : 0.0;
          a.integral = 0.0;
        }
      });
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/swerve/joint_commands", rclcpp::QoS(1),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->name.size() != actuators_.size() || msg->position.size() != msg->name.size() ||
            msg->velocity.size() != msg->name.size()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        // A late callback must not refresh freshness across an expired lease,
        // even if the physics update is paused or has not run yet.
        if (ExpireWatchdog()) return;
        const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
        if (!acknowledged_ || reset_pending_ || msg->header.stamp.nanosec >= 1000000000 ||
            stamp <= minimum_command_time_ || stamp > world_->SimTime().Double() + 0.1) return;
        std::map<std::string, double> targets;
        for (size_t i = 0; i < msg->name.size(); ++i) {
          auto it = actuators_.find(msg->name[i]);
          if (it == actuators_.end() || targets.count(msg->name[i])) return;
          double value = it->second.steering ? msg->position[i] : msg->velocity[i];
          if (!std::isfinite(value)) return;
          auto j = it->second.joint;
          value = it->second.steering ? std::clamp(value, j->LowerLimit(0), j->UpperLimit(0)) :
            std::clamp(value, -j->GetVelocityLimit(0), j->GetVelocityLimit(0));
          targets.emplace(msg->name[i], value);
        }
        for (auto &entry : targets) actuators_.at(entry.first).target = entry.second;
        received_ = std::chrono::steady_clock::now();
      });
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&SwerveActuators::Update, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Eight-joint bridge ready; wall watchdog %.3f s", watchdog_timeout_);
  }
  void Reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    ClearForReset();
  }
  void ClearForReset() {
    ++epoch_;
    reset_pending_ = true;
    acknowledged_ = false;
    received_ = {};
    last_update_ = -1;
    last_publish_ = -1;
    for (auto &[name, a] : actuators_) {
      a.target = 0.0;
      a.integral = 0.0;
    }
  }
  bool ExpireWatchdog() {
    if (acknowledged_ && received_ != std::chrono::steady_clock::time_point{} &&
        std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count() >= watchdog_timeout_) {
      ClearForReset();
      RCLCPP_WARN(node_->get_logger(), "Actuator watchdog expired; new command epoch required");
      return true;
    }
    return false;
  }
  void Update(const gazebo::common::UpdateInfo &info) {
    std::lock_guard<std::mutex> lock(mutex_);
    ExpireWatchdog();
    // A time-only reset may not invoke ModelPlugin::Reset().
    if (last_update_ >= 0 && info.simTime.Double() < last_update_) ClearForReset();
    if (reset_pending_) {
      // Read positions after Gazebo has finished resetting the physical joints.
      for (auto &[name, a] : actuators_) {
        a.target = a.steering ? a.joint->Position(0) : 0.0;
        a.integral = 0.0;
      }
      minimum_command_time_ = info.simTime.Double();
      reset_pending_ = false;
      std_msgs::msg::UInt64 notification;
      notification.data = epoch_;
      epoch_pub_->publish(notification);
    }
    const bool fresh = acknowledged_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count() < watchdog_timeout_;
    const double dt = last_update_ < 0 ? 0.0 : std::clamp(info.simTime.Double()-last_update_, 0.0, 0.02);
    last_update_ = info.simTime.Double();
    sensor_msgs::msg::JointState state;
    state.header.stamp = rclcpp::Time(static_cast<int64_t>(info.simTime.Double()*1e9));
    for (auto &[name, a] : actuators_) {
      const double position = a.joint->Position(0), velocity = a.joint->GetVelocity(0);
      // Steering PD and wheel velocity PI act through physics; never teleport joints.
      double effort;
      if (a.steering) {
        effort = 200.0*(a.target-position)-8.0*velocity;
      } else {
        const double target = fresh ? a.target : 0.0;
        const double error = target-velocity;
        if (std::abs(target) < 1e-9) a.integral = 0.0;
        const double proposed = std::clamp(a.integral+30.0*error*dt, -0.5*a.effort, 0.5*a.effort);
        const double candidate_effort = 8.0*error+proposed;
        // Integrate only when unsaturated or when unwinding a saturated command.
        if (std::abs(target) > 1e-9 &&
            (std::abs(candidate_effort) < a.effort || error*candidate_effort < 0)) a.integral = proposed;
        effort = 8.0*error+a.integral;
      }
      a.joint->SetForce(0, std::clamp(effort, -a.effort, a.effort));
      state.name.push_back(name);
      state.position.push_back(position);
      state.velocity.push_back(velocity);
      state.effort.push_back(std::clamp(effort, -a.effort, a.effort));
    }
    if (info.simTime.Double() < last_publish_ || info.simTime.Double()-last_publish_ >= 0.02) {
      pub_->publish(state);
      last_publish_ = info.simTime.Double();
    }
  }
};
GZ_REGISTER_MODEL_PLUGIN(SwerveActuators)
}  // namespace rover_sim
