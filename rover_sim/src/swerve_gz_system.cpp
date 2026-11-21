// SPDX-License-Identifier: Apache-2.0
// Gazebo Fortress (gz-sim 6) swerve actuator System for the rover.
//
// Port of the Gazebo Classic ModelPlugin (src/swerve_gazebo_plugin.cpp at tag
// classic-final) to the Entity-Component API, with the same ROS interface:
//   in : /swerve/joint_commands (sensor_msgs/JointState: kingpin positions,
//        wheel velocities), /swerve/reset_ack
//   out: /joint_states (8 actuated + passive "*_rocker_joint"),
//        /swerve/reset_epoch, and optionally /ground_truth/odom (replaces
//        Classic's libgazebo_ros_p3d)
// Same safety behaviour: reset epochs with acknowledgement, wall-clock
// watchdog, strict joint-set check, stale/future command rejection.
//
// Differences from Classic:
// - Parking hold: with a zero (or stale) wheel target the wheel is held at
//   the position it had when the hold engaged (PD, limited to the joint
//   effort limit). Classic only drove the wheel velocity to zero with a P
//   term, so the rover crept down slopes of 25 deg and more after stopping.
//   The hold engages at any speed: while the wheel still turns, the position
//   term grows and brakes it.
// - gz-sim 6 has no ISystemReset; a world reset is detected as simulation
//   time moving backwards, which clears the epoch as on Classic.
// - The ROS node is a plain rclcpp node spun on its own thread.
//
// Rocker differential (exactly two "*_rocker_joint" joints): each rocker gets
// torque = -k (left + right) - c (left rate + right rate).
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/ChildLinkName.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/plugin/Register.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rover_sim {
namespace gz = ignition::gazebo;

namespace {
bool EndsWith(const std::string &name, const std::string &suffix) {
  return name.size() >= suffix.size() &&
         name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
}
}  // namespace

class SwerveGzSystem : public gz::System,
                       public gz::ISystemConfigure,
                       public gz::ISystemPreUpdate,
                       public gz::ISystemPostUpdate {
  struct Joint {
    gz::Entity entity{gz::kNullEntity};
    double position = 0, velocity = 0, applied = 0;
  };
  struct Actuator : Joint {
    bool steering = false;
    double effort = 0, velocity_limit = 0, lower = -1e300, upper = 1e300;
    double target = 0, integral = 0;
    bool holding = false;
    double hold_position = 0;
  };

  std::map<std::string, Actuator> actuators_;
  std::map<std::string, Joint> passive_;
  gz::Model model_{gz::kNullEntity};
  gz::Entity body_{gz::kNullEntity};
  std::string body_name_;

  // Gains (SDF-overridable). Steering PD and wheel PI match the Classic plugin.
  double steering_kp_ = 200, steering_kd_ = 8;
  double wheel_kp_ = 8, wheel_ki_ = 30;
  double hold_kp_ = 40, hold_kd_ = 6;
  double differential_stiffness_ = 0, differential_damping_ = 0;
  double watchdog_timeout_ = 0.3;
  double publish_period_ = 0.02, truth_period_ = 0.02;
  bool ground_truth_ = false;
  bool enabled_ = false;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spinner_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr epoch_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr ack_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub_;

  std::mutex mutex_;
  std::chrono::steady_clock::time_point received_{};
  double sim_now_ = 0, last_update_ = -1, last_publish_ = -1, last_truth_ = -1;
  double minimum_command_time_ = 0;
  uint64_t epoch_ = 0;
  bool reset_pending_ = true;
  bool acknowledged_ = false;

 public:
  ~SwerveGzSystem() override {
    if (executor_) executor_->cancel();
    if (spinner_.joinable()) spinner_.join();
  }

  void Configure(const gz::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::EntityComponentManager &ecm, gz::EventManager &) override {
    model_ = gz::Model(entity);
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("swerve_actuators");
    epoch_ = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());

    auto get = [&](const std::string &key, double fallback) {
      return sdf->HasElement(key) ? sdf->Get<double>(key) : fallback;
    };
    watchdog_timeout_ = get("watchdog_timeout", 0.3);
    differential_stiffness_ = get("differential_stiffness", 0.0);
    differential_damping_ = get("differential_damping", 0.0);
    steering_kp_ = get("steering_kp", steering_kp_);
    steering_kd_ = get("steering_kd", steering_kd_);
    wheel_kp_ = get("wheel_kp", wheel_kp_);
    wheel_ki_ = get("wheel_ki", wheel_ki_);
    hold_kp_ = get("hold_kp", hold_kp_);
    hold_kd_ = get("hold_kd", hold_kd_);
    const double publish_rate = get("publish_rate", 50.0);
    const double truth_rate = get("ground_truth_rate", 50.0);
    ground_truth_ = sdf->HasElement("ground_truth") && sdf->Get<bool>("ground_truth");
    body_name_ = sdf->HasElement("body_name") ? sdf->Get<std::string>("body_name") : "base_root";

    for (double v : {watchdog_timeout_, publish_rate, truth_rate}) {
      if (!std::isfinite(v) || v <= 0) {
        RCLCPP_ERROR(node_->get_logger(), "watchdog_timeout and rates must be finite and positive; bridge disabled");
        return;
      }
    }
    for (double v : {differential_stiffness_, differential_damping_, steering_kp_, steering_kd_,
                     wheel_kp_, wheel_ki_, hold_kp_, hold_kd_}) {
      if (!std::isfinite(v) || v < 0) {
        RCLCPP_ERROR(node_->get_logger(), "gains must be finite and non-negative; bridge disabled");
        return;
      }
    }
    publish_period_ = 1.0 / publish_rate;
    truth_period_ = 1.0 / truth_rate;

    for (auto joint : model_.Joints(ecm)) {
      const auto name = ecm.Component<gz::components::Name>(joint)->Data();
      const bool steering = EndsWith(name, "_kingpin");
      const bool drive = EndsWith(name, "_wheel_joint");
      const bool rocker = EndsWith(name, "_rocker_joint");
      if (!steering && !drive && !rocker) continue;
      // Ask the physics system to fill in position and velocity each step.
      ecm.CreateComponent(joint, gz::components::JointPosition());
      ecm.CreateComponent(joint, gz::components::JointVelocity());
      if (rocker) {
        Joint j;
        j.entity = joint;
        passive_.emplace(name, j);
        continue;
      }
      Actuator a;
      a.entity = joint;
      a.steering = steering;
      if (auto *axis = ecm.Component<gz::components::JointAxis>(joint)) {
        a.effort = axis->Data().Effort();
        a.velocity_limit = axis->Data().MaxVelocity();
        a.lower = axis->Data().Lower();
        a.upper = axis->Data().Upper();
      }
      if (!std::isfinite(a.effort) || a.effort <= 0 ||
          (drive && (!std::isfinite(a.velocity_limit) || a.velocity_limit <= 0))) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s needs positive effort (and, for wheels, velocity) limits; "
                     "actuator bridge disabled", name.c_str());
        actuators_.clear();
        return;
      }
      actuators_.emplace(name, a);
    }
    if (!passive_.empty() && passive_.size() != 2) {
      RCLCPP_WARN(node_->get_logger(), "Found %zu rocker joints; differential needs exactly two and is off",
                  passive_.size());
    }
    if (actuators_.size() != 8) {
      RCLCPP_ERROR(node_->get_logger(), "Expected four kingpins and four drive joints; actuator bridge disabled");
      actuators_.clear();
      return;
    }

    if (ground_truth_) {
      // URDF fixed joints are lumped on import: base_link merges into base_root.
      body_ = model_.LinkByName(ecm, body_name_);
      if (body_ == gz::kNullEntity) body_ = model_.CanonicalLink(ecm);
      gz::Link(body_).EnableVelocityChecks(ecm, true);
      body_name_ = ecm.Component<gz::components::Name>(body_)->Data();
      truth_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/ground_truth/odom", 10);
    }

    epoch_pub_ = node_->create_publisher<std_msgs::msg::UInt64>(
      "/swerve/reset_epoch", rclcpp::QoS(1).transient_local());
    ack_sub_ = node_->create_subscription<std_msgs::msg::UInt64>(
      "/swerve/reset_ack", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::UInt64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (reset_pending_ || acknowledged_ || msg->data != epoch_) return;
        acknowledged_ = true;
        minimum_command_time_ = sim_now_;
        received_ = {};
        HoldCurrentState();
      });
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/swerve/joint_commands", rclcpp::QoS(1),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnCommand(*msg); });
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spinner_ = std::thread([this] { executor_->spin(); });
    enabled_ = true;
    const std::string truth = ground_truth_ ? "; ground truth on " + body_name_ : "";
    RCLCPP_INFO(node_->get_logger(),
                "Eight-joint gz-sim bridge ready (%zu passive rocker joints); wall watchdog %.3f s; "
                "parking hold kp %.1f kd %.1f%s",
                passive_.size(), watchdog_timeout_, hold_kp_, hold_kd_, truth.c_str());
  }

  void OnCommand(const sensor_msgs::msg::JointState &msg) {
    if (msg.name.size() != actuators_.size() || msg.position.size() != msg.name.size() ||
        msg.velocity.size() != msg.name.size()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    // A late callback must not refresh freshness across an expired lease.
    if (ExpireWatchdog()) return;
    const double stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
    if (!acknowledged_ || reset_pending_ || msg.header.stamp.nanosec >= 1000000000 ||
        stamp <= minimum_command_time_ || stamp > sim_now_ + 0.1) return;
    std::map<std::string, double> targets;
    for (size_t i = 0; i < msg.name.size(); ++i) {
      auto it = actuators_.find(msg.name[i]);
      if (it == actuators_.end() || targets.count(msg.name[i])) return;
      const auto &a = it->second;
      double value = a.steering ? msg.position[i] : msg.velocity[i];
      if (!std::isfinite(value)) return;
      value = a.steering ? std::clamp(value, a.lower, a.upper)
                         : std::clamp(value, -a.velocity_limit, a.velocity_limit);
      targets.emplace(msg.name[i], value);
    }
    for (auto &entry : targets) actuators_.at(entry.first).target = entry.second;
    received_ = std::chrono::steady_clock::now();
  }

  void HoldCurrentState() {
    for (auto &[name, a] : actuators_) {
      a.target = a.steering ? a.position : 0.0;
      a.integral = 0.0;
      a.holding = false;
    }
  }

  void ClearForReset() {
    ++epoch_;
    reset_pending_ = true;
    acknowledged_ = false;
    received_ = {};
    last_update_ = -1;
    last_publish_ = last_truth_ = -1;
    for (auto &[name, a] : actuators_) {
      a.target = 0.0;
      a.integral = 0.0;
      a.holding = false;
    }
  }

  bool ExpireWatchdog() {
    if (acknowledged_ && received_ != std::chrono::steady_clock::time_point{} &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() >= watchdog_timeout_) {
      ClearForReset();
      RCLCPP_WARN(node_->get_logger(), "Actuator watchdog expired; new command epoch required");
      return true;
    }
    return false;
  }

  static void Read(Joint &j, const gz::EntityComponentManager &ecm) {
    auto *p = ecm.Component<gz::components::JointPosition>(j.entity);
    auto *v = ecm.Component<gz::components::JointVelocity>(j.entity);
    j.position = p && !p->Data().empty() ? p->Data()[0] : 0.0;
    j.velocity = v && !v->Data().empty() ? v->Data()[0] : 0.0;
  }

  static void Command(gz::Entity joint, double effort, gz::EntityComponentManager &ecm) {
    auto *cmd = ecm.Component<gz::components::JointForceCmd>(joint);
    if (cmd) cmd->Data() = {effort};
    else ecm.CreateComponent(joint, gz::components::JointForceCmd({effort}));
  }

  double WheelEffort(Actuator &a, bool fresh, double dt) {
    const double target = fresh ? a.target : 0.0;
    if (std::abs(target) < 1e-9) {
      a.integral = 0.0;
      if (!a.holding) { a.holding = true; a.hold_position = a.position; }
      return hold_kp_ * (a.hold_position - a.position) - hold_kd_ * a.velocity;
    }
    a.holding = false;
    const double error = target - a.velocity;
    const double proposed = std::clamp(a.integral + wheel_ki_ * error * dt, -0.5 * a.effort, 0.5 * a.effort);
    const double candidate = wheel_kp_ * error + proposed;
    // Integrate only when unsaturated or when unwinding a saturated command.
    if (std::abs(candidate) < a.effort || error * candidate < 0) a.integral = proposed;
    return wheel_kp_ * error + a.integral;
  }

  void PreUpdate(const gz::UpdateInfo &info, gz::EntityComponentManager &ecm) override {
    if (!enabled_ || info.paused) return;
    std::lock_guard<std::mutex> lock(mutex_);
    ExpireWatchdog();
    const double now = std::chrono::duration<double>(info.simTime).count();
    sim_now_ = now;
    if (last_update_ >= 0 && now < last_update_) ClearForReset();
    for (auto &[name, a] : actuators_) Read(a, ecm);
    for (auto &[name, j] : passive_) Read(j, ecm);
    if (reset_pending_) {
      // Positions are read after the world reset has moved the joints.
      HoldCurrentState();
      minimum_command_time_ = now;
      reset_pending_ = false;
      std_msgs::msg::UInt64 notification;
      notification.data = epoch_;
      epoch_pub_->publish(notification);
    }
    const bool fresh = acknowledged_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() < watchdog_timeout_;
    const double dt = last_update_ < 0 ? 0.0 : std::clamp(now - last_update_, 0.0, 0.02);
    last_update_ = now;
    for (auto &[name, a] : actuators_) {
      // Steering PD and wheel control act through physics; joints are never teleported.
      const double effort = a.steering ? steering_kp_ * (a.target - a.position) - steering_kd_ * a.velocity
                                       : WheelEffort(a, fresh, dt);
      a.applied = std::clamp(effort, -a.effort, a.effort);
      Command(a.entity, a.applied, ecm);
    }
    if (passive_.size() == 2) {
      auto &left = passive_.begin()->second, &right = std::next(passive_.begin())->second;
      const double torque = -differential_stiffness_ * (left.position + right.position)
                            - differential_damping_ * (left.velocity + right.velocity);
      for (Joint *j : {&left, &right}) {
        j->applied = torque;
        Command(j->entity, torque, ecm);
      }
    }
  }

  void PostUpdate(const gz::UpdateInfo &info, const gz::EntityComponentManager &ecm) override {
    if (!enabled_ || info.paused) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = std::chrono::duration<double>(info.simTime).count();
    const rclcpp::Time stamp(static_cast<int64_t>(now * 1e9));
    if (last_publish_ < 0 || now < last_publish_ || now - last_publish_ >= publish_period_ - 1e-9) {
      sensor_msgs::msg::JointState state;
      state.header.stamp = stamp;
      for (auto &[name, a] : actuators_) {
        state.name.push_back(name);
        state.position.push_back(a.position);
        state.velocity.push_back(a.velocity);
        state.effort.push_back(a.applied);
      }
      for (auto &[name, j] : passive_) {
        state.name.push_back(name);
        state.position.push_back(j.position);
        state.velocity.push_back(j.velocity);
        state.effort.push_back(0.0);
      }
      pub_->publish(state);
      last_publish_ = now;
    }
    if (truth_pub_ && (last_truth_ < 0 || now < last_truth_ || now - last_truth_ >= truth_period_ - 1e-9)) {
      PublishGroundTruth(stamp, ecm);
      last_truth_ = now;
    }
  }

  // Same message as Classic's libgazebo_ros_p3d: pose and twist of the body in
  // the world frame (twist not rotated into the body frame).
  void PublishGroundTruth(const rclcpp::Time &stamp, const gz::EntityComponentManager &ecm) {
    const auto pose = gz::worldPose(body_, ecm);
    const gz::Link link(body_);
    const auto lin = link.WorldLinearVelocity(ecm).value_or(ignition::math::Vector3d::Zero);
    const auto ang = link.WorldAngularVelocity(ecm).value_or(ignition::math::Vector3d::Zero);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "world";
    odom.child_frame_id = body_name_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    odom.twist.twist.linear.x = lin.X();
    odom.twist.twist.linear.y = lin.Y();
    odom.twist.twist.linear.z = lin.Z();
    odom.twist.twist.angular.x = ang.X();
    odom.twist.twist.angular.y = ang.Y();
    odom.twist.twist.angular.z = ang.Z();
    truth_pub_->publish(odom);
  }
};
}  // namespace rover_sim

IGNITION_ADD_PLUGIN(rover_sim::SwerveGzSystem, ignition::gazebo::System,
                    rover_sim::SwerveGzSystem::ISystemConfigure,
                    rover_sim::SwerveGzSystem::ISystemPreUpdate,
                    rover_sim::SwerveGzSystem::ISystemPostUpdate)
