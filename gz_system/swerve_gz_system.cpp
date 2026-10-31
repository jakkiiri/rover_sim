// SPDX-License-Identifier: Apache-2.0
// Gazebo Fortress (gz-sim 6) System: the same swerve actuator bridge as the
// Classic plugin (src/swerve_gazebo_plugin.cpp) on the Entity-Component API.
//
// Same ROS interfaces and safety properties: /swerve/joint_commands in,
// /joint_states (12 joints) out, reset epochs with acknowledgement, wall
// watchdog, strict joint-set check, DC-motor derating, parking hold,
// /ground_truth/odom and /swerve/actuator_diagnostics.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/ChildLinkName.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rover_sim {
namespace gz = ignition::gazebo;

class SwerveGzSystem : public gz::System, public gz::ISystemConfigure, public gz::ISystemPreUpdate,
                       public gz::ISystemPostUpdate {
  enum class Kind { Steering, Drive, Suspension };
  struct Actuator {
    gz::Entity joint{gz::kNullEntity};
    gz::Entity child{gz::kNullEntity};
    Kind kind;
    double effort_limit = 0, velocity_limit = 0, lower = -1e300, upper = 1e300;
    double target = 0, integral = 0, applied = 0, available = 0, slip = 0, ground_speed = 0;
    double position = 0, velocity = 0;
    bool holding = false;
    double hold_position = 0;
  };
  std::map<std::string, Actuator> actuators_;
  std::vector<std::string> actuated_names_;
  gz::Model model_{gz::kNullEntity};
  gz::Entity base_link_{gz::kNullEntity};
  rclcpp::Node::SharedPtr node_;
  std::thread spinner_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr epoch_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr ack_sub_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point received_{};
  double sim_now_ = 0, last_update_ = -1, last_publish_ = -1, last_truth_ = -1, last_diag_ = -1;
  double watchdog_timeout_ = 0.3, publish_period_ = 0.02, truth_period_ = 0.02, diag_period_ = 0.1;
  double steering_kp_ = 60, steering_kd_ = 3, wheel_kp_ = 3, wheel_ki_ = 20, hold_kp_ = 40, hold_kd_ = 6;
  double wheel_radius_ = 0.2;
  bool ground_truth_tf_ = false, enabled_ = false;
  double minimum_command_time_ = 0;
  uint64_t epoch_ = 0;
  bool reset_pending_ = true, acknowledged_ = false;

 public:
  ~SwerveGzSystem() override {
    if (node_) rclcpp::shutdown();
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
    publish_period_ = 1.0 / get("publish_rate", 50.0);
    truth_period_ = 1.0 / get("ground_truth_rate", 50.0);
    diag_period_ = 1.0 / get("diagnostics_rate", 10.0);
    steering_kp_ = get("steering_kp", 60.0);
    steering_kd_ = get("steering_kd", 3.0);
    wheel_kp_ = get("wheel_kp", 3.0);
    wheel_ki_ = get("wheel_ki", 20.0);
    hold_kp_ = get("hold_kp", 40.0);
    hold_kd_ = get("hold_kd", 6.0);
    wheel_radius_ = get("wheel_radius", 0.2);
    ground_truth_tf_ = sdf->HasElement("ground_truth_tf") && sdf->Get<bool>("ground_truth_tf");
    for (double v : {watchdog_timeout_, publish_period_, truth_period_, diag_period_, steering_kp_, steering_kd_,
                     wheel_kp_, wheel_ki_, hold_kp_, hold_kd_, wheel_radius_}) {
      if (!std::isfinite(v) || v <= 0) {
        RCLCPP_ERROR(node_->get_logger(), "All plugin parameters must be finite and positive; bridge disabled");
        return;
      }
    }
    size_t steering = 0, drive = 0, suspension = 0;
    for (auto joint : model_.Joints(ecm)) {
      const auto name = ecm.Component<gz::components::Name>(joint)->Data();
      auto ends_with = [&](const std::string &suffix) {
        return name.size() >= suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
      };
      Kind kind;
      if (ends_with("_kingpin")) { kind = Kind::Steering; ++steering; }
      else if (ends_with("_wheel_joint")) { kind = Kind::Drive; ++drive; }
      else if (ends_with("_suspension")) { kind = Kind::Suspension; ++suspension; }
      else continue;
      Actuator a;
      a.joint = joint;
      a.kind = kind;
      if (auto *axis = ecm.Component<gz::components::JointAxis>(joint)) {
        a.effort_limit = axis->Data().Effort();
        a.velocity_limit = axis->Data().MaxVelocity();
        a.lower = axis->Data().Lower();
        a.upper = axis->Data().Upper();
      }
      if (auto *child = ecm.Component<gz::components::ChildLinkName>(joint))
        a.child = model_.LinkByName(ecm, child->Data());
      ecm.CreateComponent(joint, gz::components::JointPosition());
      ecm.CreateComponent(joint, gz::components::JointVelocity());
      actuators_.emplace(name, a);
      if (kind != Kind::Suspension) actuated_names_.push_back(name);
    }
    if (steering != 4 || drive != 4 || (suspension != 0 && suspension != 4)) {
      RCLCPP_ERROR(node_->get_logger(), "Expected four kingpins, four drive joints and zero or four suspension "
                   "joints (found %zu/%zu/%zu); actuator bridge disabled", steering, drive, suspension);
      actuators_.clear();
      return;
    }
    for (auto &[name, a] : actuators_) {
      if (a.kind == Kind::Drive && (!std::isfinite(a.effort_limit) || a.effort_limit <= 0 ||
                                    !std::isfinite(a.velocity_limit) || a.velocity_limit <= 0)) {
        RCLCPP_ERROR(node_->get_logger(), "Drive joint %s needs positive effort and velocity limits; bridge disabled",
                     name.c_str());
        actuators_.clear();
        return;
      }
    }
    base_link_ = model_.LinkByName(ecm, "base_link");
    if (base_link_ == gz::kNullEntity) base_link_ = model_.CanonicalLink(ecm);
    gz::Link(base_link_).EnableVelocityChecks(ecm, true);
    for (auto &[name, a] : actuators_) if (a.child != gz::kNullEntity) gz::Link(a.child).EnableVelocityChecks(ecm, true);

    epoch_pub_ = node_->create_publisher<std_msgs::msg::UInt64>("/swerve/reset_epoch", rclcpp::QoS(1).transient_local());
    ack_sub_ = node_->create_subscription<std_msgs::msg::UInt64>(
      "/swerve/reset_ack", rclcpp::QoS(1).transient_local(), [this](std_msgs::msg::UInt64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (reset_pending_ || acknowledged_ || msg->data != epoch_) return;
        acknowledged_ = true;
        minimum_command_time_ = sim_now_;
        received_ = {};
        HoldCurrentState();
      });
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/swerve/joint_commands", rclcpp::QoS(1), [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnCommand(msg); });
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    truth_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/ground_truth/odom", 10);
    diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/swerve/actuator_diagnostics", 10);
    if (ground_truth_tf_) tf_pub_ = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
    spinner_ = std::thread([this] { rclcpp::spin(node_); });
    enabled_ = true;
    RCLCPP_INFO(node_->get_logger(), "Swerve gz-sim bridge ready: %zu joints (%zu actuated), wall watchdog %.3f s",
                actuators_.size(), actuated_names_.size(), watchdog_timeout_);
  }

  void OnCommand(const sensor_msgs::msg::JointState::SharedPtr &msg) {
    if (msg->name.size() != actuated_names_.size() || msg->position.size() != msg->name.size() ||
        msg->velocity.size() != msg->name.size()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (ExpireWatchdog()) return;
    const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
    if (!acknowledged_ || reset_pending_ || msg->header.stamp.nanosec >= 1000000000 ||
        stamp <= minimum_command_time_ || stamp > sim_now_ + 0.1) return;
    std::map<std::string, double> targets;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = actuators_.find(msg->name[i]);
      if (it == actuators_.end() || it->second.kind == Kind::Suspension || targets.count(msg->name[i])) return;
      double value = it->second.kind == Kind::Steering ? msg->position[i] : msg->velocity[i];
      if (!std::isfinite(value)) return;
      value = it->second.kind == Kind::Steering ? std::clamp(value, it->second.lower, it->second.upper)
                                                : std::clamp(value, -it->second.velocity_limit, it->second.velocity_limit);
      targets.emplace(msg->name[i], value);
    }
    for (auto &entry : targets) actuators_.at(entry.first).target = entry.second;
    received_ = std::chrono::steady_clock::now();
  }

  // gz-sim 6 has no ISystemReset; a world reset shows up as simulation time
  // moving backwards in PreUpdate, which clears the epoch like the Classic plugin.

  void HoldCurrentState() {
    for (auto &[name, a] : actuators_) {
      a.target = a.kind == Kind::Steering ? a.position : 0.0;
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
    last_publish_ = last_truth_ = last_diag_ = -1;
    for (auto &[name, a] : actuators_) { a.target = 0.0; a.integral = 0.0; a.holding = false; }
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

  static double AvailableTorque(double proposed, double velocity, double stall, double no_load) {
    if (proposed * velocity > 0) return stall * std::max(0.0, 1.0 - std::abs(velocity) / no_load);
    return stall;
  }

  void PreUpdate(const gz::UpdateInfo &info, gz::EntityComponentManager &ecm) override {
    if (!enabled_ || info.paused) return;
    std::lock_guard<std::mutex> lock(mutex_);
    ExpireWatchdog();
    const double now = std::chrono::duration<double>(info.simTime).count();
    sim_now_ = now;
    if (last_update_ >= 0 && now < last_update_) ClearForReset();
    // Read joint state (populated by the physics system after the first step).
    for (auto &[name, a] : actuators_) {
      auto *p = ecm.Component<gz::components::JointPosition>(a.joint);
      auto *v = ecm.Component<gz::components::JointVelocity>(a.joint);
      a.position = p && !p->Data().empty() ? p->Data()[0] : 0.0;
      a.velocity = v && !v->Data().empty() ? v->Data()[0] : 0.0;
    }
    if (reset_pending_) {
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
      double effort = 0.0;
      if (a.kind == Kind::Steering) {
        effort = std::clamp(steering_kp_ * (a.target - a.position) - steering_kd_ * a.velocity,
                            -a.effort_limit, a.effort_limit);
        a.available = a.effort_limit;
      } else if (a.kind == Kind::Drive) {
        const double target = fresh ? a.target : 0.0;
        const double error = target - a.velocity;
        if (std::abs(target) < 1e-9) {
          // Parking hold: DART takes a stiff PD hold well (no ODE-style
          // rocking), limited to the stall torque so steep slopes creep. It
          // engages at any speed: while the wheel still turns the position
          // term saturates at the stall torque and acts as the brake (a rover
          // placed on a 30 deg ramp otherwise rolled away before a
          // speed-gated hold could engage).
          a.integral = 0.0;
          if (!a.holding) { a.holding = true; a.hold_position = a.position; }
          a.available = a.effort_limit;
          effort = a.holding ? hold_kp_ * (a.hold_position - a.position) - hold_kd_ * a.velocity
                             : wheel_kp_ * error;
          effort = std::clamp(effort, -a.available, a.available);
        } else {
          a.holding = false;
          const double proposed_integral = std::clamp(a.integral + wheel_ki_ * error * dt,
                                                      -0.5 * a.effort_limit, 0.5 * a.effort_limit);
          const double candidate = wheel_kp_ * error + proposed_integral;
          a.available = AvailableTorque(candidate, a.velocity, a.effort_limit, a.velocity_limit);
          if (std::abs(candidate) < a.available || error * candidate < 0) a.integral = proposed_integral;
          effort = std::clamp(wheel_kp_ * error + a.integral, -a.available, a.available);
        }
      }
      if (a.kind != Kind::Suspension) {
        auto *cmd = ecm.Component<gz::components::JointForceCmd>(a.joint);
        if (!cmd) ecm.CreateComponent(a.joint, gz::components::JointForceCmd({effort}));
        else cmd->Data() = {effort};
      }
      a.applied = effort;
    }
  }

  void PostUpdate(const gz::UpdateInfo &info, const gz::EntityComponentManager &ecm) override {
    if (!enabled_ || info.paused) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = std::chrono::duration<double>(info.simTime).count();
    const rclcpp::Time stamp(static_cast<int64_t>(now * 1e9));
    // Slip from the wheel link's world velocity along its rolling direction.
    const auto up = ignition::math::Vector3d::UnitZ;
    for (auto &[name, a] : actuators_) {
      if (a.kind != Kind::Drive || a.child == gz::kNullEntity) continue;
      auto *jp = ecm.Component<gz::components::JointAxis>(a.joint);
      auto pose = gz::worldPose(a.child, ecm);
      auto vel = gz::Link(a.child).WorldLinearVelocity(ecm);
      if (!jp || !vel) continue;
      const auto axis = pose.Rot().RotateVector(jp->Data().Xyz());
      const auto rolling = axis.Cross(up);
      a.ground_speed = vel->Dot(rolling);
      const double rim = a.velocity * wheel_radius_;
      a.slip = (rim - a.ground_speed) / std::max({std::abs(rim), std::abs(a.ground_speed), 0.05});
    }
    if (last_publish_ < 0 || now - last_publish_ >= publish_period_ - 1e-9) {
      sensor_msgs::msg::JointState state;
      state.header.stamp = stamp;
      for (auto &[name, a] : actuators_) {
        state.name.push_back(name);
        state.position.push_back(a.position);
        state.velocity.push_back(a.velocity);
        state.effort.push_back(a.applied);
      }
      pub_->publish(state);
      last_publish_ = now;
    }
    if (last_truth_ < 0 || now - last_truth_ >= truth_period_ - 1e-9) {
      PublishGroundTruth(stamp, ecm);
      last_truth_ = now;
    }
    if (last_diag_ < 0 || now - last_diag_ >= diag_period_ - 1e-9) {
      PublishDiagnostics(stamp);
      last_diag_ = now;
    }
  }

  void PublishGroundTruth(const rclcpp::Time &stamp, const gz::EntityComponentManager &ecm) {
    const auto pose = gz::worldPose(base_link_, ecm);
    auto lin = gz::Link(base_link_).WorldLinearVelocity(ecm);
    auto ang = gz::Link(base_link_).WorldAngularVelocity(ecm);
    const auto linear = pose.Rot().RotateVectorReverse(lin.value_or(ignition::math::Vector3d::Zero));
    const auto angular = pose.Rot().RotateVectorReverse(ang.value_or(ignition::math::Vector3d::Zero));
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    odom.twist.twist.linear.x = linear.X();
    odom.twist.twist.linear.y = linear.Y();
    odom.twist.twist.linear.z = linear.Z();
    odom.twist.twist.angular.x = angular.X();
    odom.twist.twist.angular.y = angular.Y();
    odom.twist.twist.angular.z = angular.Z();
    truth_pub_->publish(odom);
    if (tf_pub_) {
      tf2_msgs::msg::TFMessage tf;
      geometry_msgs::msg::TransformStamped t;
      t.header = odom.header;
      t.child_frame_id = "base_link";
      t.transform.translation.x = pose.Pos().X();
      t.transform.translation.y = pose.Pos().Y();
      t.transform.translation.z = pose.Pos().Z();
      t.transform.rotation = odom.pose.pose.orientation;
      tf.transforms.push_back(t);
      tf_pub_->publish(tf);
    }
  }

  void PublishDiagnostics(const rclcpp::Time &stamp) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    for (auto &[name, a] : actuators_) {
      if (a.kind != Kind::Drive) continue;
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "swerve_actuators/" + name;
      status.hardware_id = "gazebo_fortress";
      const bool saturated = std::abs(a.applied) >= a.available - 1e-6 && std::abs(a.applied) > 1e-6;
      status.level = saturated ? diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = saturated ? "torque saturated" : "ok";
      auto kv = [&](const std::string &key, double value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key;
        pair.value = std::to_string(value);
        status.values.push_back(pair);
      };
      kv("target_rad_s", a.target);
      kv("velocity_rad_s", a.velocity);
      kv("effort_nm", a.applied);
      kv("available_torque_nm", a.available);
      kv("stall_torque_nm", a.effort_limit);
      kv("no_load_speed_rad_s", a.velocity_limit);
      kv("ground_speed_m_s", a.ground_speed);
      kv("slip_ratio", a.slip);
      kv("parking_hold", a.holding ? 1.0 : 0.0);
      array.status.push_back(status);
    }
    diag_pub_->publish(array);
  }
};
}  // namespace rover_sim

IGNITION_ADD_PLUGIN(rover_sim::SwerveGzSystem, ignition::gazebo::System, rover_sim::SwerveGzSystem::ISystemConfigure,
                    rover_sim::SwerveGzSystem::ISystemPreUpdate, rover_sim::SwerveGzSystem::ISystemPostUpdate)
