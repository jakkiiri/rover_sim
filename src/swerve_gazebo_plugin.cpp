// SPDX-License-Identifier: Apache-2.0
// Gazebo Classic model plugin: swerve actuators with a DC-motor torque-speed
// model, passive suspension state publishing, ground truth and diagnostics.
//
// Safety properties kept from the prototype: reset epochs with an explicit
// acknowledgement handshake, a wall-clock watchdog, rejection of malformed,
// stale or future-stamped commands, and a strict joint-set check.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
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
  enum class Kind { Steering, Drive, Suspension };
  struct Actuator {
    gazebo::physics::JointPtr joint;
    Kind kind;
    double effort_limit;     // steering: PD saturation; drive: stall torque
    double velocity_limit;   // drive: no-load speed
    double target = 0;
    double integral = 0;
    double applied = 0;
    double available = 0;
    double slip = 0;
    double ground_speed = 0;
  };
  std::map<std::string, Actuator> actuators_;
  std::vector<std::string> actuated_names_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr base_link_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr epoch_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr ack_sub_;
  gazebo::event::ConnectionPtr update_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point received_{};
  double last_publish_ = -1, last_truth_ = -1, last_diag_ = -1;
  double last_update_ = -1;
  double watchdog_timeout_ = 0.3;
  double publish_period_ = 0.02, truth_period_ = 0.02, diag_period_ = 0.1;
  double steering_kp_ = 60, steering_kd_ = 3, wheel_kp_ = 3, wheel_ki_ = 20;
  double wheel_radius_ = 0.2;
  bool ground_truth_tf_ = false;
  double minimum_command_time_ = 0;
  uint64_t epoch_ = 0;
  bool reset_pending_ = true;
  bool acknowledged_ = false;

 public:
  ~SwerveActuators() override { update_.reset(); sub_.reset(); ack_sub_.reset(); }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    node_ = gazebo_ros::Node::Get(sdf);
    model_ = model;
    world_ = model->GetWorld();
    epoch_ = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
    watchdog_timeout_ = sdf->Get<double>("watchdog_timeout", 0.3).first;
    const double publish_rate = sdf->Get<double>("publish_rate", 50.0).first;
    const double truth_rate = sdf->Get<double>("ground_truth_rate", 50.0).first;
    const double diag_rate = sdf->Get<double>("diagnostics_rate", 10.0).first;
    steering_kp_ = sdf->Get<double>("steering_kp", 60.0).first;
    steering_kd_ = sdf->Get<double>("steering_kd", 3.0).first;
    wheel_kp_ = sdf->Get<double>("wheel_kp", 3.0).first;
    wheel_ki_ = sdf->Get<double>("wheel_ki", 20.0).first;
    wheel_radius_ = sdf->Get<double>("wheel_radius", 0.2).first;
    ground_truth_tf_ = sdf->Get<bool>("ground_truth_tf", false).first;
    for (double v : {watchdog_timeout_, publish_rate, truth_rate, diag_rate, steering_kp_, steering_kd_,
                     wheel_kp_, wheel_ki_, wheel_radius_}) {
      if (!std::isfinite(v) || v <= 0) {
        RCLCPP_ERROR(node_->get_logger(), "All plugin parameters must be finite and positive; bridge disabled");
        return;
      }
    }
    publish_period_ = 1.0 / publish_rate;
    truth_period_ = 1.0 / truth_rate;
    diag_period_ = 1.0 / diag_rate;

    size_t steering = 0, drive = 0, suspension = 0;
    for (auto &joint : model->GetJoints()) {
      const auto name = joint->GetName();
      auto ends_with = [&](const std::string &suffix) {
        return name.size() >= suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
      };
      Kind kind;
      if (ends_with("_kingpin")) { kind = Kind::Steering; ++steering; }
      else if (ends_with("_wheel_joint")) { kind = Kind::Drive; ++drive; }
      else if (ends_with("_suspension")) { kind = Kind::Suspension; ++suspension; }
      else continue;
      actuators_.emplace(name, Actuator{joint, kind, joint->GetEffortLimit(0), joint->GetVelocityLimit(0)});
      if (kind != Kind::Suspension) actuated_names_.push_back(name);
    }
    if (steering != 4 || drive != 4 || (suspension != 0 && suspension != 4)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Expected four kingpins, four drive joints and zero or four suspension joints "
                   "(found %zu/%zu/%zu); actuator bridge disabled", steering, drive, suspension);
      actuators_.clear();
      return;
    }
    for (auto &[name, a] : actuators_) {
      if (a.kind == Kind::Drive && (!std::isfinite(a.effort_limit) || a.effort_limit <= 0 ||
                                    !std::isfinite(a.velocity_limit) || a.velocity_limit <= 0)) {
        RCLCPP_ERROR(node_->get_logger(), "Drive joint %s needs positive effort (stall torque) and velocity "
                     "(no-load speed) limits; bridge disabled", name.c_str());
        actuators_.clear();
        return;
      }
    }
    base_link_ = model->GetLink("base_link");
    if (!base_link_) base_link_ = model->GetLink();   // fixed-joint reduction lumps base_link into the root

    epoch_pub_ = node_->create_publisher<std_msgs::msg::UInt64>("/swerve/reset_epoch", rclcpp::QoS(1).transient_local());
    ack_sub_ = node_->create_subscription<std_msgs::msg::UInt64>(
      "/swerve/reset_ack", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::UInt64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (reset_pending_ || acknowledged_ || msg->data != epoch_) return;
        acknowledged_ = true;
        minimum_command_time_ = world_->SimTime().Double();
        received_ = {};
        HoldCurrentState();
      });
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/swerve/joint_commands", rclcpp::QoS(1),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnCommand(msg); });
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    truth_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/ground_truth/odom", 10);
    diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/swerve/actuator_diagnostics", 10);
    if (ground_truth_tf_) tf_pub_ = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&SwerveActuators::Update, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Swerve bridge ready: %zu joints (%zu actuated), wall watchdog %.3f s, "
                "ground truth %.0f Hz", actuators_.size(), actuated_names_.size(), watchdog_timeout_, truth_rate);
  }

  void OnCommand(const sensor_msgs::msg::JointState::SharedPtr &msg) {
    if (msg->name.size() != actuated_names_.size() || msg->position.size() != msg->name.size() ||
        msg->velocity.size() != msg->name.size()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    // A late callback must not refresh freshness across an expired lease.
    if (ExpireWatchdog()) return;
    const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
    if (!acknowledged_ || reset_pending_ || msg->header.stamp.nanosec >= 1000000000 ||
        stamp <= minimum_command_time_ || stamp > world_->SimTime().Double() + 0.1) return;
    std::map<std::string, double> targets;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = actuators_.find(msg->name[i]);
      if (it == actuators_.end() || it->second.kind == Kind::Suspension || targets.count(msg->name[i])) return;
      double value = it->second.kind == Kind::Steering ? msg->position[i] : msg->velocity[i];
      if (!std::isfinite(value)) return;
      auto j = it->second.joint;
      value = it->second.kind == Kind::Steering ? std::clamp(value, j->LowerLimit(0), j->UpperLimit(0)) :
        std::clamp(value, -it->second.velocity_limit, it->second.velocity_limit);
      targets.emplace(msg->name[i], value);
    }
    for (auto &entry : targets) actuators_.at(entry.first).target = entry.second;
    received_ = std::chrono::steady_clock::now();
  }

  void Reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    ClearForReset();
  }

  void HoldCurrentState() {
    for (auto &[name, a] : actuators_) {
      a.target = a.kind == Kind::Steering ? a.joint->Position(0) : 0.0;
      a.integral = 0.0;
    }
  }

  void ClearForReset() {
    ++epoch_;
    reset_pending_ = true;
    acknowledged_ = false;
    received_ = {};
    last_update_ = -1;
    last_publish_ = last_truth_ = last_diag_ = -1;
    for (auto &[name, a] : actuators_) { a.target = 0.0; a.integral = 0.0; }
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

  // DC-motor derating: torque available when driving in the direction of motion
  // falls linearly from stall torque at rest to zero at the no-load speed.
  // Braking (torque opposing motion) is allowed up to the stall torque.
  static double AvailableTorque(double proposed, double velocity, double stall, double no_load) {
    if (proposed * velocity > 0) return stall * std::max(0.0, 1.0 - std::abs(velocity) / no_load);
    return stall;
  }

  void Update(const gazebo::common::UpdateInfo &info) {
    std::lock_guard<std::mutex> lock(mutex_);
    ExpireWatchdog();
    const double now = info.simTime.Double();
    if (last_update_ >= 0 && now < last_update_) ClearForReset();   // time-only reset
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

    sensor_msgs::msg::JointState state;
    state.header.stamp = rclcpp::Time(static_cast<int64_t>(now * 1e9));
    const auto up = ignition::math::Vector3d::UnitZ;
    for (auto &[name, a] : actuators_) {
      const double position = a.joint->Position(0), velocity = a.joint->GetVelocity(0);
      double effort = 0.0;
      if (a.kind == Kind::Steering) {
        effort = std::clamp(steering_kp_ * (a.target - position) - steering_kd_ * velocity,
                            -a.effort_limit, a.effort_limit);
        a.available = a.effort_limit;
      } else if (a.kind == Kind::Drive) {
        const double target = fresh ? a.target : 0.0;
        const double error = target - velocity;
        if (std::abs(target) < 1e-9) a.integral = 0.0;
        const double proposed_integral = std::clamp(a.integral + wheel_ki_ * error * dt,
                                                    -0.5 * a.effort_limit, 0.5 * a.effort_limit);
        const double candidate = wheel_kp_ * error + proposed_integral;
        a.available = AvailableTorque(candidate, velocity, a.effort_limit, a.velocity_limit);
        // Integrate only when unsaturated or when unwinding a saturated command.
        if (std::abs(target) > 1e-9 && (std::abs(candidate) < a.available || error * candidate < 0))
          a.integral = proposed_integral;
        effort = std::clamp(wheel_kp_ * error + a.integral, -a.available, a.available);
        // Slip: rim speed against the wheel centre's ground speed along the rolling direction.
        auto wheel = a.joint->GetChild();
        const auto axis = a.joint->GlobalAxis(0);
        const auto rolling = axis.Cross(up);
        a.ground_speed = wheel->WorldLinearVel().Dot(rolling);
        const double rim = velocity * wheel_radius_;
        a.slip = (rim - a.ground_speed) / std::max({std::abs(rim), std::abs(a.ground_speed), 0.05});
      }
      if (a.kind != Kind::Suspension) a.joint->SetForce(0, effort);
      a.applied = effort;
      state.name.push_back(name);
      state.position.push_back(position);
      state.velocity.push_back(velocity);
      state.effort.push_back(a.kind == Kind::Suspension ? a.joint->GetForce(0) : effort);
    }
    if (last_publish_ < 0 || now - last_publish_ >= publish_period_ - 1e-9) {
      pub_->publish(state);
      last_publish_ = now;
    }
    if (last_truth_ < 0 || now - last_truth_ >= truth_period_ - 1e-9) {
      PublishGroundTruth(now);
      last_truth_ = now;
    }
    if (last_diag_ < 0 || now - last_diag_ >= diag_period_ - 1e-9) {
      PublishDiagnostics(now, fresh);
      last_diag_ = now;
    }
  }

  void PublishGroundTruth(double now) {
    if (!base_link_) return;
    const auto pose = base_link_->WorldPose();
    const auto linear = pose.Rot().RotateVectorReverse(base_link_->WorldLinearVel());
    const auto angular = pose.Rot().RotateVectorReverse(base_link_->WorldAngularVel());
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Time(static_cast<int64_t>(now * 1e9));
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

  void PublishDiagnostics(double now, bool fresh) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = rclcpp::Time(static_cast<int64_t>(now * 1e9));
    for (auto &[name, a] : actuators_) {
      if (a.kind != Kind::Drive) continue;
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "swerve_actuators/" + name;
      status.hardware_id = "gazebo";
      const bool saturated = std::abs(a.applied) >= a.available - 1e-6 && std::abs(a.applied) > 1e-6;
      status.level = saturated ? diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = !fresh ? "no fresh command" : (saturated ? "torque saturated" : "ok");
      auto kv = [&](const std::string &key, double value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key;
        pair.value = std::to_string(value);
        status.values.push_back(pair);
      };
      kv("target_rad_s", a.target);
      kv("velocity_rad_s", a.joint->GetVelocity(0));
      kv("effort_nm", a.applied);
      kv("available_torque_nm", a.available);
      kv("stall_torque_nm", a.effort_limit);
      kv("no_load_speed_rad_s", a.velocity_limit);
      kv("ground_speed_m_s", a.ground_speed);
      kv("slip_ratio", a.slip);
      array.status.push_back(status);
    }
    diag_pub_->publish(array);
  }
};
GZ_REGISTER_MODEL_PLUGIN(SwerveActuators)
}  // namespace rover_sim
