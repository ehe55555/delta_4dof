#include "gz_delta_controller2/DeltaMotorController2.hh"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/plugin/Register.hh>

#include <gz/sim/Util.hh>

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/Link.hh>

using namespace gz;
using namespace sim;

namespace gz_delta_controller2
{

void DeltaMotorController2::Configure(
  const Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &)
{
  this->model_ = Model(_entity);

  if (!this->model_.Valid(_ecm))
  {
    gzerr << "DeltaMotorController2 must be attached to a model.\n";
    return;
  }

  if (_sdf->HasElement("kp"))
    this->kp_ = _sdf->Get<double>("kp");

  if (_sdf->HasElement("ki"))
    this->ki_ = _sdf->Get<double>("ki");

  if (_sdf->HasElement("kd"))
    this->kd_ = _sdf->Get<double>("kd");

  if (_sdf->HasElement("kv"))
    this->kv_ = _sdf->Get<double>("kv");

  if (_sdf->HasElement("ka"))
    this->ka_ = _sdf->Get<double>("ka");

  if (_sdf->HasElement("torque_limit"))
    this->torque_limit_ = _sdf->Get<double>("torque_limit");

  if (_sdf->HasElement("integral_limit"))
    this->integral_limit_ = _sdf->Get<double>("integral_limit");

  if (_sdf->HasElement("anti_windup_gain"))
    this->anti_windup_gain_ = std::max(
      0.0, _sdf->Get<double>("anti_windup_gain"));

  this->kp4_ = _sdf->HasElement("kp4") ? _sdf->Get<double>("kp4") : this->kp_;
  this->ki4_ = _sdf->HasElement("ki4") ? _sdf->Get<double>("ki4") : this->ki_;
  this->kd4_ = _sdf->HasElement("kd4") ? _sdf->Get<double>("kd4") : this->kd_;
  this->kv4_ = _sdf->HasElement("kv4") ? _sdf->Get<double>("kv4") : this->kv_;
  this->ka4_ = _sdf->HasElement("ka4") ? _sdf->Get<double>("ka4") : this->ka_;
  this->torque_limit4_ = _sdf->HasElement("torque_limit4") ?
    _sdf->Get<double>("torque_limit4") : this->torque_limit_;
  this->integral_limit4_ = _sdf->HasElement("integral_limit4") ?
    _sdf->Get<double>("integral_limit4") : this->integral_limit_;

  if (_sdf->HasElement("feedback_decimation"))
  {
    this->feedback_decimation_ = std::max(
      1, _sdf->Get<int>("feedback_decimation"));
  }

  if (_sdf->HasElement("twist_kp"))
    this->twist_kp_ = _sdf->Get<double>("twist_kp");

  if (_sdf->HasElement("twist_kd"))
    this->twist_kd_ = _sdf->Get<double>("twist_kd");

  if (_sdf->HasElement("twist_torque_limit"))
    this->twist_torque_limit_ = _sdf->Get<double>("twist_torque_limit");

  for (std::size_t i = 0; i < this->joint_names_.size(); ++i)
  {
    this->joint_entities_[i] =
      this->model_.JointByName(_ecm, this->joint_names_[i]);

    if (this->joint_entities_[i] == kNullEntity)
    {
      gzerr << "Cannot find joint [" << this->joint_names_[i] << "].\n";
      return;
    }

    if (_ecm.Component<components::JointPosition>(
          this->joint_entities_[i]) == nullptr)
    {
      _ecm.CreateComponent(
        this->joint_entities_[i],
        components::JointPosition({0.0}));
    }

    if (_ecm.Component<components::JointVelocity>(
          this->joint_entities_[i]) == nullptr)
    {
      _ecm.CreateComponent(
        this->joint_entities_[i],
        components::JointVelocity({0.0}));
    }

    if (_ecm.Component<components::JointForceCmd>(
          this->joint_entities_[i]) == nullptr)
    {
      _ecm.CreateComponent(
        this->joint_entities_[i],
        components::JointForceCmd({0.0}));
    }
  }

  this->base_entity_ = this->model_.LinkByName(_ecm, "base_link");

  if (this->base_entity_ == kNullEntity)
  {
    gzerr << "Cannot find link [base_link].\n";
    return;
  }
  else
  {
    // Cần vận tốc góc của base_link để tính vận tốc tương đối.
    Link(this->base_entity_).EnableVelocityChecks(_ecm);
  }

  this->endlink_entity_ = this->model_.LinkByName(_ecm, "end");

  if (this->endlink_entity_ == kNullEntity)
  {
    gzerr << "Cannot find link [end]. XYZ feedback will not work.\n";
  }
  else
  {
    Link(this->endlink_entity_).EnableVelocityChecks(_ecm);
  }
  this->motor4_output_link_entity_ =
    this->model_.LinkByName(_ecm, "411111");

  if (this->motor4_output_link_entity_ == kNullEntity)
  {
    gzerr << "Cannot find motor 4 output link [411111].\n";
    return;
  }
  else
  {
    Link(this->motor4_output_link_entity_)
      .EnableVelocityChecks(_ecm);
  }
  this->motor4_input_joint_entity_ =
    this->model_.JointByName(_ecm, "joint_4");

  for (std::size_t i = 0; i < this->twist_child_entities_.size(); ++i)
  {
    this->twist_parent_entities_[i] =
      this->model_.LinkByName(_ecm, this->twist_parent_names_[i]);
    this->twist_child_entities_[i] =
      this->model_.LinkByName(_ecm, this->twist_child_names_[i]);

    if (this->twist_parent_entities_[i] == kNullEntity ||
        this->twist_child_entities_[i] == kNullEntity)
    {
      gzerr << "Cannot configure twist stabilizer ["
            << this->twist_parent_names_[i] << " -> "
            << this->twist_child_names_[i] << "].\n";
      continue;
    }

    Link(this->twist_parent_entities_[i]).EnableVelocityChecks(_ecm);
    Link(this->twist_child_entities_[i]).EnableVelocityChecks(_ecm);

    const std::string child_name = this->twist_child_names_[i];
    this->twist_angle_debug_pubs_[i] =
      this->gz_node_.Advertise<gz::msgs::Double>(
        "/delta_robot/debug/twist_angle_" + child_name + "_gz");
    this->twist_rate_debug_pubs_[i] =
      this->gz_node_.Advertise<gz::msgs::Double>(
        "/delta_robot/debug/twist_rate_" + child_name + "_gz");
  }

  // Old direct q-only targets
  this->gz_node_.Subscribe(
    "/delta_robot/motor1_target_gz",
    &DeltaMotorController2::OnTarget1,
    this);

  this->gz_node_.Subscribe(
    "/delta_robot/motor2_target_gz",
    &DeltaMotorController2::OnTarget2,
    this);

  this->gz_node_.Subscribe(
    "/delta_robot/motor3_target_gz",
    &DeltaMotorController2::OnTarget3,
    this);

  this->gz_node_.Subscribe(
    "/delta_robot/motor4_target_gz",
    &DeltaMotorController2::OnTarget4,
    this);

  // New q + q_dot + q_ddot reference
  const bool joint_ref_sub_ok = this->gz_node_.Subscribe(
    "/delta_robot/joint_ref_gz",
    &DeltaMotorController2::OnJointReference,
    this);

  gzmsg << "Controller2 subscribe /delta_robot/joint_ref_gz ok="
        << (joint_ref_sub_ok ? "true" : "false")
        << "\n";

  // Full joint trajectory reference
  const bool traj_sub_ok = this->gz_node_.Subscribe(
    "/delta_robot/joint_trajectory_gz",
    &DeltaMotorController2::OnJointTrajectory,
    this);

  gzmsg << "Controller2 subscribe /delta_robot/joint_trajectory_gz ok="
        << (traj_sub_ok ? "true" : "false")
        << "\n";

  this->theta_feedback_pubs_[0] =
    this->gz_node_.Advertise<gz::msgs::Double>(
      "/delta_robot/feedback/theta1_gz");

  this->theta_feedback_pubs_[1] =
    this->gz_node_.Advertise<gz::msgs::Double>(
      "/delta_robot/feedback/theta2_gz");

  this->theta_feedback_pubs_[2] =
    this->gz_node_.Advertise<gz::msgs::Double>(
      "/delta_robot/feedback/theta3_gz");

  this->theta_feedback_pubs_[3] =
    this->gz_node_.Advertise<gz::msgs::Double>(
      "/delta_robot/feedback/theta4_gz");

  this->xyz_feedback_pubs_[0] =
    this->gz_node_.Advertise<gz::msgs::Double>(
      "/delta_robot/feedback/x_gz");

  this->xyz_feedback_pubs_[1] =
    this->gz_node_.Advertise<gz::msgs::Double>(
      "/delta_robot/feedback/y_gz");

  this->xyz_feedback_pubs_[2] =
    this->gz_node_.Advertise<gz::msgs::Double>(
      "/delta_robot/feedback/z_gz");

  this->state_feedback_pub_ =
    this->gz_node_.Advertise<gz::msgs::Double_V>(
      "/delta_robot/state_gz");

  for (std::size_t i = 0; i < 4; ++i)
  {
    const std::string index = std::to_string(i + 1);

    this->error_debug_pubs_[i] =
      this->gz_node_.Advertise<gz::msgs::Double>(
        "/delta_robot/debug/error" + index + "_gz");

    this->omega_debug_pubs_[i] =
      this->gz_node_.Advertise<gz::msgs::Double>(
        "/delta_robot/debug/omega" + index + "_gz");

    this->torque_raw_debug_pubs_[i] =
      this->gz_node_.Advertise<gz::msgs::Double>(
        "/delta_robot/debug/torque_raw" + index + "_gz");

    this->torque_cmd_debug_pubs_[i] =
      this->gz_node_.Advertise<gz::msgs::Double>(
        "/delta_robot/debug/torque_cmd" + index + "_gz");

    this->saturated_debug_pubs_[i] =
      this->gz_node_.Advertise<gz::msgs::Double>(
        "/delta_robot/debug/saturated" + index + "_gz");
  }

  this->configured_ = true;

  gzmsg << "DeltaMotorController2 loaded for model ["
        << this->model_.Name(_ecm) << "].\n";

  gzmsg << "PID2: kp=" << this->kp_
        << ", ki=" << this->ki_
        << ", kd=" << this->kd_
        << ", kv=" << this->kv_
        << ", ka=" << this->ka_
        << ", torque_limit=" << this->torque_limit_
        << ", anti_windup_gain=" << this->anti_windup_gain_
        << ", motor4=(" << this->kp4_ << ", " << this->ki4_
        << ", " << this->kd4_ << ", " << this->kv4_
        << ", " << this->ka4_ << "), torque_limit4="
        << this->torque_limit4_
        << ", twist_kp=" << this->twist_kp_
        << ", twist_kd=" << this->twist_kd_
        << ", twist_torque_limit=" << this->twist_torque_limit_
        << ", feedback_decimation=" << this->feedback_decimation_
        << "\n";
}

void DeltaMotorController2::PreUpdate(
  const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  if (!this->configured_)
    return;

  if (_info.paused)
    return;

  const double dt = std::chrono::duration<double>(_info.dt).count();

  if (dt <= 0.0)
    return;

  const double sim_time =
    std::chrono::duration<double>(_info.simTime).count();

  if (!this->targets_initialized_)
  {
    // Let the physics backend establish the closed-loop joint coordinates
    // before capturing the relative HOME position of the fourth output.
    if (sim_time < 0.1)
      return;

    std::lock_guard<std::mutex> lock(this->mutex_);

    bool all_ready = true;

    for (std::size_t i = 0; i < this->joint_entities_.size(); ++i)
    {
      auto pos_comp =
        _ecm.Component<components::JointPosition>(
          this->joint_entities_[i]);

      if (!pos_comp || pos_comp->Data().empty())
      {
        all_ready = false;
        break;
      }
    }

    if (!all_ready)
      return;

    double motor4_angle = 0.0;
    double motor4_angular_velocity = 0.0;

    if (!this->MeasureMotor4Orientation(
          _ecm,
          motor4_angle,
          motor4_angular_velocity))
    {
      return;
    }

    for (std::size_t i = 0;
        i < this->joint_entities_.size();
        ++i)
    {
      auto pos_comp =
        _ecm.Component<components::JointPosition>(
          this->joint_entities_[i]);

      if (i == 3)
      {
        // Khi controller vừa chạy, giữ nguyên hướng tuyệt đối hiện tại
        // của 411111 so với base_link.
        this->targets_[i] = motor4_angle;
      }
      else
      {
        this->targets_[i] = pos_comp->Data()[0];
      }

      this->velocity_targets_[i] = 0.0;
      this->acceleration_targets_[i] = 0.0;
      this->integrals_[i] = 0.0;
    }

    this->targets_initialized_ = true;

    gzmsg << "Initial motor targets set to current joint positions.\n";
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (this->trajectory_received_)
    {
      this->trajectory_sim_start_ = sim_time;
      this->current_trajectory_time_ = 0.0;
      this->trajectory_active_ = true;
      // Không mang tích phân của trajectory cũ sang lệnh jog mới.
      this->integrals_ = {0.0, 0.0, 0.0, 0.0};
      this->trajectory_received_ = false;
      this->last_segment_index_ = 0;

      gzmsg << "Trajectory started at sim_time="
            << this->trajectory_sim_start_ << "\n";
    }

    if (this->trajectory_active_)
    {
      const double traj_t = sim_time - this->trajectory_sim_start_;
      this->current_trajectory_time_ = std::clamp(
        traj_t, 0.0, this->trajectory_duration_);

      std::array<double, 4> q_ref;
      std::array<double, 4> qd_ref;
      std::array<double, 4> qdd_ref;

      if (this->SampleTrajectory(traj_t, q_ref, qd_ref, qdd_ref))
      {
        this->targets_ = q_ref;
        this->velocity_targets_ = qd_ref;
        this->acceleration_targets_ = qdd_ref;
      }

      if (traj_t >= this->trajectory_duration_)
      {
        this->current_trajectory_time_ = this->trajectory_duration_;
        this->trajectory_active_ = false;

        if (!this->trajectory_.empty())
        {
          this->targets_ = this->trajectory_.back().q;
        }

        this->velocity_targets_ = {0.0, 0.0, 0.0, 0.0};
        this->acceleration_targets_ = {0.0, 0.0, 0.0, 0.0};

        gzmsg << "Trajectory sampling finished. Holding final target.\n";

        gzmsg << "Saturation count: "
              << this->saturation_count_[0] << ", "
              << this->saturation_count_[1] << ", "
              << this->saturation_count_[2] << ", "
              << this->saturation_count_[3] << "\n";
      }
    }
  }

  std::array<double, 4> targets_copy;
  std::array<double, 4> velocity_targets_copy;
  std::array<double, 4> acceleration_targets_copy;
  int trajectory_id_copy{-1};
  double trajectory_time_copy{0.0};
  bool trajectory_active_copy{false};

  {
    std::lock_guard<std::mutex> lock(this->mutex_);

    targets_copy = this->targets_;
    velocity_targets_copy = this->velocity_targets_;
    acceleration_targets_copy = this->acceleration_targets_;
    trajectory_id_copy = this->last_trajectory_id_;
    trajectory_time_copy = this->current_trajectory_time_;
    trajectory_active_copy = this->trajectory_active_;
  }

  for (std::size_t i = 0; i < this->joint_entities_.size(); ++i)
  {
    auto pos_comp =
      _ecm.Component<components::JointPosition>(
        this->joint_entities_[i]);

    auto vel_comp =
      _ecm.Component<components::JointVelocity>(
        this->joint_entities_[i]);

    auto force_comp =
      _ecm.Component<components::JointForceCmd>(
        this->joint_entities_[i]);

    if (!pos_comp || !vel_comp || !force_comp)
      continue;

    if (pos_comp->Data().empty() || vel_comp->Data().empty())
      continue;

    double theta_current = pos_comp->Data()[0];
    double omega_current = vel_comp->Data()[0];

    if (i == 3)
    {
      // Motor 4 không dùng trực tiếp tọa độ joint làm hướng.
      // Đo hướng của chính link 411111 trong hệ base_link.
      if (!this->MeasureMotor4Orientation(
            _ecm,
            theta_current,
            omega_current))
      {
        continue;
      }
    }

    const double theta_target = targets_copy[i];
    const double omega_target = velocity_targets_copy[i];
    const double acceleration_target = acceleration_targets_copy[i];

    const double raw_position_error = theta_target - theta_current;
    const double position_error = i == 3 ?
      std::atan2(
        std::sin(raw_position_error),
        std::cos(raw_position_error)) :
      raw_position_error;
    const double velocity_error = omega_target - omega_current;

    const double kp = i == 3 ? this->kp4_ : this->kp_;
    const double ki = i == 3 ? this->ki4_ : this->ki_;
    const double kd = i == 3 ? this->kd4_ : this->kd_;
    const double kv = i == 3 ? this->kv4_ : this->kv_;
    const double ka = i == 3 ? this->ka4_ : this->ka_;
    const double torque_limit =
      i == 3 ? this->torque_limit4_ : this->torque_limit_;
    const double integral_limit =
      i == 3 ? this->integral_limit4_ : this->integral_limit_;

    const double feedforward_torque =
      kv * omega_target +
      ka * acceleration_target;
    const double feedback_torque_without_integral =
      kp * position_error +
      kd * velocity_error;
    const double torque_before_integral_update =
      feedback_torque_without_integral +
      ki * this->integrals_[i] +
      feedforward_torque;
    const double saturated_before_integral_update = this->Clamp(
      torque_before_integral_update,
      -torque_limit,
      torque_limit);

    double integral_rate = position_error;
    if (ki > 1e-12 && this->anti_windup_gain_ > 0.0)
    {
      integral_rate += this->anti_windup_gain_ *
        (saturated_before_integral_update - torque_before_integral_update) /
        ki;
    }

    this->integrals_[i] = this->Clamp(
      this->integrals_[i] + integral_rate * dt,
      -integral_limit,
      integral_limit);

    const double torque_raw =
      feedback_torque_without_integral +
      ki * this->integrals_[i] +
      feedforward_torque;

    const double torque_cmd = this->Clamp(
      torque_raw,
      -torque_limit,
      torque_limit);

    force_comp->Data()[0] = torque_cmd;

    // Count torque saturation on every PreUpdate step.
    const bool saturated =
      std::abs(torque_cmd) >= 0.98 * torque_limit;
    this->last_torque_commands_[i] = torque_cmd;
    this->last_saturated_[i] = saturated ? 1.0 : 0.0;

    if (saturated)
    {
      this->saturation_count_[i]++;
    }

    if (this->feedback_counter_ % this->feedback_decimation_ == 0)
    {
      gz::msgs::Double error_msg;
      gz::msgs::Double omega_msg;
      gz::msgs::Double torque_raw_msg;
      gz::msgs::Double torque_cmd_msg;
      gz::msgs::Double saturated_msg;

      error_msg.set_data(position_error);
      omega_msg.set_data(omega_current);
      torque_raw_msg.set_data(torque_raw);
      torque_cmd_msg.set_data(torque_cmd);
      saturated_msg.set_data(saturated ? 1.0 : 0.0);

      this->error_debug_pubs_[i].Publish(error_msg);
      this->omega_debug_pubs_[i].Publish(omega_msg);
      this->torque_raw_debug_pubs_[i].Publish(torque_raw_msg);
      this->torque_cmd_debug_pubs_[i].Publish(torque_cmd_msg);
      this->saturated_debug_pubs_[i].Publish(saturated_msg);
    }
  }

  this->ApplyTwistStabilizers(_ecm);

  this->feedback_counter_++;

  if (this->feedback_counter_ % this->feedback_decimation_ == 0)
  {
    std::array<double, 4> theta_actual{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> omega_actual{0.0, 0.0, 0.0, 0.0};
    bool joints_ready = true;

    for (std::size_t i = 0; i < this->joint_entities_.size(); ++i)
    {
      auto pos_comp =
        _ecm.Component<components::JointPosition>(
          this->joint_entities_[i]);
      auto vel_comp =
        _ecm.Component<components::JointVelocity>(
          this->joint_entities_[i]);

      if (!pos_comp || !vel_comp ||
          pos_comp->Data().empty() || vel_comp->Data().empty())
      {
        joints_ready = false;
        continue;
      }

      gz::msgs::Double theta_msg;

      double reported_position = pos_comp->Data()[0];
      double reported_velocity = vel_comp->Data()[0];

      if (i == 3)
      {
        if (!this->MeasureMotor4Orientation(
              _ecm,
              reported_position,
              reported_velocity))
        {
          joints_ready = false;
          continue;
        }
      }

      theta_msg.set_data(reported_position);

      this->theta_feedback_pubs_[i].Publish(theta_msg);

      theta_actual[i] = reported_position;
      omega_actual[i] = reported_velocity;
    }

    if (this->endlink_entity_ != kNullEntity)
    {
      const auto tcp_world_pose = worldPose(this->endlink_entity_, _ecm);

      math::Pose3d tcp_base_pose = tcp_world_pose;

      if (this->base_entity_ != kNullEntity)
      {
        const auto base_world_pose = worldPose(this->base_entity_, _ecm);
        tcp_base_pose = base_world_pose.Inverse() * tcp_world_pose;
      }

      const auto p = tcp_base_pose.Pos();

      gz::msgs::Double x_msg;
      gz::msgs::Double y_msg;
      gz::msgs::Double z_msg;

      x_msg.set_data(p.X());
      y_msg.set_data(p.Y());
      z_msg.set_data(p.Z());

      this->xyz_feedback_pubs_[0].Publish(x_msg);
      this->xyz_feedback_pubs_[1].Publish(y_msg);
      this->xyz_feedback_pubs_[2].Publish(z_msg);

      if (joints_ready)
      {
        gz::msgs::Double_V state_msg;
        state_msg.add_data(sim_time);
        state_msg.add_data(static_cast<double>(trajectory_id_copy));
        state_msg.add_data(trajectory_time_copy);
        state_msg.add_data(trajectory_active_copy ? 1.0 : 0.0);
        state_msg.add_data(p.X());
        state_msg.add_data(p.Y());
        state_msg.add_data(p.Z());

        // Keep the original first 28 values stable for existing tools.
        for (std::size_t i = 0; i < 3; ++i)
          state_msg.add_data(theta_actual[i]);
        for (std::size_t i = 0; i < 3; ++i)
          state_msg.add_data(omega_actual[i]);
        for (std::size_t i = 0; i < 3; ++i)
          state_msg.add_data(targets_copy[i]);
        for (std::size_t i = 0; i < 3; ++i)
          state_msg.add_data(velocity_targets_copy[i]);
        for (std::size_t i = 0; i < 3; ++i)
          state_msg.add_data(acceleration_targets_copy[i]);
        for (std::size_t i = 0; i < 3; ++i)
          state_msg.add_data(this->last_torque_commands_[i]);
        for (std::size_t i = 0; i < 3; ++i)
          state_msg.add_data(this->last_saturated_[i]);

        state_msg.add_data(theta_actual[3]);
        state_msg.add_data(omega_actual[3]);
        state_msg.add_data(targets_copy[3]);
        state_msg.add_data(velocity_targets_copy[3]);
        state_msg.add_data(acceleration_targets_copy[3]);
        state_msg.add_data(this->last_torque_commands_[3]);
        state_msg.add_data(this->last_saturated_[3]);
        const auto joint4_position =
          _ecm.Component<components::JointPosition>(
            this->motor4_input_joint_entity_);
        state_msg.add_data(
          joint4_position && !joint4_position->Data().empty() ?
          joint4_position->Data()[0] : 0.0);

        this->state_feedback_pub_.Publish(state_msg);
      }
    }
  }
}

void DeltaMotorController2::ApplyTwistStabilizers(
  EntityComponentManager &_ecm)
{
  if (this->twist_torque_limit_ <= 0.0)
    return;

  for (std::size_t i = 0; i < this->twist_child_entities_.size(); ++i)
  {
    if (this->twist_parent_entities_[i] == kNullEntity ||
        this->twist_child_entities_[i] == kNullEntity)
    {
      continue;
    }

    Link parent_link(this->twist_parent_entities_[i]);
    Link child_link(this->twist_child_entities_[i]);
    const auto parent_angular_velocity =
      parent_link.WorldAngularVelocity(_ecm);
    const auto child_angular_velocity =
      child_link.WorldAngularVelocity(_ecm);
    const math::Pose3d child_pose =
      worldPose(this->twist_child_entities_[i], _ecm);

    if (!parent_angular_velocity || !child_angular_velocity)
      continue;

    math::Vector3d axis_world =
      child_pose.Rot().RotateVector(math::Vector3d::UnitX);
    axis_world.Normalize();
    const math::Vector3d current_y =
      child_pose.Rot().RotateVector(math::Vector3d::UnitY);
    const math::Vector3d current_z =
      child_pose.Rot().RotateVector(math::Vector3d::UnitZ);

    if (!this->twist_reference_initialized_[i])
    {
      this->twist_reference_y_[i] = current_y;
      this->twist_reference_z_[i] = current_z;
      this->twist_reference_initialized_[i] = true;
      continue;
    }

    math::Vector3d reference = this->twist_reference_y_[i];
    math::Vector3d desired =
      reference - axis_world * axis_world.Dot(reference);
    if (desired.Length() < 0.2)
    {
      reference = this->twist_reference_z_[i];
      desired = reference - axis_world * axis_world.Dot(reference);
    }
    if (desired.Length() < 1e-9)
      continue;

    desired.Normalize();
    math::Vector3d current =
      current_y - axis_world * axis_world.Dot(current_y);
    if (current.Length() < 1e-9)
      continue;
    current.Normalize();

    const double twist_angle = std::atan2(
      axis_world.Dot(current.Cross(desired)),
      this->Clamp(current.Dot(desired), -1.0, 1.0));
    const double twist_rate =
      axis_world.Dot(
        *child_angular_velocity - *parent_angular_velocity);
    const double torque_scalar = this->Clamp(
      this->twist_kp_ * twist_angle - this->twist_kd_ * twist_rate,
      -this->twist_torque_limit_,
      this->twist_torque_limit_);
    const math::Vector3d torque_world = axis_world * torque_scalar;

    child_link.AddWorldWrench(
      _ecm, math::Vector3d::Zero, torque_world);
    parent_link.AddWorldWrench(
      _ecm, math::Vector3d::Zero, -torque_world);

    if (this->feedback_counter_ % this->feedback_decimation_ == 0)
    {
      gz::msgs::Double angle_msg;
      gz::msgs::Double rate_msg;
      angle_msg.set_data(twist_angle);
      rate_msg.set_data(twist_rate);
      this->twist_angle_debug_pubs_[i].Publish(angle_msg);
      this->twist_rate_debug_pubs_[i].Publish(rate_msg);
    }
  }
}

void DeltaMotorController2::OnTarget1(const gz::msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (this->joint_ref_mode_)
    return;

  this->targets_[0] = _msg.data();
  this->velocity_targets_[0] = 0.0;
  this->acceleration_targets_[0] = 0.0;
}

void DeltaMotorController2::OnTarget2(const gz::msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (this->joint_ref_mode_)
    return;

  this->targets_[1] = _msg.data();
  this->velocity_targets_[1] = 0.0;
  this->acceleration_targets_[1] = 0.0;
}

void DeltaMotorController2::OnTarget3(const gz::msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (this->joint_ref_mode_)
    return;

  this->targets_[2] = _msg.data();
  this->velocity_targets_[2] = 0.0;
  this->acceleration_targets_[2] = 0.0;
}

void DeltaMotorController2::OnTarget4(const gz::msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (this->joint_ref_mode_)
    return;

  this->targets_[3] = _msg.data();
  this->velocity_targets_[3] = 0.0;
  this->acceleration_targets_[3] = 0.0;
}

void DeltaMotorController2::OnJointReference(
  const gz::msgs::Double_V &_msg)
{
  const int size = _msg.data_size();

  const bool is_3dof =
    size == 6 || size == 9;

  const bool is_4dof =
    size == 8 || size == 12;

  if (!is_3dof && !is_4dof)
  {
    gzerr
      << "Joint reference rejected. Valid sizes are:\n"
      << "  6  = q1..q3, qd1..qd3\n"
      << "  8  = q1..q4, qd1..qd4\n"
      << "  9  = q1..q3, qd1..qd3, qdd1..qdd3\n"
      << "  12 = q1..q4, qd1..qd4, qdd1..qdd4\n"
      << "Received size=" << size << "\n";
    return;
  }

  for (int i = 0; i < size; ++i)
  {
    if (!std::isfinite(_msg.data(i)))
    {
      gzerr << "Joint reference rejected: NaN or Inf.\n";
      return;
    }
  }

  std::lock_guard<std::mutex> lock(this->mutex_);

  if (is_3dof)
  {
    for (std::size_t i = 0; i < 3; ++i)
    {
      this->targets_[i] = _msg.data(i);
      this->velocity_targets_[i] = _msg.data(3 + i);

      this->acceleration_targets_[i] =
        size == 9 ? _msg.data(6 + i) : 0.0;
    }

    // Giữ nguyên hướng q4 khi chỉ jog XYZ.
    this->velocity_targets_[3] = 0.0;
    this->acceleration_targets_[3] = 0.0;
  }
  else
  {
    for (std::size_t i = 0; i < 4; ++i)
    {
      this->targets_[i] = _msg.data(i);
      this->velocity_targets_[i] = _msg.data(4 + i);

      this->acceleration_targets_[i] =
        size == 12 ? _msg.data(8 + i) : 0.0;
    }
  }

  this->integrals_ = {0.0, 0.0, 0.0, 0.0};

  this->trajectory_received_ = false;
  this->trajectory_active_ = false;
  this->current_trajectory_time_ = 0.0;
  this->joint_ref_mode_ = true;
}

void DeltaMotorController2::OnJointTrajectory(
  const gz::msgs::Double_V &_msg)
{
  gzmsg << "OnJointTrajectory callback triggered. data_size="
        << _msg.data_size() << "\n";

  // New format:
  // data[0] = trajectory_id
  // data[1] = N
  // data[2...] = waypoint data
  //
  // Legacy waypoint: t + 3 q + 3 qd + 3 qdd = 10 values.
  // 4DOF waypoint: t + 4 q + 4 qd + 4 qdd = 13 values.

  if (_msg.data_size() < 12)
  {
    gzerr << "Joint trajectory rejected: message too short.\n";
    return;
  }

  const int trajectory_id = static_cast<int>(_msg.data(0));
  const int n = static_cast<int>(_msg.data(1));

  if (n < 2)
  {
    gzerr << "Joint trajectory rejected: N must be >= 2.\n";
    return;
  }

  const int payload_size = _msg.data_size() - 2;
  const int waypoint_size =
    payload_size >= n * 13 ? 13 : 10;
  const int expected_size = 2 + n * waypoint_size;

  if (_msg.data_size() < expected_size)
  {
    gzerr << "Joint trajectory rejected: expected "
          << expected_size << " values, got "
          << _msg.data_size() << ".\n";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (trajectory_id == this->last_trajectory_id_)
    {
      gzmsg << "Duplicate trajectory ignored: id="
            << trajectory_id << "\n";
      return;
    }

    this->last_trajectory_id_ = trajectory_id;
  }

  double q4_hold = 0.0;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    q4_hold = this->targets_[3];
  }

  std::vector<JointTrajectoryPoint2> new_traj;
  new_traj.reserve(n);

  int offset = 2;

  for (int k = 0; k < n; ++k)
  {
    JointTrajectoryPoint2 pt;

    pt.t = _msg.data(offset + 0);

    if (waypoint_size == 13)
    {
      for (std::size_t i = 0; i < 4; ++i)
      {
        pt.q[i] = _msg.data(offset + 1 + i);
        pt.qd[i] = _msg.data(offset + 5 + i);
        pt.qdd[i] = _msg.data(offset + 9 + i);
      }
    }
    else
    {
      for (std::size_t i = 0; i < 3; ++i)
      {
        pt.q[i] = _msg.data(offset + 1 + i);
        pt.qd[i] = _msg.data(offset + 4 + i);
        pt.qdd[i] = _msg.data(offset + 7 + i);
      }
      pt.q[3] = q4_hold;
    }

    new_traj.push_back(pt);
    offset += waypoint_size;
  }

  std::lock_guard<std::mutex> lock(this->mutex_);

  this->trajectory_ = new_traj;
  this->trajectory_duration_ = this->trajectory_.back().t;
  this->last_segment_index_ = 0;
  this->saturation_count_ = {0, 0, 0, 0};
  this->integrals_ = {0.0, 0.0, 0.0, 0.0};
  this->last_torque_commands_ = {0.0, 0.0, 0.0, 0.0};

  this->trajectory_received_ = true;
  this->trajectory_active_ = false;

  // Bật mode trajectory để topic q-only cũ không ghi đè.
  this->joint_ref_mode_ = true;

  gzmsg << "Received full joint trajectory: id="
        << trajectory_id
        << ", N=" << this->trajectory_.size()
        << ", duration=" << this->trajectory_duration_
        << " s.\n";
}

bool DeltaMotorController2::SampleTrajectory(
  double t,
  std::array<double, 4> &q_ref,
  std::array<double, 4> &qd_ref,
  std::array<double, 4> &qdd_ref)
{
  if (this->trajectory_.empty())
    return false;

  if (t <= this->trajectory_.front().t)
  {
    q_ref = this->trajectory_.front().q;
    qd_ref = this->trajectory_.front().qd;
    qdd_ref = this->trajectory_.front().qdd;
    return true;
  }

  if (t >= this->trajectory_.back().t)
  {
    q_ref = this->trajectory_.back().q;
    qd_ref = {0.0, 0.0, 0.0, 0.0};
    qdd_ref = {0.0, 0.0, 0.0, 0.0};
    return true;
  }

  while (
    this->last_segment_index_ + 1 < this->trajectory_.size() &&
    t >= this->trajectory_[this->last_segment_index_ + 1].t)
  {
    this->last_segment_index_++;
  }

  if (this->last_segment_index_ + 1 >= this->trajectory_.size())
  {
    q_ref = this->trajectory_.back().q;
    qd_ref = {0.0, 0.0, 0.0, 0.0};
    qdd_ref = {0.0, 0.0, 0.0, 0.0};
    return true;
  }

  const auto &a = this->trajectory_[this->last_segment_index_];
  const auto &b = this->trajectory_[this->last_segment_index_ + 1];

  const double dt = b.t - a.t;

  if (dt <= 1e-9)
  {
    q_ref = b.q;
    qd_ref = b.qd;
    qdd_ref = b.qdd;
    return true;
  }

  const double u = std::clamp((t - a.t) / dt, 0.0, 1.0);

  for (std::size_t i = 0; i < 4; ++i)
  {
    q_ref[i] = a.q[i] + u * (b.q[i] - a.q[i]);
    qd_ref[i] = a.qd[i] + u * (b.qd[i] - a.qd[i]);
    qdd_ref[i] = a.qdd[i] + u * (b.qdd[i] - a.qdd[i]);
  }

  return true;
}
bool DeltaMotorController2::MeasureMotor4Orientation(
  EntityComponentManager &_ecm,
  double &_angle,
  double &_angular_velocity) const
{
  if (this->base_entity_ == kNullEntity ||
      this->motor4_output_link_entity_ == kNullEntity)
  {
    return false;
  }

  // Pose của base_link và 411111 trong hệ world.
  const math::Pose3d base_world_pose =
    worldPose(this->base_entity_, _ecm);

  const math::Pose3d output_world_pose =
    worldPose(this->motor4_output_link_entity_, _ecm);

  // Pose của 411111 biểu diễn trong hệ base_link:
  // T_base_output = inverse(T_world_base) * T_world_output
  const math::Pose3d output_base_pose =
    base_world_pose.Inverse() * output_world_pose;

  // Local X của 411111 là trục quay.
  // Vì vậy dùng local Z làm vector biểu diễn hướng quay của khâu.
  math::Vector3d heading_base =
    output_base_pose.Rot().RotateVector(
      math::Vector3d::UnitZ);

  // Chỉ xét hướng chiếu trên mặt phẳng XY của base_link.
  heading_base = math::Vector3d(
    heading_base.X(),
    heading_base.Y(),
    0.0);

  if (heading_base.Length() < 1e-9)
  {
    return false;
  }

  heading_base.Normalize();

  // Góc tuyệt đối của hướng 411111 so với trục +X của base_link.
  // Kết quả thuộc [-pi, pi].
  _angle = std::atan2(
    heading_base.Y(),
    heading_base.X());

  Link output_link(this->motor4_output_link_entity_);
  Link base_link(this->base_entity_);

  const auto output_angular_velocity =
    output_link.WorldAngularVelocity(_ecm);

  const auto base_angular_velocity =
    base_link.WorldAngularVelocity(_ecm);

  if (!output_angular_velocity)
  {
    return false;
  }

  // Trục quay của 411111 trong hệ world.
  math::Vector3d spin_axis_world =
    output_world_pose.Rot().RotateVector(
      math::Vector3d::UnitX);

  if (spin_axis_world.Length() < 1e-9)
  {
    return false;
  }

  spin_axis_world.Normalize();

  // Vận tốc góc tương đối giữa 411111 và base_link.
  math::Vector3d relative_angular_velocity =
    *output_angular_velocity;

  if (base_angular_velocity)
  {
    relative_angular_velocity -= *base_angular_velocity;
  }

  // Thành phần vận tốc quanh trục quay của khâu.
  _angular_velocity =
    spin_axis_world.Dot(relative_angular_velocity);

  return std::isfinite(_angle) &&
         std::isfinite(_angular_velocity);
}
double DeltaMotorController2::Clamp(
  double value,
  double min_value,
  double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}

}  // namespace gz_delta_controller2

GZ_ADD_PLUGIN(
  gz_delta_controller2::DeltaMotorController2,
  gz::sim::System,
  gz_delta_controller2::DeltaMotorController2::ISystemConfigure,
  gz_delta_controller2::DeltaMotorController2::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  gz_delta_controller2::DeltaMotorController2,
  "gz::sim::systems::DeltaMotorController2")
