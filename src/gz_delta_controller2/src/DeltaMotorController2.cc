#include "gz_delta_controller2/DeltaMotorController2.hh"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <gz/math/Pose3.hh>

#include <gz/plugin/Register.hh>

#include <gz/sim/Util.hh>

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>

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

  if (_sdf->HasElement("ka"))
    this->ka_ = _sdf->Get<double>("ka");

  if (_sdf->HasElement("torque_limit"))
    this->torque_limit_ = _sdf->Get<double>("torque_limit");

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
    gzerr << "Cannot find link [base_link]. XYZ feedback will use world frame.\n";
  }

  this->endlink_entity_ = this->model_.LinkByName(_ecm, "end");

  if (this->endlink_entity_ == kNullEntity)
  {
    gzerr << "Cannot find link [end]. XYZ feedback will not work.\n";
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

  this->xyz_feedback_pubs_[0] =
    this->gz_node_.Advertise<gz::msgs::Double>(
      "/delta_robot/feedback/x_gz");

  this->xyz_feedback_pubs_[1] =
    this->gz_node_.Advertise<gz::msgs::Double>(
      "/delta_robot/feedback/y_gz");

  this->xyz_feedback_pubs_[2] =
    this->gz_node_.Advertise<gz::msgs::Double>(
      "/delta_robot/feedback/z_gz");

  for (std::size_t i = 0; i < 3; ++i)
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
        << ", ka=" << this->ka_
        << ", torque_limit=" << this->torque_limit_
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

    for (std::size_t i = 0; i < this->joint_entities_.size(); ++i)
    {
      auto pos_comp =
        _ecm.Component<components::JointPosition>(
          this->joint_entities_[i]);

      this->targets_[i] = pos_comp->Data()[0];
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
      this->trajectory_active_ = true;
      this->trajectory_received_ = false;
      this->last_segment_index_ = 0;

      gzmsg << "Trajectory started at sim_time="
            << this->trajectory_sim_start_ << "\n";
    }

    if (this->trajectory_active_)
    {
      const double traj_t = sim_time - this->trajectory_sim_start_;

      std::array<double, 3> q_ref;
      std::array<double, 3> qd_ref;
      std::array<double, 3> qdd_ref;

      if (this->SampleTrajectory(traj_t, q_ref, qd_ref, qdd_ref))
      {
        this->targets_ = q_ref;
        this->velocity_targets_ = qd_ref;
        this->acceleration_targets_ = qdd_ref;
      }

      if (traj_t >= this->trajectory_duration_)
      {
        this->trajectory_active_ = false;

        if (!this->trajectory_.empty())
        {
          this->targets_ = this->trajectory_.back().q;
        }

        this->velocity_targets_ = {0.0, 0.0, 0.0};
        this->acceleration_targets_ = {0.0, 0.0, 0.0};

        gzmsg << "Trajectory sampling finished. Holding final target.\n";

        gzmsg << "Saturation count: "
              << this->saturation_count_[0] << ", "
              << this->saturation_count_[1] << ", "
              << this->saturation_count_[2] << "\n";
      }
    }
  }

  std::array<double, 3> targets_copy;
  std::array<double, 3> velocity_targets_copy;
  std::array<double, 3> acceleration_targets_copy;

  {
    std::lock_guard<std::mutex> lock(this->mutex_);

    targets_copy = this->targets_;
    velocity_targets_copy = this->velocity_targets_;
    acceleration_targets_copy = this->acceleration_targets_;
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

    const double theta_current = pos_comp->Data()[0];
    const double omega_current = vel_comp->Data()[0];

    const double theta_target = targets_copy[i];
    const double omega_target = velocity_targets_copy[i];
    const double acceleration_target = acceleration_targets_copy[i];

    const double position_error = theta_target - theta_current;
    const double velocity_error = omega_target - omega_current;

    this->integrals_[i] += position_error * dt;

    const double torque_raw =
      this->kp_ * position_error +
      this->kd_ * velocity_error +
      this->ki_ * this->integrals_[i] +
      this->ka_ * acceleration_target;

    const double torque_cmd = this->Clamp(
      torque_raw,
      -this->torque_limit_,
      this->torque_limit_);

    force_comp->Data()[0] = torque_cmd;

    // Count torque saturation on every PreUpdate step.
    const bool saturated =
      std::abs(torque_cmd) >= 0.98 * this->torque_limit_;

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

  this->feedback_counter_++;

  if (this->feedback_counter_ % this->feedback_decimation_ == 0)
  {
    for (std::size_t i = 0; i < this->joint_entities_.size(); ++i)
    {
      auto pos_comp =
        _ecm.Component<components::JointPosition>(
          this->joint_entities_[i]);

      if (!pos_comp || pos_comp->Data().empty())
        continue;

      gz::msgs::Double theta_msg;
      theta_msg.set_data(pos_comp->Data()[0]);

      this->theta_feedback_pubs_[i].Publish(theta_msg);
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

void DeltaMotorController2::OnJointReference(
  const gz::msgs::Double_V &_msg)
{
  if (_msg.data_size() < 6)
  {
    gzerr << "Joint reference rejected. Need at least "
          << "[q1,q2,q3,qd1,qd2,qd3].\n";
    return;
  }

  std::lock_guard<std::mutex> lock(this->mutex_);

  this->targets_[0] = _msg.data(0);
  this->targets_[1] = _msg.data(1);
  this->targets_[2] = _msg.data(2);

  this->velocity_targets_[0] = _msg.data(3);
  this->velocity_targets_[1] = _msg.data(4);
  this->velocity_targets_[2] = _msg.data(5);

  if (_msg.data_size() >= 9)
  {
    this->acceleration_targets_[0] = _msg.data(6);
    this->acceleration_targets_[1] = _msg.data(7);
    this->acceleration_targets_[2] = _msg.data(8);
  }
  else
  {
    this->acceleration_targets_ = {0.0, 0.0, 0.0};
  }

  // Direct references are jog commands and override any running trajectory.
  this->trajectory_received_ = false;
  this->trajectory_active_ = false;
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
  // Each waypoint has 10 values:
  // t,
  // q1, q2, q3,
  // qd1, qd2, qd3,
  // qdd1, qdd2, qdd3

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

  const int expected_size = 2 + n * 10;

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

  std::vector<JointTrajectoryPoint2> new_traj;
  new_traj.reserve(n);

  int offset = 2;

  for (int k = 0; k < n; ++k)
  {
    JointTrajectoryPoint2 pt;

    pt.t = _msg.data(offset + 0);

    pt.q = {
      _msg.data(offset + 1),
      _msg.data(offset + 2),
      _msg.data(offset + 3)
    };

    pt.qd = {
      _msg.data(offset + 4),
      _msg.data(offset + 5),
      _msg.data(offset + 6)
    };

    pt.qdd = {
      _msg.data(offset + 7),
      _msg.data(offset + 8),
      _msg.data(offset + 9)
    };

    new_traj.push_back(pt);
    offset += 10;
  }

  std::lock_guard<std::mutex> lock(this->mutex_);

  this->trajectory_ = new_traj;
  this->trajectory_duration_ = this->trajectory_.back().t;
  this->last_segment_index_ = 0;
  this->saturation_count_ = {0, 0, 0};

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
  std::array<double, 3> &q_ref,
  std::array<double, 3> &qd_ref,
  std::array<double, 3> &qdd_ref)
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
    qd_ref = {0.0, 0.0, 0.0};
    qdd_ref = {0.0, 0.0, 0.0};
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
    qd_ref = {0.0, 0.0, 0.0};
    qdd_ref = {0.0, 0.0, 0.0};
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

  for (std::size_t i = 0; i < 3; ++i)
  {
    q_ref[i] = a.q[i] + u * (b.q[i] - a.q[i]);
    qd_ref[i] = a.qd[i] + u * (b.qd[i] - a.qd[i]);
    qdd_ref[i] = a.qdd[i] + u * (b.qdd[i] - a.qdd[i]);
  }

  return true;
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
