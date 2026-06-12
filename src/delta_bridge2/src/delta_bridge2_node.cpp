#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <gz/msgs/double_v.pb.h>
#include <gz/transport/Node.hh>

class DeltaBridge2Node : public rclcpp::Node
{
public:
  DeltaBridge2Node()
  : Node("delta_bridge2_node")
  {
    this->joint_ref_pub_ =
      this->gz_node_.Advertise<gz::msgs::Double_V>(
        "/delta_robot/joint_ref_gz");

    this->joint_ref_sub_ =
      this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/kinematic2/joint_ref_ros",
        10,
        std::bind(
          &DeltaBridge2Node::OnJointRefRos,
          this,
          std::placeholders::_1));

    this->joint_traj_pub_ =
      this->gz_node_.Advertise<gz::msgs::Double_V>(
        "/delta_robot/joint_trajectory_gz");

    this->joint_traj_sub_ =
      this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/kinematic2/joint_trajectory_ros",
        10,
        std::bind(
          &DeltaBridge2Node::OnJointTrajectoryRos,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "delta_bridge2 started:\n"
      "  /kinematic2/joint_ref_ros -> /delta_robot/joint_ref_gz\n"
      "  /kinematic2/joint_trajectory_ros -> "
      "/delta_robot/joint_trajectory_gz");
  }

private:
  void OnJointRefRos(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    const std::size_t size = msg->data.size();

    if (size != 6 && size != 8 &&
        size != 9 && size != 12)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Joint ref rejected: valid sizes are 6, 8, 9 or 12; "
        "received size=%zu",
        size);
      return;
    }

    gz::msgs::Double_V gz_msg;

    for (const auto &value : msg->data)
    {
      gz_msg.add_data(value);
    }

    const bool ok =
      this->joint_ref_pub_.Publish(gz_msg);

    if (!ok)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to publish /delta_robot/joint_ref_gz");
    }
  }

  void OnJointTrajectoryRos(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 12)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Joint trajectory rejected: data too short. size=%zu",
        msg->data.size());
      return;
    }

    const double trajectory_id = msg->data[0];
    const double n_waypoints = msg->data[1];

    if (n_waypoints < 2.0)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Joint trajectory rejected: N must be >= 2");
      return;
    }

    const std::size_t n =
      static_cast<std::size_t>(n_waypoints);

    const std::size_t legacy_size =
      2 + n * 10;

    const std::size_t four_dof_size =
      2 + n * 13;

    std::size_t expected_size = 0;

    if (msg->data.size() == four_dof_size)
    {
      expected_size = four_dof_size;
    }
    else if (msg->data.size() == legacy_size)
    {
      expected_size = legacy_size;
    }
    else
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Joint trajectory rejected: size=%zu, "
        "expected legacy=%zu or 4DOF=%zu",
        msg->data.size(),
        legacy_size,
        four_dof_size);
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Bridge2 received trajectory: "
      "size=%zu, id=%.0f, N=%.0f, expected=%zu",
      msg->data.size(),
      trajectory_id,
      n_waypoints,
      expected_size);

    gz::msgs::Double_V gz_msg;

    for (const auto &value : msg->data)
    {
      gz_msg.add_data(value);
    }

    const bool ok =
      this->joint_traj_pub_.Publish(gz_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Bridge2 published trajectory once: "
      "ok=%s, id=%.0f, N=%.0f, gz_size=%d",
      ok ? "true" : "false",
      trajectory_id,
      n_waypoints,
      gz_msg.data_size());

    if (!ok)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to publish "
        "/delta_robot/joint_trajectory_gz");
    }
  }

  gz::transport::Node gz_node_;

  gz::transport::Node::Publisher joint_ref_pub_;
  gz::transport::Node::Publisher joint_traj_pub_;

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_ref_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_traj_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<DeltaBridge2Node>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
