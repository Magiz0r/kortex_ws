#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "BaseCyclicClientRpc.h"
#include "RouterClient.h"
#include "Session.pb.h"
#include "SessionManager.h"
#include "TransportClientUdp.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
const rclcpp::Logger LOGGER = rclcpp::get_logger("kinova_read_only_joint_state_publisher");
}  // namespace

namespace k_api = Kinova::Api;

class KinovaReadOnlyJointStatePublisher : public rclcpp::Node
{
public:
  KinovaReadOnlyJointStatePublisher()
  : Node("kinova_read_only_joint_state_publisher")
  {
    this->declare_parameter<std::string>("robot_ip", "192.168.1.10");
    this->declare_parameter<std::string>("username", "admin");
    this->declare_parameter<std::string>("password", "admin");
    this->declare_parameter<int>("udp_port", 10001);
    this->declare_parameter<int>("session_inactivity_timeout_ms", 60000);
    this->declare_parameter<int>("connection_inactivity_timeout_ms", 2000);
    this->declare_parameter<double>("publish_rate_hz", 60.0);
    this->declare_parameter<std::string>("joint_state_topic", "/kinova_read_only/joint_states");
    this->declare_parameter<std::vector<std::string>>(
      "arm_joint_names",
      std::vector<std::string>{
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"});
    this->declare_parameter<bool>("publish_gripper", true);
    this->declare_parameter<std::string>("finger_joint_name", "finger_joint");
    this->declare_parameter<double>("gripper_max_joint_position", 0.81);

    robot_ip_ = this->get_parameter("robot_ip").as_string();
    username_ = this->get_parameter("username").as_string();
    password_ = this->get_parameter("password").as_string();
    udp_port_ = this->get_parameter("udp_port").as_int();
    session_inactivity_timeout_ms_ =
      this->get_parameter("session_inactivity_timeout_ms").as_int();
    connection_inactivity_timeout_ms_ =
      this->get_parameter("connection_inactivity_timeout_ms").as_int();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
    arm_joint_names_ = this->get_parameter("arm_joint_names").as_string_array();
    publish_gripper_ = this->get_parameter("publish_gripper").as_bool();
    finger_joint_name_ = this->get_parameter("finger_joint_name").as_string();
    gripper_max_joint_position_ = this->get_parameter("gripper_max_joint_position").as_double();

    if (robot_ip_.empty()) {
      throw std::runtime_error("robot_ip must not be empty");
    }
    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be greater than 0");
    }
    if (arm_joint_names_.size() != 7) {
      throw std::runtime_error("arm_joint_names must contain exactly 7 joints");
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&KinovaReadOnlyJointStatePublisher::publish_joint_state, this));

    auto joint_names = arm_joint_names_;
    if (publish_gripper_) {
      joint_names.push_back(finger_joint_name_);
    }

    RCLCPP_INFO(
      LOGGER, "Publishing read-only Kinova feedback from %s to %s for joints: %s",
      robot_ip_.c_str(), joint_state_topic_.c_str(), join(joint_names).c_str());
  }

  ~KinovaReadOnlyJointStatePublisher() override
  {
    disconnect();
  }

private:
  void publish_joint_state()
  {
    if (!ensure_connection()) {
      return;
    }

    k_api::BaseCyclic::Feedback feedback;
    try {
      feedback = base_cyclic_->RefreshFeedback();
    } catch (const std::exception & ex) {
      report_error_once(std::string("Failed to read Kinova feedback: ") + ex.what());
      disconnect();
      return;
    }

    if (feedback.actuators_size() < static_cast<int>(arm_joint_names_.size())) {
      report_error_once(
        "Received only " + std::to_string(feedback.actuators_size()) + " actuators, expected " +
        std::to_string(arm_joint_names_.size()));
      return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = arm_joint_names_;
    msg.position.reserve(arm_joint_names_.size() + (publish_gripper_ ? 1U : 0U));
    msg.velocity.reserve(arm_joint_names_.size() + (publish_gripper_ ? 1U : 0U));
    msg.effort.reserve(arm_joint_names_.size() + (publish_gripper_ ? 1U : 0U));

    for (std::size_t index = 0; index < arm_joint_names_.size(); ++index) {
      const auto & actuator = feedback.actuators(static_cast<int>(index));
      msg.position.push_back(wrap_to_pi(deg_to_rad(actuator.position())));
      msg.velocity.push_back(deg_to_rad(actuator.velocity()));
      msg.effort.push_back(actuator.torque());
    }

    if (publish_gripper_) {
      msg.name.push_back(finger_joint_name_);
      msg.position.push_back(read_gripper_position(feedback));
      msg.velocity.push_back(0.0);
      msg.effort.push_back(0.0);
    }

    publisher_->publish(msg);
  }

  bool ensure_connection()
  {
    if (connected_) {
      return true;
    }

    try {
      transport_ = std::make_unique<k_api::TransportClientUdp>();
      router_ = std::make_unique<k_api::RouterClient>(
        transport_.get(), [](k_api::KError err) {
          RCLCPP_WARN(LOGGER, "Kinova router callback reported: %s", err.toString().c_str());
        });

      transport_->connect(robot_ip_, static_cast<uint32_t>(udp_port_));

      k_api::Session::CreateSessionInfo session_info;
      session_info.set_username(username_);
      session_info.set_password(password_);
      session_info.set_session_inactivity_timeout(session_inactivity_timeout_ms_);
      session_info.set_connection_inactivity_timeout(connection_inactivity_timeout_ms_);

      session_manager_ = std::make_unique<k_api::SessionManager>(router_.get());
      session_manager_->CreateSession(session_info);
      base_cyclic_ = std::make_unique<k_api::BaseCyclic::BaseCyclicClient>(router_.get());
      connected_ = true;
      last_error_.clear();

      RCLCPP_INFO(
        LOGGER, "Connected to Kinova read-only feedback stream at %s:%d", robot_ip_.c_str(),
        udp_port_);
      return true;
    } catch (const std::exception & ex) {
      report_error_once(std::string("Failed to connect to Kinova feedback stream: ") + ex.what());
      disconnect();
      return false;
    }
  }

  void disconnect()
  {
    if (session_manager_ != nullptr) {
      try {
        session_manager_->CloseSession();
      } catch (...) {
      }
    }

    if (transport_ != nullptr) {
      try {
        transport_->disconnect();
      } catch (...) {
      }
    }

    base_cyclic_.reset();
    session_manager_.reset();
    router_.reset();
    transport_.reset();
    connected_ = false;
  }

  double read_gripper_position(const k_api::BaseCyclic::Feedback & feedback)
  {
    try {
      if (feedback.interconnect().gripper_feedback().motor_size() <= 0) {
        throw std::runtime_error("no gripper motors reported");
      }
      return feedback.interconnect().gripper_feedback().motor()[0].position() / 100.0 *
             gripper_max_joint_position_;
    } catch (const std::exception &) {
      if (!reported_missing_gripper_) {
        RCLCPP_WARN(
          LOGGER, "Gripper feedback was not available; publishing %s at 0.0",
          finger_joint_name_.c_str());
        reported_missing_gripper_ = true;
      }
      return 0.0;
    }
  }

  void report_error_once(const std::string & message)
  {
    if (message != last_error_) {
      RCLCPP_WARN(LOGGER, "%s", message.c_str());
      last_error_ = message;
    }
  }

  static double deg_to_rad(double degrees)
  {
    return degrees * kPi / 180.0;
  }

  static double wrap_to_pi(double angle_radians)
  {
    double wrapped = std::fmod(angle_radians + kPi, 2.0 * kPi);
    if (wrapped < 0.0) {
      wrapped += 2.0 * kPi;
    }
    return wrapped - kPi;
  }

  static std::string join(const std::vector<std::string> & items)
  {
    std::string output;
    for (std::size_t i = 0; i < items.size(); ++i) {
      output += items[i];
      if (i + 1 < items.size()) {
        output += ", ";
      }
    }
    return output;
  }

  std::string robot_ip_;
  std::string username_;
  std::string password_;
  std::string joint_state_topic_;
  std::vector<std::string> arm_joint_names_;
  std::string finger_joint_name_;
  int udp_port_;
  int session_inactivity_timeout_ms_;
  int connection_inactivity_timeout_ms_;
  double publish_rate_hz_;
  bool publish_gripper_;
  double gripper_max_joint_position_;

  bool connected_{false};
  bool reported_missing_gripper_{false};
  std::string last_error_;

  std::unique_ptr<k_api::TransportClientUdp> transport_;
  std::unique_ptr<k_api::RouterClient> router_;
  std::unique_ptr<k_api::SessionManager> session_manager_;
  std::unique_ptr<k_api::BaseCyclic::BaseCyclicClient> base_cyclic_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinovaReadOnlyJointStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
