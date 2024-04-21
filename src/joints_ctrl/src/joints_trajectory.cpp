// This script works with Michael's control
// And with my trajectory generation algorithm

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ctrl_interfaces/msg/franka_joints.hpp" // Use the name in snake (_._) case. Original name can be in CamelCase

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>

using namespace std::chrono_literals;

/////////////////// Global Variables ///////////////////
const std::string urdf_path = "/home/loop/project/franka_sim/src/panda_ign_moveit2/panda_description/urdf/panda.urdf";
const unsigned int n_joints = 7;

const std::vector<double> lower_joint_limits {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
const std::vector<double> upper_joint_limits {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

KDL::Tree panda_tree;
KDL::Chain panda_chain;

/////////////////// Function Declarations ///////////////////
bool within_limits(std::vector<double>& vals);
bool create_tree();
void get_chain();
void print_joint_vals(std::vector<double>& joint_vals);

/////////////// Definition of Node Class //////////////
class GazeboController : public rclcpp::Node {
public:
  std::vector<double> origin {0.4569, 0.0, 0.3853}; // Can change the task-space origin point!

  std::vector<double> curr_joint_vals = std::vector<double>(n_joints, 0.0);
  std::vector<double> last_curr_joint_vals = std::vector<double>(n_joints, 0.0);
  std::vector<double> message_joint_vals = std::vector<double>(n_joints, 0.0);
  std::vector<double> des_joint_vals = std::vector<double>(n_joints, 0.0);
  std::vector<double> last_des_joint_vals = std::vector<double>(n_joints, 0.0);

  bool control = false; 
  const float smoothing_factor = 0.01;
  int max_count = 0;
  int count = 0;
  double w = 0.0; 
  double diff_joints_magnitude = 0.0;
  double threshold_joints_change = 0.4;

  GazeboController() : Node("joints_trajectory") {
    controller_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/desired_joints", 10);
    controller_timer_ = this->create_wall_timer(10ms, std::bind(&GazeboController::controller_publisher, this));
    joint_vals_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&GazeboController::joint_states_callback, this, std::placeholders::_1));
    des_joint_vals_sub_ = this->create_subscription<ctrl_interfaces::msg::FrankaJoints>(
      "/unity_franka_joints", 10, std::bind(&GazeboController::joint_pos_from_unity_callback, this, std::placeholders::_1));

    if (!create_tree()) rclcpp::shutdown();
    get_chain();
  }

private:
  void controller_publisher() {
    if (control) {
      if (count >= max_count) {
        last_curr_joint_vals = curr_joint_vals;
        last_des_joint_vals = des_joint_vals;
        count = 0;

        std::vector<double> diff_vector = last_des_joint_vals;
        for (unsigned int i = 0; i < n_joints; i++) {
          diff_vector.at(i) -= last_curr_joint_vals.at(i); 
        }

        diff_joints_magnitude = 0.0;
        for (auto diff : diff_vector) {
          diff_joints_magnitude += diff * diff;
        }
        diff_joints_magnitude = std::sqrt(diff_joints_magnitude);
        max_count = ((float)diff_joints_magnitude/smoothing_factor) + 1;
      }

      count++;      
      if (max_count > 1 && diff_joints_magnitude >= threshold_joints_change) {
        w = 0.5 * (1.0 - cos(3.1416 * count / (max_count - 1)));

        for (unsigned int j = 0; j < n_joints; ++j) {
          message_joint_vals[j] = last_curr_joint_vals[j] + w * (last_des_joint_vals[j] - last_curr_joint_vals[j]);
        }

        auto point = sensor_msgs::msg::JointState();
        point.position = message_joint_vals;
        controller_pub_->publish(point);
      }
    }
  }

  void joint_pos_from_unity_callback(const ctrl_interfaces::msg::FrankaJoints & msg) {
      des_joint_vals = std::vector<double>(msg.joints.begin(), msg.joints.end());
      if (!control) control = true;
  }

  void joint_states_callback(const sensor_msgs::msg::JointState & msg) {
    curr_joint_vals = std::vector<double>(msg.position.begin(), msg.position.begin() + n_joints);

    // curr_joint_vals.at(0) = msg.position.at(0);
    // curr_joint_vals.at(1) = msg.position.at(1);
    // curr_joint_vals.at(2) = msg.position.at(7);
    // curr_joint_vals.at(3) = msg.position.at(2);
    // curr_joint_vals.at(4) = msg.position.at(3);
    // curr_joint_vals.at(5) = msg.position.at(4);
    // curr_joint_vals.at(6) = msg.position.at(5);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr controller_pub_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_vals_sub_;
  rclcpp::Subscription<ctrl_interfaces::msg::FrankaJoints>::SharedPtr des_joint_vals_sub_;  
};

///////////////// Other Helper Functions /////////////////
bool within_limits(std::vector<double>& vals) {
  for (unsigned int i = 0; i < n_joints; i++) {
    if (vals.at(i) > upper_joint_limits.at(i) || vals.at(i) < lower_joint_limits.at(i)) return false;
  }
  return true;
}

bool create_tree() {
  if (!kdl_parser::treeFromFile(urdf_path, panda_tree)) {
    std::cout << "Failed to construct kdl tree" << std::endl;
    return false;
  }
  return true;
}

void get_chain() {
  // panda_tree.getChain("panda_link0", "panda_grasptarget", panda_chain);
  panda_tree.getChain("panda_link1", "panda_link7", panda_chain);
}

void print_joint_vals(std::vector<double>& joint_vals) {
  std::cout << "[ ";
  for (auto val : joint_vals) {
    std::cout << val << ' ';
  }
  std::cout << "]" << std::endl;
}

//////////////////// Main Function ///////////////////
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GazeboController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
