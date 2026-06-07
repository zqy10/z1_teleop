/**
 * Teleoperation networking: frame packing and jitter buffers.
 *
 *   rosrun teleop_arm communication _role:=master
 *   rosrun teleop_arm communication _role:=slave
 */

#include <algorithm>
#include <array>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <teleop_msgs/TeleopFrame.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace {

void quatToEuler(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw) {
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
}

using Pose6 = std::array<double, 6>;

std::string roleFromArgs(int argc, char** argv) {
  const std::string role_prefix = "_role:=";
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg.compare(0, role_prefix.size(), role_prefix) == 0) {
      return arg.substr(role_prefix.size());
    }
  }
  return "";
}

std::string nodeNameForRole(const std::string& role) {
  if (role == "master") {
    return "teleop_communication_master";
  }
  if (role == "slave") {
    return "teleop_communication_slave";
  }
  return "teleop_communication";
}

template <typename T>
void paramWithLegacyNamespace(ros::NodeHandle& pnh, const std::string& name, T& value) {
  if (pnh.getParam(name, value)) {
    return;
  }
  ros::NodeHandle legacy_nh("/teleop_communication");
  legacy_nh.param(name, value, value);
}

Pose6 poseMsgTo6(const geometry_msgs::Pose& msg) {
  double roll = 0, pitch = 0, yaw = 0;
  quatToEuler(msg.orientation, roll, pitch, yaw);
  return {msg.position.x, msg.position.y, msg.position.z, roll, pitch, yaw};
}

class MasterArmDataPackager {
 public:
  void start() {
    pose_sub_ = nh_.subscribe("/master_arm/tool_pose", 10, &MasterArmDataPackager::poseCb, this);
    vel_sub_ = nh_.subscribe("/master_arm/tool_velocity", 10, &MasterArmDataPackager::velCb, this);
    wrench_sub_ = nh_.subscribe("/master_arm/wrench", 10, &MasterArmDataPackager::wrenchCb, this);
    net_pub_ = nh_.advertise<teleop_msgs::TeleopFrame>("/network/master_frame", 50);
    pack_timer_ = nh_.createTimer(ros::Duration(0.01), &MasterArmDataPackager::packAndSendCb, this);
  }

 private:
  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_lock_);
    pose_ = poseMsgTo6(msg->pose);
  }

  void velCb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_lock_);
    velocity_ = {msg->twist.linear.x,  msg->twist.linear.y,  msg->twist.linear.z,
                 msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z};
  }

  void wrenchCb(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_lock_);
    wrench_ = {msg->wrench.force.x,  msg->wrench.force.y,  msg->wrench.force.z,
               msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z};
  }

  void packAndSendCb(const ros::TimerEvent&) {
    teleop_msgs::TeleopFrame frame;
    frame.header.stamp = ros::Time::now();
    frame.seq_num = ++seq_counter_;
    {
      std::lock_guard<std::mutex> lock(state_lock_);
      std::copy(pose_.begin(), pose_.end(), frame.pose.begin());
      std::copy(velocity_.begin(), velocity_.end(), frame.velocity.begin());
      std::copy(wrench_.begin(), wrench_.end(), frame.wrench.begin());
    }
    net_pub_.publish(frame);
  }

  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber vel_sub_;
  ros::Subscriber wrench_sub_;
  ros::Publisher net_pub_;
  ros::Timer pack_timer_;

  std::mutex state_lock_;
  Pose6 pose_{};
  std::array<double, 6> velocity_{};
  std::array<double, 6> wrench_{};
  uint32_t seq_counter_ = 0;
};

class SlaveArmDataPackager {
 public:
  void start() {
    pose_sub_ = nh_.subscribe("/slave/tool_pose", 10, &SlaveArmDataPackager::poseCb, this);
    vel_sub_ = nh_.subscribe("/slave/tool_velocity", 10, &SlaveArmDataPackager::velCb, this);
    wrench_sub_ = nh_.subscribe("/slave/wrench", 10, &SlaveArmDataPackager::wrenchCb, this);
    net_pub_ = nh_.advertise<teleop_msgs::TeleopFrame>("/network/slave_frame", 50);
    pack_timer_ = nh_.createTimer(ros::Duration(0.01), &SlaveArmDataPackager::packAndSendCb, this);
  }

 private:
  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_lock_);
    pose_ = poseMsgTo6(msg->pose);
  }

  void velCb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_lock_);
    velocity_ = {msg->twist.linear.x,  msg->twist.linear.y,  msg->twist.linear.z,
                 msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z};
  }

  void wrenchCb(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_lock_);
    wrench_ = {msg->wrench.force.x,  msg->wrench.force.y,  msg->wrench.force.z,
               msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z};
  }

  void packAndSendCb(const ros::TimerEvent&) {
    teleop_msgs::TeleopFrame frame;
    frame.header.stamp = ros::Time::now();
    frame.seq_num = ++seq_counter_;
    {
      std::lock_guard<std::mutex> lock(state_lock_);
      std::copy(pose_.begin(), pose_.end(), frame.pose.begin());
      std::copy(velocity_.begin(), velocity_.end(), frame.velocity.begin());
      std::copy(wrench_.begin(), wrench_.end(), frame.wrench.begin());
    }
    net_pub_.publish(frame);
  }

  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber vel_sub_;
  ros::Subscriber wrench_sub_;
  ros::Publisher net_pub_;
  ros::Timer pack_timer_;

  std::mutex state_lock_;
  Pose6 pose_{};
  std::array<double, 6> velocity_{};
  std::array<double, 6> wrench_{};
  uint32_t seq_counter_ = 0;
};

class JitterBufferNode {
 public:
  JitterBufferNode(const std::string& sub_topic, const std::string& pub_topic) {
    ros::NodeHandle pnh("~");
    paramWithLegacyNamespace(pnh, "buffer_depth", target_depth_);
    pub_ = nh_.advertise<teleop_msgs::TeleopFrame>(pub_topic, 50);
    sub_ = nh_.subscribe(sub_topic, 50, &JitterBufferNode::networkCb, this);
    play_timer_ = nh_.createTimer(ros::Duration(0.01), &JitterBufferNode::playoutCb, this);
  }

 private:
  struct SeqFrame {
    uint32_t seq;
    teleop_msgs::TeleopFrame frame;
    bool operator>(const SeqFrame& o) const { return seq > o.seq; }
  };

  void networkCb(const teleop_msgs::TeleopFrame::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(lock_);
    if (msg->seq_num <= last_seq_) {
      return;
    }
    heap_.push(SeqFrame{msg->seq_num, *msg});
    if (!is_playing_ && static_cast<int>(heap_.size()) >= target_depth_) {
      is_playing_ = true;
      ROS_INFO("Buffer filled. Starting playout.");
    }
  }

  void playoutCb(const ros::TimerEvent&) {
    teleop_msgs::TeleopFrame output;
    {
      std::lock_guard<std::mutex> lock(lock_);
      if (!is_playing_ || heap_.empty()) {
        output = last_valid_frame_;
      } else {
        output = heap_.top().frame;
        last_seq_ = heap_.top().seq;
        heap_.pop();
        last_valid_frame_ = output;
      }
    }
    if (output.seq_num > 0) {
      pub_.publish(output);
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Timer play_timer_;

  std::mutex lock_;
  std::priority_queue<SeqFrame, std::vector<SeqFrame>, std::greater<SeqFrame>> heap_;
  bool is_playing_ = false;
  int target_depth_ = 30;
  uint32_t last_seq_ = 0;
  teleop_msgs::TeleopFrame last_valid_frame_;
};

}  // namespace

int main(int argc, char** argv) {
  std::string role = roleFromArgs(argc, argv);
  const std::string node_name = nodeNameForRole(role);
  ros::init(argc, argv, node_name);
  ros::NodeHandle pnh("~");

  if (role.empty() && !pnh.getParam("role", role)) {
    ROS_ERROR("Set _role:=master or _role:=slave (e.g. rosrun teleop_arm communication _role:=master)");
    return 1;
  }

  if (role == "master") {
    ROS_INFO("%s: role=master (packager + slave-frame buffer)", node_name.c_str());
    MasterArmDataPackager packager;
    packager.start();
    JitterBufferNode buffer("/network/slave_frame", "/master/teleop_frame");
    ros::spin();
  } else if (role == "slave") {
    ROS_INFO("%s: role=slave (packager + master-frame buffer)", node_name.c_str());
    SlaveArmDataPackager packager;
    packager.start();
    JitterBufferNode buffer("/network/master_frame", "/slave/teleop_frame");
    ros::spin();
  } else {
    ROS_ERROR("Invalid role '%s' (use master or slave)", role.c_str());
    return 1;
  }
  return 0;
}
