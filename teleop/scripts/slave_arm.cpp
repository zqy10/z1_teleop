/**
 * Slave Z1 teleop arm — C++ / unitree_arm_sdk
 *
 *   rosrun teleop_arm slave_arm
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <teleop_msgs/TeleopFrame.h>
#include <tf2/LinearMath/Quaternion.h>

#include "unitree_arm_sdk/control/unitreeArm.h"
#include "unitree_arm_sdk/math/mathTypes.h"

// =============================================================================
// ★★★ 从端配置区域 — 实机部署时在此修改 ★★★
// =============================================================================

// 从端 SDK 连接 z1_controller_111 的 IP（本机 controller 一般为 127.0.0.1）
#define SLAVE_ARM_IP "127.0.0.1"

// UDP 端口（与 z1_controller_111 ARMSDK 8074/8073 配对；与主端 8071/8072 不同）
#define SLAVE_LOCAL_PORT 8073
#define SLAVE_REMOTE_PORT 8074

#define DEFAULT_CONTROL_DT 0.02
#define DEFAULT_MASS 1.0
#define DEFAULT_DAMPING 20.0
#define DEFAULT_STIFFNESS 7.5

#define CARTESIAN_ORI_SPEED 1.0
#define CARTESIAN_POS_SPEED 0.8
#define CARTESIAN_DIR_GAIN 10

#define INIT_WAIT_SEC_DEFAULT 10.0
#define TELEOP_RUN_DURATION_SEC_DEFAULT 30.0

// =============================================================================

using namespace UNITREE_ARM;

namespace {

enum class TeleopPhase { INIT_WAIT, TELEOP, DONE };

using Pose6 = std::array<double, 6>;
using Wrench6 = std::array<double, 6>;

static Vec6 pose6ToPosture(const Pose6& p) {
  Vec6 out;
  out << p[3], p[4], p[5], p[0], p[1], p[2];
  return out;
}

static Pose6 postureToPose6(const Vec6& posture) {
  return {posture[3], posture[4], posture[5], posture[0], posture[1], posture[2]};
}

static void eulerToQuaternion(double roll, double pitch, double yaw, geometry_msgs::Quaternion& q) {
  tf2::Quaternion tq;
  tq.setRPY(roll, pitch, yaw);
  q.x = tq.x();
  q.y = tq.y();
  q.z = tq.z();
  q.w = tq.w();
}

static double clampd(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

// Reflect a 6-vector [x, y, z, rx, ry, rz] across the X–Z plane (negate Y).
// Maps the master's base frame to the slave's when the two arms are placed
// back-to-back, so the slave copies the master's motion as a mirror image.
static Pose6 mirrorXZ(const Pose6& v) {
  return {v[0], -v[1], v[2], -v[3], v[4], -v[5]};
}

static CtrlComponents* createCtrlComponents() {
  auto* ctrl = new CtrlComponents();
  ctrl->dt = 0.002;
  ctrl->udp = new UDPPort(SLAVE_ARM_IP, SLAVE_LOCAL_PORT, SLAVE_REMOTE_PORT, RECVSTATE_LENGTH, BlockYN::NO, 500000);
  ctrl->armModel = new Z1Model();
  ctrl->armModel->addLoad(0.03);
  return ctrl;
}

class ButterworthLowPass6 {
 public:
  void init(double cutoff_hz, double sample_hz) {
    const double nyq = 0.5 * sample_hz;
    const double norm = cutoff_hz / nyq;
    const double ita = 1.0 / tan(M_PI * norm);
    const double q = sqrt(2.0);
    b0_ = 1.0 / (1.0 + q * ita + ita * ita);
    b1_ = 2.0 * b0_;
    b2_ = b0_;
    a1_ = 2.0 * (ita * ita - 1.0) * b0_;
    a2_ = -(1.0 - q * ita + ita * ita) * b0_;
    initialized_ = true;
  }

  Eigen::Matrix<double, 6, 1> filter(const Eigen::Matrix<double, 6, 1>& x) {
    if (!initialized_) {
      return x;
    }
    Eigen::Matrix<double, 6, 1> y;
    for (int i = 0; i < 6; ++i) {
      const double in = x[i];
      const double out = b0_ * in + b1_ * x1_[i] + b2_ * x2_[i] + a1_ * y1_[i] + a2_ * y2_[i];
      x2_[i] = x1_[i];
      x1_[i] = in;
      y2_[i] = y1_[i];
      y1_[i] = out;
      y[i] = out;
    }
    return y;
  }

 private:
  bool initialized_ = false;
  double b0_ = 1, b1_ = 0, b2_ = 0, a1_ = 0, a2_ = 0;
  std::array<double, 6> x1_{}, x2_{}, y1_{}, y2_{};
};

class AdmittanceController {
 public:
  AdmittanceController()
      : AdmittanceController(DEFAULT_MASS, DEFAULT_DAMPING, DEFAULT_STIFFNESS, DEFAULT_CONTROL_DT) {}

  AdmittanceController(double mass, double damping, double stiffness, double dt) : M_(mass), B_(damping), K_(stiffness), dt_(dt) {
    M_inv_ = 1.0 / std::max(M_, 1e-6);
    filter_.init(10.0, 1.0 / dt);
  }

  Pose6 computeTargetPose(const Pose6& desired, const Wrench6& force) {
    Eigen::Matrix<double, 6, 1> f;
    for (int i = 0; i < 6; ++i) {
      f[i] = force[i];
    }
    f = filter_.filter(f);
    Eigen::Matrix<double, 6, 1> corr_force = f - B_ * dX_dot_ - K_ * dX_;
    dX_ddot_ = M_inv_ * corr_force;
    dX_dot_ += dX_ddot_ * dt_;
    dX_ += dX_dot_ * dt_;
    for (int i = 0; i < 6; ++i) {
      dX_[i] = clampd(dX_[i], -max_correction_, max_correction_);
      dX_dot_[i] = clampd(dX_dot_[i], -max_velocity_, max_velocity_);
    }
    Pose6 out = desired;
    for (int i = 0; i < 6; ++i) {
      out[i] = desired[i] + dX_[i];
    }
    // Debug logging change: throttle target-pose logging so real pose/force logs stay readable.
    ROS_INFO_THROTTLE(1.0, "admittance: target pose6 [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", out[0],
                      out[1], out[2], out[3], out[4], out[5]);
    return out;
  }

 private:
  double M_, B_, K_, dt_;
  double M_inv_ = 1.0;
  double max_correction_ = 0.2;
  double max_velocity_ = 0.4;
  Eigen::Matrix<double, 6, 1> dX_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> dX_dot_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> dX_ddot_ = Eigen::Matrix<double, 6, 1>::Zero();
  ButterworthLowPass6 filter_;
};

class ForceSensorTransform {
 public:
  Wrench6 transform(const Wrench6& w) const {
    return {w[2], w[0], w[1], w[5], w[3], w[4]};
  }
};

class WorkspaceLimiter {
 public:
  WorkspaceLimiter()
      : WorkspaceLimiter(Pose6{0.22, -0.42, 0.08, 0, 0, 0}, Pose6{0.78, 0.42, 0.62, 0, 0, 0}, 1.35, true) {}

  WorkspaceLimiter(const Pose6& pos_min, const Pose6& pos_max, double rpy_abs_max, bool enabled)
      : pos_min_(pos_min), pos_max_(pos_max), rpy_abs_max_(rpy_abs_max), enabled_(enabled) {}

  Pose6 clamp(const Pose6& pose) const {
    if (!enabled_) {
      return pose;
    }
    Pose6 out = pose;
    for (int i = 0; i < 3; ++i) {
      out[i] = clampd(out[i], pos_min_[i], pos_max_[i]);
    }
    if (rpy_abs_max_ > 0) {
      for (int i = 3; i < 6; ++i) {
        out[i] = clampd(out[i], -rpy_abs_max_, rpy_abs_max_);
      }
    }
    return out;
  }

 private:
  Pose6 pos_min_, pos_max_;
  double rpy_abs_max_ = 1.35;
  bool enabled_ = true;
};

class SlaveZ1Session {
 public:
  SlaveZ1Session() {
    ctrl_ = createCtrlComponents();
    arm_ = new unitreeArm(ctrl_);
    arm_->sendRecvThread->start();
    ROS_INFO("slave: Z1 SDK UDP %s  local=%u  remote=%u", SLAVE_ARM_IP, SLAVE_LOCAL_PORT, SLAVE_REMOTE_PORT);
  }

  ~SlaveZ1Session() {
    if (arm_) {
      arm_->sendRecvThread->shutdown();
      delete arm_;
      arm_ = nullptr;
    }
    if (ctrl_) {
      delete ctrl_;
      ctrl_ = nullptr;
    }
  }

  unitreeArm* arm() { return arm_; }

  void moveToInitJointPose() {
    // Debug logging change: make the temporary init control mode explicit.
    ROS_INFO("slave: control mode JOINTCTRL for init pose only [0, 1.5, -1, -0.54, 0, 0]");
    arm_->startTrack(ArmFSMState::JOINTCTRL);

    const Vec6 last_pos = arm_->lowstate->getQ();
    Vec6 target_pos;
    target_pos << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0;

    constexpr int duration = 1000;
    Timer timer(arm_->_ctrlComp->dt);
    for (int i = 0; i < duration; ++i) {
      const double alpha = static_cast<double>(i) / duration;
      arm_->q = last_pos * (1.0 - alpha) + target_pos * alpha;
      arm_->qd = (target_pos - last_pos) / (duration * arm_->_ctrlComp->dt);
      arm_->setArmCmd(arm_->q, arm_->qd);
      timer.sleep();
    }
  }

  void moveToVerticalHome() {
    ROS_INFO("slave: backToStart (vertical home)");
    arm_->backToStart();
  }

  void startCartesianTrack() {
    // Debug logging change: make the teleop control mode explicit.
    ROS_INFO("slave: control mode CARTESIAN active; command=cartesianCtrlCmd");
    arm_->startTrack(ArmFSMState::CARTESIAN);
  }

  void commandCartesianToward(const Pose6& target_pose6) {
    const Vec6 target = pose6ToPosture(target_pose6);
    const Vec6 current = arm_->lowstate->endPosture;
    Vec7 directions = Vec7::Zero();
    for (int i = 0; i < 6; ++i) {
      directions[i] = clampd((target[i] - current[i]) * CARTESIAN_DIR_GAIN, -1.0, 1.0);
    }
    arm_->cartesianCtrlCmd(directions, CARTESIAN_ORI_SPEED, CARTESIAN_POS_SPEED);
  }

  Pose6 currentPose6() const { return postureToPose6(arm_->lowstate->endPosture); }

 private:
  CtrlComponents* ctrl_ = nullptr;
  unitreeArm* arm_ = nullptr;
};

class SlaveTeleopArm {
 public:
  SlaveTeleopArm() {
    ros::NodeHandle pnh("~");
    pnh.param("enable_force_feedback", enable_force_feedback_, true);
    pnh.param("init_wait_sec", init_wait_sec_, INIT_WAIT_SEC_DEFAULT);
    pnh.param("run_duration_sec", run_duration_sec_, TELEOP_RUN_DURATION_SEC_DEFAULT);
    // Debug logging change: set to 0 to disable detailed terminal debug output.
    pnh.param("debug_log_hz", debug_log_hz_, 2.0);

    double mass = DEFAULT_MASS, damping = DEFAULT_DAMPING, stiffness = DEFAULT_STIFFNESS;
    pnh.param("mass", mass, mass);
    pnh.param("damping", damping, damping);
    pnh.param("stiffness", stiffness, stiffness);
    pnh.param("control_dt", dt_, DEFAULT_CONTROL_DT);
    admittance_ = AdmittanceController(mass, damping, stiffness, dt_);

    bool ws_en = true;
    pnh.param("workspace_limit_enable", ws_en, ws_en);
    std::vector<double> pmin_v{0.22, -0.42, 0.08}, pmax_v{0.78, 0.42, 0.62};
    pnh.param("workspace_pos_min", pmin_v, pmin_v);
    pnh.param("workspace_pos_max", pmax_v, pmax_v);
    double rpy_max = 1.35;
    pnh.param("workspace_rpy_abs_max", rpy_max, rpy_max);
    Pose6 pmin{}, pmax{};
    for (int i = 0; i < 3; ++i) {
      pmin[i] = pmin_v[i];
      pmax[i] = pmax_v[i];
    }
    limiter_ = WorkspaceLimiter(pmin, pmax, rpy_max, ws_en);

    desired_pub_ = nh_.advertise<geometry_msgs::Pose>("/slave/target_pose", 10);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/slave/tool_pose", 10);
    vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/slave/tool_velocity", 10);
    wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/slave/wrench", 10);
    teleop_sub_ = nh_.subscribe("/slave/teleop_frame", 10, &SlaveTeleopArm::teleopCb, this);

    std::string force_topic = "/slave_force";
    pnh.param("force_sensor_topic", force_topic, force_topic);
    force_sub_ = nh_.subscribe(force_topic, 10, &SlaveTeleopArm::forceCb, this);
    ROS_INFO("slave: force sensor topic %s", force_topic.c_str());

    hw_ = std::make_unique<SlaveZ1Session>();
    hw_->moveToInitJointPose();
    init_pose_ = hw_->currentPose6();
    current_pose_ = init_pose_;
    hw_->startCartesianTrack();
    state_thread_ = std::thread(&SlaveTeleopArm::stateLoop, this);

    phase_ = TeleopPhase::INIT_WAIT;
    init_wait_start_ = ros::Time::now();
    rate_hz_ = static_cast<int>(1.0 / dt_);

    ROS_INFO("slave: at init joint pose; CARTESIAN hold for %.0fs then admittance %.0fs", init_wait_sec_,
             run_duration_sec_);
  }

  ~SlaveTeleopArm() {
    shutdown_ = true;
    if (state_thread_.joinable()) {
      state_thread_.join();
    }
  }

  void finishAndExit() {
    if (phase_ == TeleopPhase::DONE) {
      return;
    }
    phase_ = TeleopPhase::DONE;
    ROS_INFO("slave: session end — backToStart and PASSIVE; CARTESIAN teleop stopped");
    if (hw_) {
      hw_->moveToVerticalHome();
      hw_->arm()->setFsm(ArmFSMState::PASSIVE);
    }
    shutdown_ = true;
    ros::requestShutdown();
  }

  void run() {
    geometry_msgs::Pose out_msg;
    ros::Rate rate(rate_hz_);
    int last_wait_log = -1;

    while (ros::ok() && !shutdown_) {
      Pose6 ref;
      Pose6 target;

      if (phase_ == TeleopPhase::INIT_WAIT) {
        const double elapsed = (ros::Time::now() - init_wait_start_).toSec();
        const int wait_left = static_cast<int>(init_wait_sec_ - elapsed);
        if (wait_left != last_wait_log && wait_left >= 0) {
          ROS_INFO_THROTTLE(5.0, "slave: init hold, admittance in %ds", wait_left);
          last_wait_log = wait_left;
        }
        if (elapsed >= init_wait_sec_) {
          finalizeForceZero();
          phase_ = TeleopPhase::TELEOP;
          teleop_start_ = ros::Time::now();
          ROS_INFO("slave: CARTESIAN admittance teleop started (%.0fs)", run_duration_sec_);
        } else {
          accumulateForceZeroSample();
        }
        ref = init_pose_;
        target = init_pose_;
      } else {
        if ((ros::Time::now() - teleop_start_).toSec() >= run_duration_sec_) {
          finishAndExit();
          break;
        }
        if (!hasTeleopReference()) {
          rate.sleep();
          ros::spinOnce();
          continue;
        }
        ref = referencePose();
        target = limiter_.clamp(admittance_.computeTargetPose(ref, admittanceForce()));
      }

      publishPose6(out_msg, target);
      desired_pub_.publish(out_msg);
      hw_->commandCartesianToward(target);

      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  void forceCb(const geometry_msgs::Wrench::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(wrench_lock_);
    latest_wrench_ = *msg;
    has_wrench_ = true;
  }

  void teleopCb(const teleop_msgs::TeleopFrame::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(teleop_lock_);
    teleop_frame_ = *msg;
    has_teleop_frame_ = true;
  }

  static Wrench6 wrenchTo6(const geometry_msgs::Wrench& w) {
    return {w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z};
  }

  static geometry_msgs::Wrench wrenchFrom6(const Wrench6& w) {
    geometry_msgs::Wrench msg;
    msg.force.x = w[0];
    msg.force.y = w[1];
    msg.force.z = w[2];
    msg.torque.x = w[3];
    msg.torque.y = w[4];
    msg.torque.z = w[5];
    return msg;
  }

  void accumulateForceZeroSample() {
    std::lock_guard<std::mutex> lock(wrench_lock_);
    if (!has_wrench_) {
      return;
    }
    const Wrench6 raw = wrenchTo6(latest_wrench_);
    for (int i = 0; i < 6; ++i) {
      force_zero_sum_[i] += raw[i];
    }
    ++force_zero_samples_;
  }

  void finalizeForceZero() {
    std::lock_guard<std::mutex> lock(wrench_lock_);
    if (force_zeroed_) {
      return;
    }
    if (force_zero_samples_ == 0) {
      ROS_WARN("slave: no force sensor samples during init; force zero offset remains 0");
      force_zeroed_ = true;
      return;
    }
    for (int i = 0; i < 6; ++i) {
      force_zero_offset_[i] = force_zero_sum_[i] / static_cast<double>(force_zero_samples_);
    }
    force_zeroed_ = true;
    ROS_INFO("slave: force zero offset from %zu samples: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
             force_zero_samples_, force_zero_offset_[0], force_zero_offset_[1], force_zero_offset_[2],
             force_zero_offset_[3], force_zero_offset_[4], force_zero_offset_[5]);
  }

  Wrench6 calibratedLocalWrenchWorld() const {
    Wrench6 calibrated{};
    {
      std::lock_guard<std::mutex> lock(wrench_lock_);
      const Wrench6 raw = wrenchTo6(latest_wrench_);
      for (int i = 0; i < 6; ++i) {
        calibrated[i] = raw[i] - force_zero_offset_[i];
      }
    }
    return force_transform_.transform(calibrated);
  }

  double debugLogPeriod() const {
    if (debug_log_hz_ <= 0.0) {
      return 0.0;
    }
    return 1.0 / debug_log_hz_;
  }

  void logDebugState(const Pose6& pose, const geometry_msgs::TwistStamped& vel, const Wrench6& wrench) const {
    const double period = debugLogPeriod();
    if (period <= 0.0) {
      return;
    }
    // Debug logging change: print real Cartesian state from lowstate->endPosture and computed velocity/wrench.
    ROS_INFO_THROTTLE(period,
                      "slave debug: real_pose(base_link) [x y z r p y]=[%.3f %.3f %.3f %.3f %.3f %.3f]",
                      pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    ROS_INFO_THROTTLE(period,
                      "slave debug: cal_vel(base_link) linear=[%.3f %.3f %.3f] angular_rpy=[%.3f %.3f %.3f]",
                      vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z, vel.twist.angular.x,
                      vel.twist.angular.y, vel.twist.angular.z);
    ROS_INFO_THROTTLE(period,
                      "slave debug: real_wrench(base_link zeroed/world) force=[%.3f %.3f %.3f] torque=[%.3f %.3f %.3f] force_feedback=%s",
                      wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5],
                      enable_force_feedback_ ? "true" : "false");
  }

  Wrench6 admittanceForce() const {
    const Wrench6 wl = calibratedLocalWrenchWorld();
    std::lock_guard<std::mutex> lock(teleop_lock_);
    if (!enable_force_feedback_ || !has_teleop_frame_) {
      const double period = debugLogPeriod();
      if (period > 0.0) {
        // Debug logging change: always show local force entering admittance, even without force feedback.
        ROS_INFO_THROTTLE(period,
                          "slave debug: admittance_force input=[%.3f %.3f %.3f %.3f %.3f %.3f] force_feedback=%s remote_frame=%s",
                          wl[0], wl[1], wl[2], wl[3], wl[4], wl[5], enable_force_feedback_ ? "true" : "false",
                          has_teleop_frame_ ? "true" : "false");
      }
      return wl;
    }
    Wrench6 wr{};
    for (int i = 0; i < 6; ++i) {
      wr[i] = teleop_frame_.wrench[i];
    }
    const Wrench6 fr = wr; //force_transform_.transform(wr);
    Wrench6 out{};
    for (int i = 0; i < 6; ++i) {
      out[i] = fr[i] + wl[i];
      out[i] = out[i] * 4.5;
    }
    const double period = debugLogPeriod();
    if (period > 0.0) {
      // Debug logging change: show local, remote, and final force when bilateral feedback is active.
      ROS_INFO_THROTTLE(period,
                        "slave debug: admittance_force local=[%.3f %.3f %.3f %.3f %.3f %.3f] remote=[%.3f %.3f %.3f %.3f %.3f %.3f] input=[%.3f %.3f %.3f %.3f %.3f %.3f]",
                        wl[0], wl[1], wl[2], wl[3], wl[4], wl[5], fr[0], fr[1], fr[2], fr[3], fr[4], fr[5],
                        out[0], out[1], out[2], out[3], out[4], out[5]);
    }
    return out;
  }

  Pose6 referencePose() const {
    std::lock_guard<std::mutex> lock(teleop_lock_);
    if (has_teleop_frame_) {
      Pose6 p{};
      for (int i = 0; i < 6; ++i) {
        p[i] = teleop_frame_.pose[i];
      }
      return mirrorXZ(p);
    }
    return current_pose_;
  }

  bool hasTeleopReference() const {
    std::lock_guard<std::mutex> lock(teleop_lock_);
    return has_teleop_frame_;
  }

  void publishPose6(geometry_msgs::Pose& msg, const Pose6& pose6) {
    msg.position.x = pose6[0];
    msg.position.y = pose6[1];
    msg.position.z = pose6[2];
    eulerToQuaternion(pose6[3], pose6[4], pose6[5], msg.orientation);
  }

  void stateLoop() {
    ros::NodeHandle pnh("~");
    int hz = 100;
    pnh.param("state_pub_hz", hz, hz);
    ros::Rate r(hz);

    Pose6 last_pose{};
    bool have_last = false;
    ros::Time last_time = ros::Time::now();

    while (ros::ok() && !shutdown_) {
      current_pose_ = hw_->currentPose6();
      const ros::Time now = ros::Time::now();

      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.stamp = now;
      pose_msg.header.frame_id = "base_link";
      publishPose6(pose_msg.pose, current_pose_);
      pose_pub_.publish(pose_msg);

      geometry_msgs::TwistStamped vel_msg;
      vel_msg.header.stamp = now;
      vel_msg.header.frame_id = "base_link";
      if (have_last) {
        const double dt = (now - last_time).toSec();
        if (dt > 0) {
          vel_msg.twist.linear.x = (current_pose_[0] - last_pose[0]) / dt;
          vel_msg.twist.linear.y = (current_pose_[1] - last_pose[1]) / dt;
          vel_msg.twist.linear.z = (current_pose_[2] - last_pose[2]) / dt;
          // Debug logging change: publish and print RPY angular velocity computed from Cartesian pose deltas.
          vel_msg.twist.angular.x = (current_pose_[3] - last_pose[3]) / dt;
          vel_msg.twist.angular.y = (current_pose_[4] - last_pose[4]) / dt;
          vel_msg.twist.angular.z = (current_pose_[5] - last_pose[5]) / dt;
        }
      }
      last_pose = current_pose_;
      last_time = now;
      have_last = true;
      vel_pub_.publish(vel_msg);

      const Wrench6 calibrated_wrench = calibratedLocalWrenchWorld();
      geometry_msgs::WrenchStamped wrench_msg;
      wrench_msg.header.stamp = now;
      wrench_msg.header.frame_id = "base_link";
      wrench_msg.wrench = wrenchFrom6(calibrated_wrench);
      wrench_pub_.publish(wrench_msg);
      logDebugState(current_pose_, vel_msg, calibrated_wrench);

      r.sleep();
    }
  }

  bool enable_force_feedback_ = true;
  double dt_ = DEFAULT_CONTROL_DT;
  double init_wait_sec_ = INIT_WAIT_SEC_DEFAULT;
  double run_duration_sec_ = TELEOP_RUN_DURATION_SEC_DEFAULT;
  double debug_log_hz_ = 2.0;
  int rate_hz_ = 50;

  TeleopPhase phase_ = TeleopPhase::INIT_WAIT;
  ros::Time init_wait_start_;
  ros::Time teleop_start_;
  Pose6 init_pose_{};
  Pose6 current_pose_{};

  AdmittanceController admittance_;
  ForceSensorTransform force_transform_;
  WorkspaceLimiter limiter_;

  ros::NodeHandle nh_;
  ros::Publisher desired_pub_, pose_pub_, vel_pub_, wrench_pub_;
  ros::Subscriber teleop_sub_, force_sub_;

  mutable std::mutex teleop_lock_;
  teleop_msgs::TeleopFrame teleop_frame_;
  bool has_teleop_frame_ = false;
  mutable std::mutex wrench_lock_;
  geometry_msgs::Wrench latest_wrench_;
  bool has_wrench_ = false;
  Wrench6 force_zero_sum_{};
  Wrench6 force_zero_offset_{};
  size_t force_zero_samples_ = 0;
  bool force_zeroed_ = false;

  std::unique_ptr<SlaveZ1Session> hw_;
  std::thread state_thread_;
  std::atomic<bool> shutdown_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "slave_arm");
  try {
    SlaveTeleopArm node;
    node.run();
  } catch (const std::exception& e) {
    ROS_FATAL("slave_arm: %s", e.what());
    return 1;
  }
  return 0;
}
