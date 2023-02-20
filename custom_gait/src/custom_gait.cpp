#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "gaitlib/gaits.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;


enum State{WAIT,
           STANDUP,
           WALK,
           STANDSTILL};

class CustomGait : public rclcpp::Node
{
public:
  CustomGait()
  : Node("custom_gait")
  {
    declare_parameter("rate", 200.0);
    double rate_hz = get_parameter("rate").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Rate is " << ((int)(1000. / rate_hz)) << "ms");
    std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));
    
    declare_parameter("seconds_per_swing", 2.0);
    double seconds_per_swing = get_parameter("seconds_per_swing").as_double();
    RCLCPP_INFO_STREAM(get_logger(), seconds_per_swing<<" seconds per swing");

    period = rate_hz * seconds_per_swing;
    RCLCPP_INFO_STREAM(get_logger(), period<<" publishes per swing");

    declare_parameter("stroke_length", 0.2);
    stroke_length = get_parameter("stroke_length").as_double();
    RCLCPP_INFO_STREAM(get_logger(), stroke_length<<"m stroke length");

    declare_parameter("stiffness", 5.0); // kp
    stiffness = get_parameter("stiffness").as_double();

    declare_parameter("damping", 1.0); // kd
    damping = get_parameter("damping").as_double();

    // declare_parameter("torque", 1.0); // tau

    // delay for 3 seconds before beginning movement
    delay = rate_hz * 3.0;

    cmd_pub_ = create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 10);

    state_sub_ = create_subscription<ros2_unitree_legged_msgs::msg::LowState>(
      "low_state", 10,
      std::bind(&CustomGait::state_cb, this, std::placeholders::_1));

    switch_gait_ = create_service<std_srvs::srv::Empty>(
      "switch",
      std::bind(&CustomGait::switch_gait, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
      rate,
      std::bind(&CustomGait::timer_callback, this));

    // initialize low_cmd fields
    low_cmd.head[0] = 0xFE;
    low_cmd.head[1] = 0xEF;
    low_cmd.level_flag = 0xFF;   // LOWLEVEL;
    for (int i = 0; i < 12; i++) {
      low_cmd.motor_cmd[i].mode = 0x0A;    // motor switch to servo (PMSM) mode
      low_cmd.motor_cmd[i].q = (2.146E+9f);   // PosStopF; // 禁止位置环
      low_cmd.motor_cmd[i].kp = 0;
      low_cmd.motor_cmd[i].dq = (16000.0f);   // VelStopF; // 禁止速度环
      low_cmd.motor_cmd[i].kd = 0;
      low_cmd.motor_cmd[i].tau = 0;
    }

    const auto lspan = 0.5 * stroke_length;   // half of "stroke length", ie how long it's on the floor
    const auto dl = 0.025;   // extra bit to extend by after leaving floor
    const auto ddl = 0.025;   // another extra bit to extend by LOL
    stand_y = -1.5 * gaitlib::LEG_LENGTH; // y distance when the foot is on the floor.
    const auto swing_height = 0.05;   // ???
    const auto dswing_height = 0.025;   // ??
    ctrl_x = {-1.0 * lspan,
      -1.0 * lspan - dl,
      -1.0 * lspan - dl - ddl,
      -1.0 * lspan - dl - ddl,
      -1.0 * lspan - dl - ddl,
      0.0,
      0.0,
      0.0,
      lspan + dl + ddl,
      lspan + dl + ddl,
      lspan + dl,
      lspan};
    ctrl_y = {stand_y,
      stand_y,
      stand_y + swing_height,
      stand_y + swing_height,
      stand_y + swing_height,
      stand_y + swing_height,
      stand_y + swing_height,
      stand_y + swing_height + dswing_height,
      stand_y + swing_height + dswing_height,
      stand_y + swing_height + dswing_height,
      stand_y,
      stand_y};

    long npoints_bezier = period*0.75;
    long npoints_sinusoid = period-npoints_bezier;
    std::vector<double> bez_x = gaitlib::bezier(ctrl_x, npoints_bezier);
    std::vector<double> bez_y = gaitlib::bezier(ctrl_y, npoints_bezier);
    RCLCPP_INFO_STREAM(get_logger(), "Size of bez_x: " << bez_x.size());
    std::vector<double> sin_x = gaitlib::linspace(ctrl_x.back(), ctrl_x.at(0), npoints_sinusoid);
    std::vector<double> sin_y = gaitlib::stance(sin_x, 0.05, ctrl_y.at(0));

    const auto final_x = gaitlib::concatenate(bez_x, sin_x);
    const auto final_y = gaitlib::concatenate(bez_y, sin_y);

    const auto fr_gaits = gaitlib::make_gait(final_x, final_y);
    fr_calf = fr_gaits.gait_calf;
    fr_thigh = fr_gaits.gait_thigh;
    // Next: MODULATE based on fr
    fl_calf = gaitlib::modulate(fr_calf, 0.5);
    fl_thigh = gaitlib::modulate(fr_thigh, 0.5);

    rr_calf = gaitlib::modulate(fr_calf, 0.5);
    rr_thigh = gaitlib::modulate(fr_thigh, 0.5);

    rl_calf = gaitlib::modulate(fr_calf, 0.0);
    rl_thigh = gaitlib::modulate(fr_thigh, 0.0);

    // create a standing up movement. Should be defined by # of seconds probably
    // the first thing the real robot does upon startup is angle its hips inwards.
    // it ends at y = some small negative #, x = 0. 
    // THEN go from (y = some small negative #, 0) to (stand_y, 0).
    // unsure if this should be in straight line
    double stand_offset = -0.1;

    long standup_pts1 = rate_hz * 2; 
    const auto stand_up_x1 = gaitlib::linspace(0.0, 0.0, standup_pts1);
    const auto stand_up_y1 = gaitlib::linspace(stand_offset, stand_offset, standup_pts1);

    long standup_pts2 = rate_hz * 3;
    const auto stand_up_x2 = gaitlib::linspace(0.0, 0.0, standup_pts2);
    const auto stand_up_y2 = gaitlib::linspace(stand_offset, stand_y, standup_pts2);

    const auto stand_up_x = gaitlib::concatenate(stand_up_x1, stand_up_x2);
    const auto stand_up_y = gaitlib::concatenate(stand_up_y1, stand_up_y2);

    const auto stand_up_gait = gaitlib::make_gait(stand_up_x, stand_up_y);

    fr_calf_stand  = stand_up_gait.gait_calf;
    fr_thigh_stand = stand_up_gait.gait_thigh;
    fl_calf_stand  = stand_up_gait.gait_calf;
    fl_thigh_stand = stand_up_gait.gait_thigh;
    rr_calf_stand  = stand_up_gait.gait_calf;
    rr_thigh_stand = stand_up_gait.gait_thigh;
    rl_calf_stand  = stand_up_gait.gait_calf;
    rl_thigh_stand = stand_up_gait.gait_thigh;


    const auto standing_joints = gaitlib::ik(0.0, stand_y);
    stand_calf = standing_joints.calf_lefty;
    stand_thigh = standing_joints.thigh_lefty;

    RCLCPP_INFO_STREAM(get_logger(), "Waiting...");
  }

private:
  /// @brief Switch gait type between standing/walking
  /// @param Request: The empty request
  /// @param Response: The empty response
  void switch_gait(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Switch Gait");
    if (state == WALK){
      state = STANDSTILL;
      motiontime = 0;
    } else if (state == STANDSTILL){
      state = WALK;
      motiontime = 0;
    }
  }
  void state_cb(const ros2_unitree_legged_msgs::msg::LowState & msg)
  {
    if (feets.size() == 0) {
      // first iteration
      // This is a really dumb way of doing it but only way I could get to build ;-;
      feets.push_back(msg.foot_force[0]);   // FR
      feets.push_back(msg.foot_force[1]);   // FL
      feets.push_back(msg.foot_force[2]);   // RR
      feets.push_back(msg.foot_force[3]);   // RL
    } else {
      feets[0] = msg.foot_force[0];
      feets[1] = msg.foot_force[1];
      feets[2] = msg.foot_force[2];
      feets[3] = msg.foot_force[3];
    }
    // RCLCPP_INFO_STREAM(get_logger(), "Foot Force:  FR:" << feets[0] << "  FL:" << feets[1] <<
    //  "  RR:" << feets[2] << "  RL:" << feets[3]);
  }
  void timer_callback()
  {
    switch(state){
      case WAIT:
      {
        count ++;
        if (count > delay) {
          RCLCPP_INFO_STREAM(get_logger(), "Stand Up!");
          state = STANDUP;
          // set params that will never change: dq, kp, kd (for PI control)
          low_cmd.motor_cmd[gaitlib::FR_CALF].dq = 0.0;
          low_cmd.motor_cmd[gaitlib::FR_CALF].kp = 5.0;
          low_cmd.motor_cmd[gaitlib::FR_CALF].kd = 1.0;
          low_cmd.motor_cmd[gaitlib::FR_THIGH].dq = 0.0;
          low_cmd.motor_cmd[gaitlib::FR_THIGH].kp = 5.0;
          low_cmd.motor_cmd[gaitlib::FR_THIGH].kd = 1.0;
          low_cmd.motor_cmd[gaitlib::FR_HIP].dq = 0.0;
          low_cmd.motor_cmd[gaitlib::FR_HIP].kp = 5.0;
          low_cmd.motor_cmd[gaitlib::FR_HIP].kd = 1.0;

          low_cmd.motor_cmd[gaitlib::FL_CALF].dq = 0.0;
          low_cmd.motor_cmd[gaitlib::FL_CALF].kp = 5.0;
          low_cmd.motor_cmd[gaitlib::FL_CALF].kd = 1.0;
          low_cmd.motor_cmd[gaitlib::FL_THIGH].dq = 0.0;
          low_cmd.motor_cmd[gaitlib::FL_THIGH].kp = 5.0;
          low_cmd.motor_cmd[gaitlib::FL_THIGH].kd = 1.0;
          low_cmd.motor_cmd[gaitlib::FL_HIP].dq = 0.0;
          low_cmd.motor_cmd[gaitlib::FL_HIP].kp = 5.0;
          low_cmd.motor_cmd[gaitlib::FL_HIP].kd = 1.0;

          low_cmd.motor_cmd[gaitlib::RR_CALF].dq = 0.0;
          low_cmd.motor_cmd[gaitlib::RR_CALF].kp = 5.0;
          low_cmd.motor_cmd[gaitlib::RR_CALF].kd = 1.0;
          low_cmd.motor_cmd[gaitlib::RR_THIGH].dq = 0.0;
          low_cmd.motor_cmd[gaitlib::RR_THIGH].kp = 5.0;
          low_cmd.motor_cmd[gaitlib::RR_THIGH].kd = 1.0;
          low_cmd.motor_cmd[gaitlib::RR_HIP].dq = 0.0;
          low_cmd.motor_cmd[gaitlib::RR_HIP].kp = 5.0;
          low_cmd.motor_cmd[gaitlib::RR_HIP].kd = 1.0;

          low_cmd.motor_cmd[gaitlib::RL_CALF].dq = 0.0;
          low_cmd.motor_cmd[gaitlib::RL_CALF].kp = 5.0;
          low_cmd.motor_cmd[gaitlib::RL_CALF].kd = 1.0;
          low_cmd.motor_cmd[gaitlib::RL_THIGH].dq = 0.0;
          low_cmd.motor_cmd[gaitlib::RL_THIGH].kp = 5.0;
          low_cmd.motor_cmd[gaitlib::RL_THIGH].kd = 1.0;
          low_cmd.motor_cmd[gaitlib::RL_HIP].dq = 0.0;
          low_cmd.motor_cmd[gaitlib::RL_HIP].kp = 5.0;
          low_cmd.motor_cmd[gaitlib::RL_HIP].kd = 1.0;
        }
        break;
      }
      case STANDUP:
      {
        low_cmd.motor_cmd[gaitlib::FR_CALF].q = fr_calf_stand.at(motiontime);
        low_cmd.motor_cmd[gaitlib::FR_CALF].tau = stand_calf_torque;
        low_cmd.motor_cmd[gaitlib::FR_THIGH].q = fr_thigh_stand.at(motiontime);
        low_cmd.motor_cmd[gaitlib::FR_HIP].q = 0.0; 
        low_cmd.motor_cmd[gaitlib::FR_HIP].tau = -1.0*stand_hip_torque;

        low_cmd.motor_cmd[gaitlib::FL_CALF].q = fl_calf_stand.at(motiontime);
        low_cmd.motor_cmd[gaitlib::FL_CALF].tau = stand_calf_torque;
        low_cmd.motor_cmd[gaitlib::FL_THIGH].q = fl_thigh_stand.at(motiontime);
        low_cmd.motor_cmd[gaitlib::FL_HIP].q = 0.0; 
        low_cmd.motor_cmd[gaitlib::FL_HIP].tau = stand_hip_torque;

        low_cmd.motor_cmd[gaitlib::RR_CALF].q = rr_calf_stand.at(motiontime);
        low_cmd.motor_cmd[gaitlib::RR_CALF].tau = stand_calf_torque;
        low_cmd.motor_cmd[gaitlib::RR_THIGH].q = rr_thigh_stand.at(motiontime);
        low_cmd.motor_cmd[gaitlib::RR_HIP].q = 0.0; 
        low_cmd.motor_cmd[gaitlib::RR_HIP].tau = -1.0*stand_hip_torque;

        low_cmd.motor_cmd[gaitlib::RL_CALF].q = rl_calf_stand.at(motiontime);
        low_cmd.motor_cmd[gaitlib::RL_CALF].tau = stand_calf_torque;
        low_cmd.motor_cmd[gaitlib::RL_THIGH].q = rl_thigh_stand.at(motiontime);
        low_cmd.motor_cmd[gaitlib::RL_HIP].q = 0.0; 
        low_cmd.motor_cmd[gaitlib::RL_HIP].tau = stand_hip_torque;

        motiontime += 1;
        // RCLCPP_INFO_STREAM(get_logger(), "Motiontime " << motiontime);
        if (motiontime >= static_cast<long>(fr_calf_stand.size())) {
          RCLCPP_INFO_STREAM(get_logger(), "Standstill!");
          motiontime = 0;
          state = STANDSTILL;
        }
        break;
      }
      case STANDSTILL:
      {
        low_cmd.motor_cmd[gaitlib::FR_CALF].q = stand_calf;
        low_cmd.motor_cmd[gaitlib::FR_CALF].tau = stand_calf_torque;
        low_cmd.motor_cmd[gaitlib::FR_THIGH].q = stand_thigh;
        low_cmd.motor_cmd[gaitlib::FR_HIP].q = 0.0; 
        low_cmd.motor_cmd[gaitlib::FR_HIP].tau = -1.0*stand_hip_torque;

        low_cmd.motor_cmd[gaitlib::FL_CALF].q = stand_calf;
        low_cmd.motor_cmd[gaitlib::FL_CALF].tau = stand_calf_torque;
        low_cmd.motor_cmd[gaitlib::FL_THIGH].q = stand_thigh;
        low_cmd.motor_cmd[gaitlib::FL_HIP].q = 0.0; 
        low_cmd.motor_cmd[gaitlib::FL_HIP].tau = stand_hip_torque;

        low_cmd.motor_cmd[gaitlib::RR_CALF].q = stand_calf;
        low_cmd.motor_cmd[gaitlib::RR_CALF].tau = stand_calf_torque;
        low_cmd.motor_cmd[gaitlib::RR_THIGH].q = stand_thigh;
        low_cmd.motor_cmd[gaitlib::RR_HIP].q = 0.0; 
        low_cmd.motor_cmd[gaitlib::RR_HIP].tau = -1.0*stand_hip_torque;

        low_cmd.motor_cmd[gaitlib::RL_CALF].q = stand_calf;
        low_cmd.motor_cmd[gaitlib::RL_CALF].tau = stand_calf_torque;
        low_cmd.motor_cmd[gaitlib::RL_THIGH].q = stand_thigh;
        low_cmd.motor_cmd[gaitlib::RL_HIP].q = 0.0; 
        low_cmd.motor_cmd[gaitlib::RL_HIP].tau = stand_hip_torque;
        break;
      }
      case WALK:
      {
        low_cmd.motor_cmd[gaitlib::FR_CALF].q = fr_calf.at(motiontime);
        low_cmd.motor_cmd[gaitlib::FR_CALF].tau = 0.0; // TODO
        low_cmd.motor_cmd[gaitlib::FR_THIGH].q = fr_thigh.at(motiontime);
        low_cmd.motor_cmd[gaitlib::FR_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::FL_CALF].q = fl_calf.at(motiontime);
        low_cmd.motor_cmd[gaitlib::FL_CALF].tau = 0.0; // TODO
        low_cmd.motor_cmd[gaitlib::FL_THIGH].q = fl_thigh.at(motiontime);
        low_cmd.motor_cmd[gaitlib::FL_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::RR_CALF].q = rr_calf.at(motiontime);
        low_cmd.motor_cmd[gaitlib::RR_CALF].tau = 0.0; // TODO
        low_cmd.motor_cmd[gaitlib::RR_THIGH].q = rr_thigh.at(motiontime);
        low_cmd.motor_cmd[gaitlib::RR_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::RL_CALF].q = rl_calf.at(motiontime);
        low_cmd.motor_cmd[gaitlib::RL_CALF].tau = 0.0; // TODO
        low_cmd.motor_cmd[gaitlib::RL_THIGH].q = rl_thigh.at(motiontime);
        low_cmd.motor_cmd[gaitlib::RL_HIP].q = 0.0; 

        motiontime += 1;
        // RCLCPP_INFO_STREAM(get_logger(), "Motiontime " << motiontime);
        if (motiontime >= static_cast<long>(fr_calf.size())) {
          RCLCPP_INFO_STREAM(get_logger(), "New Step!");
          motiontime = 0;
        }
        break;
      }
      default:
      {
        RCLCPP_INFO_STREAM(get_logger(), "INVALID STATE!!!!");
        break;
      }
    }
    // RCLCPP_INFO_STREAM(get_logger(), "FR Torque: "<<low_cmd.motor_cmd[gaitlib::FR_HIP].tau);
    cmd_pub_->publish(low_cmd);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr switch_gait_;
  ros2_unitree_legged_msgs::msg::LowCmd low_cmd;
  long motiontime = 0;
  int count = 0;
  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr state_sub_;
  std::vector<int> feets;
  long period;
  long delay;
  // these hold the primary gait
  std::vector<double> fr_calf, fl_calf, rr_calf, rl_calf, fr_thigh, fl_thigh, rr_thigh, rl_thigh;
  // these hold the standing gaits
  std::vector<double> fr_calf_stand, fl_calf_stand, rr_calf_stand, rl_calf_stand, 
                      fr_thigh_stand, fl_thigh_stand, rr_thigh_stand, rl_thigh_stand;
  // This is the length of the legs.
  double l = 0.213;
  // Define joint limits
  double calf_lo = -2.82;
  double calf_hi = -0.89;
  double thigh_lo = -0.69;
  double thigh_hi = 4.50;
  double hip_lo = -0.86;
  double hip_hi = 0.86;
  // Define nominal joint values
  double calf_base = -1.85;
  double thigh_base = 0.0;
  double hip_base = 0.0;
  // Control points for bezier curve
  std::vector<double> ctrl_x, ctrl_y;
  State state = WAIT;
  // y coordinate the foot is at WRT hip when standing still
  double stand_y, stand_calf, stand_thigh;
  double stroke_length, stiffness, damping;
  double stand_calf_torque = 3.0;
  double stand_hip_torque = 3.0;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomGait>());
  rclcpp::shutdown();
  return 0;
}
