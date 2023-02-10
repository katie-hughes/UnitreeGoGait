#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "gaitlib/gaits.hpp"

using namespace std::chrono_literals;

class CustomGait : public rclcpp::Node
{
public:
  CustomGait()
  : Node("custom_gait")
  {
    declare_parameter("rate", 200.0);
    declare_parameter("seconds_per_swing", 2.0);

    double rate_hz = get_parameter("rate").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Rate is " << ((int)(1000. / rate_hz)) << "ms");
    std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));
    
    double seconds_per_swing = get_parameter("seconds_per_swing").as_double();
    RCLCPP_INFO_STREAM(get_logger(), seconds_per_swing<<" seconds per swing");

    period = rate_hz * seconds_per_swing;
    RCLCPP_INFO_STREAM(get_logger(), period<<" publishes per swing");

    // delay for 3 seconds before beginning movement
    delay = rate_hz * 3.0;

    cmd_pub_ = create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 10);

    state_sub_ = create_subscription<ros2_unitree_legged_msgs::msg::LowState>(
      "low_state", 10,
      std::bind(&CustomGait::state_cb, this, std::placeholders::_1));

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

    const auto lspan = 0.10;   // half of "stroke length", ie how long it's on the floor
    const auto dl = 0.025;   // extra bit to extend by after leaving floor
    const auto ddl = 0.025;   // another extra bit to extend by LOL
    const auto stand_floor = -1.5 * l; // y distance when the foot is on the floor.
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
    ctrl_y = {stand_floor,
      stand_floor,
      stand_floor + swing_height,
      stand_floor + swing_height,
      stand_floor + swing_height,
      stand_floor + swing_height,
      stand_floor + swing_height,
      stand_floor + swing_height + dswing_height,
      stand_floor + swing_height + dswing_height,
      stand_floor + swing_height + dswing_height,
      stand_floor,
      stand_floor};

    long npoints_bezier = period*0.75;
    long npoints_sinusoid = period-npoints_bezier;
    std::vector<double> bez_x = gaitlib::bezier(ctrl_x, npoints_bezier);
    std::vector<double> bez_y = gaitlib::bezier(ctrl_y, npoints_bezier);
    RCLCPP_INFO_STREAM(get_logger(), "Size of bez_x: " << bez_x.size());
    std::vector<double> sin_x = gaitlib::linspace(ctrl_x.back(), ctrl_x.at(0), npoints_sinusoid);
    std::vector<double> sin_y = gaitlib::stance(sin_x, 0.05, ctrl_y.at(0));

    const auto final_x = gaitlib::concatenate(bez_x, sin_x);
    const auto final_y = gaitlib::concatenate(bez_y, sin_y);

    make_fr_gait(final_x, final_y);
    // Next: MODULATE based on fr
    fl_calf = gaitlib::modulate(fr_calf, 0.5);
    fl_thigh = gaitlib::modulate(fr_thigh, 0.5);

    rr_calf = gaitlib::modulate(fr_calf, 0.5);
    rr_thigh = gaitlib::modulate(fr_thigh, 0.5);

    rl_calf = gaitlib::modulate(fr_calf, 0.0);
    rl_thigh = gaitlib::modulate(fr_thigh, 0.0);

    RCLCPP_INFO_STREAM(get_logger(), "Waiting...");
  }

private:
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
    // RCLCPP_INFO_STREAM(get_logger(), "Timer tick!");
    if (!initiated_flag) {
      count++;
      if (count > delay) {
        RCLCPP_INFO_STREAM(get_logger(), "Start moving!");
        initiated_flag = true;
      }
    } else {
      motiontime += 1;
      // RCLCPP_INFO_STREAM(get_logger(), "Motiontime " << motiontime);
      if (motiontime >= period) {
        RCLCPP_INFO_STREAM(get_logger(), "New Step!");
        motiontime = 0;
      }
      low_cmd.motor_cmd[gaitlib::FR_CALF].q = fr_calf[motiontime];
      low_cmd.motor_cmd[gaitlib::FR_CALF].dq = 0.0;
      low_cmd.motor_cmd[gaitlib::FR_CALF].kp = 5.0;
      low_cmd.motor_cmd[gaitlib::FR_CALF].kd = 1.0;
      low_cmd.motor_cmd[gaitlib::FR_HIP].q = 0.0; // fr_hip[motiontime];
      low_cmd.motor_cmd[gaitlib::FR_HIP].dq = 0.0;
      low_cmd.motor_cmd[gaitlib::FR_HIP].kp = 5.0;
      low_cmd.motor_cmd[gaitlib::FR_HIP].kd = 1.0;
      low_cmd.motor_cmd[gaitlib::FR_THIGH].q = fr_thigh[motiontime];
      low_cmd.motor_cmd[gaitlib::FR_THIGH].dq = 0.0;
      low_cmd.motor_cmd[gaitlib::FR_THIGH].kp = 5.0;
      low_cmd.motor_cmd[gaitlib::FR_THIGH].kd = 1.0;

      low_cmd.motor_cmd[gaitlib::FL_CALF].q = fl_calf[motiontime];
      low_cmd.motor_cmd[gaitlib::FL_CALF].dq = 0.0;
      low_cmd.motor_cmd[gaitlib::FL_CALF].kp = 5.0;
      low_cmd.motor_cmd[gaitlib::FL_CALF].kd = 1.0;
      low_cmd.motor_cmd[gaitlib::FL_HIP].q = 0.0; // fl_hip[motiontime];
      low_cmd.motor_cmd[gaitlib::FL_HIP].dq = 0.0;
      low_cmd.motor_cmd[gaitlib::FL_HIP].kp = 5.0;
      low_cmd.motor_cmd[gaitlib::FL_HIP].kd = 1.0;
      low_cmd.motor_cmd[gaitlib::FL_THIGH].q = fl_thigh[motiontime];
      low_cmd.motor_cmd[gaitlib::FL_THIGH].dq = 0.0;
      low_cmd.motor_cmd[gaitlib::FL_THIGH].kp = 5.0;
      low_cmd.motor_cmd[gaitlib::FL_THIGH].kd = 1.0;

      low_cmd.motor_cmd[gaitlib::RR_CALF].q = rr_calf[motiontime];
      low_cmd.motor_cmd[gaitlib::RR_CALF].dq = 0.0;
      low_cmd.motor_cmd[gaitlib::RR_CALF].kp = 5.0;
      low_cmd.motor_cmd[gaitlib::RR_CALF].kd = 1.0;
      low_cmd.motor_cmd[gaitlib::RR_HIP].q = 0.0; // rr_hip[motiontime];
      low_cmd.motor_cmd[gaitlib::RR_HIP].dq = 0.0;
      low_cmd.motor_cmd[gaitlib::RR_HIP].kp = 5.0;
      low_cmd.motor_cmd[gaitlib::RR_HIP].kd = 1.0;
      low_cmd.motor_cmd[gaitlib::RR_THIGH].q = rr_thigh[motiontime];
      low_cmd.motor_cmd[gaitlib::RR_THIGH].dq = 0.0;
      low_cmd.motor_cmd[gaitlib::RR_THIGH].kp = 5.0;
      low_cmd.motor_cmd[gaitlib::RR_THIGH].kd = 1.0;

      low_cmd.motor_cmd[gaitlib::RL_CALF].q = rl_calf[motiontime];
      low_cmd.motor_cmd[gaitlib::RL_CALF].dq = 0.0;
      low_cmd.motor_cmd[gaitlib::RL_CALF].kp = 5.0;
      low_cmd.motor_cmd[gaitlib::RL_CALF].kd = 1.0;
      low_cmd.motor_cmd[gaitlib::RL_HIP].q = 0.0; // rl_hip[motiontime];
      low_cmd.motor_cmd[gaitlib::RL_HIP].dq = 0.0;
      low_cmd.motor_cmd[gaitlib::RL_HIP].kp = 5.0;
      low_cmd.motor_cmd[gaitlib::RL_HIP].kd = 1.0;
      low_cmd.motor_cmd[gaitlib::RL_THIGH].q = rl_thigh[motiontime];
      low_cmd.motor_cmd[gaitlib::RL_THIGH].dq = 0.0;
      low_cmd.motor_cmd[gaitlib::RL_THIGH].kp = 5.0;
      low_cmd.motor_cmd[gaitlib::RL_THIGH].kd = 1.0;
    }
    cmd_pub_->publish(low_cmd);
  }

  double get_theta_calf(double theta_thigh, double x)
  {
    return asin(-x / l - sin(theta_thigh)) - theta_thigh;
  }

  // Given an x and y (WRT hip joint), return possible joint angles
  // First two in the vector are the "lefty" solution, and last 2 are "righty"
  std::vector<double> ik(double x, double y)
  {
    double alpha = acos(sqrt(x * x + y * y) / (2 * l));
    double gamma = atan(x / y);
    double theta_thigh_left = gamma + alpha;
    double theta_calf_left = get_theta_calf(theta_thigh_left, x);
    double theta_thigh_right = gamma - alpha;
    double theta_calf_right = get_theta_calf(theta_thigh_right, x);
    std::vector<double> res;
    res.push_back(theta_thigh_left);
    res.push_back(theta_calf_left);
    res.push_back(theta_thigh_right);
    res.push_back(theta_calf_right);
    return res;
  }

  void make_fr_gait(std::vector<double> desired_x, std::vector<double> desired_y)
  {
    if (desired_x.size() != desired_y.size()) {
      RCLCPP_INFO_STREAM(get_logger(), "Desired X and Desired Y different lengths???");
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "Generating Trajectory");
      std::vector<double> ik_result;
      for (int i = 0; i < period; i++) {
        ik_result = ik(desired_x[i], desired_y[i]);
        // Here we just arbitrarily choose left result (it maintained joint limits in my example)
        // The left thigh result is 0th element and calf result is 1st
        // Keep rest of joints stationary for now
        RCLCPP_INFO_STREAM(
          get_logger(), "(x,y)=(" << desired_x[i] << "," << desired_y[i] <<
            ") ->\t(theta_t, theta_c)=(" <<
            ik_result[0] << "," << ik_result[1] << ")");
        fr_calf.push_back(ik_result[1]);
        // fl_calf.push_back(calf_base);
        // rr_calf.push_back(calf_base);
        // rl_calf.push_back(calf_base);

        fr_thigh.push_back(ik_result[0]);
        // fl_thigh.push_back(thigh_base);
        // rr_thigh.push_back(thigh_base);
        // rl_thigh.push_back(thigh_base);

        // fr_hip.push_back(hip_base);
        // fl_hip.push_back(hip_base);
        // rr_hip.push_back(hip_base);
        // rl_hip.push_back(hip_base);
      }
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  ros2_unitree_legged_msgs::msg::LowCmd low_cmd;
  long motiontime = 0;
  bool initiated_flag = false;
  int count = 0;
  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr state_sub_;
  std::vector<int> feets;
  long period;
  long delay;
  std::vector<double> fr_calf, fl_calf, rr_calf, rl_calf, fr_thigh, fl_thigh, rr_thigh, rl_thigh,
    fr_hip, fl_hip, rr_hip, rl_hip;
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
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomGait>());
  rclcpp::shutdown();
  return 0;
}
