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
           STANDSTILL,
           LIEDOWN,
           LIESTILL,
           RESET};

class CustomGait : public rclcpp::Node
{
public:
  CustomGait()
  : Node("custom_gait")
  {
    declare_parameter("rate", 200.0);
    rate_hz = get_parameter("rate").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Publish rate is " << ((int)(1000. / rate_hz)) << "ms");
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
    RCLCPP_INFO_STREAM(get_logger(), stiffness<<" kp");

    declare_parameter("damping", 1.0); // kd
    damping = get_parameter("damping").as_double();
    RCLCPP_INFO_STREAM(get_logger(), damping<<" kd");

    declare_parameter("delta", 0.05);
    delta = get_parameter("delta").as_double();
    RCLCPP_INFO_STREAM(get_logger(), delta<<" delta");

    // declare_parameter("torque", 1.0); // tau

    // delay for 3 seconds before beginning movement
    initial_delay = rate_hz * 3.0;

    cmd_pub_ = create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 10);

    state_sub_ = create_subscription<ros2_unitree_legged_msgs::msg::LowState>(
      "low_state", 10,
      std::bind(&CustomGait::state_cb, this, std::placeholders::_1));

    switch_gait_ = create_service<std_srvs::srv::Empty>(
      "switch",
      std::bind(&CustomGait::switch_gait, this, std::placeholders::_1, std::placeholders::_2));
    
    lie_down_ = create_service<std_srvs::srv::Empty>(
      "lie_down",
      std::bind(&CustomGait::lie_down, this, std::placeholders::_1, std::placeholders::_2));

    stand_up_ = create_service<std_srvs::srv::Empty>(
      "stand_up",
      std::bind(&CustomGait::stand_up, this, std::placeholders::_1, std::placeholders::_2));

    reset_torque_ = create_service<std_srvs::srv::Empty>(
      "reset_torque",
      std::bind(&CustomGait::reset_torque, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
      rate,
      std::bind(&CustomGait::timer_callback, this));

    // Provide some initializations to the low_cmd message
    init_low_cmd();

    // Create walking procedure
    // generate_trot_gait();
    generate_tripod_gait();

    // Create standup procedure
    generate_standup();

    RCLCPP_INFO_STREAM(get_logger(), "Waiting...");
  }

private:

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr switch_gait_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr lie_down_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stand_up_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_torque_;
  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr state_sub_;

  // Use one LowCmd object to repeatedly publish
  ros2_unitree_legged_msgs::msg::LowCmd low_cmd;
  // Timestep, to be used in my timer to select appropriate joint angle
  long timestep = 0;
  // Store current feet sensor information (currently unused beyond this)
  std::vector<int> feets;
  // Period of one swing
  long period;
  // Delay for a bit before beginning the gait
  long initial_delay;
  // these hold the primary gait
  std::vector<double> fr_calf_walk, fl_calf_walk, rr_calf_walk, rl_calf_walk, 
                      fr_thigh_walk, fl_thigh_walk, rr_thigh_walk, rl_thigh_walk;
  // these hold the standing gaits
  std::vector<double> fr_calf_stand, fl_calf_stand, rr_calf_stand, rl_calf_stand, 
                      fr_thigh_stand, fl_thigh_stand, rr_thigh_stand, rl_thigh_stand;
  // Current state
  State state = WAIT;
  // y coordinate the foot is at WRT hip when standing still
  double stand_y, stand_calf, stand_thigh;
  // y coordinate the foot is at WRT hip when standing still
  double liedown_y, liedown_calf, liedown_thigh;
  // for reading in parameters
  double rate_hz, stroke_length, stiffness, damping, delta;

  /// @brief Initialize the low command message for when the dog first gets connected
  void init_low_cmd(){
    low_cmd.head[0] = 0xFE;
    low_cmd.head[1] = 0xEF;
    low_cmd.level_flag = 0xFF;   // LOWLEVEL;
    for (int i = 0; i < gaitlib::NJOINTS; i++) {
      low_cmd.motor_cmd[i].mode = 0x0A;    // motor switch to servo (PMSM) mode
      low_cmd.motor_cmd[i].q = (2.146E+9f);   // PosStopF; // 禁止位置环
      low_cmd.motor_cmd[i].kp = 0;
      low_cmd.motor_cmd[i].dq = (16000.0f);   // VelStopF; // 禁止速度环
      low_cmd.motor_cmd[i].kd = 0;
      low_cmd.motor_cmd[i].tau = 0;
    }
  }

  /// @brief Generate a bezier trotting gait
  void generate_trot_gait(){
    const auto lspan = 0.5 * stroke_length;   // half of "stroke length", ie how long it's on the floor
    const auto dl = 0.025;   // extra bit to extend by after leaving floor
    const auto ddl = 0.025;   // another extra bit to extend by LOL
    stand_y = -1.5 * gaitlib::LEG_LENGTH; // y distance when the foot is on the floor.
    const auto swing_height = 0.05;   // ???
    const auto dswing_height = 0.025;   // ??
    std::vector<double> ctrl_x{-1.0 * lspan,
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
    std::vector<double> ctrl_y{stand_y,
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
    std::vector<double> sin_y = gaitlib::stance(sin_x, delta, ctrl_y.at(0));

    const auto final_x = gaitlib::concatenate(bez_x, sin_x);
    const auto final_y = gaitlib::concatenate(bez_y, sin_y);

    const auto fr_gaits = gaitlib::make_gait(final_x, final_y);
    fr_calf_walk = fr_gaits.gait_calf;
    fr_thigh_walk = fr_gaits.gait_thigh;
    // Next: MODULATE based on fr
    fl_calf_walk = gaitlib::modulate(fr_calf_walk, 0.5);
    fl_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.5);

    rr_calf_walk = gaitlib::modulate(fr_calf_walk, 0.5);
    rr_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.5);

    rl_calf_walk = gaitlib::modulate(fr_calf_walk, 0.0);
    rl_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.0);
  }

  /// @brief Generate a bezier tripod gait that moves exactly one foot at a time
  void generate_tripod_gait(){
    const auto lspan = 0.5 * stroke_length;   // half of "stroke length", ie how long it's on the floor
    const auto dl = 0.025;   // extra bit to extend by after leaving floor
    const auto ddl = 0.025;   // another extra bit to extend by LOL
    stand_y = -1.5 * gaitlib::LEG_LENGTH; // y distance when the foot is on the floor.
    const auto swing_height = 0.05;   // ???
    const auto dswing_height = 0.025;   // ??
    std::vector<double> ctrl_x{-1.0 * lspan,
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
    std::vector<double> ctrl_y{stand_y,
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

    long npoints_bezier = period;
    // this 3 is to give the 3 remaining legs time to complete the swing
    long npoints_rest = 3*npoints_bezier;
    std::vector<double> bez_x = gaitlib::bezier(ctrl_x, npoints_bezier);
    std::vector<double> bez_y = gaitlib::bezier(ctrl_y, npoints_bezier);
    RCLCPP_INFO_STREAM(get_logger(), "Size of bez_x: " << bez_x.size());
    std::vector<double> rest_x = gaitlib::linspace(ctrl_x.back(), ctrl_x.at(0), npoints_rest);
    std::vector<double> rest_y = gaitlib::linspace(ctrl_y.back(), ctrl_y.at(0), npoints_rest);

    const auto final_x = gaitlib::concatenate(bez_x, rest_x);
    const auto final_y = gaitlib::concatenate(bez_y, rest_y);

    const auto fr_gaits = gaitlib::make_gait(final_x, final_y);
    fr_calf_walk = fr_gaits.gait_calf;
    fr_thigh_walk = fr_gaits.gait_thigh;
    // Next: MODULATE based on fr
    // I want FR, RL, FL, RR for simple walk
    rl_calf_walk = gaitlib::modulate(fr_calf_walk, 0.25);
    rl_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.25);

    fl_calf_walk = gaitlib::modulate(fr_calf_walk, 0.5);
    fl_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.5);

    rr_calf_walk = gaitlib::modulate(fr_calf_walk, 0.75);
    rr_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.75);
  }


  void generate_standup(){
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

    const auto liedown_joints = gaitlib::ik(0.0, stand_offset);
    liedown_calf = liedown_joints.calf_lefty;
    liedown_thigh = liedown_joints.thigh_lefty;
  }

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
      timestep = 0;
    } else if (state == STANDSTILL){
      state = WALK;
      timestep = 0;
    }
  }

  /// @brief Switch gait type between standing/walking
  /// @param Request: The empty request
  /// @param Response: The empty response
  void lie_down(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Lie Down");
    if (state == STANDSTILL){
      state = LIEDOWN;
      timestep = 0;
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "You must be in the standstill state to lie down!");
    }
  }

  /// @brief Switch gait type between standing/walking
  /// @param Request: The empty request
  /// @param Response: The empty response
  void stand_up(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Stand Up");
    if (state == LIESTILL){
      state = STANDUP;
      timestep = 0;
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "You must be in the liestill state to lie down!");
    }
  }

  /// @brief Put the dog into a torque 0 state
  /// @param Request: The empty request
  /// @param Response: The empty response
  void reset_torque(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Reset Torque");
    state = RESET;
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
        if (timestep > initial_delay) {
          RCLCPP_INFO_STREAM(get_logger(), "Stand Up!");
          state = STANDUP;
          // set params that will never change and are the same for all joints: 
          // dq, kp, kd (for PD control)
          for (int i = 0; i < gaitlib::NJOINTS; i++){
            low_cmd.motor_cmd[i].dq = 0.0;
            low_cmd.motor_cmd[i].kp = stiffness;
            low_cmd.motor_cmd[i].kd = damping;
          }
          timestep = 0;
        }
        timestep++;
        break;
      }
      case STANDUP:
      {
        low_cmd.motor_cmd[gaitlib::FR_CALF].q = fr_calf_stand.at(timestep);
        low_cmd.motor_cmd[gaitlib::FR_THIGH].q = fr_thigh_stand.at(timestep);
        low_cmd.motor_cmd[gaitlib::FR_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::FL_CALF].q = fl_calf_stand.at(timestep);
        low_cmd.motor_cmd[gaitlib::FL_THIGH].q = fl_thigh_stand.at(timestep);
        low_cmd.motor_cmd[gaitlib::FL_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::RR_CALF].q = rr_calf_stand.at(timestep);
        low_cmd.motor_cmd[gaitlib::RR_THIGH].q = rr_thigh_stand.at(timestep);
        low_cmd.motor_cmd[gaitlib::RR_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::RL_CALF].q = rl_calf_stand.at(timestep);
        low_cmd.motor_cmd[gaitlib::RL_THIGH].q = rl_thigh_stand.at(timestep);
        low_cmd.motor_cmd[gaitlib::RL_HIP].q = 0.0; 

        timestep++;
        // RCLCPP_INFO_STREAM(get_logger(), "timestep " << timestep);
        if (timestep >= static_cast<long>(fr_calf_stand.size())) {
          RCLCPP_INFO_STREAM(get_logger(), "Standstill!");
          timestep = 0;
          state = STANDSTILL;
        }
        break;
      }
      case STANDSTILL:
      {
        low_cmd.motor_cmd[gaitlib::FR_CALF].q = stand_calf;
        low_cmd.motor_cmd[gaitlib::FR_THIGH].q = stand_thigh;
        low_cmd.motor_cmd[gaitlib::FR_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::FL_CALF].q = stand_calf;
        low_cmd.motor_cmd[gaitlib::FL_THIGH].q = stand_thigh;
        low_cmd.motor_cmd[gaitlib::FL_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::RR_CALF].q = stand_calf;
        low_cmd.motor_cmd[gaitlib::RR_THIGH].q = stand_thigh;
        low_cmd.motor_cmd[gaitlib::RR_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::RL_CALF].q = stand_calf;
        low_cmd.motor_cmd[gaitlib::RL_THIGH].q = stand_thigh;
        low_cmd.motor_cmd[gaitlib::RL_HIP].q = 0.0; 
        break;
      }
      case LIEDOWN:
      {
        const auto npoints = static_cast<long>(fr_calf_stand.size());
        // should iterate backwards through the _stand vectors.
        // ie, x_stand.size() - timestep for 0 < timestep < x_stand.size()
        low_cmd.motor_cmd[gaitlib::FR_CALF].q = fr_calf_stand.at(npoints - timestep - 1);
        low_cmd.motor_cmd[gaitlib::FR_THIGH].q = fr_thigh_stand.at(npoints - timestep - 1);
        low_cmd.motor_cmd[gaitlib::FR_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::FL_CALF].q = fl_calf_stand.at(npoints - timestep - 1);
        low_cmd.motor_cmd[gaitlib::FL_THIGH].q = fl_thigh_stand.at(npoints - timestep - 1);
        low_cmd.motor_cmd[gaitlib::FL_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::RR_CALF].q = rr_calf_stand.at(npoints - timestep - 1);
        low_cmd.motor_cmd[gaitlib::RR_THIGH].q = rr_thigh_stand.at(npoints - timestep - 1);
        low_cmd.motor_cmd[gaitlib::RR_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::RL_CALF].q = rl_calf_stand.at(npoints - timestep - 1);
        low_cmd.motor_cmd[gaitlib::RL_THIGH].q = rl_thigh_stand.at(npoints - timestep - 1);
        low_cmd.motor_cmd[gaitlib::RL_HIP].q = 0.0; 

        timestep++;
        // RCLCPP_INFO_STREAM(get_logger(), "timestep " << timestep);
        if (timestep >= npoints) {
          RCLCPP_INFO_STREAM(get_logger(), "Lie still!");
          timestep = 0;
          state = LIESTILL;
        }
        break;
      }
      case LIESTILL:
      {
        low_cmd.motor_cmd[gaitlib::FR_CALF].q = liedown_calf;
        low_cmd.motor_cmd[gaitlib::FR_THIGH].q = liedown_thigh;
        low_cmd.motor_cmd[gaitlib::FR_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::FL_CALF].q = liedown_calf;
        low_cmd.motor_cmd[gaitlib::FL_THIGH].q = liedown_thigh;
        low_cmd.motor_cmd[gaitlib::FL_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::RR_CALF].q = liedown_calf;
        low_cmd.motor_cmd[gaitlib::RR_THIGH].q = liedown_thigh;
        low_cmd.motor_cmd[gaitlib::RR_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::RL_CALF].q = liedown_calf;
        low_cmd.motor_cmd[gaitlib::RL_THIGH].q = liedown_thigh;
        low_cmd.motor_cmd[gaitlib::RL_HIP].q = 0.0; 
        break;
      }
      case WALK:
      {
        low_cmd.motor_cmd[gaitlib::FR_CALF].q = fr_calf_walk.at(timestep);
        low_cmd.motor_cmd[gaitlib::FR_THIGH].q = fr_thigh_walk.at(timestep);
        low_cmd.motor_cmd[gaitlib::FR_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::FL_CALF].q = fl_calf_walk.at(timestep);
        low_cmd.motor_cmd[gaitlib::FL_THIGH].q = fl_thigh_walk.at(timestep);
        low_cmd.motor_cmd[gaitlib::FL_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::RR_CALF].q = rr_calf_walk.at(timestep);
        low_cmd.motor_cmd[gaitlib::RR_THIGH].q = rr_thigh_walk.at(timestep);
        low_cmd.motor_cmd[gaitlib::RR_HIP].q = 0.0; 

        low_cmd.motor_cmd[gaitlib::RL_CALF].q = rl_calf_walk.at(timestep);
        low_cmd.motor_cmd[gaitlib::RL_THIGH].q = rl_thigh_walk.at(timestep);
        low_cmd.motor_cmd[gaitlib::RL_HIP].q = 0.0; 

        timestep++;
        // RCLCPP_INFO_STREAM(get_logger(), "timestep " << timestep);
        if (timestep >= static_cast<long>(fr_calf_walk.size())) {
          RCLCPP_INFO_STREAM(get_logger(), "New Step!");
          timestep = 0;
        }
        break;
      }
      case RESET:
      {
        low_cmd.motor_cmd[gaitlib::FR_CALF].tau = 0.0;
        low_cmd.motor_cmd[gaitlib::FR_THIGH].tau = 0.0;
        low_cmd.motor_cmd[gaitlib::FR_HIP].tau = 0.0;

        low_cmd.motor_cmd[gaitlib::FL_CALF].tau = 0.0;
        low_cmd.motor_cmd[gaitlib::FL_THIGH].tau = 0.0;
        low_cmd.motor_cmd[gaitlib::FL_HIP].tau = 0.0;

        low_cmd.motor_cmd[gaitlib::RR_CALF].tau = 0.0;
        low_cmd.motor_cmd[gaitlib::RR_THIGH].tau = 0.0;
        low_cmd.motor_cmd[gaitlib::RR_HIP].tau = 0.0;

        low_cmd.motor_cmd[gaitlib::RL_CALF].tau = 0.0;
        low_cmd.motor_cmd[gaitlib::RL_THIGH].tau = 0.0;
        low_cmd.motor_cmd[gaitlib::RL_HIP].tau = 0.0;
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
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomGait>());
  rclcpp::shutdown();
  return 0;
}
