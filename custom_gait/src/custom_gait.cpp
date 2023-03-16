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


enum State {WAIT,
  STANDUP,
  WALK,
  BACKWARDS,
  LEFT,
  RIGHT,
  STANDSTILL,
  LIEDOWN,
  LIESTILL,
  WALK_TO_STANDSTILL,
  STANDSTILL_TO_WALK,
  RESET};

enum GaitType {SINGLE,     // Move just the front right leg
  TRIPOD,                  // Move all four legs, but one leg at a time
  TROT,                    // Move all four legs, FR and RL together and FL and RR together
  SIDEWAYS                 // Move laterally!
  };

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
    RCLCPP_INFO_STREAM(get_logger(), seconds_per_swing << " seconds per swing");

    period = rate_hz * seconds_per_swing;
    RCLCPP_INFO_STREAM(get_logger(), period << " publishes per swing");

    declare_parameter("stroke_length_x", 0.05);
    stroke_length_x = get_parameter("stroke_length_x").as_double();
    RCLCPP_INFO_STREAM(get_logger(), stroke_length_x << "m stroke length x");

    declare_parameter("stroke_length_z", 0.025);
    stroke_length_z = get_parameter("stroke_length_z").as_double();
    RCLCPP_INFO_STREAM(get_logger(), stroke_length_z << "m stroke length z");

    declare_parameter("standup_time", 2.0);
    standup_time = get_parameter("standup_time").as_double();
    RCLCPP_INFO_STREAM(get_logger(), standup_time << "s standup_time");

    declare_parameter("stiffness", 5.0); // kp
    stiffness = get_parameter("stiffness").as_double();
    RCLCPP_INFO_STREAM(get_logger(), stiffness << " kp");

    declare_parameter("damping", 1.0); // kd
    damping = get_parameter("damping").as_double();
    RCLCPP_INFO_STREAM(get_logger(), damping << " kd");

    declare_parameter("delta", 0.05);
    delta = get_parameter("delta").as_double();
    RCLCPP_INFO_STREAM(get_logger(), delta << " delta");

    declare_parameter("stand_percentage", 0.75);
    stand_percentage = get_parameter("stand_percentage").as_double();
    RCLCPP_INFO_STREAM(get_logger(), stand_percentage << "%% stand height");

    declare_parameter("gait_type", 0);
    gait_type = static_cast<GaitType>(get_parameter("gait_type").as_int());
    RCLCPP_INFO_STREAM(get_logger(), gait_type << " gait_type");

    declare_parameter("tripod_offset", 1);
    tripod_offset = get_parameter("tripod_offset").as_int();
    RCLCPP_INFO_STREAM(get_logger(), tripod_offset << " tripod_offset");

    declare_parameter("trot_offset", 0);
    trot_offset = get_parameter("trot_offset").as_int();
    RCLCPP_INFO_STREAM(get_logger(), trot_offset << " trot_offset");

    declare_parameter("step_height", 0.05);
    step_height = get_parameter("step_height").as_double();
    RCLCPP_INFO_STREAM(get_logger(), step_height << " step_height");

    declare_parameter("dx1", 0.025);
    dx1 = get_parameter("dx1").as_double();
    RCLCPP_INFO_STREAM(get_logger(), dx1 << " dx1");

    declare_parameter("dx2", 0.025);
    dx2 = get_parameter("dx2").as_double();
    RCLCPP_INFO_STREAM(get_logger(), dx2 << " dx2");

    declare_parameter("dz1", 0.025);
    dz1 = get_parameter("dz1").as_double();
    RCLCPP_INFO_STREAM(get_logger(), dz1 << " dz1");

    declare_parameter("dz2", 0.025);
    dz2 = get_parameter("dz2").as_double();
    RCLCPP_INFO_STREAM(get_logger(), dx2 << " dz2");

    declare_parameter("dy", 0.025);
    dy = get_parameter("dy").as_double();
    RCLCPP_INFO_STREAM(get_logger(), dy << " dy");

    declare_parameter("step_limit", true);
    step_limit = get_parameter("step_limit").as_bool();
    RCLCPP_INFO_STREAM(get_logger(), step_limit << " step_limit");

    declare_parameter("nsteps", 5);
    nsteps = get_parameter("nsteps").as_int();
    RCLCPP_INFO_STREAM(get_logger(), nsteps << " nsteps");

    if ((stand_percentage < 0.05) || (stand_percentage > 0.95)) {
      throw std::logic_error("Stand percentage must be between 0 and 1!");
    }

    if (tripod_offset < 1) {
      throw std::logic_error("Tripod Offset must be 1 or more!");
    }

    // declare_parameter("torque", 1.0); // tau

    // delay for 3 seconds before beginning movement
    initial_delay = rate_hz * 3.0;

    cmd_pub_ = create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 10);

    state_sub_ = create_subscription<ros2_unitree_legged_msgs::msg::LowState>(
      "low_state", 10,
      std::bind(&CustomGait::state_cb, this, std::placeholders::_1));

    walk_forwards_ = create_service<std_srvs::srv::Empty>(
      "walk",
      std::bind(&CustomGait::walk_forwards, this, std::placeholders::_1, std::placeholders::_2));

    walk_backwards_ = create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&CustomGait::walk_backwards, this, std::placeholders::_1, std::placeholders::_2));

    walk_left_ = create_service<std_srvs::srv::Empty>(
      "left",
      std::bind(&CustomGait::walk_left, this, std::placeholders::_1, std::placeholders::_2));

    walk_right_ = create_service<std_srvs::srv::Empty>(
      "right",
      std::bind(&CustomGait::walk_right, this, std::placeholders::_1, std::placeholders::_2));

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

    stand_y = -2.0 * stand_percentage * gaitlib::LEG_LENGTH;
    stand_x = -0.05;
    stand_z = 0.0;

    // Create bezier for walking procedure
    generate_bez_controls();

    // generate a specific gait type
    if (gait_type == SINGLE) {
      generate_stupid_gait();
    } else if (gait_type == TRIPOD) {
      generate_tripod_gait();
    } else if (gait_type == TROT) {
      generate_trot_gait();
      generate_sideways_gait();
    } else if (gait_type == SIDEWAYS) {
      generate_sideways_gait();
    } else {
      throw std::logic_error("Gait Type must be 0, 1, or 2");
    }

    // Create standup procedure
    generate_standup();

    RCLCPP_INFO_STREAM(get_logger(), "Waiting...");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr walk_forwards_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr walk_backwards_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr walk_left_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr walk_right_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr lie_down_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stand_up_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_torque_;
  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr state_sub_;

  // Use one LowCmd object to repeatedly publish
  ros2_unitree_legged_msgs::msg::LowCmd low_cmd;
  // Timestep, to be used in my timer to select appropriate joint angle
  long timestep = 0;
  // Store current state of robot
  ros2_unitree_legged_msgs::msg::LowState low_state;
  // Period of one swing
  long period;
  // Delay for a bit before beginning the gait
  long initial_delay;
  // these are bezier control points
  std::vector<double> ctrl_x, ctrl_y, ctrl_z;
  // this is the bezier curve of feet position
  std::vector<double> bez_x, bez_y, bez_z;
  // these hold the primary gait
  std::vector<double> fr_calf_walk, fl_calf_walk, rr_calf_walk, rl_calf_walk,
    fr_thigh_walk, fl_thigh_walk, rr_thigh_walk, rl_thigh_walk,
    fr_hip_walk, fl_hip_walk, rr_hip_walk, rl_hip_walk;
  // these hold the sideways gait
  std::vector<double> fr_calf_side, fl_calf_side, rr_calf_side, rl_calf_side,
    fr_thigh_side, fl_thigh_side, rr_thigh_side, rl_thigh_side,
    fr_hip_side, fl_hip_side, rr_hip_side, rl_hip_side;
  // these hold the standing gaits
  std::vector<double> fr_calf_stand, fl_calf_stand, rr_calf_stand, rl_calf_stand,
    fr_thigh_stand, fl_thigh_stand, rr_thigh_stand, rl_thigh_stand;
  // these hold the transitional gaits
  std::vector<double> fr_calf_switch, fl_calf_switch, rr_calf_switch, rl_calf_switch,
    fr_thigh_switch, fl_thigh_switch, rr_thigh_switch, rl_thigh_switch;
  // Current state
  State state = WAIT;
  // Gait type to generate
  GaitType gait_type = SINGLE;
  // y coordinate the foot is at WRT hip when standing still
  double stand_x, stand_y, stand_z, stand_calf, stand_thigh;
  // y coordinate the foot is at WRT hip when standing still
  double liedown_y, liedown_calf, liedown_thigh;
  // for reading in parameters
  double rate_hz, stroke_length_x, stroke_length_z, standup_time, stiffness, damping, delta,
         stand_percentage, step_height, dx1, dx2, dz1, dz2, dy;
  bool step_limit;
  int tripod_offset, trot_offset, nsteps, step_count;

  /// @brief Initialize the low command message for when the dog first gets connected
  void init_low_cmd()
  {
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

  /// @brief Create the bezier control points for the swing in the XY plane.
  void generate_bez_controls()
  {
    const auto lspan_x = 0.5 * stroke_length_x;   // half of "stroke length", ie how long it's on the floor
    const auto lspan_z = 0.5 * stroke_length_z;
    ctrl_x = std::vector<double> {
      stand_x + -1.0 * lspan_x,
      stand_x + -1.0 * lspan_x - dx1,
      stand_x + -1.0 * lspan_x - dx1 - dx2,
      stand_x + -1.0 * lspan_x - dx1 - dx2,
      stand_x + -1.0 * lspan_x - dx1 - dx2,
      stand_x + 0.0,
      stand_x + 0.0,
      stand_x + 0.0,
      stand_x + lspan_x + dx1 + dx2,
      stand_x + lspan_x + dx1 + dx2,
      stand_x + lspan_x + dx1,
      stand_x + lspan_x};
    ctrl_y = std::vector<double> {
      stand_y,
      stand_y,
      stand_y + step_height,
      stand_y + step_height,
      stand_y + step_height,
      stand_y + step_height,
      stand_y + step_height,
      stand_y + step_height + dy,
      stand_y + step_height + dy,
      stand_y + step_height + dy,
      stand_y,
      stand_y};
    // for z controls (valid for sideways only) I just make it basically same as x.
    ctrl_z = std::vector<double> {
      stand_z + -1.0 * lspan_z,
      stand_z + -1.0 * lspan_z - dz1,
      stand_z + -1.0 * lspan_z - dz1 - dz2,
      stand_z + -1.0 * lspan_z - dz1 - dz2,
      stand_z + -1.0 * lspan_z - dz1 - dz2,
      stand_z + 0.0,
      stand_z + 0.0,
      stand_z + 0.0,
      stand_z + lspan_z + dz1 + dz2,
      stand_z + lspan_z + dz1 + dz2,
      stand_z + lspan_z + dz1,
      stand_z + lspan_z};

    // print out the controls
    RCLCPP_INFO_STREAM(get_logger(), "Stand_y " << stand_y);

    for (int i = 0; i < static_cast<int>(ctrl_x.size()); i++) {
      RCLCPP_INFO_STREAM(get_logger(), "X: " << i << " : " << ctrl_x.at(i));
    }
    for (int i = 0; i < static_cast<int>(ctrl_y.size()); i++) {
      RCLCPP_INFO_STREAM(get_logger(), "Y: " << i << " : " << ctrl_y.at(i));
    }
    for (int i = 0; i < static_cast<int>(ctrl_z.size()); i++) {
      RCLCPP_INFO_STREAM(get_logger(), "Z: " << i << " : " << ctrl_z.at(i));
    }

    // generate the curve 
    bez_x = gaitlib::bezier(ctrl_x, period);
    bez_y = gaitlib::bezier(ctrl_y, period);
    bez_z = gaitlib::bezier(ctrl_z, period);

  }

  /// @brief Generate a bezier trotting gait
  void generate_trot_gait()
  {
    RCLCPP_INFO_STREAM(get_logger(), "Generate Trot Gait");
    long npoints_wait = trot_offset*period;

    std::vector<double> final_x, final_y;

    if (trot_offset == 0){
      RCLCPP_INFO_STREAM(get_logger(), "Trot Offest 0");
      // this corresponds to constant motion
      const auto ctrl_back = ctrl_x.back();
      const auto ctrl_front = ctrl_x.front();

      RCLCPP_INFO_STREAM(get_logger(), "Back: " << ctrl_back << " Front " << ctrl_front);

      std::vector<double> sin_x = gaitlib::linspace(ctrl_back, ctrl_front, period);
      std::vector<double> sin_y = gaitlib::stance(sin_x, delta, ctrl_y.at(0));

      final_x = gaitlib::concatenate(bez_x, sin_x);
      final_y = gaitlib::concatenate(bez_y, sin_y);
    } else {
      // aitin period 
      std::vector<double> sin_x =
        gaitlib::linspace(ctrl_x.at(0), ctrl_x.at(0), period);
      std::vector<double> sin_y =
        gaitlib::stance(sin_x, delta, ctrl_y.at(0));

      const auto moving_x = gaitlib::concatenate(sin_x, bez_x);
      const auto moving_y = gaitlib::concatenate(sin_y, bez_y);

      const auto wait_x = gaitlib::linspace(moving_x.back(), moving_x.at(0), npoints_wait);
      const auto wait_y = gaitlib::linspace(moving_y.back(), moving_y.at(0), npoints_wait);

      final_x = gaitlib::concatenate(moving_x, wait_x);
      final_y = gaitlib::concatenate(moving_y, wait_y);
    }
    // print coords ???
    for (int i = 0; i < static_cast<int>(final_x.size()); i++){
      RCLCPP_INFO_STREAM(get_logger(), "Point " << i << ": (" << final_x.at(i) << ", " 
                                                              << final_y.at(i) << ")");
    }

    const auto fr_gaits = gaitlib::make_gait(final_x, final_y);
    fr_calf_walk = fr_gaits.gait_calf;
    fr_thigh_walk = fr_gaits.gait_thigh;
    // hip in this walk is just constant at 0
    fr_hip_walk = std::vector<double>(fr_calf_walk.size(), 0.0);
    // Next: MODULATE based on fr
    fl_calf_walk = gaitlib::modulate(fr_calf_walk, 0.5);
    fl_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.5);
    fl_hip_walk = gaitlib::modulate(fr_hip_walk, 0.5);

    rr_calf_walk = gaitlib::modulate(fr_calf_walk, 0.5);
    rr_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.5);
    rr_hip_walk = gaitlib::modulate(fr_hip_walk, 0.5);

    rl_calf_walk = gaitlib::modulate(fr_calf_walk, 0.0);
    rl_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.0);
    rl_hip_walk = gaitlib::modulate(fr_hip_walk, 0.5);
  }

  /// @brief Generate a SIDEWAYS bezier trotting gait
  /// essentially the same as regular trot, but x -> z
  void generate_sideways_gait()
  {
    long npoints_wait = trot_offset*period;

    std::vector<double> final_z, final_y;

    if (trot_offset == 0){
      // this corresponds to constant motion
      const auto ctrl_back = ctrl_z.back();
      const auto ctrl_front = ctrl_z.front();

      RCLCPP_INFO_STREAM(get_logger(), "Back: " << ctrl_back << " Front " << ctrl_front);

      std::vector<double> sin_z = gaitlib::linspace(ctrl_back, ctrl_front, period);
      std::vector<double> sin_y = gaitlib::stance(sin_z, delta, ctrl_y.at(0));

      final_z = gaitlib::concatenate(bez_z, sin_z);
      final_y = gaitlib::concatenate(bez_y, sin_y);
    } else {
      // waitin period 
      std::vector<double> sin_z =
        gaitlib::linspace(ctrl_x.at(0), ctrl_x.at(0), period);
      std::vector<double> sin_y =
        gaitlib::stance(sin_z, delta, ctrl_y.at(0));

      const auto moving_z = gaitlib::concatenate(sin_z, bez_z);
      const auto moving_y = gaitlib::concatenate(sin_y, bez_y);

      const auto wait_z = gaitlib::linspace(moving_z.back(), moving_z.at(0), npoints_wait);
      const auto wait_y = gaitlib::linspace(moving_y.back(), moving_y.at(0), npoints_wait);

      final_z = gaitlib::concatenate(moving_z, wait_z);
      final_y = gaitlib::concatenate(moving_y, wait_y);
    }
    // print coords ???
    for (int i = 0; i < static_cast<int>(final_z.size()); i++){
      RCLCPP_INFO_STREAM(get_logger(), "Point " << i << ": (" << final_z.at(i) << ", " 
                                                              << final_y.at(i) << ")");
    }
    // desired x coordinates are just stand_x throughout entire gait.
    std::vector<double> final_x(final_z.size(), stand_x);

    const auto fr_gaits = gaitlib::make_3Dgait(final_x, final_y, final_z);
    fr_calf_side = fr_gaits.gait_calf;
    fr_thigh_side = fr_gaits.gait_thigh;
    fr_hip_side = fr_gaits.gait_hip;
    // Next: MODULATE based on fr
    fl_calf_side = gaitlib::modulate(fr_calf_side, 0.5);
    fl_thigh_side = gaitlib::modulate(fr_thigh_side, 0.5);
    fl_hip_side = gaitlib::modulate(fr_hip_side, 0.5);

    rr_calf_side = gaitlib::modulate(fr_calf_side, 0.5);
    rr_thigh_side = gaitlib::modulate(fr_thigh_side, 0.5);
    rr_hip_side = gaitlib::modulate(fr_hip_side, 0.5);

    rl_calf_side = gaitlib::modulate(fr_calf_side, 0.0);
    rl_thigh_side = gaitlib::modulate(fr_thigh_side, 0.0);
    rl_hip_side = gaitlib::modulate(fr_hip_side, 0.0);
  }

  /// @brief Generate a bezier tripod gait that moves exactly one foot at a time
  void generate_tripod_gait()
  {
    RCLCPP_INFO_STREAM(get_logger(), "Generate Tripod Gait");
    long npoints_wait = (4 * tripod_offset - 1) * period;

    std::vector<double> tripod_sin_x =
      gaitlib::linspace(ctrl_x.at(0), ctrl_x.at(0), period);
    std::vector<double> tripod_sin_y =
      gaitlib::stance(tripod_sin_x, delta, ctrl_y.at(0));

    const auto moving_x = gaitlib::concatenate(tripod_sin_x, bez_x);
    const auto moving_y = gaitlib::concatenate(tripod_sin_y, bez_y);

    const auto wait_x = gaitlib::linspace(moving_x.back(), moving_x.at(0), npoints_wait);
    const auto wait_y = gaitlib::linspace(moving_y.back(), moving_y.at(0), npoints_wait);

    const auto final_x = gaitlib::concatenate(moving_x, wait_x);
    const auto final_y = gaitlib::concatenate(moving_y, wait_y);

    const auto fr_gaits = gaitlib::make_gait(final_x, final_y);
    fr_calf_walk = fr_gaits.gait_calf;
    fr_thigh_walk = fr_gaits.gait_thigh;
    // hip in this walk is just constant at 0
    fr_hip_walk = std::vector<double>(fr_calf_walk.size(), 0.0);
    // Next: MODULATE based on fr
    // I want FR, RL, FL, RR for simple walk
    rl_calf_walk = gaitlib::modulate(fr_calf_walk, 0.75);
    rl_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.75);
    rl_hip_walk = gaitlib::modulate(fr_hip_walk, 0.75);

    fl_calf_walk = gaitlib::modulate(fr_calf_walk, 0.5);
    fl_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.5);
    fl_hip_walk = gaitlib::modulate(fr_hip_walk, 0.5);

    rr_calf_walk = gaitlib::modulate(fr_calf_walk, 0.25);
    rr_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.25);
    rr_hip_walk = gaitlib::modulate(fr_hip_walk, 0.25);
  }

  /// @brief Generate a stupid gait that only moves FR leg with the rest standing.
  void generate_stupid_gait()
  {
    // Move from (0, stand_y) to (0 stand_y + 0.5*stroke_length) back to (0, stand_y)
    long npoints_wait = 3 * period;

    std::vector<double> single_sin_x =
      gaitlib::linspace(ctrl_x.at(0), ctrl_x.at(0), period);
    std::vector<double> single_sin_y =
      gaitlib::stance(single_sin_x, delta, ctrl_y.at(0));

    const auto moving_x = gaitlib::concatenate(single_sin_x, bez_x);
    const auto moving_y = gaitlib::concatenate(single_sin_y, bez_y);

    const auto wait_x = gaitlib::linspace(moving_x.back(), moving_x.at(0), npoints_wait);
    const auto wait_y = gaitlib::linspace(moving_y.back(), moving_y.at(0), npoints_wait);

    const auto final_x = gaitlib::concatenate(moving_x, wait_x);
    const auto final_y = gaitlib::concatenate(moving_y, wait_y);

    const auto fr_gaits = gaitlib::make_gait(final_x, final_y);
    fr_calf_walk = fr_gaits.gait_calf;
    fr_thigh_walk = fr_gaits.gait_thigh;
    fr_hip_walk = std::vector<double>(fr_calf_walk.size(), 0.0);

    // rest of legs should just be stationary
    std::vector<double> rest_x = gaitlib::linspace(stand_x, stand_x, final_x.size());
    std::vector<double> rest_y = gaitlib::linspace(stand_y, stand_y, final_y.size());
    const auto rest_gaits = gaitlib::make_gait(rest_x, rest_y);
    fl_calf_walk = rest_gaits.gait_calf;
    fl_thigh_walk = rest_gaits.gait_thigh;
    fl_hip_walk = std::vector<double>(fr_calf_walk.size(), 0.0);
    rl_calf_walk = rest_gaits.gait_calf;
    rl_thigh_walk = rest_gaits.gait_thigh;
    rl_hip_walk = std::vector<double>(fr_calf_walk.size(), 0.0);
    rr_calf_walk = rest_gaits.gait_calf;
    rr_thigh_walk = rest_gaits.gait_thigh;
    rl_hip_walk = std::vector<double>(fr_calf_walk.size(), 0.0);
  }

  /// @brief Create the standup gait procedure
  void generate_standup()
  {
    // create a standing up movement. Should be defined by # of seconds probably
    // the first thing the real robot does upon startup is angle its hips inwards.
    // it ends at y = some small negative #, x = 0.
    // THEN go from (y = some small negative #, 0) to (stand_y, 0).
    // unsure if this should be in straight line
    double stand_offset = -0.1;

    // This is a part of the standup where the legs are stationary under the hip
    long standup_pts1 = rate_hz * 1;
    const auto stand_up_x1 = gaitlib::linspace(0.0, 0.0, standup_pts1);
    const auto stand_up_y1 = gaitlib::linspace(stand_offset, stand_offset, standup_pts1);

    // This actually stands the robot up
    long standup_pts2 = rate_hz * standup_time;
    const auto stand_up_x2 = gaitlib::linspace(0.0, stand_x, standup_pts2);
    const auto stand_up_y2 = gaitlib::linspace(stand_offset, stand_y, standup_pts2);

    const auto stand_up_x = gaitlib::concatenate(stand_up_x1, stand_up_x2);
    const auto stand_up_y = gaitlib::concatenate(stand_up_y1, stand_up_y2);

    const auto stand_up_gait = gaitlib::make_gait(stand_up_x, stand_up_y);

    fr_calf_stand = stand_up_gait.gait_calf;
    fr_thigh_stand = stand_up_gait.gait_thigh;
    fl_calf_stand = stand_up_gait.gait_calf;
    fl_thigh_stand = stand_up_gait.gait_thigh;
    rr_calf_stand = stand_up_gait.gait_calf;
    rr_thigh_stand = stand_up_gait.gait_thigh;
    rl_calf_stand = stand_up_gait.gait_calf;
    rl_thigh_stand = stand_up_gait.gait_thigh;


    const auto standing_joints = gaitlib::ik(stand_x, stand_y);
    stand_calf = standing_joints.calf_lefty;
    stand_thigh = standing_joints.thigh_lefty;

    const auto liedown_joints = gaitlib::ik(0.0, stand_offset);
    liedown_calf = liedown_joints.calf_lefty;
    liedown_thigh = liedown_joints.thigh_lefty;
  }

  /// @brief Generate "switch" controls between standing and walking
  void generate_transition()
  {
    // At this point the standing and walking gaits have been calculated already
    const auto foot_current = gaitlib::fk(stand_thigh, stand_calf);
    const auto fr_desired = gaitlib::fk(fr_thigh_walk.at(0), fr_calf_walk.at(0));
    const auto fl_desired = gaitlib::fk(fl_thigh_walk.at(0), fl_calf_walk.at(0));
    const auto rr_desired = gaitlib::fk(rr_thigh_walk.at(0), rr_calf_walk.at(0));
    const auto rl_desired = gaitlib::fk(rl_thigh_walk.at(0), rl_calf_walk.at(0));
    // want to go from foot_current to fr_desired coordinates in a smooth and continuous way.
    // I know that both of these are the same y level. Just different x.
    RCLCPP_INFO_STREAM(get_logger(), "Current Stand: " << foot_current.x << " " << foot_current.y);
    RCLCPP_INFO_STREAM(get_logger(), "FR Desired: " << fr_desired.x << " " << fr_desired.y);
    RCLCPP_INFO_STREAM(get_logger(), "FL Desired: " << fl_desired.x << " " << fl_desired.y);
    RCLCPP_INFO_STREAM(get_logger(), "RR Desired: " << rr_desired.x << " " << rr_desired.y);
    RCLCPP_INFO_STREAM(get_logger(), "RL Desired: " << rl_desired.x << " " << rl_desired.y);

    // Generate switch gaits
    const auto switch_npoints = period;

    const auto fr_switch_x = gaitlib::linspace(foot_current.x, fr_desired.x, switch_npoints);
    const auto fr_switch_y = gaitlib::linspace(foot_current.y, fr_desired.y, switch_npoints);
    const auto fr_switch = gaitlib::make_gait(fr_switch_x, fr_switch_y);
    fr_calf_switch = fr_switch.gait_calf;
    fr_thigh_switch = fr_switch.gait_thigh;

    const auto fl_switch_x = gaitlib::linspace(foot_current.x, fl_desired.x, switch_npoints);
    const auto fl_switch_y = gaitlib::linspace(foot_current.y, fl_desired.y, switch_npoints);
    const auto fl_switch = gaitlib::make_gait(fl_switch_x, fl_switch_y);
    fl_calf_switch = fl_switch.gait_calf;
    fl_thigh_switch = fl_switch.gait_thigh;

    const auto rr_switch_x = gaitlib::linspace(foot_current.x, rr_desired.x, switch_npoints);
    const auto rr_switch_y = gaitlib::linspace(foot_current.y, rr_desired.y, switch_npoints);
    const auto rr_switch = gaitlib::make_gait(rr_switch_x, rr_switch_y);
    rr_calf_switch = rr_switch.gait_calf;
    rr_thigh_switch = rr_switch.gait_thigh;

    const auto rl_switch_x = gaitlib::linspace(foot_current.x, rl_desired.x, switch_npoints);
    const auto rl_switch_y = gaitlib::linspace(foot_current.y, rl_desired.y, switch_npoints);
    const auto rl_switch = gaitlib::make_gait(rl_switch_x, rl_switch_y);
    rl_calf_switch = rl_switch.gait_calf;
    rl_thigh_switch = rl_switch.gait_thigh;
  }

  /// @brief Switch between standstill and walking gaits
  /// @param Request: The empty request
  /// @param Response: The empty response
  void walk_forwards(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    if (state == STANDSTILL) {
      RCLCPP_INFO_STREAM(get_logger(), "Switch to Walk");
      // I think this might be why it slips sideways a little bit when starting.
      // generate_transition();
      // state = STANDSTILL_TO_WALK;
      state = WALK;
      timestep = 0;
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "You must be in either Standstill or Walk gait!");
    }
  }

  /// @brief Walk backwards
  /// @param Request: The empty request
  /// @param Response: The empty response
  void walk_backwards(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    if (state == STANDSTILL) {
      RCLCPP_INFO_STREAM(get_logger(), "Start walking backwards!");
      state = BACKWARDS;
      timestep = 0;
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "You must be in either Standstill to start walking backwards!");
    }
  }

  /// @brief start walking left
  /// @param Request: The empty request
  /// @param Response: The empty response
  void walk_left(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    if (state == STANDSTILL) {
      RCLCPP_INFO_STREAM(get_logger(), "Switch to Left");
      state = LEFT;
      timestep = 0;
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "You must be in either Standstill state!");
    }
  }

  /// @brief Start walking right
  /// @param Request: The empty request
  /// @param Response: The empty response
  void walk_right(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    if (state == STANDSTILL) {
      RCLCPP_INFO_STREAM(get_logger(), "Switch to Right");
      state = RIGHT;
      timestep = 0;
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "You must be in either Standstill state!");
    }
  }

  /// @brief Switch gait type between standing/walking
  /// @param Request: The empty request
  /// @param Response: The empty response
  void lie_down(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    if (state == STANDSTILL) {
      RCLCPP_INFO_STREAM(get_logger(), "Lie Down");
      state = LIEDOWN;
      timestep = 0;
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "You must be in the standstill state to lie down!");
    }
  }

  /// @brief If lying down, stand up
  /// @param Request: The empty request
  /// @param Response: The empty response
  void stand_up(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    if (state == LIESTILL) {
      RCLCPP_INFO_STREAM(get_logger(), "Stand Up");
      state = STANDUP;
      timestep = 0;
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "You must be in the liestill state to stand up!");
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

  /// @brief Callback function to save current low_state message
  /// @param msg current low_state message
  void state_cb(const ros2_unitree_legged_msgs::msg::LowState & msg)
  {
    low_state = msg;
  }

  /// @brief State machine that publishes low_cmd message
  void timer_callback()
  {
    // access state with low_state.motor_state.at(JOINT_ID).q
    switch (state) {
      case WAIT:
        {
          if (timestep > initial_delay) {
            RCLCPP_INFO_STREAM(get_logger(), "Stand Up!");
            state = STANDUP;
            // set params that will never change and are the same for all joints:
            // dq, kp, kd (for PD control)
            for (int i = 0; i < gaitlib::NJOINTS; i++) {
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
          // RCLCPP_INFO_STREAM(get_logger(), "error_q:"<<fr_calf_stand.at(timestep) -
          //                                              low_state.motor_state.at(gaitlib::FR_CALF).q);
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
      case STANDSTILL_TO_WALK:
        {
          // transition from standing to walking
          low_cmd.motor_cmd[gaitlib::FR_CALF].q = fr_calf_switch.at(timestep);
          low_cmd.motor_cmd[gaitlib::FR_THIGH].q = fr_thigh_switch.at(timestep);
          low_cmd.motor_cmd[gaitlib::FR_HIP].q = 0.0;

          low_cmd.motor_cmd[gaitlib::FL_CALF].q = fl_calf_switch.at(timestep);
          low_cmd.motor_cmd[gaitlib::FL_THIGH].q = fl_thigh_switch.at(timestep);
          low_cmd.motor_cmd[gaitlib::FL_HIP].q = 0.0;

          low_cmd.motor_cmd[gaitlib::RR_CALF].q = rr_calf_switch.at(timestep);
          low_cmd.motor_cmd[gaitlib::RR_THIGH].q = rr_thigh_switch.at(timestep);
          low_cmd.motor_cmd[gaitlib::RR_HIP].q = 0.0;

          low_cmd.motor_cmd[gaitlib::RL_CALF].q = rl_calf_switch.at(timestep);
          low_cmd.motor_cmd[gaitlib::RL_THIGH].q = rl_thigh_switch.at(timestep);
          low_cmd.motor_cmd[gaitlib::RL_HIP].q = 0.0;

          timestep++;
          // RCLCPP_INFO_STREAM(get_logger(), "timestep " << timestep);
          if (timestep >= static_cast<long>(fr_calf_switch.size())) {
            RCLCPP_INFO_STREAM(get_logger(), "Start Walking!");
            timestep = 0;
            state = WALK;
          }
          break;
        }
      case WALK:
        {
          low_cmd.motor_cmd[gaitlib::FR_CALF].q  = fr_calf_walk.at(timestep);
          low_cmd.motor_cmd[gaitlib::FR_THIGH].q = fr_thigh_walk.at(timestep);
          low_cmd.motor_cmd[gaitlib::FR_HIP].q   = fr_hip_walk.at(timestep);

          low_cmd.motor_cmd[gaitlib::FL_CALF].q  = fl_calf_walk.at(timestep);
          low_cmd.motor_cmd[gaitlib::FL_THIGH].q = fl_thigh_walk.at(timestep);
          low_cmd.motor_cmd[gaitlib::FL_HIP].q   = fl_hip_walk.at(timestep);

          low_cmd.motor_cmd[gaitlib::RR_CALF].q  = rr_calf_walk.at(timestep);
          low_cmd.motor_cmd[gaitlib::RR_THIGH].q = rr_thigh_walk.at(timestep);
          low_cmd.motor_cmd[gaitlib::RR_HIP].q   = rr_hip_walk.at(timestep);

          low_cmd.motor_cmd[gaitlib::RL_CALF].q  = rl_calf_walk.at(timestep);
          low_cmd.motor_cmd[gaitlib::RL_THIGH].q = rl_thigh_walk.at(timestep);
          low_cmd.motor_cmd[gaitlib::RL_HIP].q   = rl_hip_walk.at(timestep);

          timestep++;
          // RCLCPP_INFO_STREAM(get_logger(), "timestep " << timestep);
          if (timestep >= static_cast<long>(fr_calf_walk.size())) {
            RCLCPP_INFO_STREAM(get_logger(), "New Step!");
            step_count++;
            timestep = 0;
            if (step_limit && (step_count >= nsteps)){
              RCLCPP_INFO_STREAM(get_logger(), "Stop Stepping!");
              state = STANDSTILL;
              step_count = 0;
            }
          }
          break;
        }
      case BACKWARDS:
        {
          const auto npoints = static_cast<long>(fr_calf_walk.size());
          // should iterate backwards through the _walk vectors.
          // ie, x_walk.size() - timestep for 0 < timestep < x_walk.size()
          low_cmd.motor_cmd[gaitlib::FR_CALF].q  = fr_calf_walk.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::FR_THIGH].q = fr_thigh_walk.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::FR_HIP].q   = fr_hip_walk.at(npoints - timestep - 1);

          low_cmd.motor_cmd[gaitlib::FL_CALF].q  = fl_calf_walk.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::FL_THIGH].q = fl_thigh_walk.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::FL_HIP].q   = fl_hip_walk.at(npoints - timestep - 1);

          low_cmd.motor_cmd[gaitlib::RR_CALF].q  = rr_calf_walk.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::RR_THIGH].q = rr_thigh_walk.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::RR_HIP].q   = rr_hip_walk.at(npoints - timestep - 1);

          low_cmd.motor_cmd[gaitlib::RL_CALF].q  = rl_calf_walk.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::RL_THIGH].q = rl_thigh_walk.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::RL_HIP].q   = rl_hip_walk.at(npoints - timestep - 1);

          timestep++;
          // RCLCPP_INFO_STREAM(get_logger(), "timestep " << timestep);
          if (timestep >= static_cast<long>(fr_calf_walk.size())) {
            RCLCPP_INFO_STREAM(get_logger(), "New Step Backwards!");
            step_count++;
            timestep = 0;
            if (step_limit && (step_count >= nsteps)){
              RCLCPP_INFO_STREAM(get_logger(), "Stop Stepping!");
              state = STANDSTILL;
              step_count = 0;
            }
          }
          break;
        }
      case RIGHT:
        {
          low_cmd.motor_cmd[gaitlib::FR_CALF].q  = fr_calf_side.at(timestep);
          low_cmd.motor_cmd[gaitlib::FR_THIGH].q = fr_thigh_side.at(timestep);
          low_cmd.motor_cmd[gaitlib::FR_HIP].q   = fr_hip_side.at(timestep);

          low_cmd.motor_cmd[gaitlib::FL_CALF].q  = fl_calf_side.at(timestep);
          low_cmd.motor_cmd[gaitlib::FL_THIGH].q = fl_thigh_side.at(timestep);
          low_cmd.motor_cmd[gaitlib::FL_HIP].q   = fl_hip_side.at(timestep);

          low_cmd.motor_cmd[gaitlib::RR_CALF].q  = rr_calf_side.at(timestep);
          low_cmd.motor_cmd[gaitlib::RR_THIGH].q = rr_thigh_side.at(timestep);
          low_cmd.motor_cmd[gaitlib::RR_HIP].q   = rr_hip_side.at(timestep);

          low_cmd.motor_cmd[gaitlib::RL_CALF].q  = rl_calf_side.at(timestep);
          low_cmd.motor_cmd[gaitlib::RL_THIGH].q = rl_thigh_side.at(timestep);
          low_cmd.motor_cmd[gaitlib::RL_HIP].q   = rl_hip_side.at(timestep);

          timestep++;
          // RCLCPP_INFO_STREAM(get_logger(), "timestep " << timestep);
          if (timestep >= static_cast<long>(fr_calf_side.size())) {
            RCLCPP_INFO_STREAM(get_logger(), "New Step!");
            step_count++;
            timestep = 0;
            if (step_limit && (step_count >= nsteps)){
              RCLCPP_INFO_STREAM(get_logger(), "Stop Stepping!");
              state = STANDSTILL;
              step_count = 0;
            }
          }
          break;
        }
      case LEFT:
        {
          const auto npoints = static_cast<long>(fr_calf_side.size());
          // should iterate backwards through the _side vectors.
          // ie, x_side.size() - timestep for 0 < timestep < x_side.size()
          low_cmd.motor_cmd[gaitlib::FR_CALF].q  = fr_calf_side.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::FR_THIGH].q = fr_thigh_side.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::FR_HIP].q   = fr_hip_side.at(npoints - timestep - 1);

          low_cmd.motor_cmd[gaitlib::FL_CALF].q  = fl_calf_side.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::FL_THIGH].q = fl_thigh_side.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::FL_HIP].q   = fl_hip_side.at(npoints - timestep - 1);

          low_cmd.motor_cmd[gaitlib::RR_CALF].q  = rr_calf_side.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::RR_THIGH].q = rr_thigh_side.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::RR_HIP].q   = rr_hip_side.at(npoints - timestep - 1);

          low_cmd.motor_cmd[gaitlib::RL_CALF].q  = rl_calf_side.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::RL_THIGH].q = rl_thigh_side.at(npoints - timestep - 1);
          low_cmd.motor_cmd[gaitlib::RL_HIP].q   = rl_hip_side.at(npoints - timestep - 1);

          timestep++;
          // RCLCPP_INFO_STREAM(get_logger(), "timestep " << timestep);
          if (timestep >= static_cast<long>(fr_calf_side.size())) {
            RCLCPP_INFO_STREAM(get_logger(), "New Step Backwards!");
            step_count++;
            timestep = 0;
            if (step_limit && (step_count >= nsteps)){
              RCLCPP_INFO_STREAM(get_logger(), "Stop Stepping!");
              state = STANDSTILL;
              step_count = 0;
            }
          }
          break;
        }
      case RESET:
        {
          for (int i = 0; i < gaitlib::NJOINTS; i++) {
            low_cmd.motor_cmd[i].tau = 0.0;
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
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomGait>());
  rclcpp::shutdown();
  return 0;
}
