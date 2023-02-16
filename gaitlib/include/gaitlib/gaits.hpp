#ifndef GAITS_INCLUDE_GUARD_HPP
#define GAITS_INCLUDE_GUARD_HPP
/// \file gaits.hpp
/// \brief Helper functions for generating gait trajectories

#include<vector> // contains forward definitions for iostream objects
#include<cmath>

namespace gaitlib{

  // The following Joint IDs were taken from Unitree's quadruped.hpp
  // these are used to index the appropriate joints in their low_cmd message.

  /// @brief Length of each leg segment (thigh->calf,calf->foot) in m.
  constexpr double LEG_LENGTH = 0.213;

  /// @brief Front Right Hip Joint
  constexpr int FR_HIP = 0;
  /// @brief Front Right Thigh Joint
  constexpr int FR_THIGH = 1;
  /// @brief Front Right Calf Joint
  constexpr int FR_CALF = 2;

  /// @brief Front Left Hip Joint
  constexpr int FL_HIP = 3;
  /// @brief Front Left Thigh Joint
  constexpr int FL_THIGH = 4;
  /// @brief Front Left Calf Joint
  constexpr int FL_CALF = 5;

  /// @brief Rear Right Hip Joint
  constexpr int RR_HIP = 6;
  /// @brief Rear Right Thigh Joint
  constexpr int RR_THIGH = 7;
  /// @brief Rear Right Calf Joint
  constexpr int RR_CALF = 8;

  /// @brief Rear Left Hip Joint
  constexpr int RL_HIP = 9;
  /// @brief Rear Left Thigh Joint
  constexpr int RL_THIGH = 10;
  /// @brief Rear Left Calf Joint
  constexpr int RL_CALF = 11;

  /// @brief Way of packaging the joint trajectories for both thigh and calf.
  struct MyGait {
    /// @brief Calf trajectory
    std::vector<double> gait_calf;
    /// @brief Thigh trajectory
    std::vector<double> gait_thigh;
  };

  /// @brief 
  /// @param n 
  /// @return 
  long factorial(long n);

  /// @brief 
  /// @param n 
  /// @param k 
  /// @return 
  long choose(long n, long k);

  /// @brief 
  /// @param i 
  /// @param order 
  /// @param t 
  /// @param point 
  /// @return 
  double p_i(long i, long order, double t, double point);

  /// @brief Generate a bezier curve
  /// @param points: Control points for the bezier
  /// @param npoints: number of points in the resulting curve
  /// @return bezier curve following the control points
  std::vector<double> bezier(std::vector<double> points, long npoints);

  /// @brief Modulte a vector
  /// @param ref reference input vector
  /// @param scale scale to modulate by: between 0 and 1
  /// @return new modulated vector
  std::vector<double> modulate(std::vector<double> ref, double scale);

  /// @brief an implementation of the numpy linspace function.
  /// Used for forming/ testing simple linear trajectories
  /// @param start start of trajectory
  /// @param end end of trajectory
  /// @param npoints number of points in the trajectory
  /// @return vector of size npoints ranging from start to end
  std::vector<double> linspace(double start, double end, double npoints);

  /// @brief Generate the sinusoidal "stance" section of the gait where the foot is on the ground
  /// @param xcoords vector of the desired x coordinates of the stance
  /// should start at the final bezier control point and end at the first bezier control point
  /// such that it connects both ends of the curve!
  /// @param delta amplitude of the sinusoid
  /// @param y_level y coordinate of starting/ending bezier control points
  /// @return vector of the stance section of the gait
  std::vector<double> stance(std::vector<double> xcoords, double delta, double y_level);

  /// @brief concatenate two vectors
  /// @param v1 first vector
  /// @param v2 second vector
  /// @return their concatenation, as a new vector
  std::vector<double> concatenate(std::vector<double> v1, std::vector<double> v2);

  /// @brief helper to calculate the calf joint of the leg given theta_thigh and x.
  /// @param theta_thigh thigh angle (radians)
  /// @param x desired x coordinate WRT hip as origin (m)
  /// @return calf angle (radians)
  double get_theta_calf(double theta_thigh, double x);

  /// @brief Return joint angles that put the foot at a desired x,y for leg segment l
  /// @param x desired x coordinate WRT hip as origin (m)
  /// @param y desired y coordinate WRT hip as origin (m)
  /// @return vector [theta_thigh1, theta_calf1, theta_thigh2, theta_calf2]
  /// where (theta_thigh1, theta_calf1) and (theta_thigh2, theta_calf2) are the 2 solutions.
  std::vector<double> ik(double x, double y);

  /// @brief Create joint trajectories for calf and thigh for desired foot trajectory
  /// @param desired_x desired x trajectory of foot
  /// @param desired_y desired y trajectory of foot
  /// @return the calf and thigh joint trajectories
  MyGait make_gait(std::vector<double> desired_x, std::vector<double> desired_y);
}

#endif