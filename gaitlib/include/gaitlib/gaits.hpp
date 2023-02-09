#ifndef GAITS_INCLUDE_GUARD_HPP
#define GAITS_INCLUDE_GUARD_HPP
/// \file gaits.hpp
/// \brief Helper functions for generating gait trajectories

#include<vector> // contains forward definitions for iostream objects
#include<cmath>

namespace gaitlib{

  // The following Joint IDs were taken from Unitree's quadruped.hpp

  /// @brief Front Right Hip Joint
  constexpr int FR_0 = 0;
  /// @brief Front Right Thigh Joint
  constexpr int FR_1 = 1;
  /// @brief Front Right Calf Joint
  constexpr int FR_2 = 2;

  /// @brief Front Left Hip Joint
  constexpr int FL_0 = 3;
  /// @brief Front Left Thigh Joint
  constexpr int FL_1 = 4;
  /// @brief Front Left Calf Joint
  constexpr int FL_2 = 5;

  /// @brief Rear Right Hip Joint
  constexpr int RR_0 = 6;
  /// @brief Rear Right Thigh Joint
  constexpr int RR_1 = 7;
  /// @brief Rear Right Calf Joint
  constexpr int RR_2 = 8;

  /// @brief Rear Left Hip Joint
  constexpr int RL_0 = 9;
  /// @brief Rear Left Thigh Joint
  constexpr int RL_1 = 10;
  /// @brief Rear Left Calf Joint
  constexpr int RL_2 = 11;

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
}

#endif