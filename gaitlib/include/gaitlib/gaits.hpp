#ifndef GAITS_INCLUDE_GUARD_HPP
#define GAITS_INCLUDE_GUARD_HPP
/// \file gaits.hpp
/// \brief Helper functions for generating gait trajectories

#include<vector> // contains forward definitions for iostream objects
#include<cmath>

namespace gaitlib{

  /// @brief Length of each leg segment (thigh->calf,calf->foot) in m.
  constexpr double LEG_LENGTH = 0.213;

  /// @brief Number of joints in the unitree go1
  constexpr double NJOINTS = 12;

  // The following Joint IDs were taken from Unitree's quadruped.hpp
  // these are used to index the appropriate joints in their low_cmd message.

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

  // Joint Limits
  
  /// @brief Lower bound of calf joint (radians)
  constexpr double CALF_LO = -2.82;
  /// @brief Upper bound of calf joint (radians)
  constexpr double CALF_HI = -0.89;
  /// @brief Lower bound of thigh joint (radians)
  constexpr double THIGH_LO = -0.69;
  /// @brief Upper bound of thigh joint (radians)
  constexpr double THIGH_HI = 4.50;
  /// @brief Lower bound of hip joint (radians)
  constexpr double HIP_LO = -0.86;
  /// @brief Upper bound of hip joint (radians)
  constexpr double HIP_HI = 0.86;

  /// @brief Way of packaging the joint trajectories for both thigh and calf.
  struct MyGait {
    /// @brief Calf trajectory
    std::vector<double> gait_calf;
    /// @brief Thigh trajectory
    std::vector<double> gait_thigh;
  };

  /// @brief Way of packaging the joint trajectories for thigh, calf, and hip.
  struct MyGait3D {
    /// @brief Calf trajectory
    std::vector<double> gait_calf;
    /// @brief Thigh trajectory
    std::vector<double> gait_thigh;
    /// @brief Hip Trajectory
    std::vector<double> gait_hip;
  };

  /// @brief Way of packaging the solution for the IK of the legs
  /// (thigh_lefty, calf_lefty) is one solution and (thigh_righty, calf_righty) is the other
  struct IKResult {
    /// @brief lefty thigh angle in radians
    double thigh_lefty;
    /// @brief lefty calf angle in radians
    double calf_lefty;
    /// @brief righty thigh angle in radians
    double thigh_righty;
    /// @brief righty calf angle in radians
    double calf_righty;
  };

  /// @brief IK result involving the hip joint
  struct IKResult3D {
    /// @brief Hip angle (1 solution)
    double hip;
    /// @brief Calf and Thigh angles (2 solutions)
    IKResult leg;
  };

  /// @brief Holds Euclidean coordinates WRT the hip joint
  struct Coords {
    /// @brief X coordinate relative to hip (m)
    double x;
    /// @brief Y coordinate relative to hip (m)
    double y;
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
  /// @return the set of two solutions (righty and lefty) for joint angles thigh and calf.
  IKResult ik(double x, double y);

  /// @brief Compute the real world coordinates from the leg joint angles
  /// @param theta_thigh thigh angle in radians
  /// @param theta_calf calf angle in radians
  /// @return x,y coordinates of foot with respect to hip joint.
  Coords fk(double theta_thigh, double theta_calf);

  /// @brief Create joint trajectories for calf and thigh for desired foot trajectory
  /// @param desired_x desired x trajectory of foot
  /// @param desired_y desired y trajectory of foot
  /// @return the calf and thigh joint trajectories
  MyGait make_gait(std::vector<double> desired_x, std::vector<double> desired_y);

  /// @brief IK result of a 3D desired position for RIGHT legs
  /// (for left legs, negate the hip angle!)
  /// @param x desired x coordinate WRT hip as origin (m)
  /// @param y desired y coordinate WRT hip as origin (m)
  /// @param z desired z coordinate WRT hip as origin (m)
  /// @return IK solution: 1 hip angle, set of 2 calf/thigh angles.
  IKResult3D ik3D(double x, double y, double z);

  /// @brief Create joint trajectories for calf, thigh, and hip for desired foot trajectory
  /// @param desired_x desired x trajectory of foot
  /// @param desired_y desired y trajectory of foot
  /// @param desired_z desired z trajectory of foot
  /// @return the calf and thigh joint trajectories
  MyGait3D make_3Dgait(std::vector<double> desired_x, std::vector<double> desired_y, std::vector<double> desired_z);
}

#endif