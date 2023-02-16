#include "gaitlib/gaits.hpp"

namespace gaitlib{

long factorial(long n){
  if ((n == 1) || (n == 0)){
    return 1;
  } else {
    return n * factorial(n - 1);
  }
}

long choose(long n, long k){
  return factorial(n)/(factorial(k)*factorial(n-k));
}

double p_i(long i, long order, double t, double point){
  return choose(order,i) * pow((1-t),(order-i)) * pow(t,i)*point;
}

std::vector<double> bezier(std::vector<double> points, long npoints){
  long order = points.size();
  std::vector<double> curve;
  for (long p = 0; p<npoints; p++){
    double t = static_cast<double>(p)/static_cast<double>(npoints);
    double current = 0;
    for (long i=0; i<order; i++){
      current += p_i(i,order-1,t,points.at(i));
    }
    curve.push_back(current);
  }
  return curve;
}

std::vector<double> modulate(std::vector<double> ref, double scale){
  size_t len = ref.size();
  // unsure about this typecasting
  size_t break_at = len * scale;
  // source: 
  // https://stackoverflow.com/questions/9811235/best-way-to-split-a-vector-into-two-smaller-arrays
  std::vector<double> split1(ref.begin(), ref.begin() + break_at);
  std::vector<double> split2(ref.begin() + break_at, ref.end());
  // now put these back together in the opposite ordering. source:
  // https://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  // vector1.insert( vector1.end(), vector2.begin(), vector2.end() );
  split2.insert( split2.end(), split1.begin(), split1.end() );
  return split2;
}


std::vector<double> linspace(double start, double end, double npoints){
  std::vector<double> res;
  double step = (end - start) / npoints;
  double curr = start;
  for (int i = 0; i < npoints; i++) {
    res.push_back(curr);
    curr += step;
  }
  return res;
}

std::vector<double> stance(std::vector<double> xcoords, double delta, double y_level){
  std::vector<double> res;
  double len = xcoords.front() - xcoords.back();
  for(long i=0; i<static_cast<long>(xcoords.size()); i++){
    double ycoord = -1.0*delta*cos((M_PI/len)*xcoords.at(i)) + y_level;
    res.push_back(ycoord);
  }
  return res;
}

std::vector<double> concatenate(std::vector<double> v1, std::vector<double> v2){
  // https://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  v1.insert( v1.end(), v2.begin(), v2.end() );
  return v1;
}

double get_theta_calf(double theta_thigh, double x)
  {
    return asin(-x / LEG_LENGTH - sin(theta_thigh)) - theta_thigh;
  }

IKResult ik(double x, double y)
  {
    double alpha = acos(sqrt(x * x + y * y) / (2 * LEG_LENGTH));
    double gamma = atan(x / y);
    double theta_thigh_left = gamma + alpha;
    double theta_calf_left = get_theta_calf(theta_thigh_left, x);
    double theta_thigh_right = gamma - alpha;
    double theta_calf_right = get_theta_calf(theta_thigh_right, x);
    IKResult res{theta_thigh_left, theta_calf_left,
                 theta_thigh_right, theta_calf_right};
    return res;
  }

MyGait make_gait(std::vector<double> desired_x, std::vector<double> desired_y)
  {
    std::vector<double> gait_calf;
    std::vector<double> gait_thigh;
    if (desired_x.size() == desired_y.size()) {
      // std::vector<double> ik_result;
      for (size_t i = 0; i < desired_x.size(); i++) {
        const auto ik_result = ik(desired_x[i], desired_y[i]);
        // Here we just arbitrarily choose left result (it maintained joint limits in my example)
        // The left thigh result is 0th element and calf result is 1st
        // Keep rest of joints stationary for now
        gait_calf.push_back(ik_result.calf_lefty);
        gait_thigh.push_back(ik_result.thigh_lefty);
      }
    }
    return MyGait{gait_calf, gait_thigh};
  }

}
