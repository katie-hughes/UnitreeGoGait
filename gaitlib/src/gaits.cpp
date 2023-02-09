#include "gaitlib/gaits.hpp"

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

std::vector<double> bezier(std::vector<double> points, double step){
  long order = points.size();
  std::vector<double> curve;
  for (double t = 0.0; t<=1; t+=step){
    double current = 0;
    for (long i=0; i<order; i++){
      current += p_i(i,order-1,t,points.at(i));
    }
    curve.push_back(current);
  }
  return curve;
}
