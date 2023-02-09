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

}
