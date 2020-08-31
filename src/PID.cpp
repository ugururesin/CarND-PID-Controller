#include "PID.h"
#include <algorithm>
/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

//Initialize PID coefficients (and errors, if needed)
void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  //Previous CTE (for derivative)
  prev_cte = 0.0;

  //Initialize counters
  counter = 0;
  errorSum = 0.0;
  minError = std::numeric_limits<double>::max();
  maxError = std::numeric_limits<double>::min();
}

void PID::UpdateError(double cte) {

  p_error = cte;			//proportional error (CTE)
  i_error += cte;			//integral error (sum of CTEs)
  d_error = cte - prev_cte;	//diferential error (current-previous CTE)
  prev_cte = cte;

  errorSum += cte;
  
  counter++;
  if ( cte > maxError ) {
    maxError = cte;
  }
  if ( cte < minError ) {
    minError = cte;
  }
}

double PID::TotalError() {
  double total_error = p_error * Kp + i_error * Ki + d_error * Kd; //pid equation
  return total_error;
}

double PID::AverageError() {
  return errorSum/counter;
}

double PID::MinError() {
  return minError;
}

double PID::MaxError() {
  return maxError;
}