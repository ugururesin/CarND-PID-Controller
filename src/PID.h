#ifndef PID_H
#define PID_H

class PID {
public:
  //PID ERRORS
  double p_error;
  double i_error;
  double d_error;

  //PREVIOUS CTE (FOR DERIVATIVE)
  double prev_cte;
  
  //COUNTERS
  long counter;
  double errorSum;
  double minError;
  double maxError;
  
  //TAU COEFFICIENTS
  double Kp;
  double Ki;
  double Kd;
  
  //Constructor
  PID();

  //Destructor
  virtual ~PID();

  //INITIALIZING PID
  void Init(double Kp, double Ki, double Kd);

  
  //ERROR UPDATE 
  void UpdateError(double cte);

  //ERRORS
  double TotalError(); 		//total
  double AverageError();	//average (errorSum/counter)
  double MinError();
  double MaxError();
};

#endif /* PID_H */