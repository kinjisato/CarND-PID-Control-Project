#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
    
    double total_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
    
    /*
     * cross track error
     */
    double prev_cte;
    double int_cte;
    double max_int_cte;
    
    /*
     * steer angle
     */
    double target_steer;
    
    /*
     * twiddle variables
     */
    bool twiddle_enable;
    int steps;
    int num_loop_twiddle;
    double twiddle_err;
    double twiddle_best_err;
    std::vector<double> dp;
    std::vector<double> pid_param;
    int dp_param_index;
    
    
    

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
    
    /*
     * Calculate the target steer.
     */
    double TargetSteer();
    
    /*
     * Twiddle
     */
    void Twiddle_PID(double cte);
    void UpdatePIDParam(int index, double value);
};

#endif /* PID_H */
