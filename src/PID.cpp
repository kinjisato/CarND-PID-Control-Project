#include "PID.h"

#include <iostream>
#include <cmath>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    // Initialize error
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    
    // for anti windup
    max_int_cte = 100000.0;
    
    target_steer = 0.0;
    
    twiddle_enable = false;
    steps = 0;
    num_loop_twiddle = -2;
    twiddle_err = 0.0;
    twiddle_best_err = numeric_limits<double>::max();
    pid_param = {Kp, Ki, Kd};
    dp = {0.1*Kp, 0.1*Ki, 0.1*Kd};
    dp_param_index = 0;
    
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    
    // for anti windup
    if(i_error > max_int_cte){
        i_error = max_int_cte;
    } else if(i_error < -max_int_cte){
        i_error = -max_int_cte;
    }
}

double PID::TotalError() {
    total_error = p_error + i_error + d_error;
    
    return total_error;
}

double PID::TargetSteer(){
    target_steer = -Kp * p_error - Kd * d_error - Ki * i_error;
    
    // clip target steer in [-1.0 1.0]
    if(target_steer > 1.0){
        target_steer = 1.0;
    } else if(target_steer < -1.0){
        target_steer = -1.0;
    }
    
    return target_steer;
}

void PID::Twiddle_PID(double cte){
    twiddle_err += pow(cte,2);
    
    if((steps%3000)==0){
        if(num_loop_twiddle < 0){
            num_loop_twiddle +=1;
            twiddle_err = 0;
        } else if(num_loop_twiddle == 0){
            // update best
            twiddle_best_err = twiddle_err;
            // reset error
            twiddle_err = 0;
            // update hyper param
            UpdatePIDParam(dp_param_index, dp[dp_param_index]);
            // set for next loop
            num_loop_twiddle = 1;

        } else if(num_loop_twiddle == 1){
            if(twiddle_err < twiddle_best_err){
                // update best
                twiddle_best_err = twiddle_err;
                // reset error
                twiddle_err = 0;
                // update dp
                dp[dp_param_index] *= 1.1;
                
                // got to next param
                steps = 0;
                num_loop_twiddle = 1;
                dp_param_index = (dp_param_index +1)%3;
                UpdatePIDParam(dp_param_index, dp[dp_param_index]);
            } else {
                UpdatePIDParam(dp_param_index, (-2.0 * dp[dp_param_index]));
                num_loop_twiddle = 2;
                // reset error
                twiddle_err = 0;
            }
        } else if(num_loop_twiddle == 2){
            if(twiddle_err < twiddle_best_err){
                // update best
                twiddle_best_err = twiddle_err;
                dp[dp_param_index] *= 1.1;
            } else {
                UpdatePIDParam(dp_param_index, dp[dp_param_index]);
                dp[dp_param_index] *= 0.9;
            }
            // reset error
            twiddle_err = 0;
            // got to next param
            steps = 0;
            num_loop_twiddle = 1;
            dp_param_index = (dp_param_index +1)%3;
            UpdatePIDParam(dp_param_index, dp[dp_param_index]);
        }
    }
    steps++;
    std::cout<< "steps : " << steps <<std::endl;
    std::cout<< "num_loop : " << num_loop_twiddle <<std::endl;
    std::cout<< "param index : " << dp_param_index <<std::endl;
    std::cout<< "err : " << twiddle_err <<std::endl;
    std::cout<< "best err : " << twiddle_best_err <<std::endl;
    std::cout << "PID value : " << Kp << ":" << Ki << ":" << Kd << std::endl;
}

void PID::UpdatePIDParam(int index, double value){
    if(index == 0){
        Kp += value;
    } else if(index == 1){
        Ki += value;
    } else if(index == 2){
        Kd += value;
    }
    std::cout<< "Kp : " << Kp << "/ Ki : " << Ki << "/ Kd : " << Kd <<std::endl;
    
}
