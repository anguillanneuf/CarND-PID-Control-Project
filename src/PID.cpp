#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this -> Kp = Kp;
    this -> Ki = Ki;
    this -> Kd = Kd;

    p_error = i_error = d_error = 0.0;

    /* Twiddle params
     */
    i_step = 0;
    min_steps = 100;
    num_steps = 300;

    dKp = Kp/4.0;
    dKi = 0.0;
    dKd = 1.0;

    best_err = 1e8;
    total_err = 0.0;

    twiddle_index = 0;
    start_twiddle = true;
    increase_coefficient = false;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    i_step += 1;
    if(i_step > min_steps) total_err += cte * cte;
}

double PID::Output(){
    return -Kp * p_error - Ki * i_error - Kd * d_error;
}

double PID::TotalError() {
    if(i_step > min_steps) total_err /= i_step - min_steps;
    return total_err;
}

// Make this tolerance bigger if you are timing out!
void PID::Twiddle(double tolerance) {
    if(i_step >= num_steps){

        i_step = 0;

        double cur_err = TotalError();

        if(dKp + dKi + dKd > tolerance){

            if(start_twiddle){

                best_err = cur_err;
                Kp += dKp;
                increase_coefficient = false;
                start_twiddle = false;

            }else{
                if(cur_err < best_err){
                    best_err = cur_err;

                    switch (twiddle_index % 3) {
                        case 0:
                            dKp *= 1.1;
                        case 1:
                            dKi *= 1.1;
                        case 2:
                            dKd *= 1.1;
                    }

                    twiddle_index += 1;
                    increase_coefficient = true;

                    switch (twiddle_index % 3) {
                        case 0:
                            Kp += dKp;
                        case 1:
                            Ki += dKi;
                        case 2:
                            Kd += dKd;
                    }

                }else{
                    if(increase_coefficient){
                        increase_coefficient = false;

                        switch (twiddle_index % 3) {
                            case 0:
                                Kp -= 2 * dKp;
                            case 1:
                                Ki -= 2 * dKi;
                            case 2:
                                Kd -= 2 * dKd;
                        }
                    }else{
                        increase_coefficient = true;

                        switch (twiddle_index % 3) {
                            case 0:
                                Kp += dKp;
                                dKp *= 0.9;
                            case 1:
                                Ki += dKi;
                                dKi *= 0.9;
                            case 2:
                                Kd += dKd;
                                dKd *= 0.9;
                        }

                        twiddle_index += 1;

                        switch (twiddle_index % 3){
                            case 0:
                                Kp += dKp;
                            case 1:
                                Ki += dKi;
                            case 2:
                                Kd += dKd;
                        }
                    }
                }
            }

        }else
            return;
        total_err = 0.0;
    }
}
