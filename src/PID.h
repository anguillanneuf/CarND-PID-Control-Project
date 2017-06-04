#ifndef PID_H
#define PID_H

class PID {
public:
    /*
    * Errors
    */
    double p_error;
    double i_error;
    double d_error;

    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;

    /*
     * Params for Twiddle
     */
    int i_step;
    int min_steps;
    int num_steps;

    double dKp;
    double dKd ;
    double dKi;

    double total_err;
    double best_err;

    int twiddle_index;
    bool start_twiddle;
    bool increase_coefficient;

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
    * Calculate PID control.
    */
    double Output();

    /*
     * Twiddle to find best Kp, Ki, Kd
     */
    void Twiddle(double tolerance);

};

#endif /* PID_H */
