#include "PID.h"
#include <vector>
#include <numeric>
#include <algorithm>

#define TOLERANCE 0.001
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
    Kd = Kd_;
    Ki = Ki_;
    Kp = Kp_;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    cte_prev_ = 0.0;
    error_ = 0.0;
}

void PID::UpdateError(double cte)
{
    double max_error = std::numeric_limits<double>::max();
    double min_error = std::numeric_limits<double>::min();

    p_error = cte;
    i_error += cte;
    d_error = cte - cte_prev_;
    cte_prev_ = cte;

    error_ += cte;

    if (error_ > max_error)
    {
        error_ = max_error;
    }

    if (error_ < min_error)
    {
        error_ = min_error;
    }
}

double PID::TotalError()
{
    return -Kp * p_error - Kd * d_error - Ki * i_error;
}