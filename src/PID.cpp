#include "PID.h"
#include <vector>
#include <numeric>
#include <algorithm>

#define TOLERANCE 0.001

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
    it_counter = 0;
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
    it_counter++;

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

double PID::TotalError(double Kp_, double Ki_, double Kd_)
{
    return -Kp_ * p_error - Kd_ * d_error - Ki_ * i_error;
}

void PID::Twiddle()
{
    std::vector<double> p = {0.0, 0.0, 0.0};
    std::vector<double> dp = {1.0, 1.0, 1.0};

    double best_error = TotalError();
    double error = error_;

    while (std::accumulate(dp.begin(), dp.end(), 0) > TOLERANCE)
    {
        for (int i = 0; i < p.size(); i++)
        {
            p[i] += dp[i];
            error = TotalError(p[0], p[1], p[2]);

            if (error < best_error)
            {
                best_error = error;
                dp[i] *= 1.1;
            }
            else
            {
                p[i] -= 2 * dp[i];
                error = TotalError(p[0], p[1], p[2]);

                if (error < best_error)
                {
                    best_error = error;
                    dp[i] *= 1.1;
                }
                else
                {
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                }
            }
        }
    }
    Kp = p[0];
    Ki = p[1];
    Kd = p[2];

    error_ = best_error;
}