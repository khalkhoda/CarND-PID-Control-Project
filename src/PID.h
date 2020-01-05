#ifndef PID_H
#define PID_H

#include <iostream>
#include <uWS/uWS.h>
#include "json.hpp"
#include <chrono>

#define TWIDDLE_ENABLE          0
#define NB_CTE_COUNT            800 // 300
#define BEST_CTE_ERROR_MAX      1000000

using namespace std;
using nlohmann::json;
using namespace std::chrono;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Kd_, double Ki_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);
  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   Twiddle to optimize PID parameters

   */
#if TWIDDLE_ENABLE
   void twiddle(double tol, double cte, uWS::WebSocket<uWS::SERVER> &ws);
#endif


 private:
  /**
   * PID Errors
   */
  double p_error;
  double d_error;
  double i_error;

  /**
   * PID Coefficients
   */
  double Kp;
  double Kd;
  double Ki;

#if (TWIDDLE_ENABLE == 1)
  int nb_cte = 0;
  double cte2_sum = 0.0;
  double best_cte_error = BEST_CTE_ERROR_MAX;
  double dp[3] = {1.0, 1.0, 1.0};
  int ind = -1; // Index of parameter being tuned
  int exe_statement = 0; // Control variable of switch statement

#endif

};

#endif  // PID_H
