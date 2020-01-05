#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Kd_, double Ki_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   Kp = Kp_;
   Kd = Kd_;
   Ki = Ki_;

   p_error = 0.0; // proportional error (AKA cte)
   d_error = 0.0; // derivative error (Change of cte over time)
   i_error = 0.0; // integration error (sum of cte over time)

#if TWIDDLE_ENABLE
   cte2_sum = 0.0;
   nb_cte = 0;
#endif
}

void PID::UpdateError(double cte) {
   d_error = cte - p_error;
   p_error = cte;
   i_error += cte;
}

double PID::TotalError() {
  double total_error = -Kp*p_error -Kd*d_error - Ki*i_error; // AKA steer angle
  return total_error;
}

#if TWIDDLE_ENABLE
void PID::twiddle(double tol, double cte, uWS::WebSocket<uWS::SERVER> &ws)
{
  cte2_sum += cte*cte;
  cout << "\rStep: " << nb_cte << "/" << NB_CTE_COUNT <<flush;
  ++nb_cte;
  /*The new cte error is ready, proceed with the new twiddle cycle*/
  if (nb_cte > NB_CTE_COUNT)
  {
    printf("\n");
    double cte_error = cte2_sum / nb_cte;
    double p[3] = {Kp, Kd, Ki};
    printf("cte2_sum: %5.7f\n", cte2_sum);
    printf("nb_cte: %d\n", nb_cte);

    double sum_dp = dp[0] + dp[1] + dp[2];
    if (sum_dp > 0.02)
    {
      switch (exe_statement) {
        case 1: {
          if (cte_error < best_cte_error)
          {
            best_cte_error = cte_error;
            dp[ind] *= 1.1;
            exe_statement = 0;
          }
          else
          {
            p[ind] -= dp[ind]; // Undo increase step
            p[ind] -= dp[ind]; // decrease step
            exe_statement = 2;
          }
          break;
        }
        case 2: {
          // printf("%s\n", "if (cte_error < best_cte_error)");
          if (cte_error < best_cte_error)
          {
            // printf("%s\n", "best_cte_error = cte_error;)");
            best_cte_error = cte_error;
            // printf("%s\n", "dp[ind] *= 1.1;");
            dp[ind] *= 1.1;
          }
          else
          {
            p[ind] = p[ind] + dp[ind]; // Undo decrease step
            dp[ind] *= 0.9;
          }
          exe_statement = 0;
          break;
        }
        default: {
          printf("%s\n", "Shall never be printed");
        }
      } // switch
      if (exe_statement == 0)
      {
        ind = (ind + 1) % 3;
        printf("\nTuning p[%d] parameter now ..............................\n", ind);
        p[ind] = p[ind] + dp[ind]; // Increase step
        exe_statement = 1;
      }
      Init(p[0], p[1], p[2]);
      printf("best_cte_error: %5.7f\n", best_cte_error);
      printf(" p vector: p[0]  = %5.7f, p[1]  = %5.7f, p[2]  = %5.7f\n", Kp, Kd, Ki);
      printf("dp vector: dp[0] = %5.7f, dp[1] = %5.7f, dp[2] = %5.7f\n", dp[0], dp[1], dp[2]);

      /* Test the new set of parameters*/
      string msg = "42[\"reset\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    } // if (sum_dp > 0.02)
    else
    {
      printf("%s\n", "Converged to the required tolerance 0.02");
      printf("dp vector: dp[0] = %5.7f, dp[1] = %5.7f, dp[2] = %5.7f\n", dp[0], dp[1], dp[2]);
      printf("Optimal parameters are Kp = %f, Kd = %f, Ki = %f\n", Kp, Kd, Ki);

      // auto stop = high_resolution_clock::now();
      // auto duration = duration_cast<seconds>(stop - start);
      // cout << "Prediction step time "<<duration.count() << " seconds"<<endl;
    }
  }
}
#endif
