# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Describe the effect each of the P, I, D components.

* Proportional component P
The component p corrects the control signal by a value proportional to the error (the so called "cross track error"). That is, the bigger the error, the bigger the correct signal should be to reduce the error.

* Integral component I
The component I corrects the control signal by a value proportional to the integration (summation) of the error over time. Component I removes any systematic offset in the error measurement.

* Differential component D
The component D corrects the speed at which the control signal gets corrected by component P. It reduces overshooting range.

## Describe how the final hyperparameters were chosen.

The Twiddle algorithm described in the lesson was used. It starts off by set of parameters of 0s. Kp = 0, Kd = 0, Ki = 0.
* Calculate the mean square cross track error (CTE1)
* Increase one parameter by a small value
* Calculate the mean square cross track error (CTE2)
* If CTE2 < CTE1
  keep the new parameter
  increase that small value
* Else
  try reduce that parameter by a small value
* Calculate the mean square cross track error (CTE2)
* If CTE2 < CTE1
  keep the new parameter
* Else
  decrease that small value associated with the current parameter
* Try different parameter
* Repeat until convergence

The found parameters are then saved and used to test the PID controller.
The final parameters are:

Kp = 0.199022, Kd = 1.230227, Ki = 0.007280

It can be seen that the Kp is relatively small compared to Kd. That is to reduce the error of cte using Kp but it does that slowly by means of Kd. The integral component is very small compared to the other two parameters. It indicates that the measurement done by the simulator is quite accurate and no large offset has been observed.
