# CarND-MPC ( Model-Predictive-Control)

In this project, I have implemented Model-Predictive-Control to keep the car on track.

### Model:

I have used Kinematic bicycle model. State includes position( x, y), velocity( v), orientation( &Psi;), cross track error and orientation error. Below are the equations used to predict state. 

#### x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(&Psi;<sub>t</sub>) * &Delta;t

#### y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(&Psi;<sub>t</sub>) * &Delta;t

#### &Psi;<sub>t+1</sub> = &Psi;<sub>t</sub> + v<sub>t</sub> / Lf * &delta;<sub>t</sub> * &Delta;t

#### v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> * &Delta;t

#### cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub> * sin(e&Psi;<sub>t</sub>) * &Delta;t

#### e&Psi;<sub>t+1</sub> = &Psi;<sub>t</sub> - &Psi;<sub>des(t)</sub> + v<sub>t</sub> * &delta;<sub>t</sub> / Lf * &Delta;t
  
### Timestep Length and Elapsed Duration (N & &Delta;t):

I have used number of timestep(N) = 10 and dt = 0.1. After trying several values with higher and lower N and dt, it seems that these values work better.

### Polynomial Fitting and MPC Preprocessing:

I have convert way points into car coordinate system to simplify mathematics involved. Then I have used 3^rd^ degree polynomial to fit to way points. 3^rd^ degree polynomial is enough becuase we will not have more than 2 curves.

### Model Predictive Control with Latency:

I have used 100ms as latency of system for computation. Using latency takes into account the calculation time and delay in command execution.
