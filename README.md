# CarND-MPC ( Model-Predictive-Control)

In this project, I have implemented Model-Predictive-Control to keep the car on track.

### Model:

I have used Kinematic bicycle model. State includes position( x, y), velocity( v), orientation( psi), cross track error and orientation error. Below are the equations used to predict state. 

  // x~t+1~ = x~t~ + v~t~ * cos(psi~t~) * dt
  // y~t+1~ = y~t~ + v~t~ * sin(psi~t~) * dt
  // psi~t+1~ = psi~t~ + v~t~ / Lf * delta~t~ * dt
  // v~t+1~ = v~t~ + a~t~ * dt
  // cte~t+1~ = f(x~t~) - y~t~ + v~t~ * sin(epsi~t~) * dt
  // epsi~t+1~ = psi~t~ - psi~des(t)~ + v~t~ * delta~t~ / Lf * dt
  
### Timestep Length and Elapsed Duration (N & dt):

I have used number of timestep(N) = 10 and dt = 0.1. After trying several values with higher and lower N and dt, it seems that these values work better.

### Polynomial Fitting and MPC Preprocessing:

I have convert way points into car coordinate system to simplify mathematics involved. Then I have used 3^rd^ degree polynomial to fit to way points. 3^rd^ degree polynomial is enough becuase we will not have more than 2 curves.

### Model Predictive Control with Latency:

I have used 100ms as latency of system for computation. Using latency takes into account the calculation time and delay in command execution.