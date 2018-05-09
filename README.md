# CarND-MPC ( Model-Predictive-Control)

In this project, I have implemented Model-Predictive-Control to keep the car on track.

### Model:

I have used Kinematic bicycle model. State includes position( x, y), velocity( v), orientation( psi), cross track error and orientation error. Below are the equations used to predict state. 

#### x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(psi<sub>t</sub>) * dt

#### y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(psi<sub>t</sub>) * dt

#### psi<sub>t+1</sub> = psi<sub>t</sub> + v<sub>t</sub> / Lf * delta<sub>t</sub> * dt

#### v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> * dt

#### cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub> * sin(epsi<sub>t</sub>) * dt

#### epsi<sub>t+1</sub> = psi<sub>t</sub> - psi<sub>des(t)</sub> + v<sub>t</sub> * delta<sub>t</sub> / Lf * dt
  
### Timestep Length and Elapsed Duration (N & dt):

I have used number of timestep(N) = 10 and dt = 0.1. After trying several values with higher and lower N and dt, it seems that these values work better.

### Polynomial Fitting and MPC Preprocessing:

I have convert way points into car coordinate system to simplify mathematics involved. Then I have used 3^rd^ degree polynomial to fit to way points. 3^rd^ degree polynomial is enough becuase we will not have more than 2 curves.

### Model Predictive Control with Latency:

I have used 100ms as latency of system for computation. Using latency takes into account the calculation time and delay in command execution.