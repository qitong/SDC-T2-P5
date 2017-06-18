# SDC-T2-P5
CarND-Controls-MPC

---

## The Model
In this project, I use the model predicative control introduced by SDC course.  
The model, by taking the state of the car as and actuators (steering and accelaration) into account as well as Lf ( the distance between its center of gravity and front wheels), simulates the car in a more comprehensive way.  
The model keeps tracking Cross Tracking Error (CTE) and Orientation Error (EPSI), and by trying to minimize those two errors, it gets close to the desired reference trajectory, the predictive functions are as follow:  

```
x(t+1)   = x(t) + v(t) * cos(psi(t)) * dt
y(t+1)   = y(t) + v(t) * sin(psi(t)) * dt
psi(t+1) = psi(t) + v(t) / Lf * delta(t) * dt
v(t+1)   = v(t) + a(t) * dt
cte(t+1)  = f(x(t)) - y(t) + v(t) * sin(epsi(t)) * dt
epsi(t+1) = psi(t)  - psides(t) + v(t) * delta(t) / Lf * dt
```

--

## Timestep Length and Elapsed Duration (N & dt)

N is the number of timesteps, and dt is the time elapses between actuations. N * dt determines the total prediction horizon.  

When *N* goes up, it will help predict further future trajectory which will contribute to a smoother driving. However, the prediction is actually based on local approximation, and it gets unprecise when it predicts long future (thus I actually predict in every time step). And also, it takes more computation power when N getting larger.  

When *dt* goes up,  the result predict trajectory will lead to a large bias increase, a rough fit of reference trajectory. While, a small dt leads to a more unstable prediction, a large variation, cause the car goes crazy.  

For my own implementation, I have N=8 and dt=0.08 (instead of N=25, dt=0.05 specified in mpc-to-line quiz solution).  

--

## Polynomial Fitting and MPC Preprocessing
The waypoints to fit are given in a global coordinates, thus I first transformed it to the car's own coordinate first:
```
transformed_ptsx[i] = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
transformed_ptsy[i] = -sin(psi) * (ptsx[i] - px) + (ptsy[i] - py) * cos(psi);
```
Then, I try to fit these points into a 3rd order polynomial (instead of 1st order in the mpc-to-line solution) as suggested in the course.
```
auto coeffs = polyfit(transformed_ptsx, transformed_ptsy, 3);
```
Also, I change the implementation for *f0* and *psides0* in MPC.cpp accordingly:
```
AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
AD<double> psides0 = CppAD::atan(3 * coeffs[3] * x0 * x0 + 2 * coeffs[2] * x0 + coeffs[1]);
```

--

## Model Predictive Control with Latency

This project asked taking a delay of 100 ms into account, as the delay between sensor-processing-actuator response. That is to say, we are actually predicting from past measurement.  
To adapt to the delay, I add a *x* direction "drift", an approximation product by velocity, heading direction and delay:
```
px = v * 0.1 * cos(psi);
```
--

## Result

[![SDC MPC Qitong](https://img.youtube.com/vi/bSyojyMOa88/0.jpg)](https://youtu.be/bSyojyMOa88)
