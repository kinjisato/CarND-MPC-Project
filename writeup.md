## CarND-MPC-Project
### Kinji Sato  / 9th September 2018

---

[//]: # (Image References)
[image1]: ./examples/car_not_car.png
[video1]: ./project_video.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Compilation

Code must compile without errors with cmake and make.

#### 2. Implementation

Almost code are carried from the lecture.

##### The Model : Student describes their model in detail. This includes the state, actuators and update equations.

```C++
          fg[0] += Kcte * CppAD::pow(vars[cte_start + t], 2);
          fg[0] += Kpsi * CppAD::pow(vars[epsi_start + t], 2);
          fg[0] += Kv * CppAD::pow(vars[v_start + t] - ref_v, 2);
      }

      // Minimize the use of actuators.
      double Kdelta = 400;
      double Ka = 5; // 10
      for (int t = 0; t < N - 1; t++) {
          fg[0] += Kdelta * CppAD::pow(vars[delta_start + t], 2);
          fg[0] += Ka * CppAD::pow(vars[a_start + t], 2);
      }
      
      // Minimize the value gap between sequential actuations.
      double Kddelta = 1000;
      double Kda = 1;
      for (int t = 0; t < N - 2; t++) {
          fg[0] += Kddelta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
          fg[0] += Kda * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
```

The cost function is declared. To make easy to adjust weight of penalty, I added gain factor K*** for each cost functions.

```C++
// 3rd order
              AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
              AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
```

I choosed the 3rd order polynomial for xy tracing, so f0 and psides0 also have the 3rd order.

```C++
              fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
              fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
              // fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
              fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
              fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
              fg[1 + cte_start + t] =
              cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
              fg[1 + epsi_start + t] =
              epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```

For update, I declared as above. I defined as ``` fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt) ``` to fit positive negative value and the car steer left and right.

```C++
  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
    vector<double> result;
    
    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[a_start]);
    
    for (int i = 0; i < N-1; i++) {
        result.push_back(solution.x[x_start + i + 1]);
        result.push_back(solution.x[y_start + i + 1]);
    }
    
    return result;
}
```

I make vecotr 'result' to store the result of MPU.solver. the steer control value would be stored into result[0], throttle control value would be stored into result[1]. And N sets of car future target positions would be stored into rest of result vector. This vector result is returned to main function.

##### Timestep Length and Elapsed Duration (N & dt) : Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

First, I tried N=25 and dt = 0.05sec as same as lecture quiz. But computation cost was high. 25 * 0.05sec = 1.25sec, so with keeping the same prediction furue time period, I choosed N =10, dt = 0.1, 10 * 0.1 = 1.0sec. These were no problem to run the car, so should be reasonable value.


##### Polynomial Fitting and MPC Preprocessing : A polynomial is fitted to waypoints.

```C++
            // Transform from simulator map xy cordinates to car cordinates
            // size of ptsx
            size_t num_waypoints = ptsx.size();
            
            // declare the vectors for transformed xy
            auto ptsx_transformed = Eigen::VectorXd(num_waypoints);
            auto ptsy_transformed = Eigen::VectorXd(num_waypoints);
            
            // transform
            for (unsigned int i = 0; i<num_waypoints; i++){
                // get diff
                double dx = ptsx[i] - px;
                double dy = ptsy[i] - py;
                
                ptsx_transformed(i) = dx * cos(-psi) - dy * sin(-psi);
                ptsy_transformed(i) = dx * sin(-psi) + dy * cos(-psi);
            }
            
            // Fit polynomial order Xth
            unsigned int order_poly = 3;
            auto coeffs = polyfit(ptsx_transformed, ptsy_transformed, order_poly);
```

Simulator course map xy cordinates must to be transformed to car xy cordinates. I used similar transformation functions those used at particle filter lesson. And transoformed xy cordinates were sent to ployfit function, I choosed the 3rd order to fit.

##### Model Predictive Control with Latency : The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

```C++
            double x0 = 0;
            double y0 = 0;
            double psi0 = 0;
            double cte0 = cte;
            double epsi0 = epsi;
            double v0 = v;
            
            // delay = 100msec
            double tdelay = 0.1; // 100msec
            
            // state considered with delay
            x0 = x0 + ( v * cos(psi0) * tdelay );
            y0 = y0 + ( v * sin(psi0) * tdelay );
            psi0 = psi0 - ( v * delta * tdelay / mpc.Lf );
            v0 = v0 + a * tdelay;
            cte0 = cte0 + ( v * sin(epsi0) * tdelay );
            epsi0 = epsi0 - ( v * atan(coeffs[1]) * tdelay / mpc.Lf );

            // State vector(6)
            Eigen::VectorXd state(6);
            state(0) = x0;
            state(1) = y0;
            state(2) = psi0;
            state(3) = v0;
            state(4) = cte0;
            state(5) = epsi0;
```

The car should run with current state in delay 100msec. So state should be update by 100msec. 6 state values were considered as above formula.

#### 3. Simulation

##### The vehicle must successfully drive a lap around the track. : No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

Car should run coutinuously in the track with submitted code.

