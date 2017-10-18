## Project Model Predictive Control
---
**Optimal control with dynamic programming**
![](http://om1hdizoc.bkt.clouddn.com/17-6-5/75579128.jpg)
The goals/steps of this project are the following:

* Setting constraints and cost functions for the optimization
* Setting length and frequency for the optimization
* Transform map coordinates to car's coordinates so cte and epsi can be easily computed
* Polyfit the transformed points into a 3rd degree curve
* Show traces for map data and predicted route in real time

## Rubric points
**Here I will consider the rubric points individually and describe how I addressed each point in my implementation.**

### Describe the model in detail.
1. The model implemented is the same in the lecture. There are 6 states that needs to be considered. These states are: Px, Py, Psi, V, CTE, EPSI. And there are two actuator state needs to be taken into account as well. The actuator are Steer and Throttle. 
2. The model needs to find a set of Steer and Throttle that will drive the car to a desired position with minimum cost. There are many cost functions to consider, some will define the reference error and some will minimize the value gap between sequential actuations.

I setup the cost functions as follows:
```C++
// The part of the cost based on the reference state.
for (int i = 0; i < N; i++) {
  fg[0] += 6*CppAD::pow(vars[cte_start + i] - ref_cte, 2);
  fg[0] += 8*CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
  fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
}

// Minimize the use of actuators.
for (int i = 0; i < N - 1; i++) {
  fg[0] += CppAD::pow(vars[delta_start + i], 2);
  fg[0] += CppAD::pow(vars[a_start + i], 2);
}

// Minimize the value gap between sequential actuations.
for (int i = 0; i < N - 2; i++) {
  fg[0] += 600*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
  fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
}

```
### Describe Timestep Length and Frequency.
The timestep length and frequency is choosen based on the guideline shown on the udacity website. Initially I choosed N to 50 and dt to be 0.02. But I realized later that for my disired speed(20mph) this is a very short distance to predict. Later I changed N to be 15 and dt to be 0.07. I also set my desired speed to be 30mph. So 30mph = 13.41m/s, and 13.41m/s*0.07s = 0.938m per prediction for 15 predictions. So the total distance for my prediction is 0.938*15 = 14m and the total time length is 1.05s.


### Describe Polynomial Fitting and MPC Preprocessing
- How and why were the waypoints converted to vehicle coordinate?
  
  I converted the waypoints from map coordinates to vhicle coordinates because map coordinates wasn't working for me. Not sure why it didn't and I had been debugging it for several hours so I decided to switch to another coordinate system.

  I convert the map coordinates first by subtracting translational vector. The reason I didn't do rotational first because if you do rotational first you would also do rotation for the translational vector. However if you do translational first, you only have to do rotation once.
  
- why was a third order polynomial fitted to waypoints to get coefficients?

  The reason I use a thrid order polynomial is because 3rd polynomial fits most road condition. If I remember correctly that was mentioned in one of Udacity's lecture.

- How were the state values obtained?

  The equations for the state values were described in Udacity's lecture slide. It mostly follow's college calculus and newton's second law.
```C++
    // State equations
    x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v_[t+1] = v[t] + a[t] * dt
    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

I fitted my transformed points as follows:
```C++
  vector<double> ptsx = j[1]["ptsx"];
  vector<double> ptsy = j[1]["ptsy"];

  double px = j[1]["x"];
  double py = j[1]["y"];
  double psi = j[1]["psi"];
  double v = j[1]["speed"];
  
  Eigen::VectorXd ptsxE(ptsx.size());
  Eigen::VectorXd ptsyE(ptsy.size());
  // transforming map coords to vehicle's coords
  for(int i = 0;i < ptsx.size();i++) {
    // translational vector
    double tempx = ptsx[i] - px;
    double tempy = ptsy[i] - py;
    // rotational matrix
    ptsxE(i) = tempx*cos(psi) + tempy*sin(psi);
    ptsyE(i) = -tempx*sin(psi) + tempy*cos(psi);
    // for tracing purpose
    ptsx[i] = ptsxE(i);
    ptsy[i] = ptsyE(i);
  }
  // polyfit 3rd degree
  auto coeffs = polyfit(ptsxE,ptsyE,3);
```
I followed advices on Udacity and Slack that 3rd degree polynomial fits most of the road.

The MPC preprocessing part is shown as follows:
```C++
  // saves prtx and prty for showing traces in the simulator
  prtx.clear();
  prty.clear();

  for(int i = 0; i < N-1; i++){
    prtx.push_back(solution.x[x_start+1+i]);
    prty.push_back(solution.x[y_start+1+i]);
    //prpsi.push_back(solution.x[psi_start+1+i]);
  }
```
This code snippet is in the mpc.solve() function, I added two vector in the mpc class file as public variable to save predicted points so I could plot them.

### Video Link to the project
As the reviewer said that I could include a video file in case the car didn't driver properly under his/her test condition. Well here's a working version of it I uploaded on Youtube.
[Click here to view](https://youtu.be/_yNGCwAGa0I)

**\*updated for latency prediction and increase the speed to 30mph\***

### Describe Model Predictive Control with Latency
- Were there any implementations in the submission which helps to solve latency problem?

  To cope with latency, I followed reviewr's advice and predicted 0.1s into the future with the model equations.

  ![](http://om1hdizoc.bkt.clouddn.com/17-6-5/58965264.jpg)
  ```C++
      // update px, py, psi and v 0.1s into the future
      px = px + v*cos(psi)*latency;
      py = py + v*sin(psi)*latency;
      psi = psi + v*delta/Lf*latency;
      v = v + acceleration*latency;
      
      // update cte and epsi into the future, 0.1s
      cte = cte + (v * sin(epsi) * latency);
      epsi = epsi + (v * delta / Lf * latency);
  ```
    Since I transformed everything into the vehicle's coordinate, I only needed v, cte and epsi for solving the equations. But I needed px, py, psi and v to calculted the coefficients. That's why they were seperated.

---
### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.
There are many parameters you can play with. Like the cost functions and line fitting algorithm etc.
But I think that coordinate transformation is still the core idea of this project. Indeed I found that transforming from the map coordinates to the vehicles coordinate will be more convenient. It take some thoughts but once it's done it can save some computing time.