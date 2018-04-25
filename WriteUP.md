### Objective
The project is to create a MPC and to use it to drive the car in simulator.

### MPC
MPC stands for Model Predictive Control. It’s an advanced way of controlling a process while
fulfilling a set of constraints. We implemented the kinematic model, which is a simplified version
of dynamic model. As it is a simple model we are ignoring many forces like tire forces, gravity, mass, friction, etc….

### State
The state helps in keeping track of the vehicle. Below are the variables used to track the state model:

X: the x location in a 2 dimensional plane
Y: the y location in a 2 dimensional plane
psi: Orientation of the vehicle
V: Velocity of the vehicle
cte: Cross Track Error(Error between the reference trajectory and predicted path)
epsi: Error in vehicle orientation

### Actuators
Acutators are used to control vehicle state. We are using two actuators:
Steering: Positive value denotes right steer while negative steer denotes left steer
Throttle: Value of throttle can be between -1 and +1. Positive value means acceleration while negative value brake.

### Update equations
![alt text](https://raw.githubusercontent.com/Keshav-Aggarwal/CarND-MPC-Project/master/Equations.PNG)
dt => rate of change of state
Lf => Distance between the mass of center and front axle

### Timestep Length and Elapsed Duration (N & dt)
The timestep length defines how many states we need to prediction. The
elapsed duration defines the frequency with which the state (environment) will change.
For this project I used the timestep length (N) as 10 and the value of Elapsed duration (dt) as 100
milliseconds.

### Polynomial Fitting and MPC Preprocessing
I am using 3rd degree polynomial as it will fit mostly roads as suggested in course. 
Given waypoints were in Global coordinate system, we need to convert the global points to vehicle's coordinate system. We used below equations for this:

Finally the weights with which the car drived properly in simulator are:


### Model Predictive Control with Latency
To replicate the latency sleep method is used with a latency of 100ms to calculate the next state.

 
