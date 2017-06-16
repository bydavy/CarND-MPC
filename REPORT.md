# REPORT

## Model
The car is modelized by the global kinematic model and two actuators are controlled by the MPC: steering and throttle.

The car state is defined as the x and y coordinate of the car in the map space (1 and 2), car orientation relative to the map (2), car velocity (4), cross track error relative to the current waypoint segment and orientation error relative to the current waypoint segment. The last two components of the state incorporate waypoints to correct the car trajectory and guide it along the waypoints.

The car state is updated thanks to the following equations:
1. x = x + v * cos(psi) * dt
2. y = y + v * sin(psi) * dt
3. psi = psi + v * steering / Lf * dt
4. v = v + acceleration * dt

Where:
* dt represent the time elapsed between model updates.
* steering is the car steering angle.
* Lf is the length from front to car's center of gravity.

## Latency
Latency represents the time for the information to go through the system and for the actuators to physically apply its action. When ignored, the car has a non smooth and potentially unsafe driving.

Latency is handled in our system in two ways. First the duration between each prediction steps of the MPC procedure is greated and close to the latency. This means during prediction, we don't assume acturators will operate faster than their inner latency. And to stay as accurate as possible, the duration is very close to the latency. The second is related to the first one, as the first actuation will happen after one latency, we don't start prediction on the current car status but the predicted position after one latency. The combination of those two actions is enough to mitigate the latency problem.

## Timestep length and duration
The prediction cannot be accurate over a long periode of time. In fact, the longer it is the more likely it will diverge from reality. It can be explained by approximations done along the way in reading car status, actuators physical actions and imperfection of the model used. Therefore the total duration of the prediction is only a few seconds, in our case 1.2 seconds.

I started with 10 and 100ms for the number of step and the duration between each step and tuned it to improve the car behavior, 10 and 120ms gives me the best result so far. To summarize, during 1.2 seconds, we process 10 timestep with a duration of 120ms between timesteps. The duration between steps is greated and close to the system latency (see latency section) and the number of steps has been chosen to be a total duration of only a few seconds.


##Polynomial fiting
Given waypoints, we fit a polynomial and the solver will reason solely on the polynomial. This introduce some approximation but provide a good enough result.

I decided to solve the problem in car coordinate space instead of map coordinate system. Doing so simplifies some calculations by introducing some zero values and conveniently matches the coordinate system used by the car simulator API to draw waypoints and predicted trajectory.

To go from map to car coordinate system I used the following equation:
* w_x_car_space = (w_x - car_x) * cos(car_psi) - (w_y - car_y) * sin(car_psi)
* w_y_car_space = (w_y - car_y) * cos(car_psi) + (w_x - car_x) * sin(car_psi)

Where:
* w_x and w_y is the waypoint position in map coordinate.
* car_x and car_y are the car position in map coordinate.
* car_psi is the car orientation in the map coordinate space.
* w_x_car_space and w_y_car_space is the waypoint position in car space.
