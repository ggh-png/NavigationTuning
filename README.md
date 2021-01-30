# ROS Navigation Tuning Guide and Some Experiment
# velocity
 In ROS navigationstack, local planner takes in odometry messages (”odom” topic) and outputs velocity commands (”cmd_vel” topic) that controls the robot’s motion.
 Max/min  velocity  and  acceleration  are  two  basic  parameters  for  the  mobile  base.Setting  them  correctly  is  very  helpful  for  optimal  local  planner  behavior.
 
 Translational velocity(m/s) is the velocity when robot is moving in a straight line. Its max value is the same as the maximum velocity we obtained above.
 Rotational velocity(rad/s) is equivalent as angular velocity; its maximum value is the angularvelocity  of  the  robot  when  it  is  rotating  in  place
 
 For safety, we prefer to set maximum translational and rotational velocities to belower than their actual maximum values.

 To use the movebase node in navigation stack, we need to have a global planner and a local planner  <hr/>
<br>
 # Global Planner
  There are three global planners that adhere to navcore::BaseGlobalPlanner interface :
      carrot_planner , navfn , global_planner
      
1. carrot_planner<br>  It checks if the given goal is an obstacle, and if so it picks an alternative goal close to the original one
Eventually it passes this valid goal as a plan to the local planner or controller (internally).
Therefore, **this planner does not do any global path planning.
It is helpful if you require your robot to move close to the given goal even if the goal is unreachable.**

2. navfn and global planner<br>
navfn uses  Dijkstra’s  algorithm  to  find  a  global  path  with  minimum  cost  between start point and end point.
global planner is built as a more flexible replacement of navfn with more options.

3. more detailed global planner parameters<br>
Since globalplanneris generally the one that we prefer, let us look at some of its key parameters
**lethal_cost, neutral_cost, cost_factor are actually determine the quality of the planned global path**<br>
For lethal_cost, setting it to a low value may result in failure to produce any path, even when a feasible path isobvious.<br> 
    'cost = COSTNEUTRAL + COSTFACTOR * costmapcostvalue'
<br>incoming costmap cost values are in the range 0 to 252(Generally setting).
#### (Real Experiment)
![image](https://user-images.githubusercontent.com/70446214/106213583-5cbfbb00-6210-11eb-9a8a-300f4b79f657.png)

this ros navigation default value
lethal_cost = 253, neutral_cost=50, cost_factor = 3.

![image](https://user-images.githubusercontent.com/70446214/106213599-61846f00-6210-11eb-88be-cd047c2fbddd.png)
this is modified value 
lethal_cost = 253, neutral_cost=66, cost_factor = 0.55

Experiment Result: as you can see,neutral_cost and cost_factor is important parameter for global planning. it make more smooth global path<hr>

# Local Planner
Local planners that adhere tonavcore::BaseLocalPlanner interface :  dwa_local_planner , eband_local_planner , teb_local_planner<br>
They use different algorithms to generate velocity commands. <br>
we will discuss only dwa_local_planner.(Usually dwa_local_planner is the go to choice.)
<br>
## DWA Local Planner 
dwa_local_planner uses  Dynamic  Window  Approach  (DWA)  algorithm.<br>
 Dynamic  Window  Approach  (DWA)  algorithm Summary Description
 1. sample dx,dy,dtheta
 2. for each sampled velocity, perform simulating (current state to forward simulation) in some short period term.
 3. illegal tragectories are discard (kill off bad trajectorie) and scoring each trajectory (score considered characteristics : Proximity to Obstacle , Proximity to the goal , Proximity to the global path , and speed)
 4. highest - scoring send associated velocity
 5. repeat <br>
 
the goal of DWA is to produce (v,w) pair. based on highest scoring function (Max Objective function , cost)  
DWA consider hardware limit like transitional , rotational velocity , accel , hardware size. 
DWA  maximizes  an  objective  function that depends on (1) the progress to the target, (2) clearance from obstacles, and (3) forward velocity to produce the optimal velocity pair.
This DWA planner depends on the local costmap which provides obstacle information. therfore, ***tuning the parameters for the local costmap is crucial for optimal behavior of DWA local planner.*** 
<br><br>
### Forward Simulation Parameter

 In this step, the local planner takes the velocity samples (discard bad guys , select optimal cost).<br>
 Each velocity sampleis  simulated as if it is applied to the robot for a set time interval (sim_time)
 <br>
#### (Real Experiment)
 low sim_time parameter case
 ![image](https://user-images.githubusercontent.com/70446214/106336390-1fbbfd00-62d2-11eb-9c6f-278d88bb3296.png)
 high sim_time parameter case
![image](https://user-images.githubusercontent.com/70446214/106336356-0f0b8700-62d2-11eb-8667-0895c01d21e9.png) 
 
 Experiment Result : sim_time(time interval) mean accepted time for local path moving so every time interval renew. and on my computer, not make Local Trajectory at sim_time about 1.2<br><br>
#### time interval(sim_time) Parameter
 sim_time to low value will result in limited performance. bcuz insufficient time optimal trajectory (mean, is better more than sapling vel number managed time)
 otherwise, sim_time to high value result in the heavier the computation load. and have long curves it is not flexible in straight case (i checked, but it is    look like better than low value).
  value  of  4.0  seconds  should  be  enough  even  for  high  performance computers.
  <br><br>
#### Velocity Sample Parameter
vx_sample,vy_sample determine how many translational velocity samples, vth_sample controls the number of rotational velocities samples.
 **The number of samples you would like to take depends on how much computation power you have.** 
  In most cases we prefer to set vth_samples to be higher than translational velocity samples, because turning is generally a more complicated condition than  moving straight ahead. If you set max_vel_y to be zero, there is no need to have velocity samples in y direction since there will be no usable samples. 
  <br><br>
#### Simulation granularity Parameter
Simulation granularity  is the step size to take between points on a given trajectory in meters. test if they intersect with any obstacle or not.
  A lower value meanshigher frequency, which requires more computation power. The default value of 0.025 is generally enough for turtlebot-sized mobile base.
  <br><br>
  
### Trajactory Scoring (Objectivce Function)
DWA Local Planner maximizes an objective function to obtain optimal velocity pairs.<br>
the value of this objective function relies on three components: progress to goal, clearance from obstacles , forward velocity <br>
    cost = path_distance_bias∗(distance(m) to path from the endpoint of the trajectory)<br> + goal_distance_bias∗(distance(m) to local goal from the endpoint of the trajectory)<br> + occdist_scale∗(maximum obstacle cost along the trajectory in obstacle cost)<br>
    **The objective is to get the lowest cost**<br>
    - path_distance_bias : the weight for how much the local planner should stay close to the global path. A high value will make the local planner prefer trajectories on global path<br>
    - goal_distance_bias : the weight for how much the robot should attempt to reach the local goal, with whatever path. 이 매개 변수를 늘리면 로봇이 global path에 덜 부착될 수 있습니다.<br>
    - occdist_scale : the weight for how much the robot should attempt to avoid obstacles.***A high value for this parameter results in indecisive robot that stucks in place***
    <br><br>
#### Goal Tolerance Parameters
yaw_goal_tolerance : The tolerance in radians for the controller in yaw/rotation when achieving its goal <br><br>
xy_goal_tolerance : The tolerance in meters for the controller in the x & y distance when achieving a goal <br><br>
latch_xy_goal_tolerance : (bool) If goal tolerance is latched, 로봇이 제자리에서 간단히 회전.
<br><br>
#### Oscillation Prevention Parameters
Oscillation occur when in either of the x, y, or theta dimensions, positive and negative values are chosen consecutively. To prevent oscillations, when the robot moves in any direction, for the next cycles the opposite direction is marked invalid, until the robot has moved beyond a certain distance from the position where the flag was set. In situations such as passing a doorway, the robot may oscilate back and forth because its local planner is producing paths leading to two opposite directions. ***If the robot keeps oscilating, the navigation stack will let the robot try its recovery behaviors***<br><br>
oscillation_reset_dist : How far the robot must travel in meters before oscillation flags are reset <br>
  
  
## Costmap Parameters
costmap concepts: 
there is a global costmap, as well as a local costmap. Costmap parameters tuning is essential for the success of local planners (not only for DWA).  <br>
글로벌 비용 맵은 내비게이션 스택에 제공된 맵의 장애물을 부풀려서(inflation) 생성 / 로컬 비용 맵은 로봇의 센서가 감지한 장애물을 실시간으로 팽창시켜(inflation) 생성<br>
Costmap composed : static map layer, obstacle map layer, inflation layer <br>
a) static map layer : this is given by SLAM map <br>
b) obstacle map layer : this is include 2D obstacles and 3D obstacles (voxel layer).<br>
c) inflation layer : for calculate cost about obstacles, ***obstcle cost value propagate to close layer cells***<br>


###  Parameters Related with inflation layer
 Inflation layer is consisted of cells with cost ranging from 0 to 255. and, The value of 255 means Occupied Area and, 0 means free Area.
 ***inflation_radius Parameter controls how far away the zero cost point is from the obstacles.***



foot_print : footprint is the contour(윤곽선) of the mobile base (depends on hardware). 
Usually for safety, we want to have the footprint to be slightly larger than the robot’s real contour.

inflation_radius and cost_scaling_factor are the parameters that determine the inflation.

 (zero cost value mean free area).
 
  
  
  
  
  
reference : https://arxiv.org/pdf/1706.09068.pdf
