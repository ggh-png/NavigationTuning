# ROS Navigation Tuning Guide
## velocity
 In ROS navigationstack, local planner takes in odometry messages (”odom” topic) and outputs velocity commands (”cmd_vel” topic) that controls the robot’s motion.
 Max/min  velocity  and  acceleration  are  two  basic  parameters  for  the  mobile  base.Setting  them  correctly  is  very  helpful  for  optimal  local  planner  behavior.
 
 Translational velocity(m/s) is the velocity when robot is moving in a straight line. Its max value is the same as the maximum velocity we obtained above.
 Rotational velocity(rad/s) is equivalent as angular velocity; its maximum value is the angularvelocity  of  the  robot  when  it  is  rotating  in  place
 
 For safety, we prefer to set maximum translational and rotational velocities to belower than their actual maximum values.

 To use the movebase node in navigation stack, we need to have a global planner and a local planner  <hr/>

 ## Global Planner
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

## Lobal Planner
Local planners that adhere tonavcore::BaseLocalPlanner interface :  dwa_local_planner , eband_local_planner , teb_local_planner<br>
They use different algorithms to generate velocity commands. <br>
we will discuss only dwa_local_planner.(Usually dwa_local_planner is the go to choice.)

### DWA Local Planner 
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

### Forward Simulation Parameter
 In this step, the local planner takes the velocity samples (discard bad guys , select optimal cost).<br>
 Each velocity sampleis  simulated as if it is applied to the robot for a set time interval (sim_time)
 
#### (Real Experiment)
 low sim_time parameter case
 
 high sim_time parameter case
 
 
 sim_time(time interval) mean accepted time for local path moving so every time interval renew.
##### how can i tune time interval(sim_time)?
 sim_time to low value will result in limited performance. bcuz insufficient time optimal trajectory (mean, is better more than sapling vel number managed time)
 otherwise, sim_time to high value result in the heavier the computation load. and have long curves it is not flexible in straight case (i checked, but it is    look like better than low value).
  value  of  4.0  seconds  should  be  enough  even  for  high  performance computers.
  
  
  
  
reference : https://arxiv.org/pdf/1706.09068.pdf
