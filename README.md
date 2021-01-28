# ROS Navigation Tuning Guide
## velocity
 In ROS navigationstack, local planner takes in odometry messages (”odom” topic) and outputs velocity commands (”cmd_vel” topic) that controls the robot’s motion.
 Max/min  velocity  and  acceleration  are  two  basic  parameters  for  the  mobile  base.Setting  them  correctly  is  very  helpful  for  optimal  local  planner  behavior.
 
 Translational velocity(m/s) is the velocity when robot is moving in a straight line.Its max value is the same as the maximum velocity we obtained above.
 Rotationalvelocity(rad/s) is equivalent as angular velocity; its maximum value is the angularvelocity  of  the  robot  when  it  is  rotating  in  place
 
 For safety, we prefer to set maximum translational and rotational velocities to belower than their actual maximum values.
 
 To use the movebase node in navigation stack, we need to have a global planner and a local planner
 ## Global Planner
  There are three global planners that adhere to navcore::BaseGlobalPlanner interface :
      carrot_planner , navfn , global_planner
      
1. carrot_planner<br>  It checks if the given goal is an obstacle, and if so it picks an alternative goal close to the original one
Eventually it passes this valid goal as a plan to the local planner or controller (internally).
Therefore, **this planner does not do any global path planning.
It is helpful if you require your robot to move close to the given goaleven if the goal is unreachable.**

2. navfn and global planner<br>
navfn uses  Dijkstra’s  algorithm  to  find  a  global  path  with  minimum  cost  between start point and end point.
global planner is built as a more flexible replacement of navfn with more options.

3. more detailed global planner parameters<br>
Sinceglobalplanneris generally the one that we prefer, let us look at some of its key parameters
**lethal_cost, neutral_cost, cost_factor are actually determine the quality of the planned global path**<br>
For lethal_cost, setting it to a low value may result in failure to produce any path, even when a feasible path isobvious.<br> 

    cost = COSTNEUTRAL + COSTFACTOR * costmapcostvalue
incoming costmap cost values are in the range 0 to 252.
#### (real experiment)

this ros navigation default value
lethal_cost = 253, neutral_cost=50, cost_factor = 3.

this is modified value 
lethal_cost = 253, neutral_cost=66, cost_factor = 0.55

Experiment Result: as you can see,neutral_cost and cost_factor is important parameter for global planning. it make more smooth global path

reference : https://arxiv.org/pdf/1706.09068.pdf
