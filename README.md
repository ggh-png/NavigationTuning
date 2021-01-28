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
