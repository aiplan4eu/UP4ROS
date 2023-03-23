This repository contains a ROS wrapper for the AIPlan4EU Unified Planning library available at https://github.com/aiplan4eu/unified-planning.

## Nodes

* **up4ros**
  * Services:
    * `/up4ros/add_action` `[up_msgs/srv/AddAction]` 
    * `/up4ros/add_fluent` `[up_msgs/srv/AddFluent]` 
    * `/up4ros/add_goal` `[up_msgs/srv/AddGoal]` 
    * `/up4ros/add_object` `[up_msgs/srv/AddObject]` 
    * `/up4ros/new_problem` `[up_msgs/srv/NewProblem]` 
    * `/up4ros/set_initial_value` `[up_msgs/srv/SetInitialValue]` 
    * `/up4ros/set_problem` `[up_msgs/srv/SetProblem]`
  * Actions:
    * `/up4ros/planOneShotPDDL` `[up_msgs/action/PDDLPlanOneShot]` 
    * `/up4ros/planOneShot` `[up_msgs/action/PlanOneShot]` 
    * `/up4ros/planOneShotRemote` `[up_msgs/action/PlanOneShotRemote]` 

## Acknowledgments

<img src="https://www.aiplan4eu-project.eu/wp-content/uploads/2021/07/euflag.png" width="60" height="40">

This library is being developed for the AIPlan4EU H2020 project (https://aiplan4eu-project.eu) that is funded by the European Commission under grant agreement number 101016442.
