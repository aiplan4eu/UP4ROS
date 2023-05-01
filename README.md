![master](https://github.com/aiplan4eu/UP4ROS/actions/workflows/master.yaml/badge.svg)
![devel](https://github.com/aiplan4eu/UP4ROS/actions/workflows/devel.yaml/badge.svg)
[![codecov](https://codecov.io/github/aiplan4eu/UP4ROS/branch/master/graph/badge.svg?token=W9RX14LTPS)](https://codecov.io/github/aiplan4eu/UP4ROS)

This repository contains a ROS wrapper for the AIPlan4EU Unified Planning library available at https://github.com/aiplan4eu/unified-planning.

## Quick Overview

The **up4ros** node wraps the unified planning library.
  * It can be launched with: `roslaunch up4ros up4ros.launch`
  * Exposed services:
    * `/up4ros/add_action` `[up_msgs/srv/AddAction]` 
    * `/up4ros/add_fluent` `[up_msgs/srv/AddFluent]` 
    * `/up4ros/add_goal` `[up_msgs/srv/AddGoal]` 
    * `/up4ros/add_object` `[up_msgs/srv/AddObject]` 
    * `/up4ros/new_problem` `[up_msgs/srv/NewProblem]` 
    * `/up4ros/set_initial_value` `[up_msgs/srv/SetInitialValue]` 
    * `/up4ros/set_problem` `[up_msgs/srv/SetProblem]`
  * Exposed actions:
    * `/up4ros/planOneShotPDDL` `[up_msgs/action/PDDLPlanOneShotAction]`
    * `/up4ros/planOneShot` `[up_msgs/action/PlanOneShotAction]`
    * `/up4ros/planOneShotRemote` `[up_msgs/action/PlanOneShotRemoteAction]`

## Documentation

The documentation is available [here](https://up4ros.readthedocs.io/en/latest/)

## Acknowledgments

<img src="https://www.aiplan4eu-project.eu/wp-content/uploads/2021/07/euflag.png" width="60" height="40">

This library is being developed for the AIPlan4EU H2020 project (https://aiplan4eu-project.eu) that is funded by the European Commission under grant agreement number 101016442.
