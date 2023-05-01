#!/usr/bin/env python3

import actionlib
import rospy

import up_msgs.msg as msgs
from up4ros.test_utils import get_domain_and_problem


def run_plan_one_shot_pddl(action_name="/up4ros/action/planOneShotPDDL"):
    client = actionlib.SimpleActionClient(action_name, msgs.PDDLPlanOneShotAction)

    goal_msg = msgs.PDDLPlanOneShotGoal()
    goal_msg.plan_request.mode = msgs.PDDLPlanRequest.FILE

    domain, problem = get_domain_and_problem(
        "pddl/gripper_domain.pddl", "pddl/gripper_problem_0.pddl"
    )
    goal_msg.plan_request.domain = domain
    goal_msg.plan_request.problem = problem

    rospy.loginfo(
        "Sending to '%s' the following goal message:\n%s\n", action_name, goal_msg
    )

    def print_feedback(msg):
        rospy.loginfo("Received feedback \n%s\n", msg)

    rospy.sleep(
        0.1
    )  # sleep due to https://github.com/ros/ros_comm/issues/176#issuecomment-13930136

    client.send_goal(goal_msg, feedback_cb=print_feedback)

    # Waits for the server to finish performing the action.
    rospy.loginfo("Waiting for results...")
    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()
    rospy.loginfo("Received from '%s' the following result:\n%s\n", action_name, result)
    return result


if __name__ == "__main__":
    rospy.init_node("test_up4ros_client")
    run_plan_one_shot_pddl()
