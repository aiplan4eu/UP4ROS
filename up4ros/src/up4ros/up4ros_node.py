# Copyright 2023 Magazino GmbH
# Copyright 2022 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import tempfile

import actionlib
import rospy

from unified_planning import model
from unified_planning.io.pddl_reader import PDDLReader
from unified_planning.shortcuts import OneshotPlanner

from up4ros.ros_interface_reader import ROSInterfaceReader
from up4ros.ros_interface_writer import ROSInterfaceWriter

from up_msgs.msg import (
    PDDLPlanRequest,
    PDDLPlanOneShotAction,
    PDDLPlanOneShotFeedback,
    PDDLPlanOneShotResult,
    PlanOneShotAction,
    PlanOneShotFeedback,
    PlanOneShotResult,
)

from up_msgs.srv import (
    AddAction,
    AddActionResponse,
    AddFluent,
    AddFluentResponse,
    AddGoal,
    AddGoalResponse,
    AddObject,
    AddObjectResponse,
    GetProblem,
    GetProblemResponse,
    NewProblem,
    NewProblemResponse,
    PDDLPlanOneShot,
    PDDLPlanOneShotResponse,
    SetInitialValue,
    SetInitialValueResponse,
    SetProblem,
    SetProblemResponse,
)


class UP4ROSNode:
    def __init__(self, init_ros_interfaces=True):
        self.problems = {}
        self._ros_interface_writer = ROSInterfaceWriter()
        self._ros_interface_reader = ROSInterfaceReader()

        self._add_action = None
        self._add_fluent = None
        self._add_goal = None
        self._add_object = None
        self._get_problem = None
        self._new_problem = None
        self._pddl_plan_one_shot_server = None
        self._pddl_plan_one_shot_srv = None
        self._plan_one_shot_server = None
        self._set_initial_value = None
        self._set_problem = None

        if init_ros_interfaces:
            self.initialize_ros()

    def initialize_ros(self):
        self._pddl_plan_one_shot_server = actionlib.SimpleActionServer(
            "up4ros/action/planOneShotPDDL",
            PDDLPlanOneShotAction,
            self.pddl_plan_one_shot_callback,
        )

        self._plan_one_shot_server = actionlib.SimpleActionServer(
            "up4ros/action/planOneShot", PlanOneShotAction, self.plan_one_shot_callback
        )

        self._add_action = rospy.Service(
            "up4ros/srv/add_action", AddAction, self.add_action
        )
        self._add_fluent = rospy.Service(
            "up4ros/srv/add_fluent", AddFluent, self.add_fluent
        )
        self._add_object = rospy.Service(
            "up4ros/srv/add_object", AddObject, self.add_object
        )
        self._add_goal = rospy.Service("up4ros/srv/add_goal", AddGoal, self.add_goal)
        self._get_problem = rospy.Service(
            "up4ros/srv/get_problem", GetProblem, self.get_problem
        )
        self._new_problem = rospy.Service(
            "up4ros/srv/new_problem", NewProblem, self.new_problem
        )
        self._pddl_plan_one_shot_srv = rospy.Service(
            "up4ros/srv/planOneShotPDDL", PDDLPlanOneShot, self.pddl_plan_one_shot
        )
        self._set_initial_value = rospy.Service(
            "up4ros/srv/set_initial_value", SetInitialValue, self.set_initial_value
        )
        self._set_problem = rospy.Service(
            "up4ros/srv/set_problem", SetProblem, self.set_problem
        )

    def get_problem(self, request):
        response = GetProblemResponse()

        if request.problem_name not in self.problems:
            response.success = False
            response.message = f"Problem {request.problem_name} does not exist"
        else:
            response.problem = self._ros_interface_writer.convert(
                self.problems[request.problem_name]
            )
            response.success = True
        return response

    def new_problem(self, request):
        response = NewProblemResponse()

        if request.problem_name in self.problems:
            response.success = False
            response.message = f"Problem {request.problem_name} already exists"
        else:
            self.problems[request.problem_name] = model.Problem(request.problem_name)
            response.success = True
        return response

    def set_problem(self, request):
        response = SetProblemResponse()

        if request.problem_name in self.problems:
            response.success = False
            response.message = f"Problem {request.problem_name} already exists"
        else:
            self.problems[request.problem_name] = self._ros_interface_reader.convert(
                request.problem
            )
            response.success = True
        return response

    def add_fluent(self, request):
        response = AddFluentResponse()

        if request.problem_name not in self.problems:
            response.success = False
            response.message = f"Problem {request.problem_name} does not exist"
        else:
            problem = self.problems[request.problem_name]

            fluent = self._ros_interface_reader.convert(request.fluent, problem)
            value = self._ros_interface_reader.convert(request.default_value, problem)
            problem.add_fluent(fluent, default_initial_value=value)
            response.success = True
        return response

    def add_action(self, request):
        response = AddActionResponse()

        if request.problem_name not in self.problems:
            response.success = False
            response.message = f"Problem {request.problem_name} does not exist"
        else:
            problem = self.problems[request.problem_name]

            action = self._ros_interface_reader.convert(request.action, problem)
            problem.add_action(action)
            response.success = True
        return response

    def add_object(self, request):
        response = AddObjectResponse()

        if request.problem_name not in self.problems:
            response.success = False
            response.message = f"Problem {request.problem_name} does not exist"
        else:
            problem = self.problems[request.problem_name]

            action = self._ros_interface_reader.convert(request.object, problem)
            problem.add_object(action)
            response.success = True

        return response

    def set_initial_value(self, request):
        response = SetInitialValueResponse()

        if request.problem_name not in self.problems:
            response.success = False
            response.message = f"Problem {request.problem_name} does not exist"
        else:
            problem = self.problems[request.problem_name]

            expression = self._ros_interface_reader.convert(request.expression, problem)
            value = self._ros_interface_reader.convert(request.value, problem)
            problem.set_initial_value(expression, value)
            response.success = True
        return response

    def add_goal(self, request):
        response = AddGoalResponse()

        if request.problem_name not in self.problems:
            response.success = False
            response.message = f"Problem {request.problem_name} does not exist"
        else:
            problem = self.problems[request.problem_name]

            if len(request.goal) > 0:
                goal = self._ros_interface_reader.convert(request.goal[0].goal, problem)
                problem.add_goal(goal)
                response.success = True
            elif len(request.goal_with_cost) > 0:
                goal = self._ros_interface_reader.convert(
                    request.goal_with_cost[0].goal, problem
                )
                problem.add_goal(goal)
                response.success = True
            else:
                response.success = False
                response.message = "Goal is void"
        return response

    def pddl_plan_one_shot(self, request):
        if request.plan_request.mode == PDDLPlanRequest.RAW:
            domain_file = tempfile.NamedTemporaryFile()
            problem_file = tempfile.NamedTemporaryFile()

            with open(domain_file, "w") as pddl_writer:
                pddl_writer.write(request.plan_request.domain)
            with open(problem_file, "w") as pddl_writer:
                pddl_writer.write(request.plan_request.problem)
        else:
            domain_file = request.plan_request.domain
            problem_file = request.plan_request.problem

        reader = PDDLReader()
        response = PDDLPlanOneShotResponse()
        try:
            upf_problem = reader.parse_problem(domain_file, problem_file)
        except Exception:
            response.success = False
            response.message = "Error parsing problem"
            return response

        with OneshotPlanner(problem_kind=upf_problem.kind) as planner:
            result = planner.solve(upf_problem)
            print("%s returned: %s" % (planner.name, result.plan))

            if result.plan is not None:
                response.plan_result = self._ros_interface_writer.convert(result)
                response.success = True
                response.message = ""
            else:
                response.success = False
                response.message = "No plan found"

            return response

    def pddl_plan_one_shot_callback(self, goal):
        if goal.plan_request.mode == PDDLPlanRequest.RAW:
            domain_file = tempfile.NamedTemporaryFile()
            problem_file = tempfile.NamedTemporaryFile()

            with open(domain_file, "w") as pddl_writer:
                pddl_writer.write(goal.plan_request.domain)
            with open(problem_file, "w") as pddl_writer:
                pddl_writer.write(goal.plan_request.problem)
        else:
            domain_file = goal.plan_request.domain
            problem_file = goal.plan_request.problem

        reader = PDDLReader()
        upf_problem = reader.parse_problem(domain_file, problem_file)

        with OneshotPlanner(problem_kind=upf_problem.kind) as planner:
            result = planner.solve(upf_problem)
            print("%s returned: %s" % (planner.name, result.plan))

            feedback_msg = PDDLPlanOneShotFeedback()
            feedback_msg.plan_result = self._ros_interface_writer.convert(result)

            self._pddl_plan_one_shot_server.publish_feedback(feedback_msg)

            rospy.sleep(0.1)  # sleep to allow the feedback to be sent

            result = PDDLPlanOneShotResult()
            result.success = True
            result.message = ""
            self._pddl_plan_one_shot_server.set_succeeded(result)

    def plan_one_shot_callback(self, goal):
        upf_problem = self._ros_interface_reader.convert(goal.plan_request.problem)

        with OneshotPlanner(problem_kind=upf_problem.kind) as planner:
            result = planner.solve(upf_problem)
            print("%s returned: %s" % (planner.name, result.plan))

            feedback_msg = PlanOneShotFeedback()
            feedback_msg.plan_result = self._ros_interface_writer.convert(result)

            self._plan_one_shot_server.publish_feedback(feedback_msg)

            rospy.sleep(0.1)  # sleep to allow the feedback to be sent

            result = PlanOneShotResult()
            result.success = True
            result.message = ""
            self._plan_one_shot_server.set_succeeded(result)
