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

from mock.mock import MagicMock
from unified_planning.test.examples import get_example_problems
from up_msgs import msg as msgs
from up_msgs import srv as srvs

from up4ros.ros_interface_reader import ROSInterfaceReader
from up4ros.ros_interface_writer import ROSInterfaceWriter
from up4ros.up4ros_node import UP4ROSNode


def test_one_shot_remote_success():
    node_test = UP4ROSNode(init_ros_interfaces=False)

    # prepare the magic mock
    action_server_mock = MagicMock()

    pb_writer = ROSInterfaceWriter()
    req = srvs.SetProblemRequest()
    req.problem_name = "problem_test_robot"
    req.problem = pb_writer.convert(get_example_problems()["robot"].problem)
    response = node_test.set_problem(req)
    assert response.success
    assert response.message == ""

    problem = node_test.problems["problem_test_robot"]
    print(type(problem))

    goal_msg = msgs.PlanOneShotRemoteGoal()
    goal_msg.plan_request.problem = "problem_test_robot"

    def feedback_mock(feedback_msg):
        pb_reader = ROSInterfaceReader()
        upf_plan = pb_reader.convert(feedback_msg.plan_result.plan, problem)
        good_plan = "[move(l1, l2)]"
        assert upf_plan.__repr__() == good_plan

    action_server_mock.publish_feedback = feedback_mock

    # let's now replace the action server and plan
    node_test._plan_one_shot_remote_server = action_server_mock
    node_test.plan_one_shot_remote_callback(goal_msg)

    expected_result = msgs.PlanOneShotRemoteResult()
    expected_result.success = True
    expected_result.message = ""

    action_server_mock.set_succeeded.assert_called_with(expected_result)


def test_one_shot_remote_failure():
    node_test = UP4ROSNode(init_ros_interfaces=False)

    # prepare the magic mock
    action_server_mock = MagicMock()

    goal_msg = msgs.PlanOneShotRemoteGoal()
    goal_msg.plan_request.problem = "problem"

    # let's now replace the action server and plan
    node_test._plan_one_shot_remote_server = action_server_mock
    node_test.plan_one_shot_remote_callback(goal_msg)

    expected_result = msgs.PlanOneShotRemoteResult()
    expected_result.success = False
    expected_result.message = "Problem problem does not exist"

    action_server_mock.set_succeeded.assert_called_with(expected_result)
