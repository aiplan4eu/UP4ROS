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

from unified_planning.test.examples import get_example_problems
from up4ros.ros_interface_reader import ROSInterfaceReader
from up4ros.ros_interface_writer import ROSInterfaceWriter
from up_msgs import srv as srvs

from up4ros.up4ros_node import UP4ROSNode


def test_set_get_problem():
    node_test = UP4ROSNode(init_ros_interfaces=False)

    pb_writer = ROSInterfaceWriter()

    problems = get_example_problems()
    problem = problems['robot'].problem

    req = srvs.SetProblemRequest()
    req.problem_name = 'problem_test_robot'
    req.problem = pb_writer.convert(problem)

    response = node_test.set_problem(req)
    assert response.success
    assert(response.message == '')

    req = srvs.SetProblemRequest()
    req.problem_name = 'problem_test_robot'
    req.problem = pb_writer.convert(problem)

    response = node_test.set_problem(req)
    assert not response.success
    assert(response.message == 'Problem problem_test_robot already exists')

    pb_reader = ROSInterfaceReader()

    req2 = srvs.GetProblemRequest()
    req2.problem_name = 'problem_test_robot'

    response2 = node_test.get_problem(req2)
    assert response2.success

    problem_ret = pb_reader.convert(response2.problem)

    assert(problem == problem_ret)
